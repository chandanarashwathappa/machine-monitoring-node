"""Modbus TCP conformance of the field device, tested with pymodbus (an
independent implementation) and with raw frames for the error cases."""
import socket
import struct
import threading

import registers as R
from conftest import Device


def raw_request(port, pdu, tid=0x4242, unit=1):
    """Send one raw Modbus TCP request, return (tid, unit, response PDU)."""
    adu = struct.pack(">HHHB", tid, 0, len(pdu) + 1, unit) + pdu
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        s.sendall(adu)
        hdr = s.recv(7)
        r_tid, pid, length, r_unit = struct.unpack(">HHHB", hdr)
        assert pid == 0
        body = b""
        while len(body) < length - 1:
            body += s.recv(length - 1 - len(body))
    return r_tid, r_unit, body


def test_read_all_input_registers(mb):
    d = mb.read_inputs()
    assert 20.0 <= d["temperature_c"] <= 50.0
    assert d["state_name"] == "RUNNING"
    assert d["alarms_active"] == 0
    assert d["uptime_s"] < 60


def test_holding_register_defaults(mb):
    assert mb.read_holding(0, R.HR_COUNT) == [800, 3000, 50, 2000, 6000, 0]


def test_write_single_and_multiple(mb):
    mb.write(R.HR_TEMP_LIMIT, 650)
    assert mb.read_holding(R.HR_TEMP_LIMIT) == [650]
    rr = mb.client.write_registers(R.HR_IDEAL_CYCLE_MS, [1800, 5000], slave=1)
    assert not rr.isError()
    assert mb.read_holding(R.HR_IDEAL_CYCLE_MS, 2) == [1800, 5000]


def test_negative_value_is_twos_complement(mb):
    mb.write(R.HR_TEMP_LIMIT, -150)
    assert R.s16(mb.read_holding(R.HR_TEMP_LIMIT)[0]) == -150


def test_illegal_address_exception(mb):
    rr = mb.client.read_input_registers(R.IR_COUNT, count=1, slave=1)
    assert rr.isError() and rr.exception_code == 2
    rr = mb.client.read_holding_registers(4, count=5, slave=1)   # crosses the end
    assert rr.isError() and rr.exception_code == 2


def test_illegal_value_exception(mb):
    rr = mb.client.write_register(R.HR_CYCLE_TIMEOUT_MS, 100, slave=1)   # below 500
    assert rr.isError() and rr.exception_code == 3
    rr = mb.client.write_register(R.HR_COMMAND, 9, slave=1)               # unknown command
    assert rr.isError() and rr.exception_code == 3


def test_illegal_function_exception(mb):
    rr = mb.client.read_coils(0, count=1, slave=1)
    assert rr.isError() and rr.exception_code == 1


def test_raw_frame_header_and_quantity_limits(device):
    tid, unit, pdu = raw_request(device.port, bytes([0x04, 0, 0, 0, 2]), tid=0xBEEF, unit=17)
    assert tid == 0xBEEF and unit == 17            # echoed
    assert pdu[0] == 0x04 and pdu[1] == 4          # 2 registers = 4 bytes

    _, _, pdu = raw_request(device.port, bytes([0x03, 0, 0, 0, 0]))     # quantity 0
    assert pdu == bytes([0x83, 0x03])
    _, _, pdu = raw_request(device.port, bytes([0x03, 0, 0, 0, 126]))   # quantity 126
    assert pdu == bytes([0x83, 0x03])


def test_several_clients_at_the_same_time(device):
    clients = [Device(device.port) for _ in range(3)]
    errors = []

    def worker(c):
        try:
            for _ in range(20):
                c.read_inputs()
        except Exception as exc:          # noqa: BLE001
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(c,)) for c in clients]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    for c in clients:
        c.close()
    assert not errors, errors
