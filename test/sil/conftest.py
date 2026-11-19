"""pytest fixtures for the Software-in-the-Loop tests.

device   - a fresh device simulator process (Modbus TCP server) per test
mb       - Device helper: pymodbus client + decoding + fault injection
gateway  - the OPC UA gateway process connected to the device
opc      - OpcMachine helper: asyncua client browsing the gateway's model
"""
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
from pymodbus.client import ModbusTcpClient

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "gateway"))

import registers as R  # noqa: E402

DEVICE_SIM = ROOT / "build" / "device_sim"
GATEWAY = ROOT / "gateway" / "gateway.py"


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def start_process(cmd, ready_text, timeout=15.0):
    """Start a process and wait for its 'ready' line on stdout."""
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines = []
    deadline = time.monotonic() + timeout

    def reader():
        for line in proc.stdout:
            lines.append(line)

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    while time.monotonic() < deadline:
        if any(ready_text in line for line in lines):
            return proc
        if proc.poll() is not None:
            break
        time.sleep(0.05)
    proc.kill()
    raise RuntimeError(f"{cmd[0]} did not start; output: {''.join(lines)!r}")


def stop_process(proc):
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def wait_until(fn, timeout, what="condition", interval=0.05):
    """Call fn() until it returns a truthy value; raise AssertionError on timeout."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = fn()
        if last:
            return last
        time.sleep(interval)
    raise AssertionError(f"timeout after {timeout:.1f} s waiting for {what}")


class Device:
    """Talks to the device simulator with pymodbus, like a SCADA client."""

    def __init__(self, port: int):
        self.port = port
        self.client = ModbusTcpClient("127.0.0.1", port=port, timeout=2)
        assert self.client.connect(), "cannot connect to the device simulator"

    def read_inputs(self):
        rr = self.client.read_input_registers(0, count=R.IR_COUNT, slave=1)
        assert not rr.isError(), rr
        return R.decode_inputs(rr.registers)

    def read_holding(self, addr, count=1):
        rr = self.client.read_holding_registers(addr, count=count, slave=1)
        assert not rr.isError(), rr
        return rr.registers

    def write(self, addr, value):
        rr = self.client.write_register(addr, R.u16(value), slave=1)
        assert not rr.isError(), rr

    def wait_for(self, pred, timeout, what="condition"):
        return wait_until(lambda: (lambda d: d if pred(d) else None)(self.read_inputs()), timeout, what)

    def close(self):
        self.client.close()


class SimProcess:
    def __init__(self, port, proc):
        self.port = port
        self.proc = proc


@pytest.fixture
def device():
    port = free_port()
    proc = start_process([str(DEVICE_SIM), "--port", str(port)], "listening")
    yield SimProcess(port, proc)
    stop_process(proc)


@pytest.fixture
def mb(device):
    d = Device(device.port)
    yield d
    d.close()


@pytest.fixture
def gateway(device):
    opc_port = free_port()
    proc = start_process([sys.executable, str(GATEWAY), "--device-port", str(device.port),
                          "--opc-port", str(opc_port), "--poll", "0.2"], "running at", timeout=30)
    yield f"opc.tcp://127.0.0.1:{opc_port}/machine/"
    stop_process(proc)


class OpcMachine:
    """Browses the gateway's OPC UA model with the asyncua (sync) client."""

    def __init__(self, url: str):
        from asyncua.sync import Client
        self.client = Client(url)
        self.client.connect()
        self.idx = self.client.get_namespace_index("urn:machine-monitoring-node")
        self.machine = self.client.nodes.objects.get_child([f"{self.idx}:Machine"])

    def node(self, *path):
        return self.machine.get_child([f"{self.idx}:{p}" for p in path])

    def get(self, *path):
        return self.node(*path).read_value()

    def call(self, method):
        return self.machine.call_method(f"{self.idx}:{method}")

    def close(self):
        self.client.disconnect()


@pytest.fixture
def opc(gateway):
    m = OpcMachine(gateway)
    wait_until(lambda: m.get("DeviceConnected"), 5.0, "gateway connected to the device")
    yield m
    m.close()
