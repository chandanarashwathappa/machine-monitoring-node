"""End-to-end: device simulator -> Modbus TCP -> gateway -> OPC UA client."""
from asyncua import ua

import registers as R
from conftest import wait_until


def test_opc_ua_values_match_modbus(mb, opc):
    d = mb.read_inputs()
    assert abs(opc.get("Temperature") - d["temperature_c"]) <= 1.0
    assert opc.get("State") == "RUNNING"
    assert opc.get("StateCode") == 1
    assert opc.get("Config", "TemperatureLimit") == 80.0


def test_cycle_count_updates(opc):
    start = opc.get("CycleCount")
    wait_until(lambda: opc.get("CycleCount") >= start + 2, 8.0, "CycleCount to increase")
    assert abs(opc.get("LastCycleTime") - 2.2) <= 0.05


def test_alarm_reaches_opc_ua_and_ack_method(mb, opc):
    mb.write(R.SIM_OVERLOAD, 2000)
    wait_until(lambda: opc.get("Alarms", "OverCurrent"), 2.0, "OverCurrent in OPC UA")
    assert opc.get("Alarms", "OverCurrentUnacknowledged")
    assert opc.get("State") == "ALARM"

    mb.write(R.SIM_OVERLOAD, 0)
    wait_until(lambda: not opc.get("Alarms", "OverCurrent"), 2.0, "OverCurrent cleared")
    opc.call("AcknowledgeAlarms")
    assert mb.read_inputs()["alarms_unack"] == 0          # went through to Modbus
    wait_until(lambda: not opc.get("Alarms", "OverCurrentUnacknowledged"), 2.0, "ack in OPC UA")


def test_write_limit_from_opc_ua(mb, opc):
    opc.node("Config", "TemperatureLimit").write_value(ua.Variant(65.5, ua.VariantType.Double))
    wait_until(lambda: mb.read_holding(R.HR_TEMP_LIMIT) == [655], 2.0, "limit written to the device")


def test_reset_counters_method(mb, opc):
    wait_until(lambda: opc.get("CycleCount") >= 2, 8.0, "some cycles")
    opc.call("ResetCounters")
    assert mb.read_inputs()["cycle_count"] <= 1


def test_device_disconnect_is_reported(opc, device):
    device.proc.terminate()                                # field device goes offline
    wait_until(lambda: not opc.get("DeviceConnected"), 5.0, "DeviceConnected = false")
