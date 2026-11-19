"""Machine-monitoring behaviour: cycles, OEE figures and alarms, tested over
Modbus with fault injection through the simulator registers (100..105)."""
import time

import registers as R


def test_counts_cycles_and_measures_cycle_time(mb):
    start = mb.read_inputs()["cycle_count"]
    d = mb.wait_for(lambda d: d["cycle_count"] >= start + 3, 10.0, "3 more cycles")
    assert abs(d["last_cycle_s"] - 2.2) <= 0.05          # simulated machine: 2.2 s
    assert abs(d["avg_cycle_s"] - 2.2) <= 0.05
    # performance = ideal 2.0 s / actual 2.2 s = 90.9 %
    assert 89.0 <= d["performance_pct"] <= 92.0
    assert d["availability_pct"] >= 99.0


def test_cycle_time_follows_machine_speed(mb):
    mb.write(R.SIM_PERIOD, 1500)
    time.sleep(3.5)
    mb.wait_for(lambda d: abs(d["last_cycle_s"] - 1.5) <= 0.05, 5.0, "1.5 s cycles")
    # the average follows within a few cycles; faster than ideal is capped at 100 %
    mb.wait_for(lambda d: d["performance_pct"] == 100.0, 15.0, "performance 100 %")


def test_stopped_machine(mb):
    mb.write(R.SIM_RUNNING, 0)
    d = mb.wait_for(lambda d: d["state_name"] == "STOPPED", 1.0, "STOPPED")
    count = d["cycle_count"]
    time.sleep(3.0)
    d = mb.read_inputs()
    assert d["cycle_count"] == count                     # no production
    assert d["alarms_active"] == 0                       # no jam while stopped
    assert d["availability_pct"] < 100.0


def test_jam_alarm_and_acknowledge(mb):
    mb.write(R.HR_CYCLE_TIMEOUT_MS, 3000)
    mb.write(R.SIM_JAM, 1)
    d = mb.wait_for(lambda d: d["alarms_active"] & R.ALARM_JAM, 6.0, "JAM alarm")
    assert d["state_name"] == "ALARM"
    assert d["alarms_unack"] & R.ALARM_JAM

    mb.write(R.SIM_JAM, 0)                               # machine moves again
    d = mb.wait_for(lambda d: not d["alarms_active"] & R.ALARM_JAM, 4.0, "JAM cleared")
    assert d["alarms_unack"] & R.ALARM_JAM               # still waiting for ack
    assert d["state_name"] == "RUNNING"

    mb.write(R.HR_COMMAND, R.CMD_ACK_ALARMS)
    assert mb.read_inputs()["alarms_unack"] == 0


def test_over_temperature_with_hysteresis(mb):
    mb.write(R.SIM_HEAT, 400)                            # +40 degC
    d = mb.wait_for(lambda d: d["alarms_active"] & R.ALARM_OVER_TEMP, 15.0, "OVER_TEMP")
    assert d["temperature_c"] > 80.0

    mb.write(R.SIM_HEAT, 0)
    seen_in_band = False
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        d = mb.read_inputs()
        t, active = d["temperature_c"], d["alarms_active"] & R.ALARM_OVER_TEMP
        if 75.5 <= t <= 79.5:
            assert active, f"alarm cleared too early at {t} degC"
            seen_in_band = True
        if not active:
            assert t < 75.0, f"alarm cleared at {t} degC, expected below 75.0"
            break
        time.sleep(0.03)
    else:
        raise AssertionError("over-temperature alarm never cleared")
    assert seen_in_band, "temperature never sampled inside the hysteresis band"


def test_over_current_alarm(mb):
    mb.write(R.SIM_OVERLOAD, 2000)                       # 1.5 A + 2.0 A
    d = mb.wait_for(lambda d: d["alarms_active"] & R.ALARM_OVER_CURRENT, 1.0, "OVER_CURRENT")
    assert d["current_a"] > 3.0
    mb.write(R.SIM_OVERLOAD, 0)
    mb.wait_for(lambda d: not d["alarms_active"] & R.ALARM_OVER_CURRENT, 1.0, "cleared")


def test_sensor_fault(mb):
    mb.write(R.SIM_SENSOR_OPEN, 1)
    d = mb.wait_for(lambda d: d["alarms_active"] & R.ALARM_SENSOR, 1.0, "SENSOR alarm")
    assert not d["alarms_active"] & R.ALARM_OVER_TEMP
    assert d["temperature_c"] == -60.0


def test_reset_counters(mb):
    mb.wait_for(lambda d: d["cycle_count"] >= 2, 6.0, "some cycles")
    mb.write(R.HR_COMMAND, R.CMD_RESET_COUNTERS)
    d = mb.read_inputs()
    assert d["cycle_count"] <= 1
    assert d["run_time_s"] == 0
