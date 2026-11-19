"""Modbus register map of the machine-monitoring node (see docs/register_map.md)."""

# Input registers (function code 0x04)
IR_TEMPERATURE = 0      # int16, 0.1 degC
IR_CURRENT = 1          # uint16, mA
IR_STATE = 2            # 0 STOPPED, 1 RUNNING, 2 ALARM
IR_ALARMS_ACTIVE = 3
IR_ALARMS_UNACK = 4
IR_CYCLE_COUNT_HI = 5
IR_CYCLE_COUNT_LO = 6
IR_LAST_CYCLE_MS = 7
IR_AVG_CYCLE_MS = 8
IR_RUN_TIME_HI = 9
IR_RUN_TIME_LO = 10
IR_UPTIME_HI = 11
IR_UPTIME_LO = 12
IR_AVAILABILITY = 13    # per mille
IR_PERFORMANCE = 14     # per mille
IR_HEARTBEAT = 15
IR_COUNT = 16

# Holding registers (function codes 0x03 / 0x06 / 0x10)
HR_TEMP_LIMIT = 0       # int16, 0.1 degC
HR_CURRENT_LIMIT = 1    # mA
HR_TEMP_HYSTERESIS = 2  # 0.1 degC
HR_IDEAL_CYCLE_MS = 3
HR_CYCLE_TIMEOUT_MS = 4
HR_COMMAND = 5          # 1 = acknowledge alarms, 2 = reset counters
HR_COUNT = 6

CMD_ACK_ALARMS = 1
CMD_RESET_COUNTERS = 2

# Simulator-only holding registers (fault injection)
SIM_HEAT = 100          # int16, 0.1 degC
SIM_OVERLOAD = 101      # mA
SIM_JAM = 102
SIM_RUNNING = 103
SIM_SENSOR_OPEN = 104
SIM_PERIOD = 105        # ms

ALARM_OVER_TEMP = 0x01
ALARM_OVER_CURRENT = 0x02
ALARM_JAM = 0x04
ALARM_SENSOR = 0x08
ALARM_NAMES = {
    ALARM_OVER_TEMP: "OverTemperature",
    ALARM_OVER_CURRENT: "OverCurrent",
    ALARM_JAM: "Jam",
    ALARM_SENSOR: "SensorFault",
}

STATE_NAMES = {0: "STOPPED", 1: "RUNNING", 2: "ALARM"}


def s16(v: int) -> int:
    """uint16 register -> signed int16."""
    return v - 0x10000 if v >= 0x8000 else v


def u16(v: int) -> int:
    """signed int16 -> uint16 register."""
    return v & 0xFFFF


def u32(hi: int, lo: int) -> int:
    """Two registers, high word first -> uint32."""
    return (hi << 16) | lo


def decode_inputs(regs):
    """Decode the 16 input registers into engineering units."""
    return {
        "temperature_c": s16(regs[IR_TEMPERATURE]) / 10.0,
        "current_a": regs[IR_CURRENT] / 1000.0,
        "state": regs[IR_STATE],
        "state_name": STATE_NAMES.get(regs[IR_STATE], "?"),
        "alarms_active": regs[IR_ALARMS_ACTIVE],
        "alarms_unack": regs[IR_ALARMS_UNACK],
        "cycle_count": u32(regs[IR_CYCLE_COUNT_HI], regs[IR_CYCLE_COUNT_LO]),
        "last_cycle_s": regs[IR_LAST_CYCLE_MS] / 1000.0,
        "avg_cycle_s": regs[IR_AVG_CYCLE_MS] / 1000.0,
        "run_time_s": u32(regs[IR_RUN_TIME_HI], regs[IR_RUN_TIME_LO]),
        "uptime_s": u32(regs[IR_UPTIME_HI], regs[IR_UPTIME_LO]),
        "availability_pct": regs[IR_AVAILABILITY] / 10.0,
        "performance_pct": regs[IR_PERFORMANCE] / 10.0,
        "heartbeat": regs[IR_HEARTBEAT],
    }
