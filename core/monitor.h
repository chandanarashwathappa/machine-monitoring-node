/*
 * monitor.h - machine-monitoring logic (hardware independent)
 *
 * Inputs (sampled every control period, e.g. 10 ms):
 *   temperature, motor current, cycle sensor (digital), machine-running signal
 *
 * It computes:
 *   - cycle count, last and average cycle time (debounced cycle sensor)
 *   - run time, availability (run time / observed time) and
 *     performance (ideal cycle time / average cycle time): two OEE factors
 *   - alarms: over-temperature (with hysteresis), over-current (with 10 %
 *     hysteresis), jam (no cycle within the timeout while running),
 *     sensor fault (implausible temperature)
 *   - "unacknowledged" alarm bits, set when an alarm becomes active and
 *     cleared only by an acknowledge command
 *
 * Everything is exposed as Modbus registers: see docs/register_map.md.
 */
#ifndef MONITOR_H
#define MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/* Input registers (read only, function code 0x04) */
enum {
    IR_TEMPERATURE = 0,     /* int16, 0.1 degC */
    IR_CURRENT,             /* uint16, mA */
    IR_STATE,               /* 0 STOPPED, 1 RUNNING, 2 ALARM */
    IR_ALARMS_ACTIVE,       /* bit field, ALARM_* */
    IR_ALARMS_UNACK,        /* bit field, ALARM_* */
    IR_CYCLE_COUNT_HI,      /* uint32 cycle count, high word first */
    IR_CYCLE_COUNT_LO,
    IR_LAST_CYCLE_MS,       /* uint16, ms */
    IR_AVG_CYCLE_MS,        /* uint16, ms (moving average) */
    IR_RUN_TIME_HI,         /* uint32 run time, s, high word first */
    IR_RUN_TIME_LO,
    IR_UPTIME_HI,           /* uint32 uptime, s, high word first */
    IR_UPTIME_LO,
    IR_AVAILABILITY,        /* 0..1000 per mille */
    IR_PERFORMANCE,         /* 0..1000 per mille */
    IR_HEARTBEAT,           /* +1 every second */
    IR_COUNT
};

/* Holding registers (read/write, function codes 0x03, 0x06, 0x10) */
enum {
    HR_TEMP_LIMIT = 0,      /* int16, 0.1 degC,  -200..1500, default 800  */
    HR_CURRENT_LIMIT,       /* uint16, mA,        100..20000, default 3000 */
    HR_TEMP_HYSTERESIS,     /* int16, 0.1 degC,     0..200,  default 50   */
    HR_IDEAL_CYCLE_MS,      /* uint16, ms,         100..60000, default 2000 */
    HR_CYCLE_TIMEOUT_MS,    /* uint16, ms,         500..60000, default 6000 */
    HR_COMMAND,             /* write 1 = acknowledge alarms, 2 = reset counters; reads 0 */
    HR_COUNT
};

#define ALARM_OVER_TEMP     0x0001u
#define ALARM_OVER_CURRENT  0x0002u
#define ALARM_JAM           0x0004u
#define ALARM_SENSOR        0x0008u

#define STATE_STOPPED       0u
#define STATE_RUNNING       1u
#define STATE_ALARM         2u

#define CMD_ACK_ALARMS      1u
#define CMD_RESET_COUNTERS  2u

#define MON_DEBOUNCE_SAMPLES   3u      /* input must be stable 3 more samples */
#define MON_SENSOR_MIN_DC   (-200)     /* -20.0 degC */
#define MON_SENSOR_MAX_DC    1500      /* 150.0 degC */

typedef struct {
    float temp_c;
    float current_a;
    bool  cycle_input;      /* cycle / proximity sensor */
    bool  running;          /* machine-running signal */
} mon_inputs_t;

typedef struct {
    /* configuration (holding registers) */
    int16_t  temp_limit_dc;
    uint16_t current_limit_ma;
    int16_t  temp_hyst_dc;
    uint16_t ideal_cycle_ms;
    uint16_t cycle_timeout_ms;

    /* live values */
    int16_t  temp_dc;
    uint16_t current_ma;
    bool     running;
    uint16_t active;
    uint16_t unack;
    uint32_t cycle_count;
    uint16_t last_cycle_ms;
    uint16_t avg_cycle_ms;
    uint32_t run_ms;
    uint32_t observed_ms;
    uint32_t uptime_ms;

    /* internal */
    bool     in_raw_prev;
    bool     in_stable;
    uint8_t  in_same;
    bool     have_cycle;
    uint32_t last_cycle_at;
    uint32_t jam_ref_ms;
} monitor_t;

void     monitor_init(monitor_t *m);
void     monitor_step(monitor_t *m, const mon_inputs_t *in, uint32_t now_ms, uint32_t dt_ms);
uint16_t monitor_state(const monitor_t *m);

/* Register access, return MB_OK / MB_ERR_ADDRESS / MB_ERR_VALUE (modbus_tcp.h) */
int monitor_read_input(void *ctx, uint16_t addr, uint16_t *value);
int monitor_read_holding(void *ctx, uint16_t addr, uint16_t *value);
int monitor_write_holding(void *ctx, uint16_t addr, uint16_t value);

#endif /* MONITOR_H */
