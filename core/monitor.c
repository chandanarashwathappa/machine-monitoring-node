#include "monitor.h"
#include "modbus_tcp.h"

static int16_t to_dc(float c)
{
    float v = c * 10.0f;
    v = (v >= 0.0f) ? v + 0.5f : v - 0.5f;
    if (v > 32767.0f)  v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

static uint16_t to_ma(float a)
{
    float v = a * 1000.0f + 0.5f;
    if (v < 0.0f)     v = 0.0f;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)v;
}

void monitor_init(monitor_t *m)
{
    m->temp_limit_dc    = 800;
    m->current_limit_ma = 3000u;
    m->temp_hyst_dc     = 50;
    m->ideal_cycle_ms   = 2000u;
    m->cycle_timeout_ms = 6000u;

    m->temp_dc = 0;
    m->current_ma = 0u;
    m->running = false;
    m->active = 0u;
    m->unack = 0u;
    m->cycle_count = 0u;
    m->last_cycle_ms = 0u;
    m->avg_cycle_ms = 0u;
    m->run_ms = 0u;
    m->observed_ms = 0u;
    m->uptime_ms = 0u;

    m->in_raw_prev = false;
    m->in_stable = false;
    m->in_same = 0u;
    m->have_cycle = false;
    m->last_cycle_at = 0u;
    m->jam_ref_ms = 0u;
}

static void on_cycle(monitor_t *m, uint32_t now_ms)
{
    if (m->have_cycle) {
        uint32_t dt = now_ms - m->last_cycle_at;
        if (dt > 65535u) dt = 65535u;
        m->last_cycle_ms = (uint16_t)dt;
        /* exponential moving average, weight 1/8 */
        m->avg_cycle_ms = (m->avg_cycle_ms == 0u)
                        ? (uint16_t)dt
                        : (uint16_t)(((uint32_t)m->avg_cycle_ms * 7u + dt + 4u) / 8u);
    }
    m->cycle_count++;
    m->have_cycle = true;
    m->last_cycle_at = now_ms;
    m->jam_ref_ms = now_ms;
}

static void debounce_cycle_input(monitor_t *m, bool raw, uint32_t now_ms)
{
    if (raw != m->in_raw_prev) {
        m->in_raw_prev = raw;
        m->in_same = 0u;
    } else if (m->in_same < 255u) {
        m->in_same++;
    }
    if (m->in_same >= MON_DEBOUNCE_SAMPLES && raw != m->in_stable) {
        m->in_stable = raw;
        if (raw) on_cycle(m, now_ms);       /* rising edge = one cycle */
    }
}

static void update_alarms(monitor_t *m, uint32_t now_ms)
{
    uint16_t prev = m->active;
    uint16_t act = m->active;

    /* Sensor plausibility */
    bool sensor_ok = m->temp_dc >= MON_SENSOR_MIN_DC && m->temp_dc <= MON_SENSOR_MAX_DC;
    if (sensor_ok) act &= (uint16_t)~ALARM_SENSOR;
    else           act |= ALARM_SENSOR;

    /* Over-temperature with hysteresis (only judged with a valid sensor) */
    if (!sensor_ok) {
        act &= (uint16_t)~ALARM_OVER_TEMP;
    } else if (act & ALARM_OVER_TEMP) {
        if (m->temp_dc < m->temp_limit_dc - m->temp_hyst_dc) act &= (uint16_t)~ALARM_OVER_TEMP;
    } else if (m->temp_dc > m->temp_limit_dc) {
        act |= ALARM_OVER_TEMP;
    }

    /* Over-current with 10 % hysteresis */
    if (act & ALARM_OVER_CURRENT) {
        if (m->current_ma < (uint16_t)((uint32_t)m->current_limit_ma * 9u / 10u))
            act &= (uint16_t)~ALARM_OVER_CURRENT;
    } else if (m->current_ma > m->current_limit_ma) {
        act |= ALARM_OVER_CURRENT;
    }

    /* Jam: running but no cycle within the timeout */
    if (m->running && (uint32_t)(now_ms - m->jam_ref_ms) > m->cycle_timeout_ms)
        act |= ALARM_JAM;
    else
        act &= (uint16_t)~ALARM_JAM;

    m->active = act;
    m->unack |= (uint16_t)(act & ~prev);    /* newly active alarms need an ack */
}

void monitor_step(monitor_t *m, const mon_inputs_t *in, uint32_t now_ms, uint32_t dt_ms)
{
    m->temp_dc = to_dc(in->temp_c);
    m->current_ma = to_ma(in->current_a);

    if (in->running && !m->running) m->jam_ref_ms = now_ms;   /* machine started */
    m->running = in->running;

    debounce_cycle_input(m, in->cycle_input, now_ms);
    update_alarms(m, now_ms);

    m->uptime_ms += dt_ms;
    m->observed_ms += dt_ms;
    if (m->running) m->run_ms += dt_ms;
}

uint16_t monitor_state(const monitor_t *m)
{
    if (m->active != 0u) return STATE_ALARM;
    return m->running ? STATE_RUNNING : STATE_STOPPED;
}

int monitor_read_input(void *ctx, uint16_t addr, uint16_t *value)
{
    const monitor_t *m = (const monitor_t *)ctx;
    uint32_t run_s = m->run_ms / 1000u;
    uint32_t up_s = m->uptime_ms / 1000u;

    switch (addr) {
    case IR_TEMPERATURE:    *value = (uint16_t)m->temp_dc; break;
    case IR_CURRENT:        *value = m->current_ma; break;
    case IR_STATE:          *value = monitor_state(m); break;
    case IR_ALARMS_ACTIVE:  *value = m->active; break;
    case IR_ALARMS_UNACK:   *value = m->unack; break;
    case IR_CYCLE_COUNT_HI: *value = (uint16_t)(m->cycle_count >> 16); break;
    case IR_CYCLE_COUNT_LO: *value = (uint16_t)(m->cycle_count & 0xFFFFu); break;
    case IR_LAST_CYCLE_MS:  *value = m->last_cycle_ms; break;
    case IR_AVG_CYCLE_MS:   *value = m->avg_cycle_ms; break;
    case IR_RUN_TIME_HI:    *value = (uint16_t)(run_s >> 16); break;
    case IR_RUN_TIME_LO:    *value = (uint16_t)(run_s & 0xFFFFu); break;
    case IR_UPTIME_HI:      *value = (uint16_t)(up_s >> 16); break;
    case IR_UPTIME_LO:      *value = (uint16_t)(up_s & 0xFFFFu); break;
    case IR_AVAILABILITY:
        *value = (m->observed_ms == 0u) ? 0u
               : (uint16_t)((uint64_t)m->run_ms * 1000u / m->observed_ms);
        break;
    case IR_PERFORMANCE: {
        uint32_t perf = (m->avg_cycle_ms == 0u) ? 0u
                      : (uint32_t)m->ideal_cycle_ms * 1000u / m->avg_cycle_ms;
        *value = (uint16_t)(perf > 1000u ? 1000u : perf);
        break;
    }
    case IR_HEARTBEAT:      *value = (uint16_t)(up_s & 0xFFFFu); break;
    default:                return MB_ERR_ADDRESS;
    }
    return MB_OK;
}

int monitor_read_holding(void *ctx, uint16_t addr, uint16_t *value)
{
    const monitor_t *m = (const monitor_t *)ctx;
    switch (addr) {
    case HR_TEMP_LIMIT:       *value = (uint16_t)m->temp_limit_dc; break;
    case HR_CURRENT_LIMIT:    *value = m->current_limit_ma; break;
    case HR_TEMP_HYSTERESIS:  *value = (uint16_t)m->temp_hyst_dc; break;
    case HR_IDEAL_CYCLE_MS:   *value = m->ideal_cycle_ms; break;
    case HR_CYCLE_TIMEOUT_MS: *value = m->cycle_timeout_ms; break;
    case HR_COMMAND:          *value = 0u; break;
    default:                  return MB_ERR_ADDRESS;
    }
    return MB_OK;
}

int monitor_write_holding(void *ctx, uint16_t addr, uint16_t value)
{
    monitor_t *m = (monitor_t *)ctx;
    int16_t s = (int16_t)value;

    switch (addr) {
    case HR_TEMP_LIMIT:
        if (s < -200 || s > 1500) return MB_ERR_VALUE;
        m->temp_limit_dc = s;
        break;
    case HR_CURRENT_LIMIT:
        if (value < 100u || value > 20000u) return MB_ERR_VALUE;
        m->current_limit_ma = value;
        break;
    case HR_TEMP_HYSTERESIS:
        if (s < 0 || s > 200) return MB_ERR_VALUE;
        m->temp_hyst_dc = s;
        break;
    case HR_IDEAL_CYCLE_MS:
        if (value < 100u || value > 60000u) return MB_ERR_VALUE;
        m->ideal_cycle_ms = value;
        break;
    case HR_CYCLE_TIMEOUT_MS:
        if (value < 500u || value > 60000u) return MB_ERR_VALUE;
        m->cycle_timeout_ms = value;
        break;
    case HR_COMMAND:
        if (value == CMD_ACK_ALARMS) {
            m->unack = 0u;
        } else if (value == CMD_RESET_COUNTERS) {
            m->cycle_count = 0u;
            m->last_cycle_ms = 0u;
            m->avg_cycle_ms = 0u;
            m->have_cycle = false;
            m->run_ms = 0u;
            m->observed_ms = 0u;
        } else {
            return MB_ERR_VALUE;
        }
        break;
    default:
        return MB_ERR_ADDRESS;
    }
    return MB_OK;
}
