#include "machine_sim.h"

void machine_init(machine_t *m)
{
    m->running = true;
    m->jam = false;
    m->sensor_open = false;
    m->heat_offset_c = 0.0f;
    m->overload_a = 0.0f;
    m->cycle_period_ms = SIM_DEFAULT_PERIOD;
    m->temp_c = SIM_AMBIENT_C;
    m->phase_ms = 0u;
}

void machine_step(machine_t *m, uint32_t dt_ms)
{
    if (m->running && !m->jam) {
        m->phase_ms += dt_ms;
        if (m->phase_ms >= m->cycle_period_ms) m->phase_ms -= m->cycle_period_ms;
    }
    float target = SIM_AMBIENT_C + (m->running ? SIM_RUN_HEAT_C : 0.0f) + m->heat_offset_c;
    m->temp_c += ((float)dt_ms / 1000.0f) * (target - m->temp_c) / SIM_TAU_S;
}

bool machine_cycle_input(const machine_t *m)
{
    /* high for the first 200 ms of each cycle; frozen if the machine stops */
    return m->phase_ms < SIM_PULSE_MS;
}

float machine_current(const machine_t *m)
{
    float base = !m->running ? 0.2f : (m->jam ? 2.6f : 1.5f);
    return base + m->overload_a;
}

float machine_temp_reading(const machine_t *m)
{
    return m->sensor_open ? -60.0f : m->temp_c;
}
