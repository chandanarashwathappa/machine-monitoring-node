/*
 * machine_sim.h - simulated production machine (SIL only)
 *
 * A machine (for example a press or die-casting cell) that makes one part
 * every cycle_period_ms while running:
 *   - cycle sensor: 200 ms high pulse at the start of every cycle
 *   - motor current: 1.5 A running, 2.6 A when jammed (stalled), 0.2 A stopped
 *   - temperature: first-order model, settles at 25 degC + 20 degC while running
 *
 * Fault injection (written through simulator-only Modbus registers 100..105):
 *   heat_offset_c   extra heat, e.g. +40 degC -> over-temperature
 *   overload_a      extra motor current      -> over-current
 *   jam             machine stuck: no more cycles
 *   sensor_open     temperature reads -60 degC
 *   running         machine on / off
 *   cycle_period_ms production speed
 */
#ifndef MACHINE_SIM_H
#define MACHINE_SIM_H

#include <stdbool.h>
#include <stdint.h>

#define SIM_TAU_S            3.0f
#define SIM_AMBIENT_C       25.0f
#define SIM_RUN_HEAT_C      20.0f
#define SIM_PULSE_MS         200u
#define SIM_DEFAULT_PERIOD  2200u

typedef struct {
    bool     running;
    bool     jam;
    bool     sensor_open;
    float    heat_offset_c;
    float    overload_a;
    uint32_t cycle_period_ms;
    float    temp_c;
    uint32_t phase_ms;
} machine_t;

void  machine_init(machine_t *m);
void  machine_step(machine_t *m, uint32_t dt_ms);
bool  machine_cycle_input(const machine_t *m);
float machine_current(const machine_t *m);
float machine_temp_reading(const machine_t *m);

#endif /* MACHINE_SIM_H */
