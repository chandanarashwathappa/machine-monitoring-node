/*
 * test_core.c - unit tests for the Modbus TCP protocol and the monitor logic
 *
 * Prints "PASS: N checks, 0 errors" or FAIL lines and returns 1.
 */
#include <stdio.h>
#include <string.h>

#include "modbus_tcp.h"
#include "monitor.h"

static int g_checks = 0;
static int g_errors = 0;

#define CHECK(cond) do {                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_errors++;                                                     \
            printf("FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                   \
    } while (0)

/* ======================================================= Modbus TCP ===== */
/* Fake register map: 10 input and 10 holding registers, value = 100 * kind + addr */
static uint16_t g_hold[10];

static int fake_read_input(void *ctx, uint16_t a, uint16_t *v)
{
    (void)ctx;
    if (a >= 10u) return MB_ERR_ADDRESS;
    *v = (uint16_t)(1000u + a);
    return MB_OK;
}
static int fake_read_holding(void *ctx, uint16_t a, uint16_t *v)
{
    (void)ctx;
    if (a >= 10u) return MB_ERR_ADDRESS;
    *v = g_hold[a];
    return MB_OK;
}
static int fake_write_holding(void *ctx, uint16_t a, uint16_t v)
{
    (void)ctx;
    if (a >= 10u) return MB_ERR_ADDRESS;
    if (v == 0xDEADu) return MB_ERR_VALUE;
    g_hold[a] = v;
    return MB_OK;
}
static const modbus_map_t g_fake = { NULL, fake_read_input, fake_read_holding, fake_write_holding };

static size_t mb(const uint8_t *req, size_t n, uint8_t *resp)
{
    return modbus_tcp_process(&g_fake, req, n, resp, MB_MAX_ADU_LEN);
}

static void test_modbus(void)
{
    uint8_t r[MB_MAX_ADU_LEN];
    size_t n;

    /* FC04 read 2 input registers at address 3, transaction id 0x1234, unit 7 */
    const uint8_t rd_in[] = { 0x12, 0x34, 0, 0, 0, 6, 7, 0x04, 0x00, 0x03, 0x00, 0x02 };
    n = mb(rd_in, sizeof rd_in, r);
    CHECK(n == 13);
    CHECK(r[0] == 0x12 && r[1] == 0x34);            /* transaction id echoed */
    CHECK(r[2] == 0 && r[3] == 0);                  /* protocol id */
    CHECK(r[4] == 0 && r[5] == 7);                  /* length = unit + 6 PDU bytes */
    CHECK(r[6] == 7);                               /* unit id echoed */
    CHECK(r[7] == 0x04 && r[8] == 4);               /* fc, byte count */
    CHECK(r[9] == 0x03 && r[10] == 0xEB);           /* 1003 big-endian */
    CHECK(r[11] == 0x03 && r[12] == 0xEC);          /* 1004 */

    /* Read past the end -> exception 0x02 */
    const uint8_t rd_bad[] = { 0, 1, 0, 0, 0, 6, 1, 0x04, 0x00, 0x09, 0x00, 0x02 };
    n = mb(rd_bad, sizeof rd_bad, r);
    CHECK(n == 9 && r[7] == 0x84 && r[8] == MB_EX_ILLEGAL_ADDRESS);
    CHECK(r[4] == 0 && r[5] == 3);

    /* Quantity 0 and 126 -> exception 0x03 */
    const uint8_t rd_q0[] = { 0, 2, 0, 0, 0, 6, 1, 0x03, 0x00, 0x00, 0x00, 0x00 };
    n = mb(rd_q0, sizeof rd_q0, r);
    CHECK(n == 9 && r[7] == 0x83 && r[8] == MB_EX_ILLEGAL_VALUE);
    const uint8_t rd_q126[] = { 0, 3, 0, 0, 0, 6, 1, 0x03, 0x00, 0x00, 0x00, 126 };
    n = mb(rd_q126, sizeof rd_q126, r);
    CHECK(n == 9 && r[7] == 0x83 && r[8] == MB_EX_ILLEGAL_VALUE);

    /* Unsupported function (0x01 read coils) -> exception 0x01 */
    const uint8_t coils[] = { 0, 4, 0, 0, 0, 6, 1, 0x01, 0x00, 0x00, 0x00, 0x01 };
    n = mb(coils, sizeof coils, r);
    CHECK(n == 9 && r[7] == 0x81 && r[8] == MB_EX_ILLEGAL_FUNCTION);

    /* FC06 write single, response echoes the request */
    memset(g_hold, 0, sizeof g_hold);
    const uint8_t wr1[] = { 0, 5, 0, 0, 0, 6, 1, 0x06, 0x00, 0x02, 0xAB, 0xCD };
    n = mb(wr1, sizeof wr1, r);
    CHECK(n == 12 && memcmp(r, wr1, 12) == 0);
    CHECK(g_hold[2] == 0xABCD);

    /* FC06 value rejected by the application -> exception 0x03 */
    const uint8_t wr_bad[] = { 0, 6, 0, 0, 0, 6, 1, 0x06, 0x00, 0x02, 0xDE, 0xAD };
    n = mb(wr_bad, sizeof wr_bad, r);
    CHECK(n == 9 && r[7] == 0x86 && r[8] == MB_EX_ILLEGAL_VALUE);
    CHECK(g_hold[2] == 0xABCD);

    /* FC16 write 3 registers at 4 */
    const uint8_t wrm[] = { 0, 7, 0, 0, 0, 13, 1, 0x10, 0x00, 0x04, 0x00, 0x03, 6,
                            0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C };
    n = mb(wrm, sizeof wrm, r);
    CHECK(n == 12 && r[7] == 0x10 && r[9] == 0x04 && r[11] == 0x03);
    CHECK(g_hold[4] == 10 && g_hold[5] == 11 && g_hold[6] == 12);

    /* FC16 with a wrong byte count -> exception 0x03, nothing written */
    const uint8_t wrm_bc[] = { 0, 8, 0, 0, 0, 13, 1, 0x10, 0x00, 0x00, 0x00, 0x03, 5,
                               0x00, 0x01, 0x00, 0x02, 0x00, 0x03 };
    n = mb(wrm_bc, sizeof wrm_bc, r);
    CHECK(n == 9 && r[7] == 0x90 && r[8] == MB_EX_ILLEGAL_VALUE);
    CHECK(g_hold[0] == 0);

    /* FC16 crossing the end of the map -> exception 0x02, nothing written */
    const uint8_t wrm_addr[] = { 0, 9, 0, 0, 0, 11, 1, 0x10, 0x00, 0x09, 0x00, 0x02, 4,
                                 0x00, 0x05, 0x00, 0x06 };
    n = mb(wrm_addr, sizeof wrm_addr, r);
    CHECK(n == 9 && r[7] == 0x90 && r[8] == MB_EX_ILLEGAL_ADDRESS);
    CHECK(g_hold[9] == 0);

    /* Not Modbus TCP: protocol id != 0, or MBAP length mismatch -> no response */
    const uint8_t bad_pid[] = { 0, 1, 0, 1, 0, 6, 1, 0x04, 0x00, 0x00, 0x00, 0x01 };
    CHECK(mb(bad_pid, sizeof bad_pid, r) == 0);
    const uint8_t bad_len[] = { 0, 1, 0, 0, 0, 9, 1, 0x04, 0x00, 0x00, 0x00, 0x01 };
    CHECK(mb(bad_len, sizeof bad_len, r) == 0);
}

/* ========================================================== monitor ===== */
static mon_inputs_t inputs(float t, float i, bool cyc, bool run)
{
    mon_inputs_t in = { .temp_c = t, .current_a = i, .cycle_input = cyc, .running = run };
    return in;
}

/* Run the monitor for ms milliseconds in 10 ms steps with constant inputs */
static uint32_t run_for(monitor_t *m, uint32_t now, uint32_t ms, mon_inputs_t in)
{
    for (uint32_t t = 0; t < ms; t += 10u) {
        monitor_step(m, &in, now, 10u);
        now += 10u;
    }
    return now;
}

/* One machine cycle: 200 ms pulse, then low for period-200 ms */
static uint32_t cycle(monitor_t *m, uint32_t now, uint32_t period)
{
    now = run_for(m, now, 200u, inputs(40.0f, 1.5f, true, true));
    return run_for(m, now, period - 200u, inputs(40.0f, 1.5f, false, true));
}

static uint16_t ir(monitor_t *m, uint16_t a)
{
    uint16_t v = 0;
    CHECK(monitor_read_input(m, a, &v) == MB_OK);
    return v;
}

static void test_debounce_and_cycles(void)
{
    monitor_t m;
    monitor_init(&m);
    uint32_t now = run_for(&m, 0, 100, inputs(40.0f, 1.5f, false, true));

    /* 1- and 2-sample glitches are rejected */
    now = run_for(&m, now, 10, inputs(40.0f, 1.5f, true, true));
    now = run_for(&m, now, 100, inputs(40.0f, 1.5f, false, true));
    now = run_for(&m, now, 20, inputs(40.0f, 1.5f, true, true));
    now = run_for(&m, now, 100, inputs(40.0f, 1.5f, false, true));
    CHECK(m.cycle_count == 0);

    /* Real cycles every 2500 ms */
    for (int k = 0; k < 5; k++) now = cycle(&m, now, 2500);
    CHECK(m.cycle_count == 5);
    CHECK(ir(&m, IR_LAST_CYCLE_MS) == 2500);
    CHECK(ir(&m, IR_AVG_CYCLE_MS) == 2500);
    CHECK(ir(&m, IR_PERFORMANCE) == 800);            /* 2000 / 2500 */
    CHECK(ir(&m, IR_STATE) == STATE_RUNNING);

    /* 32-bit cycle count: high word first */
    m.cycle_count = 0x00012345u;
    CHECK(ir(&m, IR_CYCLE_COUNT_HI) == 0x0001);
    CHECK(ir(&m, IR_CYCLE_COUNT_LO) == 0x2345);

    /* Availability: running the whole time = 1000 per mille */
    CHECK(ir(&m, IR_AVAILABILITY) == 1000);
    now = run_for(&m, now, (uint32_t)m.observed_ms, inputs(40.0f, 0.2f, false, false));
    CHECK(ir(&m, IR_AVAILABILITY) == 500);           /* stopped as long as it ran */
    CHECK(ir(&m, IR_STATE) == STATE_STOPPED);
    (void)now;
}

static void test_alarms(void)
{
    monitor_t m;
    monitor_init(&m);
    uint32_t now = 0;

    /* Over-temperature: on above 80.0, off below 75.0 (hysteresis 5.0) */
    now = run_for(&m, now, 50, inputs(80.5f, 1.0f, false, false));
    CHECK(m.active & ALARM_OVER_TEMP);
    CHECK(m.unack & ALARM_OVER_TEMP);
    CHECK(monitor_state(&m) == STATE_ALARM);
    now = run_for(&m, now, 50, inputs(77.0f, 1.0f, false, false));
    CHECK(m.active & ALARM_OVER_TEMP);               /* inside hysteresis band */
    now = run_for(&m, now, 50, inputs(74.0f, 1.0f, false, false));
    CHECK(!(m.active & ALARM_OVER_TEMP));
    CHECK(m.unack & ALARM_OVER_TEMP);                /* still needs an ack */

    /* Acknowledge */
    CHECK(monitor_write_holding(&m, HR_COMMAND, CMD_ACK_ALARMS) == MB_OK);
    CHECK(m.unack == 0);

    /* Ack does not clear an alarm that is still active */
    now = run_for(&m, now, 50, inputs(40.0f, 3.5f, false, false));
    CHECK(m.active & ALARM_OVER_CURRENT);
    CHECK(monitor_write_holding(&m, HR_COMMAND, CMD_ACK_ALARMS) == MB_OK);
    CHECK(m.unack == 0 && (m.active & ALARM_OVER_CURRENT));
    now = run_for(&m, now, 50, inputs(40.0f, 2.8f, false, false));
    CHECK(m.active & ALARM_OVER_CURRENT);            /* 2.8 A > 2.7 A (90 %) */
    now = run_for(&m, now, 50, inputs(40.0f, 2.6f, false, false));
    CHECK(!(m.active & ALARM_OVER_CURRENT));

    /* Sensor fault: implausible temperature, no over-temperature judged */
    now = run_for(&m, now, 50, inputs(-60.0f, 1.0f, false, false));
    CHECK(m.active & ALARM_SENSOR);
    CHECK(!(m.active & ALARM_OVER_TEMP));
    now = run_for(&m, now, 50, inputs(40.0f, 1.0f, false, false));
    CHECK(!(m.active & ALARM_SENSOR));

    /* Jam: only while running, after the cycle timeout */
    monitor_init(&m);
    now = run_for(&m, 0, 10000, inputs(40.0f, 0.2f, false, false));
    CHECK(!(m.active & ALARM_JAM));                  /* stopped: no jam */
    now = cycle(&m, now, 2000);                      /* starts running */
    now = run_for(&m, now, 3000, inputs(40.0f, 1.5f, false, true));
    CHECK(!(m.active & ALARM_JAM));                  /* ~5 s since the last cycle */
    (void)now;
}

static void test_jam_timeout(void)
{
    monitor_t m;
    monitor_init(&m);
    uint32_t now = cycle(&m, 0, 2000);               /* last cycle edge near t = 30 ms */
    now = run_for(&m, now, 3000, inputs(40.0f, 2.6f, false, true));
    CHECK(!(m.active & ALARM_JAM));                  /* ~5 s since the edge */
    now = run_for(&m, now, 1500, inputs(40.0f, 2.6f, false, true));
    CHECK(m.active & ALARM_JAM);                     /* > 6 s */
    CHECK(m.unack & ALARM_JAM);
    now = cycle(&m, now, 2000);                      /* machine moves again */
    CHECK(!(m.active & ALARM_JAM));
    (void)now;
}

static void test_registers(void)
{
    monitor_t m;
    monitor_init(&m);
    uint16_t v;

    CHECK(monitor_read_holding(&m, HR_TEMP_LIMIT, &v) == MB_OK && v == 800);
    CHECK(monitor_read_holding(&m, HR_CURRENT_LIMIT, &v) == MB_OK && v == 3000);
    CHECK(monitor_read_holding(&m, HR_COMMAND, &v) == MB_OK && v == 0);
    CHECK(monitor_read_holding(&m, HR_COUNT, &v) == MB_ERR_ADDRESS);
    CHECK(monitor_read_input(&m, IR_COUNT, &v) == MB_ERR_ADDRESS);

    CHECK(monitor_write_holding(&m, HR_TEMP_LIMIT, 650) == MB_OK && m.temp_limit_dc == 650);
    CHECK(monitor_write_holding(&m, HR_TEMP_LIMIT, (uint16_t)(int16_t)-150) == MB_OK);
    CHECK(m.temp_limit_dc == -150);                  /* negative values allowed */
    CHECK(monitor_write_holding(&m, HR_TEMP_LIMIT, 2000) == MB_ERR_VALUE);
    CHECK(monitor_write_holding(&m, HR_CYCLE_TIMEOUT_MS, 100) == MB_ERR_VALUE);
    CHECK(monitor_write_holding(&m, HR_COMMAND, 7) == MB_ERR_VALUE);
    CHECK(monitor_write_holding(&m, HR_COUNT, 1) == MB_ERR_ADDRESS);

    /* Negative temperature as int16 two's complement */
    mon_inputs_t in = inputs(-12.3f, 0.0f, false, false);
    monitor_step(&m, &in, 0, 10);
    CHECK(monitor_read_input(&m, IR_TEMPERATURE, &v) == MB_OK && (int16_t)v == -123);

    /* Reset counters */
    m.cycle_count = 42;
    m.run_ms = 5000;
    CHECK(monitor_write_holding(&m, HR_COMMAND, CMD_RESET_COUNTERS) == MB_OK);
    CHECK(m.cycle_count == 0 && m.run_ms == 0);
}

int main(void)
{
    test_modbus();
    test_debounce_and_cycles();
    test_alarms();
    test_jam_timeout();
    test_registers();

    if (g_errors == 0) {
        printf("PASS: %d checks, 0 errors\n", g_checks);
        return 0;
    }
    printf("FAIL: %d checks, %d errors\n", g_checks, g_errors);
    return 1;
}
