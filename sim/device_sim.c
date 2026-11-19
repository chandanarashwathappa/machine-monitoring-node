/*
 * device_sim.c - Software-in-the-Loop field device (Linux / WSL)
 *
 * Runs the real monitoring core (core/) every 10 ms against the simulated
 * machine and serves Modbus TCP on 127.0.0.1 (default port 1502, so no root
 * rights are needed for port 502). Up to 4 clients at the same time, e.g. the
 * OPC UA gateway plus a test or a SCADA tool.
 *
 * Usage: device_sim [--port N]
 *
 * Simulator-only holding registers (fault injection, not on real hardware):
 *   100 heat offset, int16, 0.1 degC     103 machine running (0/1)
 *   101 motor overload, mA               104 temperature sensor open (0/1)
 *   102 jam (0/1)                        105 cycle period, ms (300..60000)
 */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "machine_sim.h"
#include "modbus_tcp.h"
#include "monitor.h"

#define TICK_MS        10u
#define DEFAULT_PORT   1502
#define MAX_CLIENTS    4

enum { SIM_HEAT = 100, SIM_OVERLOAD, SIM_JAM, SIM_RUNNING, SIM_SENSOR_OPEN, SIM_PERIOD, SIM_END };

typedef struct {
    int     fd;
    uint8_t buf[2 * MB_MAX_ADU_LEN];
    size_t  len;
} client_t;

static volatile sig_atomic_t g_running = 1;
static monitor_t g_mon;
static machine_t g_mach;

static void on_signal(int sig) { (void)sig; g_running = 0; }

static uint64_t mono_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* ---- register map: monitor registers + simulator registers 100..105 ---- */
static int sim_read_holding(void *ctx, uint16_t addr, uint16_t *v)
{
    if (addr < SIM_HEAT) return monitor_read_holding(ctx, addr, v);
    switch (addr) {
    case SIM_HEAT:        *v = (uint16_t)(int16_t)(g_mach.heat_offset_c * 10.0f); break;
    case SIM_OVERLOAD:    *v = (uint16_t)(g_mach.overload_a * 1000.0f); break;
    case SIM_JAM:         *v = g_mach.jam ? 1u : 0u; break;
    case SIM_RUNNING:     *v = g_mach.running ? 1u : 0u; break;
    case SIM_SENSOR_OPEN: *v = g_mach.sensor_open ? 1u : 0u; break;
    case SIM_PERIOD:      *v = (uint16_t)g_mach.cycle_period_ms; break;
    default:              return MB_ERR_ADDRESS;
    }
    return MB_OK;
}

static int sim_write_holding(void *ctx, uint16_t addr, uint16_t v)
{
    if (addr < SIM_HEAT) return monitor_write_holding(ctx, addr, v);
    switch (addr) {
    case SIM_HEAT:        g_mach.heat_offset_c = (float)(int16_t)v / 10.0f; break;
    case SIM_OVERLOAD:    g_mach.overload_a = (float)v / 1000.0f; break;
    case SIM_JAM:         if (v > 1u) return MB_ERR_VALUE; g_mach.jam = (v != 0u); break;
    case SIM_RUNNING:     if (v > 1u) return MB_ERR_VALUE; g_mach.running = (v != 0u); break;
    case SIM_SENSOR_OPEN: if (v > 1u) return MB_ERR_VALUE; g_mach.sensor_open = (v != 0u); break;
    case SIM_PERIOD:      if (v < 300u) return MB_ERR_VALUE; g_mach.cycle_period_ms = v; break;
    default:              return MB_ERR_ADDRESS;
    }
    return MB_OK;
}

static const modbus_map_t g_map = {
    .ctx = &g_mon,
    .read_input = monitor_read_input,
    .read_holding = sim_read_holding,
    .write_holding = sim_write_holding,
};

/* ---- TCP handling ---- */
static void client_close(client_t *c)
{
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
    c->len = 0;
}

static void client_send(client_t *c, const uint8_t *p, size_t n)
{
    while (c->fd >= 0 && n > 0) {
        ssize_t w = send(c->fd, p, n, MSG_NOSIGNAL);
        if (w < 0 && errno == EINTR) continue;
        if (w <= 0) { client_close(c); return; }
        p += w;
        n -= (size_t)w;
    }
}

/* Cut complete Modbus TCP frames out of the stream and answer them */
static void client_process(client_t *c)
{
    while (c->fd >= 0 && c->len >= MB_MBAP_LEN) {
        uint16_t mlen = (uint16_t)((c->buf[4] << 8) | c->buf[5]);
        if (mlen < 2u || mlen > MB_MAX_ADU_LEN - 6u) { client_close(c); return; }
        size_t total = 6u + mlen;
        if (c->len < total) return;                 /* wait for the rest */

        uint8_t resp[MB_MAX_ADU_LEN];
        size_t n = modbus_tcp_process(&g_map, c->buf, total, resp, sizeof resp);
        if (n == 0u) { client_close(c); return; }   /* not Modbus TCP */
        client_send(c, resp, n);

        memmove(c->buf, c->buf + total, c->len - total);
        c->len -= total;
    }
}

int main(int argc, char **argv)
{
    int port = DEFAULT_PORT;
    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) port = atoi(argv[++i]);

    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);

    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(lfd, (struct sockaddr *)&addr, sizeof addr) < 0) { perror("bind"); return 1; }
    if (listen(lfd, MAX_CLIENTS) < 0) { perror("listen"); return 1; }

    monitor_init(&g_mon);
    machine_init(&g_mach);

    client_t cli[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) { cli[i].fd = -1; cli[i].len = 0; }

    printf("Modbus TCP device simulator listening on 127.0.0.1:%d\n", port);
    fflush(stdout);

    uint64_t start = mono_ms();
    uint64_t next_tick = start;

    while (g_running) {
        uint64_t now = mono_ms();
        if (now >= next_tick) {
            mon_inputs_t in = {
                .temp_c = machine_temp_reading(&g_mach),
                .current_a = machine_current(&g_mach),
                .cycle_input = machine_cycle_input(&g_mach),
                .running = g_mach.running,
            };
            monitor_step(&g_mon, &in, (uint32_t)(next_tick - start), TICK_MS);
            machine_step(&g_mach, TICK_MS);
            next_tick += TICK_MS;
            if (now > next_tick + 1000u) next_tick = now;
            continue;
        }

        fd_set rd;
        FD_ZERO(&rd);
        FD_SET(lfd, &rd);
        int maxfd = lfd;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (cli[i].fd >= 0) {
                FD_SET(cli[i].fd, &rd);
                if (cli[i].fd > maxfd) maxfd = cli[i].fd;
            }
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = (suseconds_t)((next_tick - now) * 1000u) };
        if (select(maxfd + 1, &rd, NULL, NULL, &tv) <= 0) continue;

        if (FD_ISSET(lfd, &rd)) {
            int cfd = accept(lfd, NULL, NULL);
            if (cfd >= 0) {
                int slot = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) if (cli[i].fd < 0) { slot = i; break; }
                if (slot < 0) {
                    close(cfd);                     /* too many clients */
                } else {
                    setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
                    cli[slot].fd = cfd;
                    cli[slot].len = 0;
                }
            }
        }
        for (int i = 0; i < MAX_CLIENTS; i++) {
            client_t *c = &cli[i];
            if (c->fd < 0 || !FD_ISSET(c->fd, &rd)) continue;
            ssize_t n = recv(c->fd, c->buf + c->len, sizeof c->buf - c->len, 0);
            if (n <= 0) { client_close(c); continue; }
            c->len += (size_t)n;
            client_process(c);
        }
    }

    for (int i = 0; i < MAX_CLIENTS; i++) client_close(&cli[i]);
    close(lfd);
    return 0;
}
