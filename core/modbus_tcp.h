/*
 * modbus_tcp.h - Modbus TCP server protocol (transport independent)
 *
 * Processes one complete Modbus TCP request (ADU = 7-byte MBAP header + PDU)
 * and builds the response. Sockets are handled by the platform (PC simulator
 * or ESP32), so this file has no I/O and can be unit tested directly.
 *
 * Supported function codes:
 *   0x03 Read Holding Registers     (1..125 registers)
 *   0x04 Read Input Registers       (1..125 registers)
 *   0x06 Write Single Register
 *   0x10 Write Multiple Registers   (1..123 registers)
 * Exception responses:
 *   0x01 Illegal Function, 0x02 Illegal Data Address, 0x03 Illegal Data Value
 */
#ifndef MODBUS_TCP_H
#define MODBUS_TCP_H

#include <stddef.h>
#include <stdint.h>

#define MB_FC_READ_HOLDING        0x03u
#define MB_FC_READ_INPUT          0x04u
#define MB_FC_WRITE_SINGLE        0x06u
#define MB_FC_WRITE_MULTIPLE      0x10u

#define MB_EX_ILLEGAL_FUNCTION    0x01u
#define MB_EX_ILLEGAL_ADDRESS     0x02u
#define MB_EX_ILLEGAL_VALUE       0x03u

#define MB_MBAP_LEN               7u
#define MB_MAX_ADU_LEN            260u

/* Register access callbacks return:
 *    0  ok
 *   -1  address does not exist   -> exception 0x02
 *   -2  value not allowed        -> exception 0x03 (writes only) */
#define MB_OK              0
#define MB_ERR_ADDRESS    -1
#define MB_ERR_VALUE      -2

typedef struct {
    void *ctx;
    int (*read_input)(void *ctx, uint16_t addr, uint16_t *value);
    int (*read_holding)(void *ctx, uint16_t addr, uint16_t *value);
    int (*write_holding)(void *ctx, uint16_t addr, uint16_t value);
} modbus_map_t;

/* Returns the response length in bytes, or 0 if the request is not a valid
 * Modbus TCP frame (wrong protocol id or length): then nothing is sent. */
size_t modbus_tcp_process(const modbus_map_t *map,
                          const uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_cap);

#endif /* MODBUS_TCP_H */
