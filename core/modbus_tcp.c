#include "modbus_tcp.h"

/* Modbus is big-endian on the wire */
static uint16_t be16(const uint8_t *p)        { return (uint16_t)((p[0] << 8) | p[1]); }
static void put_be16(uint8_t *p, uint16_t v)  { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)(v & 0xFFu); }

/* Build an exception PDU, return its length */
static size_t exception_pdu(uint8_t *pdu, uint8_t fc, uint8_t code)
{
    pdu[0] = (uint8_t)(fc | 0x80u);
    pdu[1] = code;
    return 2u;
}

static size_t read_registers(const modbus_map_t *map, uint8_t fc,
                             const uint8_t *pdu, size_t pdu_len,
                             uint8_t *out, size_t out_cap)
{
    if (pdu_len != 5u) return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);

    uint16_t addr = be16(&pdu[1]);
    uint16_t qty  = be16(&pdu[3]);
    if (qty < 1u || qty > 125u) return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);
    if ((uint32_t)addr + qty > 0x10000u) return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);
    if (2u + 2u * (size_t)qty > out_cap) return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);

    int (*rd)(void *, uint16_t, uint16_t *) =
        (fc == MB_FC_READ_HOLDING) ? map->read_holding : map->read_input;

    out[0] = fc;
    out[1] = (uint8_t)(2u * qty);
    for (uint16_t i = 0; i < qty; i++) {
        uint16_t v = 0;
        if (rd(map->ctx, (uint16_t)(addr + i), &v) != MB_OK)
            return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);
        put_be16(&out[2u + 2u * i], v);
    }
    return 2u + 2u * (size_t)qty;
}

static size_t write_single(const modbus_map_t *map, const uint8_t *pdu, size_t pdu_len, uint8_t *out)
{
    const uint8_t fc = MB_FC_WRITE_SINGLE;
    if (pdu_len != 5u) return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);

    int r = map->write_holding(map->ctx, be16(&pdu[1]), be16(&pdu[3]));
    if (r == MB_ERR_ADDRESS) return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);
    if (r != MB_OK)          return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);

    for (size_t i = 0; i < 5u; i++) out[i] = pdu[i];       /* echo the request */
    return 5u;
}

static size_t write_multiple(const modbus_map_t *map, const uint8_t *pdu, size_t pdu_len, uint8_t *out)
{
    const uint8_t fc = MB_FC_WRITE_MULTIPLE;
    if (pdu_len < 6u) return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);

    uint16_t addr = be16(&pdu[1]);
    uint16_t qty  = be16(&pdu[3]);
    uint8_t  bc   = pdu[5];
    if (qty < 1u || qty > 123u || bc != 2u * qty || pdu_len != 6u + bc)
        return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);
    if ((uint32_t)addr + qty > 0x10000u)
        return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);

    /* Check that every address exists before writing anything */
    for (uint16_t i = 0; i < qty; i++) {
        uint16_t dummy;
        if (map->read_holding(map->ctx, (uint16_t)(addr + i), &dummy) != MB_OK)
            return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);
    }
    for (uint16_t i = 0; i < qty; i++) {
        int r = map->write_holding(map->ctx, (uint16_t)(addr + i), be16(&pdu[6u + 2u * i]));
        if (r == MB_ERR_ADDRESS) return exception_pdu(out, fc, MB_EX_ILLEGAL_ADDRESS);
        if (r != MB_OK)          return exception_pdu(out, fc, MB_EX_ILLEGAL_VALUE);
    }

    out[0] = fc;
    put_be16(&out[1], addr);
    put_be16(&out[3], qty);
    return 5u;
}

size_t modbus_tcp_process(const modbus_map_t *map,
                          const uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_cap)
{
    if (req_len < MB_MBAP_LEN + 1u || resp_cap < MB_MBAP_LEN + 2u) return 0;

    uint16_t tid = be16(&req[0]);
    uint16_t pid = be16(&req[2]);
    uint16_t len = be16(&req[4]);           /* unit id + PDU */
    uint8_t  uid = req[6];
    if (pid != 0u || (size_t)len + 6u != req_len) return 0;

    const uint8_t *pdu = &req[MB_MBAP_LEN];
    size_t pdu_len = req_len - MB_MBAP_LEN;
    uint8_t *out = &resp[MB_MBAP_LEN];
    size_t out_cap = resp_cap - MB_MBAP_LEN;
    size_t out_len;

    switch (pdu[0]) {
    case MB_FC_READ_HOLDING:
    case MB_FC_READ_INPUT:
        out_len = read_registers(map, pdu[0], pdu, pdu_len, out, out_cap);
        break;
    case MB_FC_WRITE_SINGLE:
        out_len = write_single(map, pdu, pdu_len, out);
        break;
    case MB_FC_WRITE_MULTIPLE:
        out_len = write_multiple(map, pdu, pdu_len, out);
        break;
    default:
        out_len = exception_pdu(out, pdu[0], MB_EX_ILLEGAL_FUNCTION);
        break;
    }

    put_be16(&resp[0], tid);                /* echo transaction id */
    put_be16(&resp[2], 0u);
    put_be16(&resp[4], (uint16_t)(out_len + 1u));
    resp[6] = uid;
    return MB_MBAP_LEN + out_len;
}
