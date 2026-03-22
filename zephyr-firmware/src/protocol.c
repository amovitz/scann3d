/*
 * protocol.c - Frame encoder and CRC-16/CCITT-FALSE implementation.
 */

#include "protocol.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>

/* CRC-16/CCITT-FALSE  (poly=0x1021, init=0xFFFF, refin=false, refout=false) */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (crc << 1) ^ 0x1021u : (crc << 1);
        }
    }
    return crc;
}

uint16_t proto_encode(uint8_t *buf, size_t buf_size,
                      scanner_pkt_type_t type, uint16_t seq,
                      const void *payload, uint16_t payload_len)
{
    uint16_t total = PROTO_OVERHEAD + payload_len;
    if (buf_size < total) {
        return 0;
    }

    uint8_t *p = buf;

    /* Magic */
    *p++ = PROTO_MAGIC_0;
    *p++ = PROTO_MAGIC_1;

    /* Type */
    *p++ = (uint8_t)type;

    /* Sequence */
    sys_put_le16(seq, p); p += 2;

    /* Payload length */
    sys_put_le16(payload_len, p); p += 2;

    /* Payload */
    memcpy(p, payload, payload_len);
    p += payload_len;

    /* CRC over [type(1) + seq(2) + len(2) + payload(N)] */
    uint16_t crc = crc16_ccitt(buf + 2, 3 + 2 + payload_len);
    sys_put_le16(crc, p);

    return total;
}
