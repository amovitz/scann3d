/*
 * protocol.c - Frame encoder and CRC-16/CCITT-FALSE implementation.
 */

#include "protocol.h"
#include <string.h>
#include <zephyr/sys/byteorder.h>

#if defined(CONFIG_OUTPUT_JSON)

static uint16_t proto_encode_json(uint8_t *buf, size_t buf_size,
                                  scanner_pkt_type_t type, uint16_t seq,
                                  const void *payload, uint16_t payload_len)
{
    // JSON ignores sequence number since we already have timestamps 
    ARG_UNUSED(seq);

    int ret = 0;
    char json_buf[JSON_BUFFER_MAX];

    // Clear the temporary JSON buffer
    memset(json_buf, 0, JSON_BUFFER_MAX);

    switch (type) {
        case PKT_TOF: {
            tof_json_t j;
            memcpy(&j, payload, sizeof(tof_payload_t));
            j.distance_mm_len        = TOF_ZONES;
            j.sigma_mm_len           = TOF_ZONES;
            j.status_len             = TOF_ZONES;
            j.nb_target_detected_len = TOF_ZONES;
            ret = json_obj_encode_buf(tof_payload_descr,
                                      ARRAY_SIZE(tof_payload_descr),
                                      &j, json_buf, sizeof(json_buf));
            break;
        }
        case PKT_IMU0:
        case PKT_IMU1: {
            imu_json_t j;
            memcpy(&j, payload, sizeof(imu_payload_t));
            j.imu_num = type - PKT_IMU0;
            ret = json_obj_encode_buf(imu_payload_descr,
                                      ARRAY_SIZE(imu_payload_descr),
                                      &j, json_buf, sizeof(json_buf));
            break;
        }
        case PKT_STATUS: {
            const status_payload_t *s = payload;
            status_json_t j = {
                .timestamp_ms = s->timestamp_ms,
                .tof_ok       = s->tof_ok,
                .imu0_ok      = s->imu0_ok,
                .imu1_ok      = s->imu1_ok,
                .wifi_ok      = s->wifi_ok,
            };
            ret = json_obj_encode_buf(status_payload_descr,
                                      ARRAY_SIZE(status_payload_descr),
                                      &j, json_buf, sizeof(json_buf));
            break;
        }
        default:
            return 0;
    }

    if (ret < 0) {
        return 0;
    }

    /* json_buf now holds the encoded string; copy to caller's buf if it fits */
    size_t json_len = strlen(json_buf);
    if (json_len + 1 > buf_size) {
        return 0;
    }
    memcpy(buf, json_buf, json_len + 1);
    return (uint16_t)json_len;
}

#else

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

#endif /* CONFIG_OUTPUT_JSON */

uint16_t proto_encode(uint8_t *buf, size_t buf_size,
                      scanner_pkt_type_t type, uint16_t seq,
                      const void *payload, uint16_t payload_len)
{
#if defined(CONFIG_OUTPUT_JSON)
    return proto_encode_json(buf, buf_size, type, seq, payload, payload_len);
#else
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
#endif
}
