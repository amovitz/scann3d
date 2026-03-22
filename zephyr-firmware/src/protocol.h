/*
 * protocol.h - Binary packet format shared across UDP and USB-serial streams.
 *
 * Wire layout (all multi-byte fields are little-endian):
 *
 *   [0x55 0xAA]   2 B  magic / frame-start
 *   [type]        1 B  packet type (enum scanner_pkt_type)
 *   [seq]         2 B  rolling sequence number (per type)
 *   [len]         2 B  payload length in bytes
 *   [payload]     N B  type-specific payload (see structs below)
 *   [crc16]       2 B  CRC-16/CCITT over bytes [type..payload]
 *
 * Total overhead: 9 bytes per frame.
 */

#ifndef SCANNER_PROTOCOL_H
#define SCANNER_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Frame constants ──────────────────────────────────────────────────────── */
#define PROTO_MAGIC_0   0x55u
#define PROTO_MAGIC_1   0xAAu
#define PROTO_HDR_LEN   7u   /* magic(2)+type(1)+seq(2)+len(2) */
#define PROTO_FTR_LEN   2u   /* crc16(2) */
#define PROTO_OVERHEAD  (PROTO_HDR_LEN + PROTO_FTR_LEN)

/* ── Packet types ─────────────────────────────────────────────────────────── */
typedef enum __attribute__((packed)) {
    PKT_TOF   = 0x01,   /* VL53L8CX 8×8 ranging frame          */
    PKT_IMU0  = 0x02,   /* LSM6DSV main board (fixed)           */
    PKT_IMU1  = 0x03,   /* LSM6DSV tracker (detachable)         */
    PKT_STATUS = 0x10,  /* heartbeat / error flags              */
} scanner_pkt_type_t;

/* ── ToF payload (329 bytes) ──────────────────────────────────────────────── */
#define TOF_ZONES  64u   /* 8×8 */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;          /* k_uptime_get_32()                  */
    uint16_t distance_mm[TOF_ZONES];/* centre distance per zone, mm       */
    uint16_t sigma_mm[TOF_ZONES];   /* ranging sigma per zone, mm         */
    uint8_t  status[TOF_ZONES];     /* per-zone ranging status (ULD codes)*/
    uint8_t  nb_target_detected[TOF_ZONES]; /* targets per zone           */
} tof_payload_t;

/* ── IMU payload (20 bytes) ───────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    int16_t  accel_x;   /* raw 16-bit - scale = ±16 g / 32768             */
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;    /* raw 16-bit - scale = ±2000 dps / 32768         */
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  temp_raw;  /* LSM6DSV: (raw / 256) + 25 °C                   */
} imu_payload_t;

/* ── Status payload (4 bytes) ────────────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  tof_ok   : 1;
    uint8_t  imu0_ok  : 1;
    uint8_t  imu1_ok  : 1;
    uint8_t  wifi_ok  : 1;
    uint8_t  _rsvd    : 4;
} status_payload_t;

/* ── Maximum serialised frame size ───────────────────────────────────────── */
#define FRAME_BUF_MAX  (PROTO_OVERHEAD + sizeof(tof_payload_t))

/* ── Utility: build a complete frame into buf[], return total length ─────── */
uint16_t proto_encode(uint8_t *buf, size_t buf_size,
                      scanner_pkt_type_t type, uint16_t seq,
                      const void *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif
#endif /* SCANNER_PROTOCOL_H */
