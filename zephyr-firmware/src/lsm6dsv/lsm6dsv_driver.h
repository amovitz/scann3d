/*
 * lsm6dsv_driver.h
 *
 * Lightweight direct-register driver for the ST LSM6DSV(TR) 6-axis IMU.
 * Bypasses the Zephyr sensor abstraction to enable maximum throughput.
 *
 * Accel full-scale  : ±16 g   (LSB = 0.488 mg)
 * Gyro  full-scale  : ±2000 dps (LSB = 0.061 dps)
 * Sample rate       : up to 7680 Hz (ODR_7680Hz)
 */

#ifndef LSM6DSV_DRIVER_H
#define LSM6DSV_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include "../protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IMU instance identifier */
typedef enum {
    IMU_MAIN  = 0,  /* fixed to main PCB, I2C0, addr 0x6A */
    IMU_TRACK = 1,  /* detachable tracker,  I2C1, addr 0x6B */
} imu_id_t;

/* Opaque context */
typedef struct lsm6dsv_ctx lsm6dsv_ctx_t;

/**
 * @brief  Initialise one LSM6DSV instance.
 *
 * @param id  Which IMU to initialise.
 * @return Pointer to context, or NULL on failure.
 */
lsm6dsv_ctx_t *lsm6dsv_init(imu_id_t id);

/**
 * @brief  Check if new data is available (DRDY bit in STATUS_REG).
 */
bool lsm6dsv_data_ready(lsm6dsv_ctx_t *ctx);

/**
 * @brief  Read one sample into @p out.
 *         Reads accel, gyro, and temperature in one 14-byte burst.
 *
 * @return true on success.
 */
bool lsm6dsv_read_sample(lsm6dsv_ctx_t *ctx, imu_payload_t *out);

/**
 * @brief  Check if the tracker IMU is still electrically present.
 *
 * Reads WHO_AM_I; returns false if the device does not respond.
 */
bool lsm6dsv_is_present(lsm6dsv_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
#endif /* LSM6DSV_DRIVER_H */
