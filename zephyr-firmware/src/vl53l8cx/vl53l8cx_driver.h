/*
 * vl53l8cx_driver.h
 *
 * Thin Zephyr wrapper around the ST VL53L8CX ULD.
 * Handles DT node resolution, initialisation sequence, and frame collection.
 */

#ifndef VL53L8CX_DRIVER_H
#define VL53L8CX_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include "../protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque sensor context - defined in vl53l8cx_driver.c */
typedef struct vl53l8cx_ctx vl53l8cx_ctx_t;

/**
 * @brief  Initialise the VL53L8CX sensor.
 *
 * Resolves the DT node, sets up the platform handle, uploads ULD firmware,
 * and starts ranging in 8×8 mode at the requested period.
 *
 * @param period_ms  Ranging integration period in ms (min ~34 for 8×8).
 * @return Pointer to context, or NULL on failure.
 */
vl53l8cx_ctx_t *vl53l8cx_init(uint32_t period_ms);

/**
 * @brief  Poll for a new ranging frame (non-blocking).
 *
 * @param ctx   Context returned by vl53l8cx_init().
 * @param out   Caller-allocated payload buffer to fill.
 * @return true if a new frame was available and written to @p out.
 */
bool vl53l8cx_get_frame(vl53l8cx_ctx_t *ctx, tof_payload_t *out);

#ifdef __cplusplus
}
#endif
#endif /* VL53L8CX_DRIVER_H */
