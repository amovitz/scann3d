/*
 * work_queues.h
 *
 * Work items and timer callbacks for IMU, ToF, and status sampling.
 * Call work_queues_init() once after sensors are initialised, then
 * work_queues_start() to arm the timers.
 */

#ifndef WORK_QUEUES_H
#define WORK_QUEUES_H

#include "vl53l8cx/vl53l8cx_driver.h"
#include "lsm6dsv/lsm6dsv_driver.h"

/**
 * @brief Provide sensor contexts to the work subsystem.
 *
 * Must be called before work_queues_start().
 * Any context pointer may be NULL if that sensor is absent.
 */
void work_queues_init(vl53l8cx_ctx_t *tof,
                       lsm6dsv_ctx_t  *imu0,
                       lsm6dsv_ctx_t  *imu1);

/**
 * @brief Arm the IMU, ToF, and status timers.
 *
 * @param imu_period_ms     IMU sampling interval      (ms)
 * @param tof_period_ms     ToF polling interval       (ms)
 * @param status_period_ms  Status reporting interval  (ms)
 */
void work_queues_start(uint32_t imu_period_ms, uint32_t tof_period_ms, uint32_t status_period_ms);

#endif /* WORK_QUEUES_H */