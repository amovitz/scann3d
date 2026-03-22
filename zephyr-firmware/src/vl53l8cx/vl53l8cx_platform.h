/*
 * vl53l8cx_platform.h
 *
 * Zephyr adaptation of the STMicroelectronics VL53L8CX ULD platform layer.
 * The ULD (Ultra Lite Driver) expects this header to define the Dev handle
 * type and the function prototypes it calls for I2C and timing.
 *
 * Clone the ULD from:
 *   https://github.com/STMicroelectronics/VL53L8CX_ULD_driver
 * and drop the *.c / *.h files into lib/vl53l8cx_uld/
 */

#ifndef VL53L8CX_PLATFORM_H
#define VL53L8CX_PLATFORM_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Platform handle ──────────────────────────────────────────────────────── */
typedef struct {
    /* I2C bus device + 7-bit address */
    const struct device  *i2c_dev;
    uint16_t              i2c_addr;

    /* Optional XSHUT GPIO (active-low reset) */
    struct gpio_dt_spec   xshut;
    bool                  has_xshut;

    /* 7-bit address actually in use (may change after set_i2c_address) */
    uint16_t              active_addr;
} VL53L8CX_Platform;

/* ── Required by ULD ──────────────────────────────────────────────────────── */
uint8_t VL53L8CX_RdByte  (VL53L8CX_Platform *p, uint16_t reg, uint8_t  *val);
uint8_t VL53L8CX_WrByte  (VL53L8CX_Platform *p, uint16_t reg, uint8_t   val);
uint8_t VL53L8CX_RdMulti (VL53L8CX_Platform *p, uint16_t reg,
                           uint8_t *data, uint32_t len);
uint8_t VL53L8CX_WrMulti (VL53L8CX_Platform *p, uint16_t reg,
                           const uint8_t *data, uint32_t len);
uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p);
void    VL53L8CX_SwapBuffer (uint8_t *buf, uint16_t size);
uint8_t VL53L8CX_WaitMs    (VL53L8CX_Platform *p, uint32_t ms);

#ifdef __cplusplus
}
#endif
#endif /* VL53L8CX_PLATFORM_H */
