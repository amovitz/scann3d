/*
 * vl53l8cx_platform.c
 *
 * Implements the platform I/O layer expected by the ST VL53L8CX ULD.
 * All calls translate to Zephyr I2C driver calls.
 *
 * The VL53L8CX uses 16-bit register addresses with MSB-first order.
 * Burst transfers concatenate the 2-byte address then the payload.
 */

#include "vl53l8cx_platform.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

/* Maximum single I2C burst the ULD issues is ~32 KB (firmware upload).
 * We split transfers > I2C_MAX_MSG_LEN into chunks here.              */
#define I2C_CHUNK_MAX  512u

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static int _write_reg(VL53L8CX_Platform *p, uint16_t reg,
                      const uint8_t *data, uint32_t len)
{
    /* Prepend the 2-byte big-endian register address */
    uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

    uint32_t offset = 0;
    while (offset < len) {
        uint32_t chunk = MIN(len - offset, I2C_CHUNK_MAX);
        struct i2c_msg msgs[2] = {
            { .buf = hdr,  .len = 2,     .flags = I2C_MSG_WRITE              },
            { .buf = (uint8_t *)data + offset,
              .len = chunk, .flags = I2C_MSG_WRITE | I2C_MSG_STOP             },
        };
        if (i2c_transfer(p->i2c_dev, msgs, 2, p->active_addr) != 0) {
            return 1;
        }
        offset += chunk;
    }
    return 0;
}

static int _read_reg(VL53L8CX_Platform *p, uint16_t reg,
                     uint8_t *data, uint32_t len)
{
    uint8_t hdr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_write_read(p->i2c_dev, p->active_addr,
                          hdr, 2, data, len) != 0 ? 1 : 0;
}

/* ── ULD platform API ─────────────────────────────────────────────────────── */
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t val)
{
    return (uint8_t)_write_reg(p, reg, &val, 1);
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *val)
{
    return (uint8_t)_read_reg(p, reg, val, 1);
}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p, uint16_t reg,
                          const uint8_t *data, uint32_t len)
{
    return (uint8_t)_write_reg(p, reg, data, len);
}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p, uint16_t reg,
                          uint8_t *data, uint32_t len)
{
    return (uint8_t)_read_reg(p, reg, data, len);
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p)
{
    if (!p->has_xshut) {
        /* Software reset via register if no XSHUT wired */
        VL53L8CX_WrByte(p, 0x7FFF, 0x00);
        VL53L8CX_WrByte(p, 0x0009, 0x04); /* system reset */
        k_msleep(10);
        VL53L8CX_WrByte(p, 0x7FFF, 0x02);
        return 0;
    }
    gpio_pin_set_dt(&p->xshut, 1);  /* assert reset (active-low) */
    k_msleep(2);
    gpio_pin_set_dt(&p->xshut, 0);  /* release reset */
    k_msleep(10);
    return 0;
}

void VL53L8CX_SwapBuffer(uint8_t *buf, uint16_t size)
{
    /* The ULD calls this to endian-swap 32-bit words in the result buffer */
    for (uint16_t i = 0; i < size; i += 4) {
        uint8_t tmp;
        tmp = buf[i];     buf[i]     = buf[i + 3]; buf[i + 3] = tmp;
        tmp = buf[i + 1]; buf[i + 1] = buf[i + 2]; buf[i + 2] = tmp;
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p, uint32_t ms)
{
    ARG_UNUSED(p);
    k_msleep((int32_t)ms);
    return 0;
}
