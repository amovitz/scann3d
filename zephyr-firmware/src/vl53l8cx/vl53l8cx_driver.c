/*
 * vl53l8cx_driver.c
 *
 * Wraps the ST ULD API in a Zephyr-friendly interface.
 * The ULD types (VL53L8CX_Configuration, VL53L8CX_ResultsData, …) come from
 * the ULD headers placed in lib/vl53l8cx_uld/.
 */

#include "vl53l8cx_driver.h"
#include "vl53l8cx_platform.h"

/* ST ULD headers (provided by lib/vl53l8cx_uld/) */
#include "vl53l8cx_api.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vl53l8cx, LOG_LEVEL_INF);

/* DT aliases ----------------------------------------------------------------
 * The overlay declares  vl53l8cx: vl53l8cx@29  on &i2c0.
 * We resolve the parent I2C bus and child address at compile time.          */
#define VL53_NODE        DT_NODELABEL(vl53l8cx)
#define VL53_I2C_BUS     DT_BUS(VL53_NODE)
#define VL53_I2C_ADDR    DT_REG_ADDR(VL53_NODE)
#define VL53_HAS_XSHUT   DT_NODE_HAS_PROP(VL53_NODE, xshut_gpios)

struct vl53l8cx_ctx {
    VL53L8CX_Configuration   dev;   /* ULD configuration block (includes Dev) */
    VL53L8CX_ResultsData      res;   /* latest ranging results                */
    bool                      ready;
};

static struct vl53l8cx_ctx _ctx;

vl53l8cx_ctx_t *vl53l8cx_init(uint32_t period_ms)
{
    const struct device *i2c = DEVICE_DT_GET(VL53_I2C_BUS);
    if (!device_is_ready(i2c)) {
        LOG_ERR("I2C bus not ready");
        return NULL;
    }

    /* Populate the platform handle embedded in VL53L8CX_Configuration */
    VL53L8CX_Platform *plat = &_ctx.dev.platform;
    plat->i2c_dev    = i2c;
    plat->i2c_addr   = VL53_I2C_ADDR;
    plat->active_addr = VL53_I2C_ADDR;

#if VL53_HAS_XSHUT
    static const struct gpio_dt_spec xshut =
        GPIO_DT_SPEC_GET(VL53_NODE, xshut_gpios);
    if (!gpio_is_ready_dt(&xshut)) {
        LOG_ERR("XSHUT GPIO not ready");
        return NULL;
    }
    gpio_pin_configure_dt(&xshut, GPIO_OUTPUT_INACTIVE);
    plat->xshut     = xshut;
    plat->has_xshut = true;
#else
    plat->has_xshut = false;
#endif

    /* Hardware reset */
    uint8_t status;
    status = VL53L8CX_Reset_Sensor(plat);
    if (status != 0U) {
        LOG_ERR("Sensor reset failed: %u", status);
        return NULL;
    }

    /* Init ULD - uploads ~84 KB firmware via I2C, takes ~300 ms */
    LOG_INF("Uploading VL53L8CX firmware (this takes ~300 ms)…");
    status = vl53l8cx_init(&_ctx.dev);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("ULD init failed: %u", status);
        return NULL;
    }

    /* Configure 8×8 mode */
    status = vl53l8cx_set_resolution(&_ctx.dev, VL53L8CX_RESOLUTION_8X8);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_resolution failed: %u", status);
        return NULL;
    }

    /* Ranging frequency - period_ms to Hz, clamped to sensor limits */
    uint32_t freq = 1000u / period_ms;
    if (freq < 1)  { freq = 1;  }
    if (freq > 60) { freq = 60; }
    status = vl53l8cx_set_ranging_frequency_hz(&_ctx.dev, (uint8_t)freq);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_frequency failed: %u", status);
        return NULL;
    }

    /* Integration time - use autonomous mode */
    status = vl53l8cx_set_ranging_mode(&_ctx.dev,
                                        VL53L8CX_RANGING_MODE_CONTINUOUS);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_ranging_mode failed: %u", status);
        return NULL;
    }

    /* Start ranging */
    status = vl53l8cx_start_ranging(&_ctx.dev);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("start_ranging failed: %u", status);
        return NULL;
    }

    LOG_INF("VL53L8CX ready - 8×8 @ %u Hz", (unsigned)freq);
    _ctx.ready = true;
    return &_ctx;
}

bool vl53l8cx_get_frame(vl53l8cx_ctx_t *ctx, tof_payload_t *out)
{
    if (!ctx || !ctx->ready) {
        return false;
    }

    uint8_t data_ready = 0;
    if (vl53l8cx_check_data_ready(&ctx->dev, &data_ready)
            != VL53L8CX_STATUS_OK) {
        return false;
    }
    if (!data_ready) {
        return false;
    }

    if (vl53l8cx_get_ranging_data(&ctx->dev, &ctx->res)
            != VL53L8CX_STATUS_OK) {
        return false;
    }

    out->timestamp_ms = k_uptime_get_32();

    for (int i = 0; i < TOF_ZONES; i++) {
        /* ULD stores up to VL53L8CX_NB_TARGET_PER_ZONE targets; we take [0] */
        out->distance_mm[i] = (uint16_t)
            ctx->res.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i];
        out->sigma_mm[i]    = (uint16_t)
            ctx->res.range_sigma_mm[VL53L8CX_NB_TARGET_PER_ZONE * i];
        out->status[i]      = (uint8_t)
            ctx->res.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i];
        out->nb_target_detected[i] =
            ctx->res.nb_target_detected[i];
    }

    return true;
}
