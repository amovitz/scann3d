/*
 * vl53l8cx_driver.c
 *
 * Wraps the ST ULD API in a Zephyr-friendly interface.
 * The ULD types (VL53L8CX_Configuration, VL53L8CX_ResultsData, …) come from
 * the ULD headers placed in lib/vl53l8cx_uld/.
 */

/* ST ULD headers (provided by lib/vl53l8cx_uld/) */
#include "vl53l8cx_driver.h"
#include "vl53l8cx_api.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(vl53l8cx, LOG_LEVEL_INF);

#define VL53_NODE       DT_NODELABEL(vl53l8cx)
#define VL53_I2C_BUS    DT_BUS(VL53_NODE)
#define VL53_I2C_ADDR   DT_REG_ADDR(VL53_NODE)   /* 7-bit: 0x29 */
#define VL53_HAS_XSHUT  DT_NODE_HAS_PROP(VL53_NODE, xshut_gpios)

/*
 * Zephyr resources kept separately from the ULD's VL53L8CX_Platform.
 * The ULD platform shim accesses these via extern — see note below.
 */
const struct device     *vl53l8cx_i2c_dev;
uint16_t                 vl53l8cx_i2c_addr;   /* 7-bit address for i2c_transfer */

#if VL53_HAS_XSHUT
static const struct gpio_dt_spec _xshut =
    GPIO_DT_SPEC_GET(VL53_NODE, xshut_gpios);
#endif

struct vl53l8cx_ctx {
    VL53L8CX_Configuration  dev;
    VL53L8CX_ResultsData    res;
    bool                    ready;
};

static struct vl53l8cx_ctx _ctx;

vl53l8cx_ctx_t *vl53l8cx_driver_init(uint32_t period_ms)
{
    /* Resolve I2C bus */
    vl53l8cx_i2c_dev = DEVICE_DT_GET(VL53_I2C_BUS);
    if (!device_is_ready(vl53l8cx_i2c_dev)) {
        LOG_ERR("I2C bus not ready");
        return NULL;
    }
    vl53l8cx_i2c_addr = VL53_I2C_ADDR;   /* 0x29 */

    /* Set the only field ST's VL53L8CX_Platform actually has */
    _ctx.dev.platform.address = VL53_I2C_ADDR << 1;  /* ULD wants 8-bit: 0x52 */

#if VL53_HAS_XSHUT
    if (!gpio_is_ready_dt(&_xshut)) {
        LOG_ERR("XSHUT GPIO not ready");
        return NULL;
    }
    gpio_pin_configure_dt(&_xshut, GPIO_OUTPUT_INACTIVE);

    /* Hardware reset via XSHUT */
    gpio_pin_set_dt(&_xshut, 0);
    k_sleep(K_MSEC(10));
    gpio_pin_set_dt(&_xshut, 1);
    k_sleep(K_MSEC(10));
#endif

    uint8_t status;

    LOG_INF("Uploading VL53L8CX firmware (~300 ms)...");
    status = vl53l8cx_init(&_ctx.dev);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("ULD init failed: %u", status);
        return NULL;
    }

    status = vl53l8cx_set_resolution(&_ctx.dev, VL53L8CX_RESOLUTION_8X8);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_resolution failed: %u", status);
        return NULL;
    }

    uint32_t freq = 1000u / period_ms;
    freq = CLAMP(freq, 1u, 15u);   /* 8x8 max is 15 Hz */
    status = vl53l8cx_set_ranging_frequency_hz(&_ctx.dev, (uint8_t)freq);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_frequency failed: %u", status);
        return NULL;
    }

    status = vl53l8cx_set_ranging_mode(&_ctx.dev,
                                        VL53L8CX_RANGING_MODE_CONTINUOUS);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("set_ranging_mode failed: %u", status);
        return NULL;
    }

    status = vl53l8cx_start_ranging(&_ctx.dev);
    if (status != VL53L8CX_STATUS_OK) {
        LOG_ERR("start_ranging failed: %u", status);
        return NULL;
    }

    LOG_INF("VL53L8CX ready - 8x8 @ %u Hz", (unsigned)freq);
    _ctx.ready = true;
    return &_ctx;
}

bool vl53l8cx_driver_get_frame(vl53l8cx_ctx_t *ctx, tof_payload_t *out)
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
