/*
 * lsm6dsv_driver.c
 *
 * Direct-register driver for LSM6DSV(TR).
 *
 * Register map references:
 *   AN5763 - LSM6DSV application note
 *   DS13771 - LSM6DSV datasheet
 */

#include "lsm6dsv_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lsm6dsv, LOG_LEVEL_INF);

/* ── Register addresses ──────────────────────────────────────────────────── */
#define REG_FUNC_CFG_ACCESS  0x01u
#define REG_WHO_AM_I         0x0Fu
#define REG_CTRL1            0x10u   /* XL ODR / FS */
#define REG_CTRL2            0x11u   /* GY ODR / FS */
#define REG_CTRL3            0x12u   /* BDU / IF_INC */
#define REG_CTRL4            0x13u
#define REG_STATUS_REG       0x1Eu
#define REG_OUT_TEMP_L       0x20u
#define REG_OUTX_L_G         0x22u   /* gyro X-low (6 bytes: X,Y,Z) */
#define REG_OUTX_L_A         0x28u   /* accel X-low (6 bytes: X,Y,Z) */

/* ── Bit fields ──────────────────────────────────────────────────────────── */
#define STATUS_XLDA  BIT(0)  /* accel data ready */
#define STATUS_GDA   BIT(1)  /* gyro data ready  */
#define STATUS_TDA   BIT(2)  /* temp data ready  */
#define CTRL3_BDU    BIT(6)  /* block data update */
#define CTRL3_IF_INC BIT(2)  /* auto-increment register address */

/* WHO_AM_I value for LSM6DSV */
#define LSM6DSV_WHO_AM_I_VAL  0x70u

/*
 * ODR codes for CTRL1/CTRL2 [3:0]:
 *   0001 = 7.5 Hz   0110 = 240 Hz   1010 = 3840 Hz
 *   0010 = 15 Hz    0111 = 480 Hz   1011 = 7680 Hz
 *   0011 = 30 Hz    1000 = 960 Hz
 *   0100 = 60 Hz    1001 = 1920 Hz
 *   0101 = 120 Hz
 *
 * We set 3840 Hz so the FIFO fills faster than our 1 kHz drain timer.
 * Read the STATUS register and drain one sample per timer tick.
 */
#define ODR_3840HZ  0x0Au

/*
 * Full-scale codes:
 *   Accel FS [5:4]:  00=±2g  01=±4g  10=±8g  11=±16g  → use 11
 *   Gyro  FS [5:4]:  00=±125dps  01=±250  10=±500  11=±1000  100=±2000 → 100
 *
 * Gyro FS lives in CTRL2[5:3] for LSM6DSV (differs from LSM6DS3).
 * Accel FS = CTRL1[5:4]
 */
#define CTRL1_VAL  ((ODR_3840HZ) | (0x3u << 4))  /* 3840 Hz, ±16 g   */
#define CTRL2_VAL  ((ODR_3840HZ) | (0x4u << 3))  /* 3840 Hz, ±2000 dps */

/* ── DT resolution ───────────────────────────────────────────────────────── */
#define IMU0_NODE  DT_NODELABEL(imu0)
#define IMU1_NODE  DT_NODELABEL(imu1)

struct lsm6dsv_ctx {
    const struct device *i2c_dev;
    uint16_t             addr;
    imu_id_t             id;
    bool                 present;
};

static struct lsm6dsv_ctx _ctx[2];

/* ── Internal helpers ────────────────────────────────────────────────────── */
static int _wr(lsm6dsv_ctx_t *c, uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(c->i2c_dev, c->addr, reg, val);
}

static int _rd(lsm6dsv_ctx_t *c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return i2c_burst_read(c->i2c_dev, c->addr, reg, buf, len);
}

/* ── Public API ──────────────────────────────────────────────────────────── */
lsm6dsv_ctx_t *lsm6dsv_init(imu_id_t id)
{
    lsm6dsv_ctx_t *c = &_ctx[id];
    c->id = id;

    if (id == IMU_MAIN) {
        c->i2c_dev = DEVICE_DT_GET(DT_BUS(IMU0_NODE));
        c->addr    = DT_REG_ADDR(IMU0_NODE);
    } else {
        c->i2c_dev = DEVICE_DT_GET(DT_BUS(IMU1_NODE));
        c->addr    = DT_REG_ADDR(IMU1_NODE);
    }

    if (!device_is_ready(c->i2c_dev)) {
        LOG_ERR("IMU%d I2C bus not ready", id);
        return NULL;
    }

    /* Verify chip identity */
    uint8_t who = 0;
    if (_rd(c, REG_WHO_AM_I, &who, 1) != 0 || who != LSM6DSV_WHO_AM_I_VAL) {
        LOG_WRN("IMU%d not found (WHO_AM_I=0x%02X)", id, who);
        c->present = false;
        /* Return context anyway so callers can poll lsm6dsv_is_present() */
        return c;
    }

    /* Software reset - bit 0 of CTRL3 */
    _wr(c, REG_CTRL3, BIT(0));
    k_msleep(5);

    /* Configure: BDU + auto-increment */
    _wr(c, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);

    /* Accel: 3840 Hz, ±16 g */
    _wr(c, REG_CTRL1, CTRL1_VAL);

    /* Gyro: 3840 Hz, ±2000 dps */
    _wr(c, REG_CTRL2, CTRL2_VAL);

    c->present = true;
    LOG_INF("LSM6DSV IMU%d ready at 0x%02X", id, c->addr);
    return c;
}

bool lsm6dsv_is_present(lsm6dsv_ctx_t *ctx)
{
    if (!ctx) { return false; }
    uint8_t who = 0;
    if (_rd(ctx, REG_WHO_AM_I, &who, 1) != 0) {
        ctx->present = false;
        return false;
    }
    ctx->present = (who == LSM6DSV_WHO_AM_I_VAL);
    return ctx->present;
}

bool lsm6dsv_data_ready(lsm6dsv_ctx_t *ctx)
{
    if (!ctx || !ctx->present) { return false; }
    uint8_t status = 0;
    _rd(ctx, REG_STATUS_REG, &status, 1);
    /* Require both accel AND gyro data ready */
    return (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool lsm6dsv_read_sample(lsm6dsv_ctx_t *ctx, imu_payload_t *out)
{
    if (!ctx || !ctx->present) { return false; }

    /*
     * OUT_TEMP_L (0x20) through OUTX_H_A (0x2D) = 14 bytes:
     *   [0-1]  OUT_TEMP
     *   [2-7]  OUTX/Y/Z_G  (gyro)
     *   [8-13] OUTX/Y/Z_A  (accel)
     */
    uint8_t raw[14];
    if (_rd(ctx, REG_OUT_TEMP_L, raw, sizeof(raw)) != 0) {
        return false;
    }

    out->timestamp_ms = k_uptime_get_32();

    out->temp_raw = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);

    out->gyro_x = (int16_t)((uint16_t)raw[3]  << 8 | raw[2]);
    out->gyro_y = (int16_t)((uint16_t)raw[5]  << 8 | raw[4]);
    out->gyro_z = (int16_t)((uint16_t)raw[7]  << 8 | raw[6]);

    out->accel_x = (int16_t)((uint16_t)raw[9]  << 8 | raw[8]);
    out->accel_y = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);
    out->accel_z = (int16_t)((uint16_t)raw[13] << 8 | raw[12]);

    return true;
}
