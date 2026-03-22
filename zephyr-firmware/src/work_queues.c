/*
 * work_queues.c
 *
 * Work handlers and k_timer instances for the three sampling loops:
 *
 *   imu_timer    fires every imu_period_ms
 *   tof_timer    fires every tof_period_ms
 *   status_timer fires every 5 s
 *
 * Timer callbacks are ISR-safe (they only submit work items).
 * All I2C access and packet dispatch happen in the system workqueue.
 */

#include "work_queues.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "protocol.h"
#include "output/data_output.h"
#include "led/led_ctrl.h"

LOG_MODULE_REGISTER(work_queues, LOG_LEVEL_INF);

/* ── Sensor contexts (set via work_queues_init) ─────────────────────────── */
static vl53l8cx_ctx_t *tof_ctx;
static lsm6dsv_ctx_t  *imu0_ctx;
static lsm6dsv_ctx_t  *imu1_ctx;

/* ── Rolling sequence numbers per packet type ────────────────────────────── */
static uint16_t seq_tof    = 0;
static uint16_t seq_imu0   = 0;
static uint16_t seq_imu1   = 0;
static uint16_t seq_status = 0;

/* ─────────────────────────────────────────────────────────────────────────
 *  IMU work
 * ───────────────────────────────────────────────────────────────────────── */
static void imu_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    imu_payload_t sample;

    /* ── IMU 0 (main board) ──────────────────────────────────────────────── */
    if (imu0_ctx && lsm6dsv_data_ready(imu0_ctx)) {
        if (lsm6dsv_read_sample(imu0_ctx, &sample)) {
            data_output_send(PKT_IMU0, seq_imu0++,
                             &sample, sizeof(sample));
        }
    }

    /* ── IMU 1 (detachable tracker) ──────────────────────────────────────── */
    if (imu1_ctx) {
        /*
         * Periodically re-check presence so the tracker can be hot-plugged.
         * We gate the check on seq_imu1 & 0x3FF (every ~1024 ticks ≈ 1 s).
         */
        if ((seq_imu1 & 0x3FFu) == 0u) {
            lsm6dsv_is_present(imu1_ctx);
        }
        if (lsm6dsv_data_ready(imu1_ctx)) {
            if (lsm6dsv_read_sample(imu1_ctx, &sample)) {
                data_output_send(PKT_IMU1, seq_imu1++,
                                 &sample, sizeof(sample));
            }
        }
    }
}
K_WORK_DEFINE(imu_work, imu_work_handler);

static void imu_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&imu_work);
}
K_TIMER_DEFINE(imu_timer, imu_timer_cb, NULL);

/* ─────────────────────────────────────────────────────────────────────────
 *  ToF work
 * ───────────────────────────────────────────────────────────────────────── */
static void tof_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    if (!tof_ctx) { return; }

    tof_payload_t frame;
    if (vl53l8cx_driver_get_frame(tof_ctx, &frame)) {
        data_output_send(PKT_TOF, seq_tof++,
                         &frame, sizeof(frame));
    }
}
K_WORK_DEFINE(tof_work, tof_work_handler);

static void tof_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&tof_work);
}
K_TIMER_DEFINE(tof_timer, tof_timer_cb, NULL);

/* ─────────────────────────────────────────────────────────────────────────
 *  Status heartbeat (every 5 s)
 * ───────────────────────────────────────────────────────────────────────── */
static void status_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    status_payload_t st = {
        .timestamp_ms = k_uptime_get_32(),
        .tof_ok   = (tof_ctx  != NULL),
        .imu0_ok  = (imu0_ctx != NULL && lsm6dsv_is_present(imu0_ctx)),
        .imu1_ok  = (imu1_ctx != NULL && lsm6dsv_is_present(imu1_ctx)),
        .wifi_ok  = 1,   /* if we got here, WiFi was up at init */
    };

    /* D2 indicates sensor readiness: green = ToF+IMU OK, yellow = ToF only, blue = IMU only */
    if (st.tof_ok && st.imu0_ok) {
        if (st.imu1_ok) {
            led_set_blink(LED_D2, LED_COLOR_GREEN, 500);
        } else {
            led_set_color(LED_D2, LED_COLOR_GREEN);
        }
    } else if (st.tof_ok) {
        led_set_color(LED_D2, LED_COLOR_YELLOW);
    } else if (st.imu0_ok) {
        if (st.imu1_ok) {
            led_set_blink(LED_D2, LED_COLOR_BLUE, 500);
        } else {
            led_set_color(LED_D2, LED_COLOR_BLUE);
        }
    }

    data_output_send(PKT_STATUS, seq_status++, &st, sizeof(st));
}
K_WORK_DEFINE(status_work, status_work_handler);

static void status_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&status_work);
}
K_TIMER_DEFINE(status_timer, status_timer_cb, NULL);

/* ─────────────────────────────────────────────────────────────────────────
 *  Public API
 * ───────────────────────────────────────────────────────────────────────── */
void work_queues_init(vl53l8cx_ctx_t *tof,
                       lsm6dsv_ctx_t  *imu0,
                       lsm6dsv_ctx_t  *imu1)
{
    tof_ctx  = tof;
    imu0_ctx = imu0;
    imu1_ctx = imu1;
}

void work_queues_start(uint32_t imu_period_ms, uint32_t tof_period_ms, uint32_t status_period_ms)
{
    k_timer_start(&imu_timer,
                  K_MSEC(imu_period_ms),
                  K_MSEC(imu_period_ms));

    k_timer_start(&tof_timer,
                  K_MSEC(tof_period_ms),
                  K_MSEC(tof_period_ms));

    k_timer_start(&status_timer,
                  K_MSEC(status_period_ms),
                  K_MSEC(status_period_ms));

    LOG_INF("Timers armed - IMU @%u ms, ToF @%u ms, Status @%u ms",
            imu_period_ms, tof_period_ms, status_period_ms);
}