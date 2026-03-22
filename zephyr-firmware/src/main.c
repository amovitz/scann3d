/*
 * main.c
 *
 * scann3d - ESP32-C3-WROOM-02-N4
 *
 * Two k_timer instances drive sensor sampling:
 *
 *   imu_timer  - fires every CONFIG_SCANNER_IMU_PERIOD_MS (default 1 ms)
 *                Drains one DRDY sample from each LSM6DSV per tick.
 *
 *   tof_timer  - fires every CONFIG_SCANNER_TOF_PERIOD_MS (default 66 ms)
 *                Polls the VL53L8CX data-ready flag and reads the frame
 *                when available.
 *
 * Timer callbacks run in interrupt context so they are minimal: they post
 * a work item to the system workqueue, which does the actual I2C reads and
 * frame dispatch.
 *
 * Data flow:
 *   work handler -> proto_encode() -> data_output_push() -> output thread
 *                                                          ├─ udp_out_send()
 *                                                          └─ usb_serial_write()
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "protocol.h"
#include "vl53l8cx/vl53l8cx_driver.h"
#include "lsm6dsv/lsm6dsv_driver.h"
#include "output/data_output.h"
#include "led/led_ctrl.h"
#include "servo/servo_ctrl.h"
#include "uart_aux.h"
#include "wifi_mgr.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ── Sensor contexts (initialised in main) ───────────────────────────────── */
static vl53l8cx_ctx_t *tof_ctx;
static lsm6dsv_ctx_t  *imu0_ctx;
static lsm6dsv_ctx_t  *imu1_ctx;

/* ── Rolling sequence numbers per packet type ────────────────────────────── */
static uint16_t seq_tof  = 0;
static uint16_t seq_imu0 = 0;
static uint16_t seq_imu1 = 0;

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
static uint16_t seq_status = 0;

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
 *  Entry point
 * ───────────────────────────────────────────────────────────────────────── */
int main(void)
{
    LOG_INF("=== scann3d booting ===");

    /* 1. LED controller - init first so we can signal status visually */
    if (led_ctrl_init() != 0) {
        LOG_ERR("LED init failed");
        /* Non-fatal - continue booting */
    }

    /* Boot indicator: both LEDs white, fast blink */
    led_set_blink(LED_D1, LED_COLOR_WHITE, 100);
    led_set_blink(LED_D2, LED_COLOR_WHITE, 100);

    /* 2. Servo controller */
    if (servo_ctrl_init() != 0) {
        LOG_ERR("Servo init failed");
        led_set_color(LED_D2, LED_COLOR_CYAN);    /* steady cyan = fatal */
    }

    /* 3. Auxiliary UART0 (IO20/IO21 - available for future use) */
    if (uart_aux_init() != 0) {
        LOG_WRN("Auxiliary UART init failed (non-fatal)");
        led_set_color(LED_D2, LED_COLOR_MAGENTA);  /* steady magenta = fatal */
    }

    /* 4. Output subsystem (USB serial + UDP socket) */
    if (data_output_init() != 0) {
        LOG_ERR("Output init failed - halting");
        led_set_color(LED_D2, LED_COLOR_RED);      /* steady red = fatal */
        return -1;
    }

    /* 5. WiFi - non-fatal; UDP won't work but USB-serial will */
    led_set_blink(LED_D1, LED_COLOR_YELLOW, 500);  /* yellow blink = connecting */
    if (wifi_mgr_connect() != 0) {
        LOG_WRN("WiFi unavailable - UDP output disabled");
        led_set_blink(LED_D1, LED_COLOR_RED, 500); /* blinking red = no WiFi */
    } else {
        led_set_color(LED_D1, LED_COLOR_GREEN);    /* steady green = WiFi up */
    }

    /* 6. Sensors */
    tof_ctx  = vl53l8cx_driver_init(CONFIG_SCANNER_TOF_PERIOD_MS);
    if (!tof_ctx) {
        LOG_ERR("VL53L8CX init failed");
    }

    imu0_ctx = lsm6dsv_init(IMU_MAIN);
    if (!imu0_ctx) {
        LOG_ERR("LSM6DSV main init failed");
    }

    imu1_ctx = lsm6dsv_init(IMU_TRACK);
    if (!imu1_ctx) {
        LOG_WRN("LSM6DSV tracker not present (detachable - OK)");
    }

    /* D2 indicates sensor readiness: green = ToF+IMU OK, yellow = ToF only, blue = IMU only */
    if ((tof_ctx != NULL) && (imu0_ctx != NULL)) {
        if (imu1_ctx != NULL) {
            led_set_blink(LED_D2, LED_COLOR_GREEN, 500);
        } else {
            led_set_color(LED_D2, LED_COLOR_GREEN);
        }
    } else if (tof_ctx != NULL) {
        led_set_color(LED_D2, LED_COLOR_YELLOW);
    } else if (imu0_ctx != NULL) {
        if (imu1_ctx != NULL) {
            led_set_blink(LED_D2, LED_COLOR_BLUE, 500);
        } else {
            led_set_color(LED_D2, LED_COLOR_BLUE);
        }
    }

    /* 7. Start sampling timers */
    k_timer_start(&imu_timer,
                  K_MSEC(CONFIG_SCANNER_IMU_PERIOD_MS),
                  K_MSEC(CONFIG_SCANNER_IMU_PERIOD_MS));

    k_timer_start(&tof_timer,
                  K_MSEC(CONFIG_SCANNER_TOF_PERIOD_MS),
                  K_MSEC(CONFIG_SCANNER_TOF_PERIOD_MS));

    k_timer_start(&status_timer,
                  K_SECONDS(5),
                  K_SECONDS(5));

    LOG_INF("Streaming started - IMU @%d ms, ToF @%d ms",
            CONFIG_SCANNER_IMU_PERIOD_MS,
            CONFIG_SCANNER_TOF_PERIOD_MS);

    /* Main thread has nothing left to do - everything runs via timers/work. */
    while (true) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
