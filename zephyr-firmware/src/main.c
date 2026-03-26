/*
 * main.c
 *
 * scann3d - ESP32-C3-WROOM-02-N4
 *
 * Boots peripherals in order, hands sensor contexts to the work subsystem,
 * then arms the sampling timers. All periodic work runs via k_timer ->
 * k_work -> system workqueue (see work_queues.c).
 *
 * Data flow:
 *   work handler -> proto_encode() -> data_output_push() -> output thread
 *                                                          ├─ udp_out_send()
 *                                                          └─ usb_serial_write()
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "work_queues.h"
#include "protocol.h"
#include "vl53l8cx/vl53l8cx_driver.h"
#include "lsm6dsv/lsm6dsv_driver.h"
#include "output/data_output.h"
#include "led/led_ctrl.h"
#include "servo/servo_ctrl.h"
#include "uart_aux.h"
#include "wifi_mgr.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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
        led_set_color(LED_D2, LED_COLOR_CYAN);     /* steady cyan = fatal */
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
    vl53l8cx_ctx_t *tof_ctx  = vl53l8cx_driver_init(CONFIG_SCANNER_TOF_PERIOD_MS);
    if (!tof_ctx) {
        LOG_ERR("VL53L8CX init failed");
    }

    lsm6dsv_ctx_t *imu0_ctx = lsm6dsv_init(IMU_MAIN);
    if (!imu0_ctx) {
        LOG_ERR("LSM6DSV main init failed");
    }

    lsm6dsv_ctx_t *imu1_ctx = lsm6dsv_init(IMU_TRACK);
    if (!imu1_ctx) {
        LOG_WRN("LSM6DSV tracker not present (detachable - OK)");
    }

    /* D2 indicates sensor readiness: green = ToF+IMU OK, yellow = ToF only, blue = IMU only */
    if (tof_ctx && imu0_ctx) {
        imu1_ctx ? led_set_blink(LED_D2, LED_COLOR_GREEN, 500)
                 : led_set_color(LED_D2, LED_COLOR_GREEN);
    } else if (tof_ctx) {
        led_set_color(LED_D2, LED_COLOR_YELLOW);
    } else if (imu0_ctx) {
        imu1_ctx ? led_set_blink(LED_D2, LED_COLOR_BLUE, 500)
                 : led_set_color(LED_D2, LED_COLOR_BLUE);
    }

    /* 7. Hand contexts to the work subsystem and start timers */
    work_queues_init(tof_ctx, imu0_ctx, imu1_ctx);
    work_queues_start(CONFIG_SCANNER_IMU_PERIOD_MS,
                      CONFIG_SCANNER_TOF_PERIOD_MS,
                      CONFIG_SCANNER_STATUS_PERIOD_MS);

    /* Wait a few seconds and turn off LEDs for darkroom conditions */
    k_sleep(K_SECONDS(3));
    led_set_color(LED_D1, LED_COLOR_OFF);
    led_set_color(LED_D2, LED_COLOR_OFF);

    /* Main thread has nothing left to do - everything runs via timers/work. */
    while (true) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
