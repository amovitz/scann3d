/*
 * servo_ctrl.c
 *
 * 360-degree servo controller using ESP32-C3 LEDC hardware PWM.
 *
 * The PWM hardware autonomously generates the 50 Hz signal once set.
 * The servo thread blocks on the message queue; when a new position arrives
 * it calls pwm_set_dt() and returns to blocking - no busy-waiting.
 *
 * Pulse-width calculation (all in nanoseconds):
 *
 *   period    = 20 000 000 ns  (20 ms)
 *   mid       =  1 500 000 ns  (1500 µs - stop)
 *   range     =    500 000 ns  (±500 µs either side of mid)
 *   pulse(p)  = mid + (int32_t)(p × range)
 *
 *   p = -1.0  ->  pulse = 1 000 000 ns  (1 ms, full CW)
 *   p =  0.0  ->  pulse = 1 500 000 ns  (1.5 ms, stopped)
 *   p = +1.0  ->  pulse = 2 000 000 ns  (2 ms, full CCW)
 *
 * DT node used:
 *   servo0_pwm  - defined in boards/esp32c3_devkitm.overlay
 *   ledc0 ch0   - IO9, 50 Hz, timer 0
 */

#include "servo_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(servo_ctrl, LOG_LEVEL_INF);

/* ── PWM timing constants (nanoseconds) ──────────────────────────────────── */
#define SERVO_PERIOD_NS   20000000u   /* 20 ms  - 50 Hz                  */
#define SERVO_MID_NS       1500000u   /* 1500 µs - neutral / stopped      */
#define SERVO_RANGE_NS      500000u   /* ±500 µs - full travel either way */

/* ── DT PWM specs - one entry per servo ─────────────────────────────────── */
static const struct pwm_dt_spec servo_pwm[NUM_SERVOS] = {
    [SERVO_0] = PWM_DT_SPEC_GET(DT_NODELABEL(servo0_pwm)),
};

/* ── Message queue ────────────────────────────────────────────────────────── */
#define SERVO_QUEUE_DEPTH  8u
K_MSGQ_DEFINE(servo_msgq, sizeof(servo_msg_t), SERVO_QUEUE_DEPTH, 4);

/* ── Thread ───────────────────────────────────────────────────────────────── */
#define SERVO_THREAD_STACK  1024u
#define SERVO_THREAD_PRIO   8

static K_THREAD_STACK_DEFINE(servo_stack, SERVO_THREAD_STACK);
static struct k_thread servo_thread_data;

/**
 * Clamp a float to [lo, hi] without pulling in libm.
 * Using a conditional expression keeps it fully inline.
 */
static inline float fclamp(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void servo_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    servo_msg_t msg;

    while (true) {
        /* Block indefinitely until a new command arrives */
        if (k_msgq_get(&servo_msgq, &msg, K_FOREVER) != 0) {
            continue;
        }

        if (msg.servo >= NUM_SERVOS) {
            LOG_WRN("servo_ctrl: invalid servo index %u", msg.servo);
            continue;
        }

        /* Map position [-1.0, +1.0] -> pulse width in nanoseconds */
        float pos = fclamp(msg.position, SERVO_POS_FULL_CW, SERVO_POS_FULL_CCW);
        uint32_t pulse_ns = (uint32_t)((int32_t)SERVO_MID_NS
                            + (int32_t)(pos * (float)SERVO_RANGE_NS));

        int rc = pwm_set_dt(&servo_pwm[msg.servo],
                            SERVO_PERIOD_NS, pulse_ns);
        if (rc != 0) {
            LOG_ERR("servo_ctrl: pwm_set_dt failed: %d", rc);
        } else {
            LOG_DBG("servo %u -> pos=%.3f pulse=%u ns",
                    msg.servo, (double)pos, pulse_ns);
        }
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */
int servo_ctrl_init(void)
{
    /* Verify PWM devices are ready */
    for (uint8_t i = 0u; i < NUM_SERVOS; i++) {
        if (!pwm_is_ready_dt(&servo_pwm[i])) {
            LOG_ERR("PWM device not ready for servo %u", i);
            return -ENODEV;
        }
        /* Park at neutral (stopped) */
        int rc = pwm_set_dt(&servo_pwm[i], SERVO_PERIOD_NS, SERVO_MID_NS);
        if (rc != 0) {
            LOG_ERR("servo_ctrl: init pwm_set_dt[%u] failed: %d", i, rc);
            return rc;
        }
    }

    /* Start consumer thread */
    k_thread_create(&servo_thread_data, servo_stack,
                    K_THREAD_STACK_SIZEOF(servo_stack),
                    servo_thread_fn, NULL, NULL, NULL,
                    SERVO_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&servo_thread_data, "servo");

    LOG_INF("Servo controller ready (%u servo(s), IO9, 50 Hz)", NUM_SERVOS);
    return 0;
}

int servo_set_position(uint8_t servo, float position)
{
    if (servo >= NUM_SERVOS) {
        return -EINVAL;
    }
    servo_msg_t msg = {
        .servo    = servo,
        .position = position,
    };
    return k_msgq_put(&servo_msgq, &msg, K_NO_WAIT);
}
