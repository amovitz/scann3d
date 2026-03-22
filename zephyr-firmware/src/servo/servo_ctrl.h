/*
 * servo_ctrl.h
 *
 * 360-degree continuous-rotation servo controller for scann3d.
 *
 * Uses the ESP32-C3 LEDC hardware PWM peripheral (channel 0, IO9).
 * Standard RC servo protocol:
 *
 *   Period   : 20 ms  (50 Hz)
 *   Pulse min: 1000 µs  → full clockwise
 *   Pulse mid: 1500 µs  → stopped
 *   Pulse max: 2000 µs  → full counter-clockwise
 *
 * The position value maps linearly:
 *   -1.0  →  1000 µs pulse  (full CW)
 *    0.0  →  1500 µs pulse  (stopped)
 *   +1.0  →  2000 µs pulse  (full CCW)
 *
 * Messages are queued and consumed by a dedicated low-priority thread.
 * Once a position is set, the PWM hardware holds it autonomously until
 * the next message arrives.
 *
 * Message format:
 *   { .servo = SERVO_0, .position = 0.5f }  → 75% CCW speed
 */

#ifndef SERVO_CTRL_H
#define SERVO_CTRL_H

#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Servo indices ────────────────────────────────────────────────────────── */
#define SERVO_0    0u
#define NUM_SERVOS 1u   /* expandable - add PWM specs and DT nodes as needed */

/* ── Position constants ───────────────────────────────────────────────────── */
#define SERVO_POS_STOP     ( 0.0f)
#define SERVO_POS_FULL_CW  (-1.0f)
#define SERVO_POS_FULL_CCW ( 1.0f)

/* ── Message ──────────────────────────────────────────────────────────────── */
typedef struct {
    uint8_t servo;     /* Servo index - SERVO_0, etc.                */
    float   position;  /* Clamped to [-1.0, +1.0]; 0.0 = stop       */
} servo_msg_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise PWM hardware and start the servo consumer thread.
 *
 * All servos are placed at SERVO_POS_STOP (1500 µs) on init.
 *
 * @return 0 on success, negative errno on failure.
 */
int servo_ctrl_init(void);

/**
 * @brief  Command a servo to a position.  Non-blocking; queued.
 *
 * @param servo     Servo index (SERVO_0 … NUM_SERVOS-1).
 * @param position  Target position [-1.0, +1.0].
 * @return 0 on success, -ENOMEM if the queue is full, -EINVAL if out of range.
 */
int servo_set_position(uint8_t servo, float position);

/** @brief Direct queue access for callers that build their own servo_msg_t. */
extern struct k_msgq servo_msgq;

#ifdef __cplusplus
}
#endif
#endif /* SERVO_CTRL_H */
