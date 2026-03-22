/*
 * led_ctrl.h
 *
 * GPIO-muxed RGB LED controller for scann3d.
 *
 * Two common-cathode RGB LEDs (D1 and D2) are driven by individual GPIO pins -
 * no PWM. Mixed colours (yellow, cyan, magenta, white) are produced by rapid
 * time-division multiplexing at ~750 Hz, which is well above the flicker-
 * fusion threshold (~60 Hz). Each active channel in a mixed colour receives
 * an equal share of the duty cycle:
 *
 *   WHITE  -> R/G/B each at ~250 Hz  (750 Hz ÷ 3)
 *   YELLOW -> R/G   each at ~375 Hz  (750 Hz ÷ 2)
 *   etc.
 *
 * Two message types share a single queue (tagged union):
 *
 *   LED_MSG_COLOR  {diode, color}             - set instant colour
 *   LED_MSG_BLINK  {diode, on_ms, off_ms}     - set blink timing
 *
 * Setting on_ms = 0 on a blink message disables blinking and returns the
 * LED to its current colour, steady-on.
 *
 * Convenience wrappers (led_set_color, led_set_blink) are provided so callers
 * don't need to touch the queue directly.
 */

#ifndef LED_CTRL_H
#define LED_CTRL_H

#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Diode indices ────────────────────────────────────────────────────────── */
#define LED_D1  0u
#define LED_D2  1u
#define NUM_LEDS 2u

/* ── Colour palette ───────────────────────────────────────────────────────── */
typedef enum {
    LED_COLOR_OFF     = 0,
    LED_COLOR_RED     = 1,
    LED_COLOR_GREEN   = 2,
    LED_COLOR_BLUE    = 3,
    LED_COLOR_YELLOW  = 4,   /* R + G muxed */
    LED_COLOR_CYAN    = 5,   /* G + B muxed */
    LED_COLOR_MAGENTA = 6,   /* R + B muxed */
    LED_COLOR_WHITE   = 7,   /* R + G + B muxed */
    _LED_COLOR_MAX
} led_color_t;

/* ── Message types ────────────────────────────────────────────────────────── */
typedef enum {
    LED_MSG_COLOR = 0,   /* set colour immediately */
    LED_MSG_BLINK = 1,   /* configure blink timing */
} led_msg_type_t;

typedef struct {
    led_msg_type_t type;
    uint8_t        diode;     /* LED_D1 or LED_D2 */
    led_color_t    color;     /* LED_MSG_COLOR */
    uint32_t       blink_ms;  /* LED_MSG_BLINK */
} led_msg_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise GPIOs, start the mux timer and message-queue thread.
 * @return 0 on success, negative errno on failure.
 */
int led_ctrl_init(void);

/**
 * @brief  Set the colour of one LED. Non-blocking; queued.
 *
 * @param diode  LED_D1 or LED_D2
 * @param color  Desired colour (LED_COLOR_OFF turns the LED off)
 * @return 0 on success, -ENOMEM if the queue is full.
 */
int led_set_color(uint8_t diode, led_color_t color);

/**
 * @brief  Configure blink timing for one LED. Non-blocking; queued.
 *         The LED cycles between the current colour (on_ms) and off (off_ms).
 *         Pass on_ms = 0 to disable blinking.
 *
 * @param diode     LED_D1 or LED_D2
 * @param color     Desired colour (LED_COLOR_OFF turns the LED off)
 * @param blink_ms  ON/OFF duration in milliseconds
 * @return 0 on success, -ENOMEM if the queue is full.
 */
int led_set_blink(uint8_t diode, led_color_t color, uint32_t blink_ms);

/** @brief Direct queue access for callers that build their own led_msg_t. */
extern struct k_msgq led_msgq;

#ifdef __cplusplus
}
#endif
#endif /* LED_CTRL_H */
