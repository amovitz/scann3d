/*
 * led_ctrl.c
 *
 * Time-division multiplexed RGB LED driver.
 *
 * Architecture
 * ─────────────
 *  led_mux_timer  - k_timer firing every MUX_PERIOD_US µs (~750 Hz).
 *                   Submits led_mux_work to the system workqueue.
 *
 *  led_mux_work   - Workqueue handler; does the real work:
 *                   1. Drain all pending messages from led_msgq.
 *                   2. For each LED, determine visibility (blink state).
 *                   3. Advance the mux slot and set the correct GPIO.
 *
 * GPIO indexing (per-LED channel array index -> colour bit):
 *   [0] = Red   [1] = Green   [2] = Blue
 *
 * colour_mask[] encodes which channels are active as a 3-bit field:
 *   bit 0 = Red,  bit 1 = Green,  bit 2 = Blue
 */

#include "led_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_ctrl, LOG_LEVEL_INF);

/* ── Tuning ───────────────────────────────────────────────────────────────── */
#define MUX_PERIOD_US    1333u   /* ~750 Hz timer -> 250 Hz per channel in WHITE */
#define LED_QUEUE_DEPTH  8u

/* ── DT GPIO specs  [led_index][channel: 0=R 1=G 2=B] ────────────────────── */
static const struct gpio_dt_spec led_gpios[NUM_LEDS][3] = {
    [LED_D1] = {
        [0] = GPIO_DT_SPEC_GET(DT_NODELABEL(d1_r), gpios),
        [1] = GPIO_DT_SPEC_GET(DT_NODELABEL(d1_g), gpios),
        [2] = GPIO_DT_SPEC_GET(DT_NODELABEL(d1_b), gpios),
    },
    [LED_D2] = {
        [0] = GPIO_DT_SPEC_GET(DT_NODELABEL(d2_r), gpios),
        [1] = GPIO_DT_SPEC_GET(DT_NODELABEL(d2_g), gpios),
        [2] = GPIO_DT_SPEC_GET(DT_NODELABEL(d2_b), gpios),
    },
};

/* ── Colour -> active-channel bitmask ─────────────────────────────────────── */
#define CH_R  BIT(0)
#define CH_G  BIT(1)
#define CH_B  BIT(2)

static const uint8_t color_mask[_LED_COLOR_MAX] = {
    [LED_COLOR_OFF]     = 0,
    [LED_COLOR_RED]     = CH_R,
    [LED_COLOR_GREEN]   = CH_G,
    [LED_COLOR_BLUE]    = CH_B,
    [LED_COLOR_YELLOW]  = CH_R | CH_G,
    [LED_COLOR_CYAN]    = CH_G | CH_B,
    [LED_COLOR_MAGENTA] = CH_R | CH_B,
    [LED_COLOR_WHITE]   = CH_R | CH_G | CH_B,
};

/* ── Per-LED runtime state ────────────────────────────────────────────────── */
struct led_state {
    led_color_t color;
    uint8_t     mux_slot;      /* index into active channels list, 0-based */
    uint32_t    blink_on_ms;   /* 0 = blink disabled                       */
    uint32_t    blink_off_ms;
};

static struct led_state led_state[NUM_LEDS];

/* ── Message queue ────────────────────────────────────────────────────────── */
K_MSGQ_DEFINE(led_msgq, sizeof(led_msg_t), LED_QUEUE_DEPTH, 4);

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/** Return the index (0/1/2) of the n-th set bit in mask, or 0xFF if none. */
static uint8_t nth_set_bit(uint8_t mask, uint8_t n)
{
    for (uint8_t i = 0u; i < 3u; i++) {
        if (mask & BIT(i)) {
            if (n == 0u) {
                return i;
            }
            n--;
        }
    }
    return 0xFFu;
}

/** Count set bits in a 3-bit mask. */
static uint8_t popcount3(uint8_t mask)
{
    return (uint8_t)(__builtin_popcount(mask & 0x07u));
}

/** Turn off all three channels of one LED. */
static inline void led_all_off(uint8_t led)
{
    gpio_pin_set_dt(&led_gpios[led][0], 0);
    gpio_pin_set_dt(&led_gpios[led][1], 0);
    gpio_pin_set_dt(&led_gpios[led][2], 0);
}

/** Set exactly one channel of one LED on, the rest off. */
static inline void led_set_channel(uint8_t led, uint8_t ch)
{
    gpio_pin_set_dt(&led_gpios[led][0], ch == 0u ? 1 : 0);
    gpio_pin_set_dt(&led_gpios[led][1], ch == 1u ? 1 : 0);
    gpio_pin_set_dt(&led_gpios[led][2], ch == 2u ? 1 : 0);
}

/* ── Mux work handler ─────────────────────────────────────────────────────── */
static void led_mux_work_handler(struct k_work *w)
{
    ARG_UNUSED(w);

    /* 1. Drain all queued messages ─────────────────────────────────────────── */
    led_msg_t msg;
    while (k_msgq_get(&led_msgq, &msg, K_NO_WAIT) == 0) {
        if (msg.diode >= NUM_LEDS) {
            continue;
        }
        struct led_state *s = &led_state[msg.diode];

        if (msg.type == LED_MSG_COLOR) {
            if (msg.color < _LED_COLOR_MAX) {
                s->color    = msg.color;
                s->mux_slot = 0u;
            }
        } else {  /* LED_MSG_BLINK */
            s->blink_on_ms  = msg.blink.on_ms;
            s->blink_off_ms = msg.blink.off_ms;
        }
    }

    /* 2. Update each LED ───────────────────────────────────────────────────── */
    int64_t now_ms = k_uptime_get();

    for (uint8_t led = 0u; led < NUM_LEDS; led++) {
        struct led_state *s = &led_state[led];

        /* ── Blink gating ──────────────────────────────────────────────────── */
        bool visible = true;
        if (s->blink_on_ms > 0u) {
            uint32_t cycle = s->blink_on_ms + s->blink_off_ms;
            if (cycle > 0u) {
                uint32_t phase = (uint32_t)(now_ms % (int64_t)cycle);
                visible = (phase < s->blink_on_ms);
            }
        }

        /* ── Apply colour via mux ──────────────────────────────────────────── */
        uint8_t mask = color_mask[s->color];

        if (!visible || mask == 0u) {
            led_all_off(led);
            continue;
        }

        uint8_t nch = popcount3(mask);
        if (nch == 1u) {
            /* Pure colour - no muxing needed, hold the channel steady */
            led_set_channel(led, nth_set_bit(mask, 0u));
        } else {
            /* Mixed colour - advance slot and activate next channel */
            s->mux_slot = (uint8_t)((s->mux_slot + 1u) % nch);
            led_set_channel(led, nth_set_bit(mask, s->mux_slot));
        }
    }
}

static K_WORK_DEFINE(led_mux_work, led_mux_work_handler);

/* ── Mux timer callback (ISR context - just submits work) ─────────────────── */
static void led_mux_timer_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_work_submit(&led_mux_work);
}

static K_TIMER_DEFINE(led_mux_timer, led_mux_timer_cb, NULL);

/* ── Public API ───────────────────────────────────────────────────────────── */
int led_ctrl_init(void)
{
    /* Configure all LED GPIOs as outputs, initially LOW (off) */
    for (uint8_t led = 0u; led < NUM_LEDS; led++) {
        for (uint8_t ch = 0u; ch < 3u; ch++) {
            const struct gpio_dt_spec *g = &led_gpios[led][ch];
            if (!gpio_is_ready_dt(g)) {
                LOG_ERR("LED GPIO not ready: led=%u ch=%u", led, ch);
                return -ENODEV;
            }
            gpio_pin_configure_dt(g, GPIO_OUTPUT_INACTIVE);
        }
        led_state[led].color        = LED_COLOR_OFF;
        led_state[led].mux_slot     = 0u;
        led_state[led].blink_on_ms  = 0u;
        led_state[led].blink_off_ms = 0u;
    }

    /* Start mux timer */
    k_timer_start(&led_mux_timer,
                  K_USEC(MUX_PERIOD_US),
                  K_USEC(MUX_PERIOD_US));

    LOG_INF("LED controller started (mux @ ~%u Hz)",
            (unsigned)(1000000u / MUX_PERIOD_US));
    return 0;
}

int led_set_color(uint8_t diode, led_color_t color)
{
    if (diode >= NUM_LEDS || color >= _LED_COLOR_MAX) {
        return -EINVAL;
    }
    led_msg_t msg = {
        .type  = LED_MSG_COLOR,
        .diode = diode,
        .color = color,
    };
    return k_msgq_put(&led_msgq, &msg, K_NO_WAIT);
}

int led_set_blink(uint8_t diode, uint32_t on_ms, uint32_t off_ms)
{
    if (diode >= NUM_LEDS) {
        return -EINVAL;
    }
    led_msg_t msg = {
        .type           = LED_MSG_BLINK,
        .diode          = diode,
        .blink.on_ms    = on_ms,
        .blink.off_ms   = off_ms,
    };
    return k_msgq_put(&led_msgq, &msg, K_NO_WAIT);
}
