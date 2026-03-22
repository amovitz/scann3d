/*
 * data_output.c
 *
 * A k_msgq holds up to OUTPUT_QUEUE_DEPTH frames.
 * The output thread pulls frames and dispatches them to both sinks in series.
 * UDP and USB-serial writes are fire-and-forget; a failed send is counted
 * but does not stall the pipeline.
 */

#include "data_output.h"
#include "udp_out.h"
#include "usb_serial.h"
#include "../protocol.h"
#include "led/led_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(data_output, LOG_LEVEL_INF);

#define OUTPUT_QUEUE_DEPTH  16
#define OUTPUT_THREAD_STACK 2048
#define OUTPUT_THREAD_PRIO  5

K_MSGQ_DEFINE(output_q, sizeof(output_frame_t), OUTPUT_QUEUE_DEPTH, 4);

static K_THREAD_STACK_DEFINE(output_stack, OUTPUT_THREAD_STACK);
static struct k_thread output_thread_data;

static void output_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    output_frame_t frame;

    while (true) {
        if (k_msgq_get(&output_q, &frame, K_FOREVER) == 0) {
            led_set_color(LED_D1, LED_COLOR_OFF);
            udp_out_send(frame.buf, frame.len);
            usb_serial_write(frame.buf, frame.len);
            led_set_color(LED_D1, LED_COLOR_GREEN);
        }
    }
}

int data_output_init(void)
{
    int rc;

    rc = udp_out_init();
    if (rc != 0) {
        LOG_ERR("UDP init failed: %d", rc);
        /* Non-fatal - USB serial still works */
    }

    rc = usb_serial_init();
    if (rc != 0) {
        LOG_ERR("USB serial init failed: %d", rc);
    }

    k_thread_create(&output_thread_data, output_stack,
                    K_THREAD_STACK_SIZEOF(output_stack),
                    output_thread, NULL, NULL, NULL,
                    OUTPUT_THREAD_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&output_thread_data, "output");

    LOG_INF("Output subsystem started");
    return 0;
}

int data_output_push(const output_frame_t *frame)
{
    return k_msgq_put(&output_q, frame, K_NO_WAIT);
}

int data_output_send(scanner_pkt_type_t type, uint16_t seq,
                     const void *payload, uint16_t payload_len)
{
    output_frame_t frame;
    frame.len = proto_encode(frame.buf, sizeof(frame.buf),
                             type, seq, payload, payload_len);
    if (frame.len == 0) {
        return -EINVAL;
    }
    return data_output_push(&frame);
}
