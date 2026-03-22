/*
 * usb_serial.c
 *
 * Writes binary scanner frames to the USB CDC-ACM interface.
 * Zephyr's CDC-ACM UART driver is interrupt-driven; we use uart_fifo_fill()
 * which blocks only until the TX ring buffer accepts the bytes, not until
 * they are sent - so we never stall the output thread on USB flow control.
 *
 * The chosen node (cdc_acm_uart0) is declared in the board overlay and
 * aliased via the zephyr,console chosen property.
 */

#include "usb_serial.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_serial, LOG_LEVEL_INF);

#define USB_SERIAL_NODE DT_NODELABEL(usb_serial)

static const struct device *_dev;

/* Required by CDC-ACM driver — must be set even if unused */
static void _uart_irq_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
}


int usb_serial_init(void)
{
    _dev = DEVICE_DT_GET(USB_SERIAL_NODE);
    if (!device_is_ready(_dev)) {
        LOG_ERR("USB serial device not ready");
        return -ENODEV;
    }
    LOG_INF("USB serial ready");
    return 0;
}

void usb_serial_write(const uint8_t *buf, uint16_t len)
{
    if (!_dev) {
        return;
    }

    /*
     * uart_poll_out() is the safe path for CDC-ACM — it doesn't require
     * the IRQ TX path to be active and works correctly whether or not a
     * host is connected (bytes are silently dropped if no host).
     */
    for (uint16_t i = 0; i < len; i++) {
        uart_poll_out(_dev, buf[i]);
    }
}
