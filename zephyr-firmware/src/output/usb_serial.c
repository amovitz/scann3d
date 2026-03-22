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
#include <zephyr/usb/usb_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_serial, LOG_LEVEL_INF);

#define CDC_UART_NODE  DT_NODELABEL(cdc_acm_uart0)

static const struct device *_cdc_dev;

int usb_serial_init(void)
{
    _cdc_dev = DEVICE_DT_GET(CDC_UART_NODE);
    if (!device_is_ready(_cdc_dev)) {
        LOG_ERR("CDC-ACM device not ready");
        return -ENODEV;
    }

    int rc = usb_enable(NULL);
    if (rc != 0 && rc != -EALREADY) {
        LOG_ERR("usb_enable failed: %d", rc);
        return rc;
    }

    /*
     * Wait for the host to open the port (DTR set).
     * We poll up to 5 s; if no host connects we continue anyway -
     * the TX bytes will simply be discarded until a host opens the port.
     */
    uint32_t dtr  = 0;
    int      wait = 50;
    while (wait-- > 0 && !dtr) {
        uart_line_ctrl_get(_cdc_dev, UART_LINE_CTRL_DTR, &dtr);
        k_msleep(100);
    }

    if (!dtr) {
        LOG_WRN("No USB host detected - streaming will begin when connected");
    } else {
        LOG_INF("USB CDC-ACM ready");
    }
    return 0;
}

void usb_serial_write(const uint8_t *buf, uint16_t len)
{
    if (!_cdc_dev) { return; }

    /*
     * uart_fifo_fill() returns the number of bytes actually accepted by the
     * TX FIFO.  Loop until all bytes are queued to handle partial fills.
     */
    uint16_t sent = 0;
    while (sent < len) {
        int n = uart_fifo_fill(_cdc_dev, buf + sent, len - sent);
        if (n <= 0) {
            /* TX buffer full - yield and retry */
            k_yield();
            continue;
        }
        sent += (uint16_t)n;
    }
}
