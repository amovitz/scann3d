/*
 * uart_aux.c
 *
 * Interrupt-driven driver for the auxiliary UART0 (IO20 RX / IO21 TX).
 *
 * The RX ISR pushes received bytes into uart_aux_rx_msgq.  If the queue is
 * full, bytes are silently dropped - the application should drain the queue
 * frequently.  TX uses uart_fifo_fill() in a loop; it is blocking in the
 * sense that it yields until all bytes are accepted by the hardware FIFO,
 * but it does not busy-wait - k_yield() gives other threads CPU time.
 */

#include "uart_aux.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_aux, LOG_LEVEL_INF);

#define UART0_NODE       DT_ALIAS(aux_uart)    /* &uart0 via alias in overlay */
#define RX_QUEUE_DEPTH   256u

/* ── RX message queue (1-byte items, ISR → thread) ─────────────────────── */
K_MSGQ_DEFINE(uart_aux_rx_msgq, sizeof(uint8_t), RX_QUEUE_DEPTH, 1);

static const struct device *_uart_dev;

/* ── ISR ─────────────────────────────────────────────────────────────────── */
static void uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

        if (uart_irq_rx_ready(dev)) {
            uint8_t byte;
            int n = uart_fifo_read(dev, &byte, 1);
            if (n == 1) {
                /* k_msgq_put is ISR-safe with K_NO_WAIT */
                k_msgq_put(&uart_aux_rx_msgq, &byte, K_NO_WAIT);
            }
        }

        /* TX IRQ - not used; we fill the FIFO directly in uart_aux_write() */
        if (uart_irq_tx_ready(dev)) {
            uart_irq_tx_disable(dev);
        }
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */
int uart_aux_init(void)
{
    _uart_dev = DEVICE_DT_GET(UART0_NODE);
    if (!device_is_ready(_uart_dev)) {
        LOG_ERR("UART0 (aux) device not ready");
        return -ENODEV;
    }

    uart_irq_callback_set(_uart_dev, uart_isr);
    uart_irq_rx_enable(_uart_dev);

    LOG_INF("Auxiliary UART0 ready (IO20 RX / IO21 TX, 115200 baud)");
    return 0;
}

void uart_aux_write(const uint8_t *buf, size_t len)
{
    if (!_uart_dev || len == 0u) {
        return;
    }

    size_t sent = 0u;
    while (sent < len) {
        int n = uart_fifo_fill(_uart_dev, buf + sent, (int)(len - sent));
        if (n <= 0) {
            k_yield();   /* TX FIFO full - yield, don't spin */
            continue;
        }
        sent += (size_t)n;
    }
}
