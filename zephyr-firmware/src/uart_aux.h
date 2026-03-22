/*
 * uart_aux.h
 *
 * Auxiliary UART (IO20 RX / IO21 TX) interface.
 *
 * UART0 is distinct from the USB CDC-ACM console. This module provides a
 * simple write path so any subsystem can send bytes out UART0 without
 * duplicating the device lookup. Receive is interrupt-driven and delivered
 * through a k_msgq for non-blocking consumption by any thread.
 *
 * Usage - transmit:
 *   uart_aux_write(buf, len);
 *
 * Usage - receive (one byte at a time from the ISR ring):
 *   uint8_t byte;
 *   while (k_msgq_get(&uart_aux_rx_msgq, &byte, K_NO_WAIT) == 0) {
 *       // process byte
 *   }
 */

#ifndef UART_AUX_H
#define UART_AUX_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Receive queue - filled by ISR, drained by application threads */
extern struct k_msgq uart_aux_rx_msgq;

/**
 * @brief  Initialise UART0 interrupt handler and RX queue.
 * @return 0 on success, negative errno on failure.
 */
int uart_aux_init(void);

/**
 * @brief  Transmit @p len bytes from @p buf over UART0.
 *         Blocks until all bytes are accepted by the TX FIFO.
 */
void uart_aux_write(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* UART_AUX_H */
