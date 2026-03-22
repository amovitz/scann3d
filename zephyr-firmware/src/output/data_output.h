/*
 * data_output.h
 *
 * Unified output layer. Callers push encoded frames into a k_msgq;
 * a dedicated output thread drains it to both UDP and USB-serial in parallel.
 */

#ifndef DATA_OUTPUT_H
#define DATA_OUTPUT_H

#include <stdint.h>
#include <stddef.h>
#include "../protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Frame message handed to the output queue */
typedef struct {
    uint8_t  buf[FRAME_BUF_MAX];
    uint16_t len;
} output_frame_t;

/**
 * @brief  Initialise the output subsystem (starts output thread).
 * @return 0 on success, negative errno on failure.
 */
int data_output_init(void);

/**
 * @brief  Enqueue a pre-encoded frame for transmission.
 *         Non-blocking: drops the frame if the queue is full.
 *
 * @param frame  Pointer to the frame to enqueue (copied into the queue).
 * @return 0 on success, -ENOMEM if queue is full.
 */
int data_output_push(const output_frame_t *frame);

/**
 * @brief  Convenience: encode + push in one call.
 */
int data_output_send(scanner_pkt_type_t type, uint16_t seq,
                     const void *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif
#endif /* DATA_OUTPUT_H */
