/* udp_out.h */
#ifndef UDP_OUT_H
#define UDP_OUT_H
#include <stdint.h>
#include <stddef.h>
int  udp_out_init(void);
void udp_out_send(const uint8_t *buf, uint16_t len);
#endif
