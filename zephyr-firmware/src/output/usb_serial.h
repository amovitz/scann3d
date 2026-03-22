/* usb_serial.h */
#ifndef USB_SERIAL_H
#define USB_SERIAL_H
#include <stdint.h>
#include <stddef.h>
int  usb_serial_init(void);
void usb_serial_write(const uint8_t *buf, uint16_t len);
#endif
