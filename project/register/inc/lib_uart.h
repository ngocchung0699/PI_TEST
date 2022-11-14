
#ifndef	__LIB_UART_H__
#define	__LIB_UART_H__

#include <stdint.h>

int serial_open (const char *device, unsigned long baud_rate);
void serial_close (const int fd);

void serial_send_char (const int serial_port, const unsigned char c);
void serial_send_string (const int serial_port, const char *s);
void serial_send (const int serial_port, const char *message, ...);
int serial_data_avail (const int serial_port);
uint8_t serial_get_char (const int serial_port);
#endif
