
#ifndef	__LIB_UART_H__
#define	__LIB_UART_H__

int   serial_open      (const char *device, unsigned long baud_rate) ;
void  serial_close     (const int fd) ;

void  serial_send_char   (const int serial_port, unsigned long baud_rate, const unsigned char c) ;
void  serial_send_string      (const int serial_port, unsigned long baud_rate, const char *s) ;
void  serial_send    (const int fd, const char *message, ...) ;
int   serial_data_avail (const int serial_port, unsigned long baud_rate) ;
int   serial_get_char   (const int serial_port, unsigned long baud_rate) ;

#endif
