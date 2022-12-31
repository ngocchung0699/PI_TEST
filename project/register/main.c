#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "lib.h"


int main() 
{   
    lib_init();
    uart_init(UART0, 115200);
    while (1)
    {
        // char data = uart_receive();
        // printf("value: %d \n", data);
        // delay_ms(100);
        uart_send_string(UART0, "hello\r\n");
        // spi_send(0, (uint8_t*) "hello", 6);
        delay_ms(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    


