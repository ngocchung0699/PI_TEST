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

#define PIN 18

int main() 
{   
    lib_init();
    aux_uart_setup(115200);
    aux_uart_send_char(0x41);
    while (1)
    {
        aux_uart_send_string("hello \r\n");
        delay_ms(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    


