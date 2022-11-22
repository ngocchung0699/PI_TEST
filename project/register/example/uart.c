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
    uart_setup(115200);
    uart_send_char(0x41);
    while (1)
    {
        // char data = uart_receive();
        // printf("value: %d \n", data);
        // delay_ms(100);
        uart_send_char(0x41);
        delay_ms(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    


