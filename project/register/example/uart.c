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

uint8_t data = 0x53;

int main() 
{   
    lib_init();
    uart_hw_setup(115200);

    uart_setup(13, 19, 9600);
    
    while (1)
    {
        uart_send_string("hello world\r\n");
        delay_us(500);
    }
    lib_close();
    return (EXIT_SUCCESS);
}


