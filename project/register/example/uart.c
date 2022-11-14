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
    uart_setup(0, 9600);
    
    while (1)
    {
        uart_putc(0x41);
        delay_us(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
