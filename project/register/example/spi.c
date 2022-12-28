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
    spi_init();
    while (1)
    {
        spi_send(0, (uint8_t*) "hello\r\n", 8);
        delay_ms(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    


