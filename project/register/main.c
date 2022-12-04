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

uint8_t data1[] = {0x80, 0x28};
uint8_t data2[] = {0x80, 0x0c};
uint8_t data3[] = {0x80, 0x01};
uint8_t data4[] = {0x80, 0x06};
uint8_t data5[] = {0x80, 0x01};

uint8_t data6[] = {0x80, 0xc1};
uint8_t data7[] = {0x40, 0x41};

char rec[5];
int main() 
{   
    lib_init();
    i2c_setup();

    i2c_start();
    i2c_set_slave_address(0x3E);
    i2c_write(data1, 2);
    i2c_write(data2, 2);
    i2c_write(data3, 2);
    i2c_write(data4, 2);
    i2c_write(data5, 2);
    delay_ms(1);
    i2c_write(data6, 2);
    i2c_write(data7, 2);

    i2c_end();


    while (1)
    {

    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
