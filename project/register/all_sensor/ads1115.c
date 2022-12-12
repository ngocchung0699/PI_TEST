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

#define addr 0x48

#define addr_read 0x83

#define addr_write 0x01
#define addr_read 0x00
#define A0 0xC1
#define A1 0xD1
#define A2 0xE1
#define A3 0xF1


int ads_read(uint8_t chanel){
    uint8_t data_send[] = {0x01, 0x83, chanel};
    uint8_t data_read[10];
    i2c_send(addr, data_send, 3);
    i2c_receive(addr, data_read, 3);
    int data;
    data = data_read[1] << 8 | data_read[2];
    //int data_read = wiringPiI2CReadReg16(addr_setup, addr_read);
    return data;
}


int main() 
{   
    lib_init();
    i2c_init();
    while (1)
    {   
        printf("value: %d \n", ads_read(A0));
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
