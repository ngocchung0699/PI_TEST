#include <ads.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

int ads_read(uint8_t chanel){
    int data_send = (addr_confirm<<8) | chanel;
    wiringPiI2CWriteReg16(addr_setup, addr_write, data_send);
    int data_read = wiringPiI2CReadReg16(addr_setup, addr_read);
    return (((uint8_t)data_read)<<8) | (data_read>>8);
}


