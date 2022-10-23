#include <ads.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define addr 0x48

#define addr_read 0x83

#define addr_setup wiringPiI2CSetup(addr)

#define addr_write 0x01
#define addr_read 0x00
#define A0 0xC1
#define A1 0xD1
#define A2 0xE1
#define A3 0xF1

int ads_read(uint8_t chanel){
    int data_send = (0x83<<8) | chanel;
    wiringPiI2CWriteReg16(addr_setup, 0x01, data_send);
    int data_read = wiringPiI2CReadReg16(addr_setup, addr_read);
    return (((uint8_t)data_read)<<8) | (data_read>>8);
}


