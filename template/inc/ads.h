#ifndef __ADS_H
#define __ADS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define addr 0x48

#define addr_confirm 0x83

#define addr_setup wiringPiI2CSetup(addr)

#define addr_write 0x01
#define addr_read 0x00
#define A0 0xC1
#define A1 0xD1
#define A2 0xE1
#define A3 0xF1

int ads_read(uint8_t chanel);

#endif
