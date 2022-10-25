#ifndef __DHT11_H
#define __DHT11_H

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>


#define MAXTIMINGS	85
#define DHTPIN		26

bool read_dht11(float *data);

#endif
