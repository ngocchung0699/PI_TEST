#ifndef __DHT11_H
#define __DHT11_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <wiringPi/wiringPi.h>

#define MAXTIMINGS 85
#define count 6

#define PIN 26    // pin data 

bool dht11_read(void);
float readHumidity(void);
float readTemperature(void);
int readTempAndHumidity(float* data);

#endif
