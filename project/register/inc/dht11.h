#ifndef __DHT11_H
#define __DHT11_H


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


#define MAXTIMINGS	85
#define DHTPIN		18

bool read_dht11(float *data);

#endif
