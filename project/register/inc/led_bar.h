#ifndef	__LED_BAR_H__
#define	__LED_BAR_H__

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib.h"

#define pin_clock 13
#define pin_data 19

void start_led(void);
void end_led(void);
void set_led(int num, bool status);
void set_level_led(int num, bool dir);
void random_led(void);

#endif