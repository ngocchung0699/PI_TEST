#include "stdio.h"
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "lib.h"
#include "lib_uart.h"

uint8_t var[10];

#define PIN 26

int main() {
	lib_init();
	pinMode(PIN, INPUT_PULLUP);
	gpio_rising_enable(PIN);
	while (1)
	{
		if(gpio_eds_flag(PIN) ==1){
			gpio_eds_clear_flag(PIN);
			printf("ON");
		}
	}
	
	return 0;
}
