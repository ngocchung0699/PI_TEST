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

int main() {
	int fd = serial_open ("/dev/serial0", 9600);
	while(1) {
		int i;
		while (serial_data_avail(fd) > 0 )
		{
			int c = serial_get_char(fd);
			var[i++] = c;
			if(var[i-1] == 0x0A && var[i] == 0x0D){
				break;
			}
		}
		printf("receive: %s", var);
		
	}
	return 0;
}
