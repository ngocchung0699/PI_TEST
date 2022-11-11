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

int main() {

	while(1) {
		serial_send_char(0, 9600, 0x41);
	}
	return 0;
}