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

int main() {

	int fd;
 
	printf("Raspberry's sending : \n");
    if((fd = serialOpen ("/dev/serial0", 9600)) < 0 ){
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
	}
	while(1) {
		
		serialPuts(fd, "hello");
		serialFlush(fd);
		printf("%s\n", "hello");
		fflush(stdout);
		delay(1000);
	}
	return 0;
}