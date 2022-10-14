#include <wiringPi/wiringPi.h>
#include <wiringPi/wiringPiI2C.h>>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <led.h>
#include <dht11.h>

#define addrSlave 0x08
 
int val = 0x10;   // DEC = 16
 
int main(){
 
	int fd = wiringPiI2CSetup(addrSlave);
 
	while(1){
		wiringPiI2CWrite(fd,val);
		printf("%d",val);
		fflush(stdout);
		delay(1000);
	}
	return 0;
}

