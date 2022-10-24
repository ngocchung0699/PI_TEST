#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <lib_i2c.h>
#include <ds1307.h>

uint8_t dt[10];

int main( void )
{
	printf( "Raspberry Pi wiringPi test program\n" );
	
	if ( wiringPiSetupGpio() == -1 ){
		exit( 1 );
	}
	rtc rtc;
	
	rtc.second = 15;
	rtc.minute = 45;
	rtc.hours = 10;
	rtc.day = 30;
	rtc.month = 9;
	rtc.year = 22;
	
	int fd = i2c_setup(1, 0x68);
	set_time(fd, rtc);
	while ( 1 ){
		//get_time(fd, &rtc);
		//i2c_smbus_read_byte_data(fd,0x00);
		//i2c_smbus_read_block_data(fd, 0x00, dt);
		//delay(1000);
	}
	return(0);
}
