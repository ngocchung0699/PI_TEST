#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

int data_read;

int main( void )
{
	printf( "Raspberry Pi wiringPi program\n" );
	
	if ( wiringPiSetupGpio() == -1 ){ exit( 1 ); }
	int fd = wiringPiI2CSetup(0x48);
	delay(1000);
	
	while ( 1 ){
		int data_send = (0x83<<8) | 0xC1;
		wiringPiI2CWriteReg16(fd,0x01,data_send);
		data_read = wiringPiI2CReadReg16(fd,0x00);
		printf("adc 0: %d\n", data_read);
		int data = (((uint8_t)data_read)<<8) | (data_read>>8);
		printf("data : %d\n", data);
		delay(5000);
	}
	return(0);
}

