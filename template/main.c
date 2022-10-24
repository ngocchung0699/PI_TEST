#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ads.h>

int main( void )
{
	printf( "Raspberry Pi wiringPi program\n" );
	
	if ( wiringPiSetupGpio() == -1 ){ exit( 1 ); }
	pinMode(26, OUTPUT);
	while ( 1 ){
		digitalWrite(26,HIGH);
	}
	return(0);
}

