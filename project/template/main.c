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
	while ( 1 ){
		printf("a0: %d", ads_read(A0));
		delay(1000);
	}
	return(0);
}

