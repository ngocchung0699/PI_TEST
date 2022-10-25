#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <dht11.h>

float data[2];

int main( void )
{
	printf( "Raspberry Pi wiringPi DHT11 Temperature test program\n" );
 
	if ( wiringPiSetupGpio() == -1 )
		exit( 1 );
 
	while ( 1 )
	{
		if (!read_dht11(data)){
			printf("\n nhiet do: %f", data[1]);
			printf("\n do am: %f", data[0]);
		}
		
		else{
			printf( "Failed to get temprature and humidity value");
		}
		delay( 1000 ); 
	}
 
	return(0);
}