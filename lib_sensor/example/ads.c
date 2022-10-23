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
	printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|               ADS1115              |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |    Sensor Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       SDA1       |       SDA       |\n");
    printf("|       SCL1       |       SCL       |\n");
    printf("+------------------------------------+\n");
	
	if ( wiringPiSetupGpio() == -1 ){ exit( 1 ); }
	while ( 1 ){
		printf("A0: %d\n", ads_read(A0));
        printf("A1: %d\n", ads_read(A1));
        printf("A2: %d\n", ads_read(A2));
        printf("A3: %d\n", ads_read(A3));
		delay(1000);
	}
	return(0);
}

