#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <rgbled.h>
#include <pthread.h>

#define rgbled_pin 19

PI_THREAD (blinky)
{
	
  	for (;;)
  	{
		led_reset();
		for (int i = 0; i < MAX_LED; i++) { led_set(i, 35,70,0); }
		led_print(rgbled_pin);
		delay(1);

  	}
}

int main( void )
{
	printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|          Grove RGBLED STICK        |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |    Sensor Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       X          |       NC        |\n");
    printf("|       19          |       SIG       |\n");
    printf("+------------------------------------+\n");
	
	if ( wiringPiSetupGpio() == -1 ){ exit( 1 ); }
	piThreadCreate (blinky);

	pinMode(rgbled_pin,OUTPUT);

	while ( 1 ){

	}
	return(0);
}
