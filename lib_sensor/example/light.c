#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define light_pin 6

int main (void)
{
    printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|            Grove  Light            |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |    Sensor Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       X          |       NC        |\n");
    printf("|       6          |       SIG       |\n");
    printf("+------------------------------------+\n");

    if ( wiringPiSetupGpio() == -1 ){ exit(1); }

    pinMode(light_pin, INPUT);
    
    while (1)
    {
        if(digitalRead(light_pin)){
            printf("ON");
        }
        else{
            printf("OFF");
        }
    }

    return 0 ;
}