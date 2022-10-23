#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define flame_pin 5

int main (void)
{
    printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|            Grove  Flame            |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |    Sensor Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       X          |       NC        |\n");
    printf("|       5          |       SIG       |\n");
    printf("+------------------------------------+\n");

    if ( wiringPiSetupGpio() == -1 ){ exit(1); }

    pinMode(flame_pin, INPUT);
    
    while (1)
    {
        if(digitalRead(flame_pin)){
            printf("ON");
        }
        else{
            printf("OFF");
        }
    }

    return 0 ;
}