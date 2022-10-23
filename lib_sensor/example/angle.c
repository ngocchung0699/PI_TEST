#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ads.h>

int main (void)
{
    printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|             Grove Angle            |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |   ADS1115 Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       SDA1       |       SDA       |\n");
    printf("|       SCL1       |       SCL       |\n");
    printf("+------------------------------------+\n");
    printf("|    ADS1115 Pin   |    Angle Pin    |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       X          |       NC        |\n");
    printf("|       A0         |       SIG       |\n");
    printf("+------------------------------------+\n");

    if ( wiringPiSetupGpio() == -1 ){ exit(1); }
    
    while (1)
    {
        printf("ANGLE VALUE: %d", ads_read(A0));
        delay(1000);
    }

    return 0 ;
}