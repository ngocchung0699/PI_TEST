#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "lib.h"

#define PIN 18

void angle_servo(int angle)
{
    int angle_to_pwm = map(angle, -90, 90, 1000, 2000);
    digitalWrite(PIN, 1);
    delay_us(angle_to_pwm);
    digitalWrite(PIN, 0);
    delay_us(20000-angle_to_pwm);
}

int main() 
{   
    lib_init();
    pinMode(PIN, OUTPUT);

    while (1)
    {   
        angle_servo(60);
        delay_ms(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
