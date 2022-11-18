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

int main() 
{   
    lib_init();
    // pwm_hw_setup(PWM_ENABLE, 19.2, 4);  // (T = 1*range us)
    // pwm_hw_write(1);
    pinMode(PIN, OUTPUT);
    pwm_write(PIN, 20, 1000);
    delay_ms(2000);
    pwm_write(PIN, 70, 1000);
    delay_ms(2000);
    pwm_off(PIN);
    delay_ms(2000);
    pwm_on(PIN);
    
    while (1)
    {

    }
    lib_close();
    return (EXIT_SUCCESS);
}
