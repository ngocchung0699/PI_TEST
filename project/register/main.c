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
    pinMode(PIN, ALT5);
    pwm_setup(PWM0, PWM_ENABLE, 16, 1024);
    pwm_write(PWM0, 512);

    while (1)
    {

    }
    lib_close();
    return (EXIT_SUCCESS);
}
