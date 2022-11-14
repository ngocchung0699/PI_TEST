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

#define PIN 26

#define DIVISOR 16
#define RANGE 1024

int main() 
{   
    lib_init();
    pwm_setup(PWM0, PWM_ENABLE, DIVISOR, RANGE);
    pwm_write(PWM0, 512);
    while (1)
    {

    }
    lib_close();
    return (EXIT_SUCCESS);
}
