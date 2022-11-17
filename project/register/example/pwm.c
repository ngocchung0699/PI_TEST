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

int main() 
{   
    lib_init();
    pwm_hw_setup(PWM_ENABLE, 19.2, 4);  // (T = 1*range us)
    pwm_hw_write(1);
    while (1)
    {

    }
    lib_close();
    return (EXIT_SUCCESS);
}
