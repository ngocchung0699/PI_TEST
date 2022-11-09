#include "stdio.h"
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib.h"

#define PIN 18


int main()
{
    lib_init();
    pwm_setup(PIN, PWM_ENABLE, 1, 1024);
    pwm_write(PIN, 512);
    while(1)
    {  
        
    }  
    lib_close();
    return 0;
}
