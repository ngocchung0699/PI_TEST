#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "timer_sys.h"
#include "gpio.h"



int main() 
{
    pinMode(26,OUTPUT);
    while (1)
    {
        digitalWrite(26,1);
        delay_sys_ms(1);
        digitalWrite(26,0);
        delay_sys_ms(1);
    }
    return (EXIT_SUCCESS);
}
