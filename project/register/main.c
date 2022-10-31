#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "timer.h"
#include "gpio.h"



int main() 
{
    pinMode(26,OUTPUT);
    timer_init();
    while (1)
    {
        digitalWrite(26,1);
        timer_delay(1);
        digitalWrite(26,0);
        timer_delay(1);
    }
    return (EXIT_SUCCESS);
}
