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
#include "led_bar.h"

int main() 
{   
    lib_init();

    pinMode(pin_clock, OUTPUT);
    pinMode(pin_data, OUTPUT);

    while (1)
    {   
        for(int i = 1; i <= 10; i++)
        {
            set_led(i, 1);
            delay_ms(200);
        }
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
