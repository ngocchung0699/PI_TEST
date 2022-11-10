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
#include <pthread.h>
#include "lib.h"

#define PIN 26

int main()
{
    lib_init();
    pinMode(PIN, INPUT_PULLUP);
    gpio_set_pud(PIN, INPUT_PULLUP);
    gpio_rising_enable(PIN);
    while(1)
    {  
        if(gpio_eds_flag(PIN) == 1){
            gpio_eds_clear_flag(PIN);
            printf("iqr on");
        }
        if(gpio_eds_flag(PIN) == 0){
            printf("iqr off");
        }
    }  
    lib_close();
    return 0;
}
