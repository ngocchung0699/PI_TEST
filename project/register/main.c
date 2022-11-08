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

#define PIN 26

int i=0;
void iqr(){
    i++;
    gpio_eds_clear_flag(PIN);
}

int main()
{
    lib_init();
    iqr_setup(PIN, HIGH, iqr);
    while(1)
    {  
        printf("value: %d", i);
        delay_ms(100);
    }  
    lib_close();
    return 0;
}
