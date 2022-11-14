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

int main() 
{   
    lib_init();
    pinMode(PIN, OUTPUT);
    
    while (1)
    {
        digitalWrite(PIN, HIGH);
        delay_us(100);
        digitalWrite(PIN, LOW);
        delay_us(100);
    }

    return (EXIT_SUCCESS);
}
