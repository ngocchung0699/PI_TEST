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
        delay_ms(500);
        digitalWrite(PIN, LOW);
        delay_ms(500);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
