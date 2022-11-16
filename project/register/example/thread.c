#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <stdint.h>
#include "lib.h"

#define PIN 26
#define PIN1 19

void *thr(){
    while (1)
    {
        // pwm_write(PWM2, 512);
        // uart_putc(0x41);
        digitalWrite(PIN1, HIGH);
        delay_us(100);
        digitalWrite(PIN1, LOW);
        delay_us(100);
    }
}

int main() 
{   
    lib_init();
    pthread_t threadId ;
    pthread_create (&threadId, NULL, thr, NULL) ;
    pinMode(PIN1, OUTPUT);
    pinMode(PIN, OUTPUT);

    while (1)
    {
        
        digitalWrite(PIN, HIGH);
        delay_us(100);
        digitalWrite(PIN, LOW);
        delay_us(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
