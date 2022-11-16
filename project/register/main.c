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

#define PIN1 26
#define PIN2 19
#define PIN3 13

void thr1(){
    while (1)
    {
        // pwm_write(PWM2, 512);
        // uart_putc(0x41);
        digitalWrite(PIN2, HIGH);
        delay_us(100);
        digitalWrite(PIN2, LOW);
        delay_us(100);
    }
}
void thr2(){
    while (1)
    {
        // pwm_write(PWM2, 512);
        // uart_putc(0x41);
        digitalWrite(PIN3, HIGH);
        delay_us(100);
        digitalWrite(PIN3, LOW);
        delay_us(100);
    }
}

int main() 
{   
    lib_init();
    pthread_t threadId1 ;
    pthread_t threadId2 ;
    pthread_create (&threadId1, NULL, thr1, NULL) ;
    pthread_create (&threadId2, NULL, thr2, NULL) ;
    pinMode(PIN1, OUTPUT);
    pinMode(PIN2, OUTPUT);
    pinMode(PIN3, OUTPUT);
    // uart_setup(9600);
    //pwm_setup(PWM2, PWM_ENABLE, 16, 1024);
    //  pwm_set();
    
    while (1)
    {
        // pwm_write(PWM2, 512);
        // uart_putc(0x41);
        digitalWrite(PIN1, HIGH);
        delay_us(100);
        digitalWrite(PIN1, LOW);
        delay_us(100);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
