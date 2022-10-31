
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

#include "gpio.h"

static volatile unsigned int *gpio ;
static volatile unsigned int *pwm ;

// gpio_GPFSEL:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPFSEL 0-5)
//	Groups of 10 - 3 bits per Function - 30 bits per port
static uint8_t GPIO_GPFSEL [] =
{
  GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,GPFSEL0,
  GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,GPFSEL1,
  GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,GPFSEL2,
  GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,GPFSEL3,
  GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,GPFSEL4,
  GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5,GPFSEL5
} ;


// gpioToShift
//	Define the shift up for the 3 bits per pin in each GPFSEL port
static uint8_t GPIO_SHIFT [] =
{
    0,3,6,9,12,15,18,21,24,27,
    0,3,6,9,12,15,18,21,24,27,
    0,3,6,9,12,15,18,21,24,27,
    0,3,6,9,12,15,18,21,24,27,  
    0,3,6,9,12,15,18,21,24,27,
    0,3,6,9,12,15,18,21,24,27
} ;

// gpioToGPSET:
//	(Word) offset to the GPIO Set registers for each GPIO pin
static uint8_t GPIO_GPSET [] =
{
    GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, 
    GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, 
    GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, 
    GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0, GPSET0,
    GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, 
    GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, 
    GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, 
    GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1, GPSET1
} ;

static uint8_t GPIO_GPCLR [] =
{
  GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,
  GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,
  GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,
  GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,GPCLR0,
  GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,
  GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,
  GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,
  GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1,GPCLR1
} ;

// gpioToGPLEV:
//	(Word) offset to the GPIO Input level registers for each GPIO pin
static uint8_t GPIO_GPLEV [] =
{
    GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0,
    GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0,
    GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0,
    GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0, GPLEV0,
    GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1,
    GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1,
    GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1,
    GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1, GPLEV1
} ;


void pinMode(int pin, int mode){
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);
    gpio = (uint32_t *)mmap(NULL, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, memfd, GPIO_BASE);
    if (gpio == MAP_FAILED)
        printf("mmap gpio failed: %s\n", strerror(errno));    
    close(memfd);

    pwm = (uint32_t *)mmap(NULL, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, memfd, PWM0_ADR);
    if (pwm == MAP_FAILED)
        printf("mmap pwm failed: %s\n", strerror(errno));    


    if(mode == INPUT){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) ;
    }
    else if (mode == OUTPUT)
      *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (1 << GPIO_SHIFT[pin]) ;
    else{}
}

void digitalWrite(int pin, int value){   
    if (value == LOW){ *(gpio + GPIO_GPCLR [pin]) = 1 << pin;}
    else{ *(gpio + GPIO_GPSET [pin]) = 1 << pin;}
}
bool digitalRead(int pin){
    if((*(gpio + GPIO_GPLEV [pin]) & (1 << (pin & 31))) != 0){
        return HIGH;
    }
    else{
        return LOW;
    }
}


