#ifndef	__GPIO_H__
#define	__GPIO_H__

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#define INPUT 0
#define OUTPUT 1

#define LOW 0
#define HIGH 1

#define GPIO_BASE                   0xfe200000

#define GPFSEL0                     (0x00/4)
#define GPFSEL1                     (0x04/4)
#define GPFSEL2                     (0x08/4)
#define GPFSEL3                     (0x0c/4)
#define GPFSEL4                     (0x10/4)
#define GPFSEL5                     (0x14/4)
#define GPSET0                      (0x1c/4)
#define GPSET1                      (0x20/4)
#define GPCLR0                      (0x28/4)
#define GPCLR1                      (0x2c/4)
#define GPLEV0                      (0x34/4)
#define GPLEV1                      (0x38/4)
#define GPEDS0                      (0x40/4)
#define GPEDS1                      (0x44/4)
#define GPREN0                      (0x4c/4)
#define GPREN1                      (0x50/4)
#define GPPEN0                      (0x58/4)
#define GPPEN1                      (0x5c/4)
#define GPHEN0                      (0x64/4)
#define GPHEN1                      (0x68/4)
#define GPLEN0                      (0x70/4)
#define GPLEN1                      (0x74/4)
#define GPAREN0                     (0x7c/4)
#define GPAREN1                     (0x80/4)
#define GPAPEN0                     (0x88/4)
#define GPAPEN1                     (0x8c/4)
#define GPIO_PUP_PDN_CNTRL_REG0     (0xe4/4)
#define GPIO_PUP_PDN_CNTRL_REG1     (0xe8/4)
#define GPIO_PUP_PDN_CNTRL_REG2     (0xec/4)
#define GPIO_PUP_PDN_CNTRL_REG3     (0xf0/4)

#define	FSEL_INPUT		0b000
#define	FSEL_OUTPUT		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)


#define PWM0_ADR 0x7e20c000
#define PWM1_ADR 0x7e20c800


void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
bool digitalRead(int pin);


#endif