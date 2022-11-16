
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

static volatile uint32_t *base ;
static volatile uint32_t *gpio ;
static volatile uint32_t *clk ;
static volatile uint32_t *pwm ;
static volatile uint32_t *timer ;
static volatile uint32_t *uart ;


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

static uint8_t GPIO_PUP_PDN_CNTRL [] = 
{
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30
} ;

// GPIO GPFSEL PULL UP/PULL DOWN:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPIO_PUP_PDN_CNTRL_REG 0-3)

static uint8_t GPIO_GPFSEL_PUD [] =
{
    GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0,
    GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0,
    GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0,
    GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0, GPIO_PUP_PDN_CNTRL_REG0,

    GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1,
    GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1,
    GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1,
    GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1, GPIO_PUP_PDN_CNTRL_REG1,

    GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2,
    GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2,
    GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2,
    GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2, GPIO_PUP_PDN_CNTRL_REG2,

    GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3,
    GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3,
    GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3,
    GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3, GPIO_PUP_PDN_CNTRL_REG3
} ;


void lib_init(){
    int memfd;
    
    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) 
	{
	  printf("base_init: Unable to open /dev/mem: %s\n", strerror(errno));
      exit(1);
	}
        
    base = (uint32_t *)mmap(NULL, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, memfd, BASE_ADR);
    if (base == MAP_FAILED)
        printf("mmap gpio failed: %s\n", strerror(errno));  
        
    gpio = base + GPIO_REG;
    clk = base + CLK_REG;
    pwm = base + PWM0_REG;
    timer = base + TIMER_REG;
    uart = base + UART_REG;
    close(memfd);
}

void lib_close(){
    munmap(&base, BLOCK_SIZE);
    gpio = MAP_FAILED;
    timer = MAP_FAILED;
    uart = MAP_FAILED;
}


void pinMode(int pin, int mode){
    nopull_mode(pin);
    if(mode == INPUT){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_INPUT << GPIO_SHIFT[pin]) ;
    }
    else if(mode == INPUT_PULLUP){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_INPUT << GPIO_SHIFT[pin]) ;
        pullup_mode(pin);
    }
    else if(mode == INPUT_PULLDOWN){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_INPUT << GPIO_SHIFT[pin]) ;
        pulldown_mode(pin);
    }
    else if(mode == OUTPUT){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_OUTPUT << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT0){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT0 << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT1){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT1 << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT2){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT2 << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT3){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT3 << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT4){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT4 << GPIO_SHIFT[pin]) ;
    }
    else if(mode == ALT5){
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT5 << GPIO_SHIFT[pin]) ;
    }
}

void nopull_mode(int pin)
{
    *(gpio + GPIO_GPFSEL_PUD[pin]) = (*(gpio + GPIO_GPFSEL_PUD[pin]) & ~(3 << GPIO_PUP_PDN_CNTRL[pin])) | (NO_PULL << GPIO_PUP_PDN_CNTRL[pin]) ;
}

void pullup_mode(int pin)
{
    *(gpio + GPIO_GPFSEL_PUD[pin]) = (*(gpio + GPIO_GPFSEL_PUD[pin]) & ~(3 << GPIO_PUP_PDN_CNTRL[pin])) | (PULLUP << GPIO_PUP_PDN_CNTRL[pin]) ;
}

void pulldown_mode(int pin)
{
    *(gpio + GPIO_GPFSEL_PUD[pin]) = (*(gpio + GPIO_GPFSEL_PUD[pin]) & ~(3 << GPIO_PUP_PDN_CNTRL[pin])) | (PULLDOWN << GPIO_PUP_PDN_CNTRL[pin]) ;
}

void digitalWrite(int pin, int value){   
    if (value == LOW)   { *(gpio + GPIO_GPCLR [pin]) = 1 << pin;}
    if (value == HIGH)  { *(gpio + GPIO_GPSET [pin]) = 1 << pin;}
}
bool digitalRead(int pin){
    if((*(gpio + GPIO_GPLEV [pin]) & (1 << (pin & 31))) != 0){
        return HIGH;
    }
    else{
        return LOW;
    }
}
/* miliseconds */
void delay_ms(uint64_t milis)
{
    delay_us(milis * 1000);
}


/* microseconds */
void delay_us(uint64_t micros)
{
    uint64_t        start;

    start =  sys_timer_read();

    sys_timer_delay(start, micros);
}

/* Read the System Timer Counter (64-bits) */
uint64_t sys_timer_read(void)
{
    volatile uint32_t* paddr;
    uint32_t hi, lo;
    uint64_t st;

    if (timer==MAP_FAILED)
	return 0;

    paddr = timer + TIMER_CHI;
    hi = peri_read(paddr);

    paddr = timer + TIMER_CLO;
    lo = peri_read(paddr);
    
    paddr = timer + TIMER_CHI;
    st = peri_read(paddr);
    
    /* Test for overflow */
    if (st == hi)
    {
        st <<= 32;
        st += lo;
    }
    else
    {
        st <<= 32;
        paddr = timer + TIMER_CLO;
        st += peri_read(paddr);
    }
    return st;
}

/* Delays for the specified number of microseconds with offset */
void sys_timer_delay(uint64_t offset_micros, uint64_t micros)
{
    uint64_t compare = offset_micros + micros;

    while(sys_timer_read() < compare);
}

uint32_t peri_read(volatile uint32_t* paddr)
{
    uint32_t ret;
    ret = *paddr;
    return ret;
}

//----------PWM-----------//

void pwm_set_clock(int divisor)
{
    if ( clk == MAP_FAILED || pwm == MAP_FAILED)
        return;
    divisor &= 0xfff;

    *(clk + CLK_CNTL) = CLK_PASSWRD | 0x01;           // Enable clock oscillator

    while ((*(clk + CLK_CNTL) & 0x80) != 0);            // Wait for reset

    *(clk + CLK_DIV) = CLK_PASSWRD | (divisor << 12);   // Set divisor

    *(clk + CLK_CNTL) = CLK_PASSWRD | 0x11;           // Enable the clock generator
}

void pwm_set_mode(bool channel, bool pwm_mode)
{
    if (clk == MAP_FAILED || pwm == MAP_FAILED)
        return;

    uint32_t control = *(pwm + PWM_CTL);

    if(channel==0){
        control = 1 << 7 | pwm_mode << 0;
    }
    else{
        control = 1 << 15 | pwm_mode << 8;
    }
    *(clk + CLK_CNTL) = control;
}

void pwm_set_range(bool channel, uint32_t range)
{
    if (clk == MAP_FAILED || pwm == MAP_FAILED)
        return;

    if(channel == 0){
        *(pwm + PWM_RNG1) = range;
    }
    else{
        *(pwm + PWM_RNG2) = range;
    }
}

void pwm_setup(int PWM_pin, bool pwm_mode, uint32_t divisor, uint32_t range)
{
    int pin;
    bool channel;
    if(PWM_pin == PWM0){ 
        pin = PWM_pin;
        channel = 0;
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT0 << GPIO_SHIFT[pin]) ;
    }
    else if(PWM_pin == PWM1){ 
        pin = PWM_pin;
        channel = 1;
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT0 << GPIO_SHIFT[pin]) ;
    }
    else if(PWM_pin == PWM2){ 
        pin = PWM_pin;
        channel = 0;
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT5 << GPIO_SHIFT[pin]) ;
    }
    else if(PWM_pin == PWM3){ 
        pin = PWM_pin;
        channel = 1;
        *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_ALT5 << GPIO_SHIFT[pin]) ;
    }

    pwm_set_mode(channel, pwm_mode);

    pwm_set_clock(divisor);

    pwm_set_range(channel, range);
}

void pwm_write(int PWM_pin, uint32_t data)
{
    if (clk == MAP_FAILED || pwm == MAP_FAILED)
        return;

    if(PWM_pin == PWM0){ 
        *(pwm + PWM_DAT1) = data;
    }
    else if(PWM_pin == PWM1){    
        *(pwm + PWM_DAT2) = data;
    }
    else if(PWM_pin == PWM2){ 
        *(pwm + PWM_DAT1) = data;
    }
    else if(PWM_pin == PWM3){ 
        *(pwm + PWM_DAT2) = data;
    }
}

//----------UART----------//

// uart0
void uart_setup(unsigned long baud)
{
    // Disable pull up/down for pin 14,15 & delay for 150 cycles.
    nopull_mode(14);
    nopull_mode(15);

    int pin_tx = 14;
    int pin_rx = 15;
    *(gpio + GPIO_GPFSEL[pin_tx]) = (*(gpio + GPIO_GPFSEL[pin_tx]) & ~(7 << GPIO_SHIFT[pin_tx])) | (FSEL_ALT0 << GPIO_SHIFT[pin_tx]) ;
    *(gpio + GPIO_GPFSEL[pin_rx]) = (*(gpio + GPIO_GPFSEL[pin_rx]) & ~(7 << GPIO_SHIFT[pin_rx])) | (FSEL_ALT0 << GPIO_SHIFT[pin_rx]) ;

    //Disable UART
    *(uart + UART_CR) = 0;

    // Clear uart Flag Register
    *(uart + UART_FR) = 0;

    // Clear pending interrupts.
    *(uart + UART_ICR) = 0x7FF;

    // Divider = 3000000 / (16 * baud)
    *(uart + UART_IBRD) = (int) 3000000 / (16 * baud);

    // Fractional part register
    *(uart + UART_FBRD) = (int)(((3000000 / (16 * baud)) - (int)(3000000 / (16 * baud)))*64 + 0.5);

    //Clear UART FIFO by writing 0 in FEN bit of LCRH register
    *(uart + UART_LCRH) = (0 << 4);

    // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity)
	*(uart + UART_LCRH) = (1 << 4) | (1 << 5) | (1 << 6);

    // Mask all interrupts.
	*(uart + UART_IMSC) = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                               (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable UART0, receive & transfer part of UART.                  
	*(uart + UART_CR) = (1 << 0) | (1 << 8) | (1 << 9);
}


void uart_putc(unsigned char c)
{
    while (*(uart + UART_FR) & (1 << 5)); // Wait until there is room for new data in Transmission fifo
    *(uart + UART_DR) = (unsigned char) c; // Write data in transmission fifo
}

uint8_t uart_getc()
{
    while (*(uart + UART_FR) & (1 << 4)); // Wait until data arrives in Rx fifo. Bit 4 is set when RX fifo is empty
    return (char)*(uart + UART_DR);
}

void uart_puts(const char *str)
{
    for (size_t i = 0; str[i] != '\0'; i++)
        uart_putc( (char) str[i] );
}

