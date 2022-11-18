
#include "stdio.h"
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib.h"

static volatile uint32_t *base ;
static volatile uint32_t *gpio ;
static volatile uint32_t *clk ;
static volatile uint32_t *pwm ;
static volatile uint32_t *timer ;
static volatile uint32_t *uart ;


static volatile uint32_t reg_pwm;

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

    // pthread_t threadId ;
    // pthread_create (&threadId, NULL, thr, NULL) ;
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


//---------PWM-HARDWARE----------//
////only used for GPIO 18

/* Read with memory barriers from peripheral
 *
 */
uint32_t peri_read(volatile uint32_t* paddr)
{
    uint32_t ret;
    __sync_synchronize();
    ret = *paddr;
    __sync_synchronize();
    return ret;
}

void peri_write(volatile uint32_t* paddr, uint32_t value)
{
    __sync_synchronize();
    *paddr = value;
    __sync_synchronize();
}

void pwm_hw_set_clock(float divisor)
{
    if (   clk == MAP_FAILED || pwm == MAP_FAILED)
      return; /* bcm2835_init() failed or not root */
    uint32_t div;

    div = (int) divisor*2.8125;  // divisor*540/192
    /* From Gerts code */
    div &= 0xfff;
    /* Stop PWM clock */
    peri_write(clk + CLK_CNTL, CLK_PASSWRD | 0x01);
    // delay_us(110); /* Prevents clock going slow */
    /* Wait for the clock to be not busy */
    while ((peri_read(clk + CLK_CNTL) & 0x80) != 0)
	// delay_us(1); 
    /* set the clock divider and enable PWM clock */
    peri_write(clk + CLK_DIV, CLK_PASSWRD | (div << 12));
    peri_write(clk + CLK_CNTL, CLK_PASSWRD | 0x11); /* Source=osc and enable */
}

void pwm_hw_set_mode(uint8_t channel, uint8_t enabled)
{
  if (   clk == MAP_FAILED || pwm == MAP_FAILED)
    return; /* bcm2835_init() failed or not root */

  uint32_t control = peri_read(pwm + PWM_CTL);

  if (channel == 0)
    {
      if (enabled)
	control |= PWM0_ENABLE;
      else
	control &= ~PWM0_ENABLE;
    }
  else if (channel == 1)
    {
      if (enabled)
	control |= PWM1_ENABLE;
      else
	control &= ~PWM1_ENABLE;
    }
  peri_write(pwm + PWM_CTL, control);
}

void pwm_hw_set_range(uint8_t channel, uint32_t range)
{
  if (   clk == MAP_FAILED || pwm == MAP_FAILED)
    return;

  if (channel == 0)
        peri_write(pwm + PWM0_RANGE, range);
  else if (channel == 1)
        peri_write(pwm + PWM1_RANGE, range);
}

void pwm_hw_setup(bool pwm_mode, uint32_t divisor, uint32_t range)
{
    pinMode(18, ALT5);
    pwm_hw_set_clock(divisor);
    pwm_hw_set_mode(0, pwm_mode);
    pwm_hw_set_range(0, range);
}

void pwm_hw_write(uint32_t data)
{
    if (   clk == MAP_FAILED || pwm == MAP_FAILED)
        return;
    int channel =0;
    if (channel == 0)
        peri_write(pwm + PWM0_DATA, data);
    else if (channel == 1)
        peri_write(pwm + PWM1_DATA, data);
}


//---------PWM-SOFTWARE---------//

void *thr()
{
    while (1)
    {
        uint8_t pin;
        uint8_t duty;
        uint16_t freq;
        pin = reg_pwm>>24;
        duty = reg_pwm>>16;
        freq = reg_pwm;
        if(pin != 0)
        {
            digitalWrite(pin, 1);
            delay_us(duty*freq*0.01);
            digitalWrite(pin, 0);
            delay_us(freq-duty*freq*0.01);
        }
    }
}

void pwm_write(uint8_t pin, uint8_t duty, uint16_t freq)
{
    reg_pwm = 0;
    
    reg_pwm |= pin;
    reg_pwm = reg_pwm<<8;

    reg_pwm |= duty;
    reg_pwm = reg_pwm<<16;

    reg_pwm |= freq;

}

void pwm_on(uint8_t pin)
{
    *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_OUTPUT << GPIO_SHIFT[pin]) ;
}

void pwm_off(uint8_t pin)
{
    *(gpio + GPIO_GPFSEL[pin]) = (*(gpio + GPIO_GPFSEL[pin]) & ~(7 << GPIO_SHIFT[pin])) | (FSEL_INPUT << GPIO_SHIFT[pin]) ;
}


//----------UART----------//

// uart0
void uart_hw_setup(unsigned long baud)
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

    uint32_t value = 16*baud;
    uint32_t value_i = (3000000 / value);
    uint32_t value_f = (3000000000/value - value_i* 1000)*64/1000 + 0.5;

    // Divider = 3000000 / (16 * baud)
    *(uart + UART_IBRD) = (int) value_i;

    // Fractional part register
    *(uart + UART_FBRD) = (int) value_f;

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
    delay_us(150);
}

char uart_getc()
{
    while (*(uart + UART_FR) & (1 << 4)); // Wait until data arrives in Rx fifo. Bit 4 is set when RX fifo is empty
    char data = *(uart + UART_DR);
    return (unsigned char) data;
}

void uart_puts(const char *str)
{
    for (size_t i = 0; str[i] != '\0'; i++)
    {
        uart_putc( (char) str[i] );
    }
    delay_us(150);
}


//-----------UART-SOFTWARE-----------//

uart_struct uart_set;

void uart_setup(uint8_t pin_tx, uint8_t pin_rx,unsigned long baud)
{
    
    pinMode(pin_tx, OUTPUT);
    pinMode(pin_rx, INPUT);

    int time_bit = 0;
    int time_error = 0;

    if      (baud == 300)       { time_bit = B300;      time_error = E300; }
    else if (baud == 600 )      { time_bit = B600;      time_error = E600;}
    else if (baud == 1200 )     { time_bit = B1200;     time_error = E1200;}
    else if (baud == 2400 )     { time_bit = B1200;     time_error = E1200;}
    else if (baud == 4800 )     { time_bit = B4800;     time_error = E4800;}
    else if (baud == 9600 )     { time_bit = B9600;     time_error = E9600;}
    else if (baud == 14400 )    { time_bit = B14400;    time_error = E14400;}
    else if (baud == 19200 )    { time_bit = B19200;    time_error = E19200;}
    else if (baud == 28800 )    { time_bit = B28800;    time_error = E28800;}
    else if (baud == 38400 )    { time_bit = B38400;    time_error = E38400;}
    else if (baud == 56000 )    { time_bit = B56000;    time_error = E56000;}
    else if (baud == 57600 )    { time_bit = B57600;    time_error = E57600;}
    else if (baud == 115200 )   { time_bit = B115200;   time_error = E115200;}
    else if (baud == 128000 )   { time_bit = B128000;   time_error = E128000;}
    else if (baud == 256000 )   { time_bit = B256000;   time_error = E256000;}
    else if (baud == 300000 )   { time_bit = B300000;   time_error = E300000;}
    else if (baud == 500000 )   { time_bit = B500000;   time_error = E500000;}
    else if (baud == 1000000 )  { time_bit = B1000000;  time_error = E1000000;}

    uart_set.pin_tx = pin_tx;
    uart_set.pin_rx = pin_rx;
    uart_set.t_bit = time_bit;
    uart_set.data = 0;
    uart_set.t_error = time_error;

}

void uart_send_char(char data)
{
    uart_set.data = data;
    
    digitalWrite(uart_set.pin_tx, 1);
    digitalWrite(uart_set.pin_tx, 0);
    delay_us(uart_set.t_bit - uart_set.t_error);
    for (int i = 0; i < 8; i++) 
    {
        
        if(data&(1<<i))     { digitalWrite(uart_set.pin_tx, 1); }
            
        else                { digitalWrite(uart_set.pin_tx, 0); }
        delay_us(uart_set.t_bit);
        
    }
    
    digitalWrite(uart_set.pin_tx, 1);
    delay_us(uart_set.t_bit * 10);
}

void uart_send_string(const char *data)
{
    for (size_t i = 0; data[i] != '\0'; i++)
    {
        uart_send( (char) data[i] );
    }
}

