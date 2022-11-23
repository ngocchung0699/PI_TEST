
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
static volatile uint32_t *aux ;
static volatile uint32_t *uart0 ;
static volatile uint32_t *uart2 ;
static volatile uint32_t *uart3 ;
static volatile uint32_t *uart4 ;
static volatile uint32_t *uart5 ;
static volatile uint32_t *i2c0 ;
static volatile uint32_t *i2c1 ;
static volatile uint32_t *i2c3 ;
static volatile uint32_t *i2c4 ;
static volatile uint32_t *i2c5 ;
static volatile uint32_t *i2c6 ;

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
    uart0 = base + UART0_REG;
    uart2 = base + UART2_REG;
    uart3 = base + UART3_REG;
    uart4 = base + UART4_REG;
    uart5 = base + UART5_REG;
    aux = base + AUX_REG;
    i2c0 = base + BSC0_REG;
    i2c1 = base + BSC1_REG;
    i2c3 = base + BSC3_REG;
    i2c4 = base + BSC4_REG;
    i2c5 = base + BSC5_REG;
    i2c6 = base + BSC6_REG;

    //pthread_t threadId ;
    //pthread_create (&threadId, NULL, thr, NULL);
    close(memfd);
}

void lib_close(){
    munmap(&base, BLOCK_SIZE);
    gpio = MAP_FAILED;
    timer = MAP_FAILED;
    aux = MAP_FAILED;
    uart0 = MAP_FAILED;
    uart2 = MAP_FAILED;
    uart3 = MAP_FAILED;
    uart4 = MAP_FAILED;
    uart5 = MAP_FAILED;
    i2c0 = MAP_FAILED;
    i2c1 = MAP_FAILED;
    i2c3 = MAP_FAILED;
    i2c4 = MAP_FAILED;
    i2c5 = MAP_FAILED;
    i2c6 = MAP_FAILED;
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
void uart_setup(unsigned long baud)
{
    // Disable pull up/down for pin 14,15 & delay for 150 cycles.
    pinMode(4, ALT4);
    pinMode(5, ALT4);
    //Disable UART
    *(uart3 + UART_CR) = 0;

    // Clear uart Flag Register
    *(uart4 + UART_FR) = 0;

    // Clear pending interrupts.
    *(uart4 + UART_ICR) = 0x7FF;

    uint32_t value = 16*baud;
    uint32_t value_i = (3000000 / value);
    uint32_t value_f = (3000000000/value - value_i* 1000)*64/1000 + 0.5;

    // Divider = 3000000 / (16 * baud)
    *(uart4 + UART_IBRD) = (int) value_i;

    // Fractional part register
    *(uart4 + UART_FBRD) = (int) value_f;

    //Clear UART FIFO by writing 0 in FEN bit of LCRH register
    *(uart4 + UART_LCRH) = (0 << 4);

    // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity)
	*(uart4 + UART_LCRH) = (1 << 4) | (1 << 5) | (1 << 6);

    // Mask all interrupts.
	*(uart4 + UART_IMSC) = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                               (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10);

    // Enable uart4, receive & transfer part of UART.                  
	*(uart4 + UART_CR) = (1 << 0) | (1 << 8) | (1 << 9);
}

void uart_send_char(unsigned char data)
{
    while (*(uart4 + UART_FR) & (1 << 5)); // Wait until there is room for new data in Transmission fifo
    *(uart4 + UART_DR) = (unsigned char) data; // Write data in transmission fifo
    delay_us(150);
}

char uart_receive()
{
    while (!(*(uart4 + UART_FR) & (1 << 4))); // Wait until data arrives in Rx fifo. Bit 4 is set when RX fifo is empty
    return (unsigned char) *(uart4 + UART_DR);
}

void uart_send_string(const char *data)
{
    for (size_t i = 0; data[i] != '\0'; i++)
    {
        uart_send_char( (char) data[i] );
    }
    delay_us(150);
}


void aux_uart_setup(long baud)
{
        /* set gpio 14 and 15 to UART1 (mini-uart) */
    pinMode(14, ALT5);
    pinMode(15, ALT5);

    /* enable mini-uart */
    *(aux + AUX_ENABLES) = 1 << 0;
    /* disable TX and RX and auto flow control */
    *(aux + AUX_MU_CNTL_REG) = 0;
    /* enable receive interrupts, check bcm errata */
    *(aux + AUX_MU_IER_REG) = 0<<0 | 0<<1;
    /* set 8-bit mode */
    *(aux + AUX_MU_LCR_REG) = 0<<0 ;
    /* set RTS to always high */
    *(aux + AUX_MU_MCR_REG) = 0<<0;
    /* 115200 @ 500 MHz */
    *(aux + AUX_MU_BAUD_REG) = (500000000/(8*(baud + 1)));
    /* enable TX and RX */
    *(aux + AUX_MU_CNTL_REG) = 1<<0 | 1<<1;
}

void aux_uart_send_char(char data)
{
    /* keep looping if the 5th bit is 0 */
    while (!(*(aux + AUX_MU_LSR_REG) & 0x20));

    *(aux + AUX_MU_IO_REG) = data;
}

char aux_uart_receive()
{
    while (!(*(aux + AUX_MU_LSR_REG) & 0x01));

    return *(aux + AUX_MU_IO_REG) & 0xFF;
}
void aux_uart_send_string(const char *data)
{
    for(int i=0; data[i] != '\0';i++)
    {
        aux_uart_send_char(data[i]);
    }
}


//-----------I2C-------------//

void i2c_setup()
{
    pinMode(2, ALT0);   // PIN 2 IS SDA
    pinMode(3, ALT0);   // PIN 2 IS SCL
}











