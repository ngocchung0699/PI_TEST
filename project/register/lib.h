#ifndef	__LIB_H__
#define	__LIB_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#define	BLOCK_SIZE		            0x01000000

#define BASE_ADR                    0xfe000000

static volatile uint32_t* base;


void lib_init();
void lib_close();

//--------GPIO----------//



#define GPIO_REG                    (0x200000/4)

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
#define GPFEN0                      (0x58/4)
#define GPFEN1                      (0x5c/4)
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

#define	FSEL_INPUT		            0b000
#define	FSEL_OUTPUT		            0b001
#define	FSEL_ALT0		            0b100
#define	FSEL_ALT1		            0b101
#define	FSEL_ALT2		            0b110
#define	FSEL_ALT3		            0b111
#define	FSEL_ALT4		            0b011
#define	FSEL_ALT5		            0b010

typedef enum
{
    INPUT           = 0x00,
    OUTPUT          = 0x01
} MODE;

typedef enum
{
    NO_PULL         = 0x00,   /*!< Off ? disable pull-up/down 0b00 */
    INPUT_PULLUP    = 0x01,   /*!< Enable Pull Down control 0b01 */
    INPUT_PULLDOWN  = 0x02    /*!< Enable Pull Up control 0b10  */
} PU_PD_CONTROL;

typedef enum
{
    LOW,
    HIGH,
    RISING,
    FALLING
} MODE_STATUS;


void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
bool digitalRead(int pin);

//--------TIMER--------//

#define TIMER_REG                   (0x3000/4)

#define TIMER_CS                    (0x0000/4)
#define TIMER_CLO                   (0x0004/4)
#define TIMER_CHI                   (0x0008/4)

#define TIMER_C0                    (0x000c/4)
#define TIMER_C1                    (0x0010/4)
#define TIMER_C2                    (0x0014/4)
#define TIMER_C3                    (0x0018/4)

#define TIMER_M0                    0
#define TIMER_M1                    1
#define TIMER_M2                    2
#define TIMER_M3                    3

void delay_ms(uint64_t milis);
void delay_us(uint64_t micros);
uint64_t sys_timer_read(void);
void sys_timer_delay(uint64_t offset_micros, uint64_t micros);
unsigned long millis(void);
unsigned long micros(void);

uint32_t peri_read(volatile uint32_t* paddr);

//---------CLOCK---------//

#define CLK_REG                     (0x101000/4)
#define CLK_CNTL                    40
#define CLK_DIV                     41
#define CLK_PASSWRD                 (0x5A << 24)  //Password to enable setting PWM clock 

//---------PWM----------//

#define PWM0_REG                    (0x20c000/4)
#define PWM1_REG                    (0x20c800/4)   // do not use
#define PWM_CTL                     (0x00/4)
#define PWM_STA                     (0x04/4)
#define PWM_DMAC                    (0x08/4)
#define PWM_RNG1                    (0x10/4)
#define PWM_DAT1                    (0x14/4)
#define PWM_FIF1                    (0x18/4)
#define PWM_RNG2                    (0x20/4)
#define PWM_DAT2                    (0x24/4)

#define PWM0                        12             // GPIO 12 (PWM0_0)
#define PWM1                        13             // GPIO 13 (PWM0_1)
#define PWM2                        18             // GPIO 18 (PWM0_0)
#define PWM3                        19             // GPIO 19 (PWM0_1)

typedef enum
{
    PWM_ENABLE,
    PWM_DISABLE
}PWM_MODE;

typedef enum
{
    PWM_CH0,
    PWM_CH1
}PWM_CHANEL;

void pwm_set_clock(uint32_t divisor);
void pwm_set_mode(bool channel, bool pwm_mode);
void pwm_set_range(bool channel, uint32_t range);

void pwm_setup(int PWM_pin, bool pwm_mode,uint32_t divisor, uint32_t range);
void pwm_set();
void pwm_write(int PWM_pin, uint32_t data);

//---------IRQ---------//
/*
void iqr_setup(int pin, int mode, void (*function)(void));
void iqr_close(int pin, int mode);
*/

void gpio_rising_enable(int pin);
void gpio_rising_disable(int pin);

void gpio_falling_enable(int pin);
void gpio_falling_disable(int pin);

void gpio_high_enable(int pin);
void gpio_high_disable(int pin);

void gpio_low_enable(int pin);
void gpio_low_disable(int pin);


bool gpio_eds_flag(int pin);

void gpio_eds_clear_flag(int pin);


#endif

