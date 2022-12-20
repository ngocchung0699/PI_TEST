#ifndef	__LIB_H__
#define	__LIB_H__

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

#define	    BLOCK_SIZE		            0x01000000

#define     BASE_ADR                    0xfe000000

#define     __I     volatile const       /*!< Defines 'read only' permissions */
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

void lib_init();
void lib_close();

//--------GPIO----------//
typedef enum
{
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    ALT0,
    ALT1,
    ALT2,
    ALT3,
    ALT4,
    ALT5
} GPIO_MODE;

typedef enum
{
    NO_PULL         = 0x00,  
    PULLUP          = 0x01,   
    PULLDOWN        = 0x02   
} GPIO_PU_PD_CONTROL;

typedef enum
{
    LOW,
    HIGH
} GPIO_STATUS;

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

#define	FSEL_INPUT		            0b000
#define	FSEL_OUTPUT		            0b001
#define	FSEL_ALT0		            0b100
#define	FSEL_ALT1		            0b101
#define	FSEL_ALT2		            0b110
#define	FSEL_ALT3		            0b111
#define	FSEL_ALT4		            0b011
#define	FSEL_ALT5		            0b010

void pinMode(int pin, int mode);
void nopull_mode(int pin);
void pullup_mode(int pin);
void pulldown_mode(int pin);
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
uint32_t peri_read(volatile uint32_t* paddr);

//---------CLOCK---------//

#define CLK_REG                     (0x101000/4)
#define CLK_CNTL                    40
#define CLK_DIV                     41
#define CLK_PASSWRD                 (0x5A << 24)  //Password to enable setting PWM clock 

//---------PWM-HARDWARE----------//

//only used for GPIO 18

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

#define PWM0                        12             // GPIO 12 (PWM0_0, ALT0)
#define PWM1                        13             // GPIO 13 (PWM0_1, ALT0)
#define PWM2                        18             // GPIO 18 (PWM0_0, ALT5)
#define PWM3                        19             // GPIO 19 (PWM0_1, ALT5)

#define PWM0_ENABLE 0x0081
#define PWM1_ENABLE 0x8100
#define PWM0_RANGE  4
#define PWM1_RANGE  8
#define PWM0_DATA  5
#define PWM1_DATA  9

typedef enum
{   
    PWM_DISABLE,
    PWM_ENABLE
}PWM_MODE;

typedef enum
{
    PWM_CH0,
    PWM_CH1
}PWM_CHANEL;

void pwm_hw_set_clock(float divisor);
void pwm_hw_set_mode(uint8_t channel, uint8_t enabled);
void pwm_hw_set_range(uint8_t channel, uint32_t range);
void pwm_hw_setup(bool pwm_mode,uint32_t divisor, uint32_t range);
void pwm_hw_write(uint32_t data);

//---------PWM-SOFTWARE---------//

void *thr();
void pwm_write(uint8_t pin, uint8_t duty, uint16_t freq);
void pwm_on(uint8_t pin);
void pwm_off(uint8_t pin);


//---------UART---------//

#define UART0_REG                    (0x201000/4)
#define UART2_REG                    (0x201400/4)
#define UART3_REG                    (0x201600/4)
#define UART4_REG                    (0x201800/4)
#define UART5_REG                    (0x201a00/4)

#define UART_DR                     (0x00/4)
#define UART_RSRECR                 (0x04/4)
#define UART_FR                     (0x18/4)
#define UART_ILPR                   (0x20/4)
#define UART_IBRD                   (0x24/4)
#define UART_FBRD                   (0x28/4)
#define UART_LCRH                   (0x2C/4)
#define UART_CR                     (0x30/4)
#define UART_IFLS                   (0x34/4)
#define UART_IMSC                   (0x38/4)
#define UART_RIS                    (0x3C/4)
#define UART_MIS                    (0x40/4)
#define UART_ICR                    (0x44/4)
#define UART_DMACR                  (0x48/4)

typedef struct
{
    __IO    uint32_t    DR;         /*!< UART Data register,                        Address offset: 0x00 */
    __IO    uint32_t    RSRECR;     /*!<                                            Address offset: 0x04 */
    __IO    uint32_t    temp1;      /*!<                                            Address offset: 0x08 */
    __IO    uint32_t    temp2;      /*!<                                            Address offset: 0x0C */
    __IO    uint32_t    temp3;      /*!<                                            Address offset: 0x10 */
    __IO    uint32_t    temp4;      /*!<                                            Address offset: 0x14 */
    __IO    uint32_t    FR;         /*!< UART Flag register,                        Address offset: 0x18 */
    __IO    uint32_t    temp5;      /*!<                                            Address offset: 0x1C */
    __IO    uint32_t    ILPR;       /*!< UART not in use,                           Address offset: 0x20 */
    __IO    uint32_t    IBRD;       /*!< UART Integer Baud rate divisor,            Address offset: 0x24 */
    __IO    uint32_t    FBRD;       /*!< UART Fractional Baud rate divisor,         Address offset: 0x28 */
    __IO    uint32_t    LCRH;       /*!< UART Line Control register,                Address offset: 0x2C */
    __IO    uint32_t    CR;         /*!< UART Control register,                     Address offset: 0x30 */
    __IO    uint32_t    IFLS;       /*!< UART Interrupt FIFO Level Select Register, Address offset: 0x34 */
    __IO    uint32_t    IMSC;       /*!< UART Interrupt Mask Set Clear Register,    Address offset: 0x38 */
    __IO    uint32_t    RIS;        /*!< UART Raw Interrupt Status Register,        Address offset: 0x3C */
    __IO    uint32_t    MIS;        /*!< UART Masked Interrupt Status Register,     Address offset: 0x40 */
    __IO    uint32_t    ICR;        /*!< UART Interrupt Clear Register,             Address offset: 0x44 */
    __IO    uint32_t    DMACR;      /*!< UART DMA Control Register,                 Address offset: 0x48 */
} UART_TypeDef;


void uart_init(unsigned long baud);
void uart_deinit();
void uart_send_char(unsigned char data);
char uart_receive();
void uart_send_string(const char *data);

//-----------I2C-------------//

#define BSC0_REG            (0x205000/4)     // use by boot
#define BSC1_REG            (0x804000/4)
// BSC2  -  use by the HDMI
#define BSC3_REG            (0x205600/4)
#define BSC4_REG            (0x205800/4)
#define BSC5_REG            (0x205a00/4)
#define BSC6_REG            (0x205c00/4)
// BSC7  -  use by the HDMI
#define BSC_C           (0x00/4)
#define BSC_S           (0x04/4)
#define BSC_DLEN        (0x08/4)
#define BSC_A           (0x0C/4)
#define BSC_FIFO        (0x10/4)
#define BSC_DIV         (0x14/4)
#define BSC_DEL         (0x18/4)
#define BSC_CLKT        (0x1C/4)

#define CORE_CLK_HZ     25000000

void i2c_init();
void i2c_deinit();
void i2c_start();
void i2c_end();
void i2c_set_slave_address(uint8_t addr);
void i2c_set_clock_divider(uint16_t div);
void i2c_set_baudrate(uint32_t baudrate);
uint8_t i2c_write(const uint8_t *data, int len);
uint8_t i2c_read(uint8_t *data, int len);

void i2c_send(uint8_t addr, const uint8_t *data, int len);
void i2c_receive(uint8_t addr, uint8_t *data, int len);


//-----------SPI------------//

#define SPI0_REG  (0x204000/4)
//   SPI1 - use for AUX
//   SPI2 - use for AUX
#define SPI3_REG  (0x204600/4)
#define SPI4_REG  (0x204800/4)
#define SPI5_REG  (0x204a00/4)
#define SPI6_REG  (0x204c00/4)

#define SPI_CS (0x00/4)
#define SPI_FIFO (0x04/4)
#define SPI_CLK (0x08/4)
#define SPI_DLEN (0x0c/4)
#define SPI_LTOH (0x10/4)
#define SPI_OC (0x14/4)

void spi_init();
void spi_deinit();
void spi_send_receive(uint8_t chip_select, uint8_t *sbuffer, uint8_t *rbuffer, uint8_t size);
void spi_send(uint8_t chip_select, uint8_t *data, uint32_t size);
void spi_receive(uint8_t chip_select, uint8_t *data, uint32_t size); 


//------------FUNCTION-----------//
double map(double x, double in_min, double in_max, double out_min, double out_max);

#endif




