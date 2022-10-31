#ifndef	__TIMER_H__
#define	__TIMER_H__

#include <stdint.h>

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define TIMER_BASE                   0x7e003000

#define TIMER_CS        (0x00/4)
#define TIMER_CLO       (0x04/4)
#define TIMER_CHI       (0x08/4)

#define TIMER_C0       (0x0c/4)
#define TIMER_C1       (0x10/4)
#define TIMER_C2       (0x14/4)
#define TIMER_C3       (0x18/4)

#define TIMER_M0       0
#define TIMER_M1       1
#define TIMER_M2       2
#define TIMER_M3       3

void timer_init();
uint32_t timer_peripheral_read(volatile uint32_t* paddr);
uint64_t timer_read(void);
void timer_delay(uint64_t ms);

#endif
