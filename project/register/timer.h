#ifndef	__TIMER_H__
#define	__TIMER_H__

#include <stdint.h>

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define TIMER_BASE                   0x7e003000

#define CLOCKHZ 1000000

struct timer_regs {
    uint32_t control_status;
    uint32_t counter_lo;
    uint32_t counter_hi;
    uint32_t compare[4];
};

enum vc_irqs {
    SYS_TIMER_IRQ_0 = 1,
    SYS_TIMER_IRQ_1 = 2,
    SYS_TIMER_IRQ_2 = 4,
    SYS_TIMER_IRQ_3 = 8,
    AUX_IRQ = (1 << 29)
};

#define REGS_TIMER ((struct timer_regs *)(TIMER_BASE))

void timer_init();
void handle_timer_1();
void handle_timer_3();
void timer_sleep(uint32_t ms);
uint64_t timer_get_ticks();


#endif