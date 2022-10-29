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
#include "timer.h"

static volatile unsigned int *timer ;

void timer_init(){
    int memfd = open("/dev/mem", O_RDWR | O_SYNC);

    timer = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, TIMER_BASE) ;
    if (timer == MAP_FAILED)
        printf("mmap gpio failed: %s\n", strerror(errno));
    close(memfd);
}

const uint32_t interval_1 = CLOCKHZ;
uint32_t cur_val_1 = 0;

const uint32_t interval_3 = CLOCKHZ / 4;
uint32_t cur_val_3 = 0;

void timer_init() {
    cur_val_1 = REGS_TIMER->counter_lo;
    cur_val_1 += interval_1;
    REGS_TIMER->compare[1] = cur_val_1;

    cur_val_3 = REGS_TIMER->counter_lo;
    cur_val_3 += interval_3;
    REGS_TIMER->compare[3] = cur_val_3;
}

void handle_timer_1() {
    cur_val_1 += interval_1;
    REGS_TIMER->compare[1] = cur_val_1;
    REGS_TIMER->control_status |= SYS_TIMER_IRQ_1;

    //printf("Timer 1 received.\n");
}

void handle_timer_3() {
    cur_val_3 += interval_3;
    REGS_TIMER->compare[3] = cur_val_3;
    REGS_TIMER->control_status |= SYS_TIMER_IRQ_3;

    //printf("Timer 3 received.\n");
}

uint64_t timer_get_ticks() {
    uint32_t hi = REGS_TIMER->counter_hi;
    uint32_t lo = REGS_TIMER->counter_lo;

    //double check hi value didn't change after setting it...
    if (hi != REGS_TIMER->counter_hi) {
        hi = REGS_TIMER->counter_hi;
        lo = REGS_TIMER->counter_lo;
    }

    return ((uint64_t)hi << 32) | lo;
}

//sleep in milliseconds.
void timer_sleep(uint32_t ms) {
    uint64_t start = timer_get_ticks();

    while(timer_get_ticks() < start + (ms * 1000)) {

    }
}