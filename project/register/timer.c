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
    timer = (uint32_t *)mmap(NULL, BLOCK_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, memfd, TIMER_BASE);
    if (timer == MAP_FAILED)
        printf("mmap timer failed: %s\n", strerror(errno));
    close(memfd);
}

/* Read the System Timer Counter (64-bits) */
uint64_t timer_read(void)
{

    uint32_t hi, lo;
    uint64_t st;

    hi = *(timer + TIMER_CHI/4);

    lo = *(timer + TIMER_CLO/4);;
    
    st = *(timer + TIMER_CHI/4);;
    
    /* Test for overflow */
    if (st == hi)
    {
        st <<= 32;
        st += lo;
    }
    else
    {
        st <<= 32;
        st += *(timer + TIMER_CLO/4);
    }
    return st;
}

/* Delays for the specified number of microseconds with offset */
void timer_delay(uint64_t ms)
{
    uint64_t compare = ms;

    while(timer_read() < compare);
}
