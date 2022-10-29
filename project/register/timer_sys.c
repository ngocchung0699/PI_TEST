#include"timer_sys.h"
#include <time.h>

void delay_sys_ms (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000) ;
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

    nanosleep (&sleeper, &dummy) ;
}

void delay_sys_us (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000000) ;
    sleeper.tv_nsec = (long)(howLong % 1000000) * 1000 ;

    nanosleep (&sleeper, &dummy) ;
}

void delay_sys_ns (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000000000) ;
    sleeper.tv_nsec = (long)(howLong % 1000000000) ;

    nanosleep (&sleeper, &dummy) ;
}