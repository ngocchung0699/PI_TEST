#include <delay_time.h>
#include <time.h>

void delay_ms (unsigned long howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}
void delay_us (unsigned long howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000000) ;
  sleeper.tv_nsec = (long)(howLong % 1000000) * 1000 ;

  nanosleep (&sleeper, &dummy) ;
}
void delay_ns (unsigned long howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000000000) ;
  sleeper.tv_nsec = (long)(howLong % 1000000000) ;

  nanosleep (&sleeper, &dummy) ;
}

