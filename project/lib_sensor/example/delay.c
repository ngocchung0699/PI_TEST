#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <delay_time.h>


int main (void)
{
  wiringPiSetupGpio () ;

  while (1)
  {
    delay_ns(1000);
    delay_us(1000);
    delay_ms(1000);
  }

  return 0 ;
}
