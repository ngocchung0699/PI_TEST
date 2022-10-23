#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>


#define pin 0


void myInterrupt (void) { 
    // working
}


int main (void)
{
  wiringPiSetupGpio () ;
  pinMode(pin, INPUT);

  wiringPiISR (pin, INT_EDGE_FALLING, &myInterrupt) ;  // 

  while (1)
  {
    /* code */
  }

  return 0 ;
}
