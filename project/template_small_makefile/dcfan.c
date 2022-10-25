#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


#define FAN 		26


int main(void)
{
  unsigned char cnt=0;

  printf("***** Raspberry pi DC FAN Test ******\n") ;
  wiringPiSetupGpio();


  pinMode(FAN, OUTPUT);
  
  while(1){
    digitalWrite(FAN, HIGH);
    printf("26 HIGH \n") ;
    delay(1000);
    digitalWrite(FAN, LOW);
    printf("26 LOW \n") ;
    delay(1000);
    }
  
  return 0 ;
}
