
#include <stdio.h>
#include <wiringPi.h>
#include <ads1115.h>

#define	AD_BASE 120


int main (void)
{

  char * Vref[]={"6.144","4.096","2.048","1.024","0.512","0.256"};

  int i;
  int g;
  int value;

  wiringPiSetupGpio () ;
  ads1115Setup(AD_BASE,0x48);

  for(g=0;g<6;g++)
  {
    digitalWrite(AD_BASE,g);
    printf("\nGain with Vref=%sV\n",Vref[g]);
    printf("------------\n");
    for (i=0;i<4;i++)
     printf("Ch%d:%d\n",i,analogRead(AD_BASE+i));
  }
  
  while(1){
	  digitalWrite(AD_BASE,2);
	  printf("Ch%d:%d\n",2,analogRead(AD_BASE+2));
	  delay(5000);
	}
  return 0 ;
}

