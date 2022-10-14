#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <lcd_i2c.h>


int main (void)
{
  wiringPiSetupGpio () ;
  LCD_Init();

  while (1)
  {
    LCD_Clear();
    delay(1);
    LCD_SendFloat(0, 2, 123.456, 2);
    delay(500);
  }

  return 0 ;
}
