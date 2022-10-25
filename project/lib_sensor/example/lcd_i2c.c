#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <lcd_i2c.h>


int main (void)
{
    printf("+------------------------------------+\n");
    printf("|   Raspberry Pi wiringPi program    |\n");
    printf("+------------------------------------+\n");
    printf("|             Sensor name            |\n");
    printf("|              Grove LCD             |\n");
    printf("|------------------------------------|\n");
    printf("|               Connect              |\n");
    printf("|    BCM (PI)      |    Sensor Pin   |\n");
    printf("|       GND        |       GND       |\n");
    printf("|       5V         |       VCC       |\n");
    printf("|       SDA1       |       SDA       |\n");
    printf("|       SCL1       |       SCL       |\n");
    printf("+------------------------------------+\n");
    wiringPiSetupGpio () ;
    LCD_Init();

  while (1)
  {
    LCD_Clear();
    delay(1);
    LCD_SendChar(0, 2, "Hello World!", 12);
    delay(500);
  }

  return 0 ;
}
