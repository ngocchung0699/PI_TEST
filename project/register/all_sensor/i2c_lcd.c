#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "lib.h"
#include "i2c_lcd.h"

int main() 
{   
    lib_init();
    i2c_init();
    LCD_Init();


    while (1)
    {   
        LCD_Clear();
        LCD_SendChar(1, 1, "hello", 5);
        delay_ms(500);

        LCD_Clear();
        LCD_SendInt(0, 2, 1234567);
        delay_ms(500);

        LCD_Clear();
        LCD_SendFloat(0, 3, 123.4564, 3);
        delay_ms(500);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
