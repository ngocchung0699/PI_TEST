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

// Convert Decimal to Binary Coded Decimal (BCD)
uint8_t ds1307_dec2bcd(uint8_t num)
{
  return ((num/10 * 16) + (num % 10));
}

// Convert Binary Coded Decimal (BCD) to Decimal
uint8_t ds1307_bcd2dec(uint8_t num)
{
  return ((num/16 * 10) + (num % 16));
}
//                      reg    s     m     h    w      d     m     y
uint8_t data_send[] = {0x00, 0x30, 0x14, 0x13, 0x04, 0x07, 0x12, 0x22};

uint8_t data_read[100];

int main() 
{   
    lib_init();
    i2c_setup();
  
    // set time
    // i2c_send(0x68, data_send, 8);

    delay_ms(100);

    while (1)
    {   
        // get time
        i2c_send(0x68, &data_send[0], 1);
        
        i2c_receive(0x68, data_read, 7);
        
        printf("time now: %d:%d:%d .date: %d - %d,%d,20%d \n",    ds1307_bcd2dec(data_read[2]), ds1307_bcd2dec(data_read[1]), ds1307_bcd2dec(data_read[0]), 
                            ds1307_bcd2dec(data_read[3]), ds1307_bcd2dec(data_read[4]), ds1307_bcd2dec(data_read[5]), ds1307_bcd2dec(data_read[6]));
        delay_ms(1000);
    }
    lib_close();
    return (EXIT_SUCCESS);
}
    
