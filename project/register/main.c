#include <stdio.h>
#include "lib.h"
 
int main()
{
    int fd;
    if(wiringPiSetupGpio() < 0)return 1;
    if((fd = serialOpen("/dev/serial0",9600)) < 0)return 1;
    printf("serial test start ...\n");
    serialPrintf(fd,"Hello World!!!\n");
    while(1)
    {  
        serialPutchar(fd,serialGetchar(fd));
    }  
    serialClose(fd);
    return 0;
}
