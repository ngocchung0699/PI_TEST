#include "stdio.h"
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib.h"


#define PIN 26

int main()
{
    lib_init();

    while(1)
    {  
        if(digitalRead(26) == HIGH){
            printf("high \n");
        }
        else{
            printf("low \n");
        }
        delay_ms(1000);
    }  
    lib_close();
    return 0;
}
