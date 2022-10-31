#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "timer_sys.h"
#include "gpio.h"
#include "sys/bcm2835.h"

#define PIN 26

int main() 
{
    bcm2835_init();
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
    while (1)
    {
                // Turn it on
        bcm2835_gpio_write(PIN, HIGH);
        
        // wait a bit
        bcm2835_delay(500);
        
        // turn it off
        bcm2835_gpio_write(PIN, LOW);
        
        // wait a bit
        bcm2835_delay(500);
    }
    bcm2835_close();
    return (EXIT_SUCCESS);
}
