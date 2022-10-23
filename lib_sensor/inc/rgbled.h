#ifndef __RGBLED_H
#define __RGBLED_H

#define MAX_LED 10



void pulse_low(int pin);
void pulse_high(int pin);
void led_set(int LEDnum, int red, int green, int blue);
void led_print (int pin);
void led_reset(void);


#endif
