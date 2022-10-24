#include <rgbled.h>
#include <F:/PI-IOT/project/template/wiringPi/wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum status{
    g,
    r,
    b
}status;

typedef struct led_rgb{
    uint8_t LED_Data[MAX_LED][3];
}led_rgb;

led_rgb rgb;


void pulse_low(int pin){
    digitalWrite(pin, HIGH);
    for(int i = 0; i < 40; i++);
    digitalWrite(pin, LOW);
    for(int i = 0; i < 175; i++);
}
void pulse_high(int pin){
    digitalWrite(pin, HIGH);
    for(int i = 0; i < 130; i++);
    digitalWrite(pin, LOW);
    for(int i = 0; i < 85; i++);
}

void led_set(int LEDnum, int red, int green, int blue){
	rgb.LED_Data[LEDnum][g] = green;
	rgb.LED_Data[LEDnum][r] = red;
	rgb.LED_Data[LEDnum][b] = blue;
}

void led_print (int pin){
    uint32_t color;
	for (int i= 0; i<MAX_LED; i++)
	{
		color = ((rgb.LED_Data[i][g]<<16) | (rgb.LED_Data[i][r]<<8) | (rgb.LED_Data[i][b]));

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i)){ pulse_high(pin); }
			else {pulse_low(pin);}
		}
	}
}

void led_reset (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		rgb.LED_Data[i][g] = 0;
		rgb.LED_Data[i][r] = 0;
		rgb.LED_Data[i][b] = 0;
	}
}

