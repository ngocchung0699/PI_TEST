#include "led_bar.h"

void start_led(void)
{
    for(int i = 0; i < 11; i++)
    {
        digitalWrite(pin_clock, 1);
        delay_us(9);
        digitalWrite(pin_clock, 0);
        delay_us(9);
    }
}

void end_led(void)
{
    for(int i = 0; i < 12; i++)
    {
        digitalWrite(pin_clock, 1);
        delay_us(9);
        digitalWrite(pin_clock, 0);
        delay_us(9);
    }
    digitalWrite(pin_clock, 1);
    delay_us(14);

    digitalWrite(pin_clock, 0);
    delay_us(4);
    digitalWrite(pin_clock, 1);
    delay_us(4);
    digitalWrite(pin_clock, 0);

    delay_us(240);
        
    for(int i = 0; i < 4; i++)
    {
        digitalWrite(pin_data, 1);
        delay_us(4);
        digitalWrite(pin_data, 0);
        delay_us(4);
    }
    digitalWrite(pin_clock, 1);
    delay_us(4);
    digitalWrite(pin_clock, 0);
}

// led on: status = 1
// led off: status = 0
void set_led(int num, bool status)
{
    int led_num = 1;

    start_led();

    for(int i = 0 ; i < 10 ; i++)
    {
        for(int j = 0 ; j < 8; j++)
        {
            digitalWrite(pin_clock, 1);
            if((led_num == num) && (status == 1) &&(j == 0) ){digitalWrite(pin_data, 1);}
            if( (led_num == num) && (j == 4) ){digitalWrite(pin_data, 0);}
            delay_us(9);
            digitalWrite(pin_clock, 0);
            delay_us(9);
        }
        led_num++;
    }

    end_led();

}

// reverse direction : dir = 0
// forward direction : dir = 1
void set_level_led(int num, bool dir)
{
    int led_num = 0;

    start_led();

    if(dir == 1)
    {
        led_num = 0;
        for(int i = 0 ; i < 10 ; i++)
        {
            for(int j = 0 ; j < 8; j++)
            {
                digitalWrite(pin_clock, 1);
                if((led_num < num) && (j == 0) ){digitalWrite(pin_data, 1);}
                if( (led_num < num) && (j == 4) ){digitalWrite(pin_data, 0);}
                delay_us(9);
                digitalWrite(pin_clock, 0);
                delay_us(9);
            }
            led_num++;
        }
    }
    else
    {
        led_num = 9;

        for(int i = 0 ; i < 10 ; i++)
        {
            for(int j = 0 ; j < 8; j++)
            {
                digitalWrite(pin_clock, 1);
                if( (led_num < num) && (j == 0) ){digitalWrite(pin_data, 1);}
                if( (led_num < num) && (j == 4) ){digitalWrite(pin_data, 0);}
                delay_us(9);
                digitalWrite(pin_clock, 0);
                delay_us(9);
            }
            led_num--;
        }
    }

    end_led();
}

void random_led(void)
{
    unsigned int n = rand();
    int num = map(n, 0, 2147483646, 1, 10);
    printf("value: %d\n", num);
    set_led(num, 1);
}

