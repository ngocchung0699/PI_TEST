#ifndef __DS1307_H
#define __DS1307_H
#include <stdint.h>


typedef struct
{
	uint8_t hours;
	uint8_t minute;
	uint8_t second;
	uint8_t day;
	uint8_t month;
	uint8_t year;
}rtc;

void set_time(int fd, rtc RTC);
void get_time(int fd, rtc *RTC);

#endif
