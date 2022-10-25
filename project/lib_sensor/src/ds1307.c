#include <ds1307.h>
#include <lib_i2c.h>

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

void set_time(int fd, rtc RTC){
	
	
    uint8_t times[7];
    times[0] =0x80;
	times[1] =ds1307_dec2bcd(RTC.minute);
	times[2] =ds1307_dec2bcd(RTC.hours);
	times[3] =0x00;
	times[4] =ds1307_dec2bcd(RTC.day);
	times[5] =ds1307_dec2bcd(RTC.month);
	times[6] =ds1307_dec2bcd(RTC.year);

    i2c_smbus_write_block_data(fd, 0x00, 7, times);
	i2c_smbus_write_byte_data(fd, 0x00, ds1307_dec2bcd(RTC.second));
}

void get_time(int fd, rtc *RTC){
	uint8_t data[10];

	//i2c_smbus_read_block_data(fd,0x00, data);
	//i2c_smbus_read_byte_data(fd, 0x00);
	RTC->second = i2c_smbus_read_byte_data(fd, 0x00);
	RTC->minute = i2c_smbus_read_byte_data(fd, 0x00);
	RTC->hours = i2c_smbus_read_byte_data(fd, 0x00);
	RTC->day = i2c_smbus_read_byte_data(fd, 0x00);
	RTC->month = i2c_smbus_read_byte_data(fd, 0x00);
	RTC->year = i2c_smbus_read_byte_data(fd, 0x00);
}
