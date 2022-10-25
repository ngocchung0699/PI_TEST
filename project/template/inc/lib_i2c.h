

#ifndef LIB_I2C_SMBUS_H
#define LIB_I2C_SMBUS_H


#include <linux/types.h>
#include <linux/i2c.h>
#include <wiringPiI2C.h>


extern __s32 i2c_smbus_write_quick(int file, __u8 value);

extern __s32 i2c_smbus_read_byte(int file);

extern __s32 i2c_smbus_write_byte(int file, __u8 value);

extern __s32 i2c_smbus_read_byte_data(int file, __u8 command);

extern __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);

extern __s32 i2c_smbus_read_word_data(int file, __u8 command);

extern __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);

extern __s32 i2c_smbus_process_call(int file, __u8 command, __u16 value);

/* Returns the number of read bytes */
extern __s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values);

extern __s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length, const __u8 *values);

/* Returns the number of read bytes */
extern __s32 i2c_smbus_block_process_call(int file, __u8 command, __u8 length, __u8 *values);

int i2c_setup_interface(const char *device, int devId);

int i2c_setup (int choose_device, const int devId);

#endif /* LIB_I2C_SMBUS_H */







