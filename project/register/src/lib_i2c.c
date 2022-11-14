


#include <errno.h>
#include "lib_i2c.h"	// NB: Path changed!
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */


//NB: Added by John Burns



#ifndef NULL



#define NULL 0



#endif



/* Compatibility defines */



#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN



#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA



#endif



#ifndef I2C_FUNC_SMBUS_PEC



#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC



#endif





static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;
	__s32 err;
	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

	err = ioctl(file, I2C_SMBUS, &args);
	if (err == -1)
		err = -errno;
	return err;
}

__s32 i2c_smbus_write_quick(int file, __u8 value)
{
	return i2c_smbus_access(file, value, 0, I2C_SMBUS_QUICK, NULL);
}

__s32 i2c_smbus_read_byte(int file)
{
	union i2c_smbus_data data;
	int err;

	err = i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);

	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte(int file, __u8 value)
{
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE, NULL);
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FF & data.byte;
}

__s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

__s32 i2c_smbus_read_word_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	int err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data);
	if (err < 0)
		return err;
	return 0x0FFFF & data.word;
}

__s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value)
{

	union i2c_smbus_data data;
	data.word = value;
	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}


__s32 i2c_smbus_process_call(int file, __u8 command, __u16 value)
{
	union i2c_smbus_data data;
	data.word = value;
	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_PROC_CALL, &data))
		return -1;
	else
		return 0x0FFFF & data.word;

}

/* Returns the number of read bytes */
__s32 i2c_smbus_read_block_data(int file, __u8 command, __u8 *values)
{
	union i2c_smbus_data data;
	int i, err;
	err = i2c_smbus_access(file, I2C_SMBUS_READ, command, 3, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= 10; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

__s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length, const __u8 *values)

{
	union i2c_smbus_data data;
	int i;
	if (length >= I2C_SMBUS_BLOCK_MAX){
		length = I2C_SMBUS_BLOCK_MAX;
	}

	for (i = 0; i < length; i++){
		data.block[0] = length;
		data.block[i+1] = values[i];
	}

	return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, length+1, &data);
}

/* Returns the number of read bytes */
__s32 i2c_smbus_block_process_call(int file, __u8 command, __u8 length, __u8 *values)
{
	union i2c_smbus_data data;
	int i, err;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	err = i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BLOCK_PROC_CALL, &data);
	if (err < 0)
		return err;
	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}



// Open a connection to the bmp085

// Returns a file id

int i2c_setup_interface(const char *device, int devId)

{
	int fd = 0;
	// Open port for reading and writing

	if ((fd = open(device, O_RDWR)) < 0)
	{
		exit(1);
	}
	// Set the port options and set the address of the device

	if (ioctl(fd, I2C_SLAVE, devId) < 0) 
	{					
		exit(1);

	}
	return fd;
}

/*

 * wiringPiI2CSetup:

 *	Open the I2C device, and regsiter the target device

 *********************************************************************************

 */
int i2c_setup (int choose_device, const int devId)
{

  	//int rev ;

  	const char *device ;

	//rev = piGpioLayout () ;

  	if (choose_device == 1)
    	device = "/dev/i2c-1" ;
  	if (choose_device == 0)
    	device = "/dev/i2c-0" ;
  	return i2c_setup_interface (device, devId) ;
}



