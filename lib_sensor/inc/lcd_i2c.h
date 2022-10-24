#ifndef __LCD_I2C_H
#define __LCD_I2C_H

#include <F:/PI-IOT/project/lcd_i2c/wiringPi/wiringPi.h>
#include <F:/PI-IOT/project/lcd_i2c/wiringPi/wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// Define some device parameters
#define LCD_I2C_ADDR   0x3E // I2C device address

#define LCD_I2C_ADDR_SETUP wiringPiI2CSetup(LCD_I2C_ADDR) // setup i2c wiringPi library

#define LCD_ADDR_SET_LINE 0x80  // adress setup line ()

#define LCD_LINE1  0x80 // 1st line
#define LCD_LINE2  0xC0 // 2nd line
#define LCD_ADDR_DATA  0x40 // data display
#define LCD_CLEAR_DATA  0x01 // clear data display
#define LCD_ADDR_CLEAR 0x80

#define LCD_SET_MODE 0x28  // 4-bit mode, 2 line, small font size
#define LCD_DISPLAY_ON 0X0C  // Display ON, Cursor OFF, Cursor blink OFF
#define LCD_POINTER 0x06   // Cursor move direction left to right, no display shift

void LCD_Init(void);
void LCD_Send_Byte(int AD, int reg, int data);
void LCD_Clear();
void LCD_SetCursor(int column, int row);
void LCD_SendChar(int row, int column, char *data, size_t length_data);  
void LCD_SendInt(int row, int column, int number);
void LCD_SendFloat(int row, int column, double number, int number_decimal);

#endif
