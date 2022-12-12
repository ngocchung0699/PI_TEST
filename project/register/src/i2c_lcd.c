#include "i2c_lcd.h"

void LCD_Init()
{
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, LCD_SET_MODE);
    delay_ms(1);
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, LCD_DISPLAY_ON);
    delay_ms(1);
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, LCD_CLEAR_DATA);
    delay_ms(1);
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, LCD_POINTER);
    delay_ms(1);
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, LCD_CLEAR_DATA);
    delay_ms(10);
}

uint8_t data_s[] = {0, 0};
void LCD_Send_Byte(uint8_t ADDR, uint8_t reg, uint8_t data)
{
    data_s[0] = reg;
    data_s[1] = data;
    i2c_send(ADDR, data_s, 2);
}

void LCD_Clear(){
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_CLEAR, LCD_CLEAR_DATA);
    delay_ms(1);
}
void LCD_SetCursor(int column, int row){
    //column = (row == 0 ? column | 0x80 : column | 0xc0);
    if    (row == 0)  { column = column | 0x80; }
    if    (row == 1)  { column = column | 0xc0; }
    LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_SET_LINE, column);
}

void LCD_SendChar(int row, int column, char *data, size_t length_data){
    LCD_SetCursor(column, row);
    for(unsigned int i=0; i < length_data; i++){
        LCD_Send_Byte(LCD_I2C_ADDR, LCD_ADDR_DATA, (int) *data++ );
    }
}

void LCD_SendInt(int row, int column, int number){
    char arr[100];
    sprintf(arr, "%d", number);
    LCD_SendChar(row, column, arr, strlen(arr));
}

void LCD_SendFloat(int row, int column, double number, int number_decimal){
    char arr[100];
    char nd[2];
    char var_temp[6];
    char endChar[1] = {0x00};

    memset(var_temp, 0x00, 6);
    memcpy(var_temp, "%0.", 3);
    sprintf(nd, "%d", number_decimal);
    memcpy(var_temp + strlen(var_temp), nd, 1);
    memcpy(var_temp + strlen(var_temp), "f", 1);

    memcpy(var_temp + strlen(var_temp), endChar, 1);

    sprintf(arr, (char*)var_temp , number);
  
    LCD_SendChar(row, column, arr, strlen(arr));
}



