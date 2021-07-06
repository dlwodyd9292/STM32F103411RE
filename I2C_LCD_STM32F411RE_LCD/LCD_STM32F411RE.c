/*
 * LCD_STM32F411RE.c
 *
 *  Created on: Jul 6, 2021
 *      Author: kccistc
 *
 */

#include "LCD_STM32F411RE.h"

extern I2C_HandleTypeDef hi2c1;

void lcd_send_cmd(char cmd){
	char upperNibble, lowerNibble;
	uint8_t tdata[4];
	upperNibble = cmd & 0xf0;
	lowerNibble = (cmd << 4) & 0xf0;
	tdata[0] = upperNibble | 0x0c;
	tdata[1] = upperNibble | 0x08;
	tdata[2] = lowerNibble | 0x0c;
	tdata[3] = lowerNibble | 0x08;
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)tdata, 0x4, 100);
}
void lcd_send_data(char data){
	char upperNibble, lowerNibble;
	uint8_t tdata[4];
	upperNibble = data & 0xf0;
	lowerNibble = (data << 4) & 0xf0;
	tdata[0] = upperNibble | 0x0d;
	tdata[1] = upperNibble | 0x09;
	tdata[2] = lowerNibble | 0x0d;
	tdata[3] = lowerNibble | 0x09;
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)tdata, 0x4, 100);
}
void lcd_send_string(char *str){
	while(*str)
		lcd_send_data(*str++);
}
void lcd_setCursor(uint8_t row, uint8_t col)
{
	col %= 16;
	row %= 2;
	uint8_t address = (0x40 * row) + col;
	uint8_t command = 0x80 + address;        // 1?��, 0?��
	lcd_send_cmd(command);
}


void lcd_init(void){
	HAL_Delay(50);
	lcd_send_cmd(0x30);
	HAL_Delay(5);
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd(0x30);
	HAL_Delay(10);

	lcd_send_cmd(0x20);	//4bit
	HAL_Delay(10);
	lcd_send_cmd(0x28);	// 4bit, 2line, 5x8
	HAL_Delay(1);
	lcd_send_cmd(0x08);	// display off
	HAL_Delay(1);
	lcd_send_cmd(0x01);	// clear display
	HAL_Delay(1);
	lcd_send_cmd(0x06);	// cursor increment, no shift
	HAL_Delay(1);
	lcd_send_cmd(0x0C);	// display on/off
	HAL_Delay(1);
}

