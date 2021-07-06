#ifndef __LCD_STM32F411RE_H__
#define __LCD_STM32F411RE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SLAVE_ADDRESS_LCD 0x4E

void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_setCursor(uint8_t row, uint8_t col);  //row, col

#ifdef __cplusplus
}
#endif

#endif /* __LCD_STM32F411RE_H__ */
