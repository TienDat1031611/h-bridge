/*
 * lcd.h
 *
 *  Created on: 26 Apr 2024
 *      Author: Nguyễn Tiến Đạt
 */

#ifndef LCD_H_
#define LCD_H_
#include "main.h"
#include "stdio.h"
#include "I2C.h"
#include "adc.h"
extern void lcd_send_cmd (char cmd);
extern void send_data(uint8_t *data, int length);
extern void lcd_send_data (char data_lcd);
static void delay_ms(uint16_t t);
extern void lcd_line(int row, int col);
extern void lcd_send_string (char *str);
extern void lcd_init (void);
extern void check_battery(float  voltage, int voltageInt, int voltageFrac, char *buffer) ;
extern void read_adc_battery(void);
#endif /* LCD_H_ */
