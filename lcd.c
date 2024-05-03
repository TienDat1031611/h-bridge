/*
 * lcd.c
 *
 *  Created on: 26 Apr 2024
 *      Author: Nguyễn Tiến Đạt
 */
#include "lcd.h"

#define address_lcd 0x7E

void lcd_send_cmd (char cmd) {
	char data_m, data_l;
	uint8_t data_t[4];
	data_m = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_m|0x0C; /*bit2=E;bit1=RW;bit0=RS ; C=1100 ~ EN=1,RS=0*/
	data_t[1] = data_m|0x08; /*bit2=E;bit1=RW;bit0=RS ; 8=1000 ~ EN=0,RS=0*/
	data_t[2] = data_l|0x0C; /*bit2=E;bit1=RW;bit0=RS ; C=1100 ~ EN=1,RS=0*/
	data_t[3] = data_l|0x08; /*bit2=E;bit1=RW;bit0=RS ; 8=1000 ~ EN=0,RS=0*/
	send_data(data_t,4);
}

void lcd_send_data (char data_lcd) {
	char data_m, data_l;
	uint8_t data_t[4];
	data_m = (data_lcd&0xf0);
	data_l = ((data_lcd<<4)&0xf0);
	data_t[0] = data_m|0x0D; /*bit2=E;bit1=RW;bit0=RS ; D=1101 ~ EN=1,RS=1*/
	data_t[1] = data_m|0x09; /*bit2=E;bit1=RW;bit0=RS ; 9=1001 ~ EN=0,RS=1*/
	data_t[2] = data_l|0x0D; /*bit2=E;bit1=RW;bit0=RS ; D=1101 ~ EN=1,RS=1*/
	data_t[3] = data_l|0x09; /*bit2=E;bit1=RW;bit0=RS ; 9=1001 ~ EN=0,RS=1*/
	send_data(data_t,4);
}
void lcd_line(int row, int col){
	switch(row)
	{
	case 0:
		col|=0x80;
		break;
	case 1:
		col |=0xC0;
		break;
	}
	lcd_send_cmd(col);
}

void lcd_init (void) {
	// setup 4 bit
	delay_ms(50);
	lcd_send_cmd(0x30);
	delay_ms(5);
	lcd_send_cmd(0x30);
	delay_ms(1);
	lcd_send_cmd(0x30);
	delay_ms(10);
	lcd_send_cmd(0x20);
	delay_ms(10);
	//setup display
	lcd_send_cmd(0x28);
	delay_ms(1);
	lcd_send_cmd(0x08);
	delay_ms(1);
	lcd_send_cmd(0x01);
	delay_ms(5);
	lcd_send_cmd(0x06);
	delay_ms(1);
	lcd_send_cmd(0x0C);

}
void lcd_send_string (char *str)
{
	while(*str) lcd_send_data(*str++);
}
void send_data(uint8_t *data, int length) {
    for (int i = 0; i < length; i++) {
    	I2C_Start();
    	I2C_Address(address_lcd);
        I2C_Write(data[i]);
        I2C_Stop();
    }
}
void check_battery(float  voltage, int voltageInt, int voltageFrac, char *buffer) {
	if ((voltage > 11.96) && (voltage <= 12.41)) {
		snprintf(buffer, 32, "B:%d.%02dV_NORMAL", voltageInt, voltageFrac);
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string("                ");
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string(buffer);
	}
	if ((voltage > 12.41) && (voltage<= 12.98)) {
		snprintf(buffer, 32, "B:%d.%02dV_good", voltageInt, voltageFrac);
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string("                ");
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string(buffer);
	}
	if ( voltage > 12.98) {
		snprintf(buffer, 32, "B:%d.%02dV_high", voltageInt, voltageFrac);
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string("                ");
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string(buffer);
	}
	if ( voltage <= 11.7) {
		snprintf(buffer, 32, "B:%d.%02dV_runout", voltageInt, voltageFrac);
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string("                ");
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string(buffer);
	}
	if ((voltage <= 11.96) && (voltage > 11.7)) {
		snprintf(buffer, 32, "B:%d.%02dV_low", voltageInt, voltageFrac);
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string("                ");
		lcd_line(1,1); // Set cursor to first line, first column
		lcd_send_string(buffer);
	}
}
void read_adc_battery(void) {
	float voltage = readADC(4);
	int voltageInt = (int)voltage;
	int voltageFrac = (int)((voltage - voltageInt) * 100);
	char buffer[32];

	check_battery(voltage, voltageInt, voltageFrac, buffer);

}
void delay_ms(uint16_t t) {
	volatile unsigned long l = 0;
	for(uint16_t n = 0; n < t; n++) {
		for(l = 0; l < 72000; l++) {}
	}
}
