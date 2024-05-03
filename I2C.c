/*
 * I2C.c
 *
 *  Created on: 23 Apr 2024
 *      Author: Nguyễn Tiến Đạt
 */
#include "I2C.h"


void I2C_Config(void) {
/*** The following is the required sequence in master mode.
● Program the peripheral input clock in I2C_CR2 Register in order to generate correct
timings
● Configure the clock control registers
● Configure the rise time register
● Program the I2C_CR1 register to enable the peripheral
● Set the START bit in the I2C_CR1 register to generate a Start condition
**/

/*Enable I2C Clock */
	RCC->APB1ENR |= (1<<21);
/*Enable GPIO Clock*/
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

	GPIO_InitStruct.Pin = SCL1_Pin|SDA1_Pin ;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*Reset I2C*/
	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);
/*Program the peripheral input clock in I2C_CR2 Register in order to generate correct
timings*/
	I2C1->CR2 |=(36<<0);   //the maximum frequency is limited by the maximum APB frequency (36 MHz)
/*Configure the clock control registers*/
	I2C1->CCR = (180<<0); //CCR = (Tr(SCL)+Tw(SCLH))/TPCLK1
/*Configure the rise time register*/
	I2C1->TRISE = 37; //Trise = Tr(SCL)/TPCLK1 + 1
/*Set the START bit in the I2C_CR1 register to generate a Start condition*/
	I2C1->CR1 |=(1<<0);

}
void I2C_Start(void) {
/***
● Send the Start Condition
● Wait the SB bit is set. This indicates that the start condition is generated.
**/
	I2C1->CR1 |= (1<<8);
	while (!(I2C1->SR1 & (1<<0)));
}
void I2C_Write(uint8_t data) {
/***
 ● Wait for the TXE(bit 7 SR1) is set. This indicate that DR is empty
 ● Send data to the DR register
 ● Wait for the BTF (bit 2 in SR1) is set. This indicate that the end of last data transmission
 */
	while (!(I2C1->SR1 &(1<<7)));
	I2C1->DR = data;
	while (!(I2C1->SR1 &(1<<2)));
}
void I2C_Address(uint8_t address) {
/***
 ● The address byte is sent to DR register
 ● Wait the ADDR bit(bit 1 SR1) is set. This indicate the address byte is send successful.
 ● Clear ADDR bit by reading SR1 and SR2, a read of the SR1 register followed by a read of the SR2 register
 */
	I2C1->DR = address;
	while (!(I2C1->SR1 & (1<<1)));
	uint8_t temp = I2C1->SR1 | I2C1 -> SR2;
}
void I2C_Stop(void) {
	I2C1->CR1 |= (1<<9);
}

