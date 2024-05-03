/*
 * spi.c
 *
 *  Created on: 3 May 2024
 *      Author: Nguyễn Tiến Đạt
 */
#include "spi.h"

void SPIConfig (void)
{
  RCC->APB1ENR |= (1<<14);  // Enable SPI2 CLock

  SPI2->CR1 &= ~ (1<<0)&(1<<1);   // CPOL=0, CPHA=0

  SPI2->CR1 |= (1<<2);  // Master Mode

  SPI2->CR1 |= (2<<3);  // BR[2:0] = 010: fPCLK/8, PCLK1 = 36MHz, SPI clk = 4.5MHz

  SPI2->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first

  SPI2->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management

  SPI2->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex

  SPI2->CR1 &= ~(1<<11);  // DFF=0, 8 bit data

  SPI2->CR2 = 0;

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /*Configure MOSI and SCK Pin output*/
  GPIO_InitStruct.Pin = MOSI2_Pin|SCK2_Pin ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure MISO Pin input*/
  GPIO_InitStruct.Pin = MISO2_Pin ;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure CS and CSN Pin output*/
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void SPI_Enable (void)
{
	SPI2->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	SPI2->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}
void SPI_Transmit (uint8_t *data, int size)
{
	/*
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	*/

	int i=0;
	while (i<size)
	{
	   while (!((SPI2->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI2->DR = data[i];  // load the data into the Data Register
	   i++;
	}
		while (!((SPI2->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
		while (((SPI2->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

		//  Clear the Overrun flag by reading DR and SR
		uint8_t temp_spi = SPI2->DR;
				temp_spi = SPI2->SR;

	}

void SPI_Receive (uint8_t *data, int size)
{
	/*
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	*/

	while (size)
	{
		while (((SPI2->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI2->DR = 0;  // send dummy data
		while (!((SPI2->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI2->DR);
		size--;
	}
}
