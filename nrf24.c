/*
 * nrf24.c
 *
 *  Created on: May 3, 2024
 *      Author: Nguyễn Tiến Đạt
 */

#include "nrf24.h"

void CS_Select (void) {
	LL_GPIO_ResetOutputPin(GPIOB, CSN_Pin);
}
void CS_UnSelect (void) {
	LL_GPIO_SetOutputPin(GPIOB, CSN_Pin);
}
void CE_Enable (void) {
	LL_GPIO_SetOutputPin(GPIOB, CE_Pin);
}
void CE_Disable (void) {
	LL_GPIO_ResetOutputPin(GPIOB, CE_Pin);
}
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;  //W_REGISTER: 001A AAAA
	buf[1] = Data;
	// Pull the CS Pin LOW to select the device
	CS_Select();
	SPI_Transmit(&Data,2);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
}
uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data_nrf=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	SPI_Transmit(&Reg,1);
	SPI_Receive(&data_nrf,1);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data_nrf;
}

// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	SPI_Transmit(&cmd,1);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}
void NRF24_Init (void)
{
	// disable the chip before configuring the device
	CE_Disable();
	CS_UnSelect();


	nrf24_WriteReg(CONFIG, 0);  // will be configured later

	nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

	nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

	nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

	nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

	nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	CE_Enable();
}
