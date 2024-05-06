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
	SPI_Transmit(buf,2);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
}
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[1];
	buf[0] = Reg|1<<5;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	SPI_Transmit(buf,1);
	SPI_Transmit(data,size);

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
	CS_Select();
}
void NRF24_TxMode (uint8_t *Address, uint8_t channel) {
	CE_Disable();
	CS_UnSelect();
	nrf24_WriteReg (RF_CH, channel);  // select the channel
	nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Write the TX address
	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
//	config = config | (1<<1);   // write 1 in the PWR_UP bit
	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_WriteReg (CONFIG, config);

	// Enable the chip after configuring the device
	CE_Enable();

}

// transmit the data

uint8_t NRF24_Transmit (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// select the device
	CS_Select();

	// payload command
	cmdtosend = W_TX_PAYLOAD;
	SPI_Transmit(&cmdtosend,1);

	// send the payload
	SPI_Transmit( data, 32);

	// Unselect the device
	CS_UnSelect();
	delay_ms(1);
	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

		// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
		if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
		{
			cmdtosend = FLUSH_TX;
			nrfsendCmd(cmdtosend);

			// reset FIFO_STATUS
			//nrf24_reset (FIFO_STATUS);

			return 1;
		}

		return 0;

}
void NRF24_RxMode (uint8_t *Address, uint8_t channel)
{
	// disable the chip before configuring the device
	CE_Disable();
	nrf24_WriteReg (RF_CH, channel);  // select the channel
	// select data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_WriteReg (EN_RXADDR, en_rxaddr);
	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address

	nrf24_WriteReg (RX_PW_P1, 32);   // 32 bit payload size for pipe 1

	// power up the device in Rx mode
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg (CONFIG, config);

}
uint8_t isDataAvailable (int pipenum)
{
	uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF24_Receive (uint8_t *data)
{
	uint8_t cmdtosend = 0;

	// select the device
	CS_Select();

	// payload command
	cmdtosend = R_RX_PAYLOAD;
	SPI_Transmit(&cmdtosend, 1);

	// Receive the payload
	SPI_Receive(data,32);

	// Unselect the device
	CS_UnSelect();

	delay_ms(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}

void delay_ms(uint16_t t) {
	volatile unsigned long l = 0;
	for(uint16_t n = 0; n < t; n++) {
		for(l = 0; l < 72000; l++) {}
	}
}



