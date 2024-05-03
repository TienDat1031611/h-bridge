/*
 * spi.h
 *
 *  Created on: 3 May 2024
 *      Author: Nguyễn Tiến Đạt
 */

#ifndef SPI_H_
#define SPI_H_

#include "main.h"

extern void SPIConfig (void);
extern void SPI_Transmit (uint8_t *data, int size);
extern void SPI_Receive (uint8_t *data, int size);

#endif /* SPI_H_ */
