/*
 * I2C.h
 *
 *  Created on: 23 Apr 2024
 *      Author: Nguyễn Tiến Đạt
 */

#ifndef I2C_H_
#define I2C_H_
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

extern void I2C_Config(void);
extern void I2C_Start(void);
extern void I2C_Write(uint8_t data);
extern void I2C_Address(uint8_t address);
extern void I2C_Stop(void);

#endif /* I2C_H_ */
