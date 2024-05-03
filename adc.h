/*
 * adc.h
 *
 *  Created on: 2 May 2024
 *      Author: Nguyễn Tiến Đạt
 */

#ifndef ADC_H_
#define ADC_H_
#include "main.h"

extern void ADC_Init(void);
extern float readADC(uint8_t channel);

#endif /* ADC_H_ */
