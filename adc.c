/*
 * adc.c
 *
 *  Created on: 2 May 2024
 *      Author: Nguyễn Tiến Đạt
 */
#include "adc.h"

void ADC_Init(void) {
/*Enable GPIO and ADC clock*/
	RCC->APB2ENR |=(1<<2)|(1<<9);
/*Set the prescaler div 4 */
	RCC->CFGR |=(1<<14);   // ADCCLK = 18MHz
/*Set the sampling time for the channels*/
	ADC1->SMPR2 |= (1<<12); // sampling time of 7.5 cycles for channel 4
							// conversion time = (sampling time + 12.5)/ADCCLK = 1us
/*Configure GPIO*/
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

	GPIO_InitStruct.Pin = ADCVol_Pin; // Specify the pin to be configured
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG; // Set the pin mode to analog mode
	LL_GPIO_Init(ADCVol_GPIO_Port, &GPIO_InitStruct); // Initialize GPIO port with the configuration
// Perform calibration
	    ADC1->CR2 |= ADC_CR2_CAL; // Start calibration
	    while (ADC1->CR2 & ADC_CR2_CAL); // Wait for calibration to complete
}
float readADC(uint8_t channel) {
    // Configure ADC channel
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (channel << 0); // Set channel in sequence
    ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC)); // Wait for conversion to complete

    // Read conversion result
    uint16_t adcValue = ADC1->DR;

    float dividerRatio = 4.0;
    // Convert ADC value to voltage (assuming Vref+ = 3.3V)
    float voltage = (3.3f * adcValue * dividerRatio) / 4096.0f;

    return voltage;
}
