/*
 * adc.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"

typedef struct{
	uint32_t adc_buffer[50]; //ADC BUFFER FOR COLLECT ADC DATA
	uint32_t adc_avg; //AVERAGE ADC DATA
} ADC;

void ADC_init(ADC_HandleTypeDef* hadc, ADC* adc);
uint32_t Update_adc(ADC* adc);

#endif /* ADC_H_ */
