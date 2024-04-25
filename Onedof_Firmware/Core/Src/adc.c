/*
 * adc.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#include "adc.h"

void ADC_init(ADC_HandleTypeDef* hadc, ADC* adc){
	HAL_ADC_Start_DMA(hadc, adc -> adc_buffer, 50);
}

uint32_t Update_adc(ADC* adc){
	uint64_t sum_adc = 0;
	for (uint8_t i = 0; i < 50; i++){
		sum_adc += adc -> adc_buffer[i];
	}
	return adc -> adc_avg = sum_adc / 50;
}
