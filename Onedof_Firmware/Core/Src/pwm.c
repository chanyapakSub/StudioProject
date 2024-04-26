/*
 * pwm.c
 *
 *  Created on: Apr 24, 2024
 *      Author: naker
 */

#include "pwm.h"

void Update_pwm(TIM_HandleTypeDef* htim, uint16_t htim_channel,GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int32_t pwm_signal) {
	__HAL_TIM_SET_COMPARE(htim, htim_channel, fabs(pwm_signal));
	if (pwm_signal < 0) {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
	} else {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
	}
}
