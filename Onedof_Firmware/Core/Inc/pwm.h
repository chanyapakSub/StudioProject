/*
 * pwm.h
 *
 *  Created on: Apr 24, 2024
 *      Author: naker
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
#include "math.h"

// USE FOR MD20A MOTOR DRIVER
void Update_pwm(TIM_HandleTypeDef* htim, uint16_t htim_channel, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int32_t pwm_signal);
#endif /* INC_PWM_H_ */
