/*
 * state.h
 *
 *  Created on: May 3, 2024
 *      Author: naker
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_
#include "main.h"
#include "pwm.h"
#include "ModBusRTU.h"
#include "Basesystem.h"
#include "qei.h"

typedef struct{
	uint8_t homing_state[2];	/* Channel 0 is for Homing state was selected
								Channel 1 is for selected state  */
	uint8_t homing_command;
	uint8_t is_home;

} HOME;

typedef struct{
	uint8_t emer_state;
} EMER;
void homing(HOME* home, GPIO_TypeDef* GPIO_Prox, uint16_t GPIO_Pin_Prox, TIM_HandleTypeDef* htim, uint16_t htim_channel
		, GPIO_TypeDef* GPIO_PWM, uint16_t GPIO_Pin_PWM, u16u8_t* registerFrame[200]
		, GPIO_TypeDef* GPIO_HomingButton, uint16_t GPIO_Pin_HomingButton, QEI* encoder);
void emergency(EMER* emer, GPIO_TypeDef* GPIO_EmerLight, uint16_t GPIO_Pin_EmerLight);
#endif /* INC_STATE_H_ */
