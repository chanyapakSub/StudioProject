/*
 * eff.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#include "eff.h"

void Update_actual_eff(EFF* eff, u16u8_t* registerFrame[200], GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull, GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push){
	eff -> actual_status[0] = HAL_GPIO_ReadPin(GPIO_Pull, GPIO_Pin_Pull);
	eff -> actual_status[1] = HAL_GPIO_ReadPin(GPIO_Push, GPIO_Pin_Push);
	if(eff -> actual_status[0] && !eff -> actual_status[1]){
		// Pull reed switch on and Push reed switch off
		registerFrame[0x04] -> U16 = 1;
	}
	else if(!eff -> actual_status[0] && eff -> actual_status[1]){
		// Pull reed switch off and Push reed switch on
		registerFrame[0x04] -> U16 = 2;
	}
	else{registerFrame[0x04] = 0;} // Pull reed switch off and Push reed switch off
}

void Update_eff(EFF* eff, GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull,
		GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push, GPIO_TypeDef* GPIO_Vacuum, uint16_t GPIO_Pin_Vacuum)
{
	HAL_GPIO_WritePin(GPIO_Vacuum, GPIO_Pin_Vacuum, eff -> solenoid_command[0]);
	HAL_GPIO_WritePin(GPIO_Pull, GPIO_Pin_Pull, eff -> solenoid_command[1]);
	HAL_GPIO_WritePin(GPIO_Push, GPIO_Pin_Push, eff -> solenoid_command[2]);
}
