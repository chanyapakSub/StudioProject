/*
 * eff.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef INC_EFF_H_
#define INC_EFF_H_

#include "main.h"
#include "ModBusRTU.h"

typedef struct{
	uint8_t actual_status[2]; // 0 = Pull reed switch, 1 = Push reed switch
	uint8_t solenoid_command[3]; // 0 = vacuum, 1 = pull, 2 = push
	u16u8_t update_actual_status[200];
} EFF;

void Update_actual_eff(EFF* eff, GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull, GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push);
void Update_eff(EFF* eff, GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull, GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push, GPIO_TypeDef* GPIO_Vacuum, uint16_t GPIO_Pin_Vacuum);

#endif /* INC_EFF_H_ */
