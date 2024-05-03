/*
 * eff.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "eff.h"

<<<<<<< Updated upstream
//void eff_init(EFF *eff){
//
//}
=======
void Update_endeffector_status(EFF* eff, GPIO_TypeDef* GPIO_Pull,uint16_t GPIO_Pin_Pull,/* Reed switch pull */
		GPIO_TypeDef* GPIO_Push,uint16_t GPIO_Pin_Push /* Reed switch push*/)
{
	if((HAL_GPIO_ReadPin(GPIO_Pull, GPIO_Pin_Pull) == 1)
			&& (HAL_GPIO_ReadPin(GPIO_Push, GPIO_Pin_Push) == 0))
	{
		// Pull Reed switch was detect
		eff -> gripper_actual_status = 0b0001;
	}
	else if((HAL_GPIO_ReadPin(GPIO_Pull, GPIO_Pin_Pull) == 0)
			&& (HAL_GPIO_ReadPin(GPIO_Push, GPIO_Pin_Push) == 1))
	{
		// Push Reed switch was detect
		eff -> gripper_actual_status = 0b0010;
	}
	else
	{
		eff -> gripper_actual_status = 0b0000;
	}
}
void Update_endeffector_command(EFF* eff, GPIO_TypeDef* GPIO_Pull, uint16_t GPIO_Pin_Pull,/* Solenoid valve pull */
		GPIO_TypeDef* GPIO_Push, uint16_t GPIO_Pin_Push, /* Solenoid valve push*/
		GPIO_TypeDef* GPIO_Vacuum, uint16_t GPIO_Pin_Vacuum /* Solenoid valve vacuum*/)
{
	if(eff -> gripper_status == 0){
		// Pull gripper
		HAL_GPIO_WritePin(GPIO_Pull, GPIO_Pin_Pull, SET);
		HAL_GPIO_WritePin(GPIO_Push, GPIO_Pin_Push, RESET);
	}
	else if(eff -> gripper_status == 1){
		// Push gripper
		HAL_GPIO_WritePin(GPIO_Pull, GPIO_Pin_Pull, RESET);
		HAL_GPIO_WritePin(GPIO_Push, GPIO_Pin_Push, SET);
	}
	if(eff -> vacuum_status == 0){
		// Deactivate vacuum
		HAL_GPIO_WritePin(GPIO_Vacuum, GPIO_Pin_Vacuum, RESET);
	}
	else if(eff -> vacuum_status == 1){
		// Activate vacuum
		HAL_GPIO_WritePin(GPIO_Vacuum, GPIO_Pin_Vacuum, SET);
	}
}
>>>>>>> Stashed changes
