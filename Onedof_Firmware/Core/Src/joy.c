/*
 * joy.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#include "joy.h"

void Update_joy(JOY *joy){
	uint8_t s_1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	uint8_t s_2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
	uint8_t s_3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);
	uint8_t s_4 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	if(!s_1 || !s_2 || !s_3 || !s_4){
		static uint8_t is_first = 1;
		static uint64_t timestamp = 0;
		if(is_first){
			timestamp = HAL_GetTick() + 10;
			is_first = 0;
		}
		if (HAL_GetTick() > timestamp){
			timestamp = HAL_GetTick() + 10;
			joy -> s_1 = s_1;
			joy -> s_2 = s_2;
			joy -> s_3 = s_3;
			joy -> s_4 = s_4;
			joy -> is_place = 1;
		}
	}else{
		joy -> s_1 = 1;
		joy -> s_2 = 1;
		joy -> s_3 = 1;
		joy -> s_4 = 1;
		joy -> is_place = 0;
	}
}

