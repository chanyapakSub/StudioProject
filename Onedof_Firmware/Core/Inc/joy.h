/*
 * joy.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef INC_JOY_H_
#define INC_JOY_H_

#include "main.h"
typedef struct{
	uint8_t s_1, s_2, s_3, s_4;
	uint8_t is_place;
	uint16_t shelves_position[5];
}JOY;

void Update_joy(JOY *joy);

#endif /* INC_JOY_H_ */
