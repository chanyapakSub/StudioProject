/*
 * lowpass.h
 *
 *  Created on: May 7, 2024
 *      Author: naker
 */

#ifndef INC_LOWPASS_H_
#define INC_LOWPASS_H_
#include "main.h"
#include "arm_math.h"

typedef struct{
	float filtered_data;
} LOWPASS;

void Update_lowpass(LOWPASS* lowpass, float input);
void Reset_lowpass(LOWPASS* lowpass);
#endif /* INC_LOWPASS_H_ */
