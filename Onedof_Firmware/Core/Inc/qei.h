/*
 * qei.h
 *
 *  Created on: Apr 24, 2024
 *      Author: naker
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "math.h"
#include "main.h"

//VARIABLE SETTING
typedef struct{
	int32_t ppr; // ENCODER PPR
	int32_t freq; // READING FREQUENCY
	int32_t period; // QEI PERIOD
	uint32_t counter_value[2]; // COUNTER VALUE NEW & OLD
	int32_t diff_counter_value; // DIFFERNCE OF NEW & OLD VALUE OF COUNTER
	int64_t pulse; // POSITION AT PULSE UNIT
	float rev; // POSITION AT REVOLUTION UNIT
	float rad; // POSITION AT RADIANT UNIT
	float pps; // POSITION AT PULSE UNIT
	float rpm; // POSITION AT PULSE UNIT
	float radps; // POSITION AT PULSE UNIT
	float mmps; // POSITION AT MILLIMETER UNIT
} QEI;
enum{
	NEW,OLD
};

//FUNCTION SETTING
void QEI_init(QEI* qei, int32_t ppr, int32_t freq, int32_t period);
void Update_qei(QEI* qei, TIM_HandleTypeDef* htim);
void Reset_qei(QEI* qei);

#endif /* INC_QEI_H_ */
