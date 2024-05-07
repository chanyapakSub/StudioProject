/*
 * qei.h
 *
 *  Created on: Apr 24, 2024
 *      Author: naker
 */

#ifndef INC_QEI_H_
#define INC_QEI_H_

#include "math.h"
#include "arm_math.h"
#include "main.h"

//VARIABLE SETTING
typedef struct{
	int32_t ppr; // ENCODER PPR
	int32_t freq; // READING FREQUENCY
	int32_t period; // QEI PERIOD
	uint32_t counter_value[2]; // COUNTER VALUE NEW & OLD
	int32_t diff_counter_value; // DIFFERNCE OF NEW & OLD VALUE OF COUNTER
	int64_t pulse; // POSITION AT PULSE UNIT
	float32_t rev; // POSITION AT REVOLUTION UNIT
	float32_t rad; // POSITION AT RADIANT UNIT
	float32_t mm; // POSITION AT MILLIMETER UNIT
	float32_t pps; // POSITION AT PULSE UNIT
	float32_t rpm; // POSITION AT PULSE UNIT
	float32_t radps; // POSITION AT PULSE UNIT
	float32_t mmps; // POSITION AT MILLIMETER UNIT
	float32_t velocity_value[2];
	float32_t diff_velocity_value;
	float32_t ppss;
	float32_t radpss;
	float32_t rpms;
	float32_t mmpss; //AC AT MILLIMETER UNIT
} QEI;
enum{
	NEW,OLD
};

//FUNCTION SETTING
void QEI_init(QEI* qei, int32_t ppr, int32_t freq, int32_t period);
void Update_qei(QEI* qei, TIM_HandleTypeDef* htim);
void Reset_qei(QEI* qei);
float32_t Get_mm(QEI* qei);
float32_t Get_mmps(QEI* qei);

#endif /* INC_QEI_H_ */
