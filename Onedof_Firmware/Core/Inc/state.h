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
	float64_t goal;
} POINT;

typedef struct{

} JOG;

typedef struct{
	uint8_t homing_state[2];	/* Channel 0 is for Homing state was selected
								Channel 1 is for selected state  */
	uint8_t homing_command;
	uint8_t is_home;
	uint8_t homing_first;
	uint8_t state_check;
	uint64_t homing_ts;
	int16_t pwm;

} HOME;

typedef struct{
	uint8_t emer_state;
} EMER;

void point_init(POINT* point);
void run_point(POINT* point);
void homing_init(HOME* home);
void homing(HOME* home, GPIO_TypeDef* GPIO_Prox, uint16_t GPIO_Pin_Prox);
void emer_init(EMER* emer);
void emergency(EMER* emer, GPIO_TypeDef* GPIO_EmerLight, uint16_t GPIO_Pin_EmerLight);
#endif /* INC_STATE_H_ */
