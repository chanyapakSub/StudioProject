/*
 * pid.h
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "arm_math.h"

//INTEGRAL TERM IS USE BACKWARD EULER RULE
typedef struct{
	float32_t kp; //P GAIN FOR PID
	float32_t ki; //I GAIN FOR PID
	float32_t kd; //D GAIN FOR PID
	float32_t sampt; //SAMPLING TIME

} PID;

void PID_init(PID* pid, float32_t _kp,  float32_t _ki, float32_t _kd, float32_t _sampt);
int32_t Update_pid(PID *pid, float32_t error, int32_t pid_sat, int32_t plant_sat);
#endif /* INC_PID_H_ */
