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
	float kp; //P GAIN FOR PID
	float ki; //I GAIN FOR PID
	float kd; //D GAIN FOR PID
	float sampt; //SAMPLING TIME
	float y_n; //Initial output
	float e_n; //Initial error
	float e_n_1;
	float e_n_2;

} PID;

void PID_init(PID* pid, float _kp,  float _ki, float _kd, float _sampt);
float Update_pid(PID *pid, float error, float pid_sat, float plant_sat);
void Reset_pid(PID* pid);
#endif /* INC_PID_H_ */
