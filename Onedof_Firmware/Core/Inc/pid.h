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
	float kp_up; //P GAIN FOR PID UP CASE
	float ki_up; //I GAIN FOR PID UP CASE
	float kd_up; //D GAIN FOR PID UP CASE
	float kp_down; //P GAIN FOR PID DOWN CASE
	float ki_down; //I GAIN FOR PID DOWN CASE
	float kd_down; //D GAIN FOR PID DOWN CASE
	double y_n; //Initial output
	double y_n_1;
	double e_n; //Initial error
	double e_n_1;
	double e_n_2;

} PID;

void PID_init(PID* pid, float _kp_u,  float _ki_u, float _kd_u,float _kp_d,  float _ki_d, float _kd_d);
double Update_pid(PID *pid, double error, float pid_sat, float plant_sat);
void Reset_pid(PID* pid);
#endif /* INC_PID_H_ */
