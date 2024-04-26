/*
 * pid.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "pid.h"

void PID_init(PID* pid, float32_t _kp,  float32_t _ki, float32_t _kd, float32_t _sampt){
	pid -> kp = _kp;
	pid -> ki = _ki;
	pid -> kd = _kd;
	pid -> sampt = _sampt;
}
int32_t Update_pid(PID *pid, float32_t error, int32_t pid_sat, int32_t plant_sat) {
	static float32_t y_n = 0; // Output[n]
	static float32_t y_n_1 = 0; // Output[n-1]
	float32_t e_n = error; // error[n]
	static float32_t e_n_1 = 0; // error[n-1]

	float32_t p_term = e_n * pid -> kp;
	float32_t d_term = ((e_n - e_n_1) * pid -> kd) / pid -> sampt;
	float32_t i_term = y_n_1 + (e_n * pid -> sampt * pid -> ki);
	// Check kp ki kd
	if (pid -> kp == 0){
		p_term = 0;
	}
	else if(pid -> ki == 0){
		i_term = 0;
	}
	else if(pid -> kd == 0){
		d_term = 0;
	}
	y_n = p_term + d_term + i_term; // pid output
	uint8_t is_sat = 0;
	// check is pid output is saturating
	if((int32_t)y_n > pid_sat){
		is_sat = 1;
	}
	else if((int32_t)y_n < -(pid_sat)){
		is_sat = 1;
	}
	// check is error sign and output sign is equal
	if(e_n * y_n == fabs(e_n * y_n)){
		// if pid output is saturating and error sign and output sign is  i_term = 0;
		if(is_sat == 1){
			y_n = p_term + d_term;
		}
	}
	// Plant saturation
	if((int32_t)y_n > plant_sat){
		y_n = plant_sat;
	}
	else if((int32_t)y_n < -(plant_sat)){
		y_n = (-(plant_sat));
	}
	// Update value
	y_n_1 = y_n;
	e_n_1 = e_n;
	return y_n;
}
