/*
 * pid.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "pid.h"

void PID_init(PID* pid, float _kp,  float _ki, float _kd, float _sampt){
	pid -> kp = _kp;
	pid -> ki = _ki;
	pid -> kd = _kd;
	pid -> sampt = _sampt;
	pid -> y_n = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}
float Update_pid(PID *pid, float error, float pid_sat, float plant_sat) {
//	static float e_n_1 = 0.0; // error[n-1]
//	static float p_term = 0.0; // p_n
//	static float i_term = 0.0; // i_n
//	static float d_term = 0.0; // d_n
//	static float i_term_1 = 0.0; // i_n_1
//	static float d_term_1 = 0.0; // d_n_1
//
//	p_term = e_n * pid -> kp;
//	i_term = ((pid -> ki * pid -> sampt * (e_n + e_n_1)) * 0.5) + (i_term_1);
//	d_term = (((e_n - e_n_1) * pid -> kd * 2.0) / pid -> sampt) - (d_term_1);
//
//	if(pid -> ki == 0){
//		i_term = 0.0;
//	}
//	if(pid -> kd == 0){
//		d_term = 0.0;
//	}
//
//	pid -> y_n = p_term + d_term + i_term; // pid output
//
//	// Clamp I term implement from MATLAB
//	uint8_t is_sat = 0;
//	// check is pid output is saturating
//	if(pid -> y_n > pid_sat){
//		is_sat = 1;
//	}
//	else if(pid -> y_n < -(pid_sat)){
//		is_sat = 1;
//	}
//	// check is error sign and output sign is equal
//	if(e_n * pid -> y_n == fabs(e_n * pid -> y_n)){
//		// if pid output is saturating and error sign and output sign is i_term = 0;
//		if(is_sat == 1){
//			pid -> y_n = pid -> y_n - i_term;
//		}
//	}
//
//	// Plant saturation
//	if(pid -> y_n > plant_sat){
//		pid -> y_n = plant_sat;
//	}
//	else if(pid -> y_n < -(plant_sat)){
//		pid -> y_n = (-(plant_sat));
//	}
//
//	// Update value
//	i_term_1 = i_term;
//	d_term_1 = d_term;
//	e_n_1 = e_n;

	float e_n = error; // error[n]

	if(!(((pid -> y_n >= pid_sat) && e_n > 0) || ((pid -> y_n <= -(pid_sat)) && e_n < 0 ))){
		pid -> y_n += ((pid -> kp + pid -> ki + pid -> kd) * e_n)
						- ((pid -> kp + (2 * pid -> kd)) * pid -> e_n_1)
						+ (pid -> kd * pid -> e_n_2);
	}
	if(pid -> y_n >= pid_sat){
		pid -> y_n = pid_sat;

	}else if(pid -> y_n < -pid_sat){
		pid -> y_n = -pid_sat;
	}

	pid -> e_n_2 = pid -> e_n_1;
	pid -> e_n_1 = pid -> e_n;

	return pid -> y_n;
}
void Reset_pid(PID* pid){
	pid -> kp = 0.0;
	pid -> ki = 0.0;
	pid -> kd = 0.0;
	pid -> y_n = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}


