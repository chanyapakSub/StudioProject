/*
 * pid.c
 *
 *  Created on: Apr 26, 2024
 *      Author: naker
 */
#include "pid.h"

void PID_init(PID* pid, float _kp_u,  float _ki_u, float _kd_u,float _kp_d,  float _ki_d, float _kd_d){
	pid -> kp_up = _kp_u;
	pid -> ki_up = _ki_u;
	pid -> kd_up = _kd_u;
	pid -> kp_down = _kp_d;
	pid -> ki_down = _ki_d;
	pid -> kd_down = _kd_d;
	pid -> y_n = 0.0;
	pid -> y_n_1 = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}
double Update_pid(PID *pid, double error, float pid_sat, float plant_sat) {

	float e_n = error; // error[n]

	// For upcase
	if(!(((pid -> y_n >= pid_sat) && e_n > 0) || ((pid -> y_n <= -(pid_sat)) && e_n < 0 ))){
		pid -> y_n += ((pid -> kp_up + pid -> ki_up + pid -> kd_up) * e_n)
						- ((pid -> kp_up + (2 * pid -> kd_up)) * pid -> e_n_1)
						+ (pid -> kd_up * pid -> e_n_2);
	}
	// For down case
//	else if(e_n < 0){
//		if(!(((pid -> y_n >= pid_sat) && e_n > 0) || ((pid -> y_n <= -(pid_sat)) && e_n < 0 ))){
//			pid -> y_n =	pid -> y_n_1
//							+ ((pid -> kp_down + pid -> ki_down + pid -> kd_down) * e_n)
//							- ((pid -> kp_down + (2 * pid -> kd_down)) * pid -> e_n_1)
//							+ (pid -> kd_down * pid -> e_n_2);
//		}
//	}

	if(pid -> y_n >= pid_sat){
		pid -> y_n = pid_sat;

	}else if(pid -> y_n < -pid_sat){
		pid -> y_n = -pid_sat;
	}

	pid -> e_n_2 = pid -> e_n_1;
	pid -> e_n_1 = pid -> e_n;
	pid -> y_n_1 = pid -> y_n;

	return pid -> y_n;
}
void Reset_pid(PID* pid){
	pid -> y_n = 0.0;
	pid -> y_n_1 = 0.0;
	pid -> e_n = 0.0;
	pid -> e_n_1 = 0.0;
	pid -> e_n_2 = 0.0;
}


