/*
 * Trapezoidal.c
 *
 *  Created on: May 16, 2024
 *      Author: naker
 */
#include "Trapezoidal.h"

void trapezoidalGeneration(trapezoidalGen* genTrapezoidal, double initial_position, double target_position,
		double vel_max, double acc_max){
	// Set parameters
	double distance = 0.0; // qf-qi
	double d_acc = 0.0; // acc and dcc distancea
//	double vel_peak = 0.0;
	uint8_t pattern = 0;
	double t = 0.0;
	double vel_check = 0.0;

	genTrapezoidal -> total_time = 0.0;
	genTrapezoidal -> t0 = 0.0;
	genTrapezoidal -> t1 = 0.0;
	genTrapezoidal -> t2 = 0.0;
	genTrapezoidal -> t3 = 0.0;
	genTrapezoidal -> initial_position = initial_position;
	genTrapezoidal -> final_position = target_position;

	// Calculate distance
	distance = fabs((target_position - initial_position));
	d_acc = ((vel_max * vel_max * 0.5) / acc_max);
	// Check case
	t = sqrt(distance / acc_max);
	vel_check = acc_max * t;

	// Direction choose
	if((target_position - initial_position) >= 0){
		// Move down
		genTrapezoidal -> dir = 1;
	}
	else if((target_position - initial_position) < 0){
		// Move up
		genTrapezoidal -> dir = -1;
	}

	// Find trajectory pattern
	if(vel_check >= vel_max){
		// Reach max velocity
		pattern = 1;
	}
	else if(vel_check < vel_max){
		// Not reach max velocity
		pattern = 2;
	}

	switch(pattern){
	case 1:
		genTrapezoidal -> t0 = 0.0;
		genTrapezoidal -> t1 = vel_max / acc_max;
		genTrapezoidal -> t2 = genTrapezoidal -> t1 + ((distance - (2.0 * d_acc)) / vel_max);
		genTrapezoidal -> t3 = genTrapezoidal -> t2 + genTrapezoidal -> t1;
		genTrapezoidal -> total_time = genTrapezoidal -> t3;
		break;
	case 2:
//		vel_peak = sqrt(2 * acc_max * distance);
		genTrapezoidal -> t0 = 0.0;
		genTrapezoidal -> t1 = t;
		genTrapezoidal -> t2 = genTrapezoidal -> t1 + genTrapezoidal -> t1;
		genTrapezoidal -> t3 = genTrapezoidal -> t2;
		genTrapezoidal -> total_time = genTrapezoidal -> t3;
		break;
	default:
		break;
	}
	if(distance == 0){
		genTrapezoidal -> t0 = 0.0;
		genTrapezoidal -> t1 = 0.0;
		genTrapezoidal -> t2 = 0.0;
		genTrapezoidal -> t3 = 0.0;
		genTrapezoidal -> total_time = 0.0;
	}
}

void trapezoidalComputation(trapezoidalCompute* computeTrapezoidal, trapezoidalGen* genTrapezoidal, double vel_max, double acc_max){

	static double p_i, v_i = 0.0;
	static uint8_t pass_1 = 1;
	static uint8_t pass_2 = 0;
	static uint8_t pass_3 = 0;
	computeTrapezoidal -> t += 1.0/1000.0; // plus time every 0.001 sec or 1000 hz
	computeTrapezoidal -> is_finish = 0;
	if(genTrapezoidal -> t2 != genTrapezoidal -> t3){
		if((genTrapezoidal -> t0 <= computeTrapezoidal -> t) && (computeTrapezoidal -> t <= genTrapezoidal -> t1)){
			if(pass_1 == 1){
				p_i = genTrapezoidal -> initial_position;
				v_i = 0.0;
				pass_1 = 0;
				pass_2 = 1;
			}
			computeTrapezoidal -> set_pos = p_i  + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t0)) + ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t0)  * (computeTrapezoidal -> t - genTrapezoidal -> t0)));
			computeTrapezoidal -> set_vel = v_i + (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t0));
		}
		else if((genTrapezoidal -> t1 < computeTrapezoidal -> t) && (computeTrapezoidal -> t <= genTrapezoidal -> t2)){
			if(pass_2 == 1){
				p_i = computeTrapezoidal -> set_pos;
				v_i = computeTrapezoidal -> set_vel;
				pass_2 = 0;
				pass_3 = 1;
			}
			computeTrapezoidal -> set_pos = p_i + ((genTrapezoidal -> dir * vel_max * (computeTrapezoidal -> t - genTrapezoidal -> t1)));
			computeTrapezoidal -> set_vel = vel_max * genTrapezoidal -> dir;
		}
		else if((genTrapezoidal -> t2 < computeTrapezoidal -> t) && (computeTrapezoidal -> t <= genTrapezoidal -> t3)){
			if(pass_3 == 1){
				p_i = computeTrapezoidal -> set_pos;
				v_i = computeTrapezoidal -> set_vel;
				pass_3 = 0;
			}
			computeTrapezoidal -> set_pos = p_i + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t2)) - ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t2)) * (computeTrapezoidal -> t - genTrapezoidal -> t2));
			computeTrapezoidal -> set_vel = v_i - (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t2));
		}
		else if(genTrapezoidal -> t3 < computeTrapezoidal -> t){
			pass_1 = 1;
			pass_2 = 0;
			pass_3 = 0;
			if(computeTrapezoidal -> set_pos > genTrapezoidal -> final_position){
				if(genTrapezoidal -> dir == -1){
					computeTrapezoidal -> set_pos = genTrapezoidal -> final_position;
				}
			}
			if(computeTrapezoidal -> set_pos < genTrapezoidal -> final_position){
				if(genTrapezoidal -> dir == 1){
					computeTrapezoidal -> set_pos = genTrapezoidal -> final_position;
				}
			}
			if(computeTrapezoidal -> set_pos != genTrapezoidal -> final_position){
				computeTrapezoidal -> set_pos = p_i + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t2)) - ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t2)) * (computeTrapezoidal -> t - genTrapezoidal -> t2));
				computeTrapezoidal -> set_vel = v_i - (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t2));
			}
			if(computeTrapezoidal -> set_pos == genTrapezoidal -> final_position) {
				computeTrapezoidal -> set_vel = 0.0;
				computeTrapezoidal -> is_finish = 1;
			}
		}
	}
	else if(genTrapezoidal -> t2 == genTrapezoidal -> t3){
		if((genTrapezoidal -> t0 <= computeTrapezoidal -> t) && (computeTrapezoidal -> t <= genTrapezoidal -> t1)){
			if(pass_1 == 1){
				p_i = genTrapezoidal -> initial_position;
				v_i = 0.0;
				pass_1 = 0;
				pass_2 = 1;
			}
			computeTrapezoidal -> set_pos = p_i + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t0)) + ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t0) * (computeTrapezoidal -> t - genTrapezoidal -> t0)));
			computeTrapezoidal -> set_vel = v_i + (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t0));
		}
		else if((genTrapezoidal -> t1 < computeTrapezoidal -> t) && (computeTrapezoidal -> t <= genTrapezoidal -> t2)){
			if(pass_2 == 1){
				p_i = computeTrapezoidal -> set_pos;
				v_i = computeTrapezoidal -> set_vel;
				pass_2 = 0;
			}
			computeTrapezoidal -> set_pos = p_i + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t1)) - ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t1) * (computeTrapezoidal -> t - genTrapezoidal -> t1)));
			computeTrapezoidal -> set_vel = v_i - (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t1));
		}
		else if(genTrapezoidal -> t2 < computeTrapezoidal -> t){
			pass_1 = 1;
			pass_2 = 0;
			pass_3 = 0;
			if(computeTrapezoidal -> set_pos != genTrapezoidal -> final_position){
				computeTrapezoidal -> set_pos = p_i + (v_i * (computeTrapezoidal -> t - genTrapezoidal -> t1)) - ((genTrapezoidal -> dir * 0.5 * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t1) * (computeTrapezoidal -> t - genTrapezoidal -> t1)));
				computeTrapezoidal -> set_vel = v_i - (genTrapezoidal -> dir * acc_max * (computeTrapezoidal -> t - genTrapezoidal -> t1));
			}
			if(computeTrapezoidal -> set_pos > genTrapezoidal -> final_position){
				if(genTrapezoidal -> dir == -1){
					computeTrapezoidal -> set_pos = genTrapezoidal -> final_position;
				}
			}
			if(computeTrapezoidal -> set_pos < genTrapezoidal -> final_position){
				if(genTrapezoidal -> dir == 1){
					computeTrapezoidal -> set_pos = genTrapezoidal -> final_position;
				}
			}
			if(computeTrapezoidal -> set_pos == genTrapezoidal -> final_position) {
				computeTrapezoidal -> set_vel = 0.0;
				computeTrapezoidal -> is_finish = 1;
			}
		}
	}
}
