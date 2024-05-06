/*
 * Trajectory.c
 *
 *  Created on: May 1, 2024
 *      Author: chanyapak
 */

#ifndef SRC_TRAJECTORY_C_
#define SRC_TRAJECTORY_C_
#include "Trajectory.h"


void checkStartMoving(){
	if(q_f != q_f_old){
		q_i = q_f_old;   //Define q_i
		delta_q = q_f - q_i;
		if(delta_q < 0){
			dir = -1;
		}
		else{
			dir = 1;
		}

		float q_check = fabs(q_d_i + q_2d_max*sqrt(fabs(delta_q)/q_2d_max)*dir);

		if(q_check >= q_d_max){ //Accelerate to maximum speed
			calmode = 1;
			t_acc = fabs((q_d_max-q_d_i)/q_2d_max);
			q_acc = q_d_i*t_acc*dir + 0.5*q_2d_max*t_acc*t_acc*dir;
			t_const = fabs((q_f-q_i-2*q_acc)/q_d_max);
			q_const = q_acc + q_d_max*t_const*dir;
			total_t = 2*t_acc + t_const;
		}
		else{ //Accelerating does not reach maximum speed
			calmode = 2;
			t_acc = sqrt(fabs(delta_q)/q_2d_max);
			q_acc = q_d_i*t_acc*dir + 0.5*q_2d_max*t_acc*t_acc*dir;
			q_d_acc = q_d_i + q_2d_max*t_acc*dir;
			total_t = 2*t_acc;
		}
		T_start = micros();
		q_f_old = q_f;
	}
}

void createTrajectory(){
	if(calmode == 1){
		t = (float)(micros()-T_start)/1000000; //sec
		if((0 <= t) && (t < t_acc)){ //Acceleration Segment
			ref_p = q_i + q_d_i*t*dir + 0.5*q_2d_max*t*t*dir;
			ref_v = q_d_i + q_2d_max*t*dir;
			ref_a = q_2d_max*dir;
		}
		else if((t_acc <= t) && (t < t_acc+t_const)){ //Constant Velocity Segment
			ref_p = q_i + q_acc + q_d_max*(t-t_acc)*dir;
			ref_v = q_d_max*dir;
			ref_a = 0;
		}
		else if((t_acc+t_const <= t) && (t < 2*t_acc+t_const)){ //Deceleration Segment
			ref_p = q_i + q_const + q_d_max*(t-t_acc-t_const)*dir-0.5*q_2d_max*(t-t_acc-t_const)*(t-t_acc-t_const)*dir;
			ref_v = -q_2d_max*(t-t_acc-t_const)*dir + q_d_max*dir;
			ref_a = -q_2d_max*dir;
		}
		else{
			calmode = 3;
		}
	}
	else if(calmode == 2){
		t = (float)(micros()-T_start)/1000000; //sec
		if((0 <= t) && (t < t_acc)){ //Acceleration Segment
			ref_p = q_i + q_d_i*t*dir + 0.5*q_2d_max*t*t*dir;
			ref_v = q_d_i + q_2d_max*t*dir;
			ref_a = q_2d_max*dir;
		}
		else if((t_acc <= t) && (t < t_acc*2)){
			ref_p = q_i + q_acc + q_d_acc*(t-t_acc) - 0.5*q_2d_max*(t-t_acc)*(t-t_acc)*dir;
			ref_v = q_d_acc - q_2d_max*(t-t_acc)*dir;
			ref_a = -q_2d_max*dir;
		}
		else{
			calmode = 3;
		}
	}
	else if(calmode == 3){ //stop
		ref_p = q_f;
		ref_v = 0;
		ref_a = 0;
	}
}


#endif /* SRC_TRAJECTORY_C_ */
