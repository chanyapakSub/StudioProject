/*
 * Trajectory.h
 *
 *  Created on: May 1, 2024
 *      Author: chanyapak
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_
#include "main.h"

//Define
float q_d_i = 0;
float q_d_max = 650; //Maximum velocity
float q_2d_max = 700; //Maximum acc
//init
float q_f = 600;
float q_i = 600;
float delta_q = 0;
//output trajectory
float ref_p = 0;
float ref_v = 0;
float ref_a = 0;
//other parameters
uint8_t calmode = 3; //stop mode
float q_f_old = 600;
float t_acc = 0;
float q_acc = 0;
float t_const = 0;
float q_const = 0;
float q_d_acc = 0;
uint64_t T_start = 0;
float t = 0;
int dir = 1;
float total_t = 0;

#endif /* INC_TRAJECTORY_H_ */
