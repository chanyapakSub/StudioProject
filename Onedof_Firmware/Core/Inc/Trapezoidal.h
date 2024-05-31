/*
 * Trapezoidal.h
 *
 *  Created on: May 16, 2024
 *      Author: naker
 */

#ifndef INC_TRAPEZOIDAL_H_
#define INC_TRAPEZOIDAL_H_
#include "main.h"
#include "math.h"

typedef struct{
	uint64_t m_t0, m_t1, m_t2, m_t3, m_t_time;
	double t0, t1, t2, t3, t_acc, total_time;
	int8_t dir;
	double initial_position, final_position;
} trapezoidalGen;

typedef struct{
	uint64_t m_t;
	double t;
	uint8_t is_finish;
	double set_vel, set_pos;
} trapezoidalCompute;

void trapezoidalGeneration(trapezoidalGen* genTrapezoidal, double initial_position, double target_position,
		double vel_max, double acc_max);
void trapezoidalComputation(trapezoidalCompute* computeTrapezoidal, trapezoidalGen* genTrapezoidal, double vel_max, double acc_max);

#endif /* INC_TRAPEZOIDAL_H_ */
