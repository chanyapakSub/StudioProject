/*
 * pidNing.h
 *
 *  Created on: May 6, 2024
 *      Author: chanyapak
 */

#ifndef INC_PIDNING_H_
#define INC_PIDNING_H_


#include "main.h"
#include "arm_math.h"
typedef struct {

	/* Controller gains */
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;

	/* Derivative low-pass filter time constant */
	float32_t tau;

	/* Output limits */
	float32_t limMin;
	float32_t limMax;

	/* Integrator limits */
	float32_t limMinInt;
	float32_t limMaxInt;

	/* Sample time (in seconds) */
	float32_t T;

	/* Controller "memory" */
	float32_t integrator;
	float32_t prevError;			/* Required for integrator */
	float32_t differentiator;
	float32_t prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float32_t sampt; //SAMPLING TIME
	float32_t out;

} PIDController;

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif


