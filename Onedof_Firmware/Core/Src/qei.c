/*
 * qei.c
 *
 *  Created on: Apr 24, 2024
 *      Author: naker
 */
#include "qei.h"

void QEI_init(QEI* qei, int32_t ppr, int32_t freq, int32_t period){
	qei -> ppr = ppr;
	qei -> freq = freq;
	qei -> period = period;
	qei -> counter_value[NEW] = 0;
	qei -> counter_value[OLD] = 0;
	qei -> diff_counter_value = 0;
	qei -> pulse = 0;
	qei -> rad = 0.0;
	qei -> rev = 0.0;
	qei -> mm = 0.0;
	qei -> pps = 0.0;
	qei -> rpm = 0.0;
	qei -> radps = 0.0;
	qei -> mmps = 0.0;
<<<<<<< Updated upstream
=======
	qei -> radpss = 0.0;
	qei -> rpms = 0.0;
	qei -> velocity_value[NEW] = 0;
	qei -> velocity_value[OLD] = 0;
	qei -> ppss = 0.0;
	qei -> mmpss = 0.0;


>>>>>>> Stashed changes
}

void Update_qei(QEI* qei, TIM_HandleTypeDef* htim){
	// Update counter
	qei -> counter_value[NEW] = __HAL_TIM_GET_COUNTER(htim);
	// Update difference of counter's value = NEW - OLD
	qei -> diff_counter_value = qei -> counter_value[NEW] - qei -> counter_value[OLD];
	if(qei -> diff_counter_value > qei -> period / 2){
		qei -> diff_counter_value -= qei -> period;
	}
	// over flow problem
	if(qei -> diff_counter_value < -(qei -> period) / 2){
		qei -> diff_counter_value += qei -> period;
	}
	// Update velocity at difference unit
	qei -> pps = (float)qei -> diff_counter_value * (float)(qei -> freq); // Difference counter * frequency
	qei -> radps = qei -> pps * 2.0 * M_PI / (float)(qei -> ppr);
	qei -> rpm = qei -> pps * 60.0 / (float)(qei -> ppr) ;
	qei -> mmps = qei -> pps * 16.0 / (float)(qei -> ppr);
	// Update position at difference unit
	qei -> pulse += qei -> diff_counter_value;
	qei -> rad += (float)qei -> diff_counter_value * 2.0 * M_PI / (float)(qei -> ppr);
	qei -> rev += (float)qei -> diff_counter_value / (float)(qei -> ppr);
	qei -> mm += (float)qei -> diff_counter_value * 16.0 / (float)(qei -> ppr); // for lead 16 mm.
<<<<<<< Updated upstream
	// Update acceleration at difference
=======

	// Update acceleration at difference unit
	qei -> velocity_value[NEW] = qei -> pps;
	qei -> diff_velocity_value = qei -> velocity_value[NEW] - qei -> velocity_value[OLD];
	qei -> ppss = qei -> diff_velocity_value * (float)(qei -> freq); // Difference of velocity * frequency
	qei -> radpss = qei -> ppss * 2.0 * M_PI / (float)(qei -> ppr);
	qei -> rpms = qei -> ppss * 60.0 / (float)(qei -> ppr);
	qei -> mmpss = qei -> ppss * 16.0 / (float)(qei -> ppr); // Acceleration in mm/s^2
>>>>>>> Stashed changes

	// Update OLD value
	qei -> counter_value[OLD] = qei -> counter_value[NEW];
	qei -> velocity_value[OLD] = qei -> velocity_value[NEW];

}

void Reset_qei(QEI* qei){
	qei -> counter_value[NEW] = 0;
	qei -> counter_value[OLD] = 0;
	qei -> diff_counter_value = 0;
	qei -> pulse = 0;
	qei -> rad = 0.0;
	qei -> rev = 0.0;
	qei -> mm = 0.0;
	qei -> pps = 0.0;
	qei -> rpm = 0.0;
	qei -> radps = 0.0;
	qei -> mmps = 0.0;
	qei -> radpss = 0.0;
	qei -> rpms = 0.0;
	qei -> mmpss = 0.0;
}
float32_t Get_mmpss(QEI* qei){
	return qei -> mmpss;
}

float32_t Get_mmps(QEI* qei){
	return qei -> mmps;
}
float32_t Get_mm(QEI* qei){
	return qei -> mm;
}
