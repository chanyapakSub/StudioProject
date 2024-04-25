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
	qei -> rad = 0;
	qei -> rev = 0;
	qei -> pps = 0;
	qei -> rpm = 0;
	qei -> radps =0;
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
	qei -> pps = (float)qei -> diff_counter_value * (float)qei -> freq;
	qei -> radps = qei -> pps * 2.0 * M_PI / (float)(qei -> ppr);
	qei -> rpm = qei -> pps * 60.0 / (float)(qei -> ppr) ;
	qei -> mmps = qei -> pps * 16.0 / (float)(qei -> ppr);
	// Update position at difference unit
	qei -> pulse += qei -> diff_counter_value;
	qei -> rad += (float)qei -> diff_counter_value * 2.0 * M_PI / (float)(qei -> ppr);
	qei -> rev += (float)qei -> diff_counter_value / (float)(qei -> ppr);
	// Update acceleration at difference

	// Update OLD value
	qei -> counter_value[OLD] = qei -> counter_value[NEW];

}

void Reset_qei(QEI* qei){
	qei -> counter_value[NEW] = 0;
	qei -> counter_value[OLD] = 0;
	qei -> diff_counter_value = 0;
	qei -> pulse = 0;
	qei -> rad = 0;
	qei -> rev = 0;
	qei -> pps = 0;
	qei -> rpm = 0;
	qei -> radps =0;
}

