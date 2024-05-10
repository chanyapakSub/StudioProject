/*
 * lowpass.c
 *
 *  Created on: May 7, 2024
 *      Author: naker
 */
#include "lowpass.h"
void Update_lowpass(LOWPASS* lowpass, float32_t input){
	float32_t x_n = input;
	static float32_t x_n_1 = 0.0;
	static float32_t y_n_1 = 0.0;

	lowpass -> filtered_data = 0.969 * y_n_1 + 0.0155 * x_n + 0.0155 * x_n_1;

	x_n_1 = x_n;
	y_n_1 = lowpass -> filtered_data;

}
