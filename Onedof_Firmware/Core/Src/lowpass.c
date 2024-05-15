/*
 * lowpass.c
 *
 *  Created on: May 7, 2024
 *      Author: naker
 */
#include "lowpass.h"
void Update_lowpass(LOWPASS* lowpass, float input){
	float x_n = input;
	static float x_n_1 = 0.0;
	static float y_n_1 = 0.0;

	if(input == 0.0){
		lowpass -> filtered_data = 0.0;
	}
	else{
		lowpass -> filtered_data = 0.969 * y_n_1 + 0.0155 * x_n + 0.0155 * x_n_1;
	}
	x_n_1 = x_n;
	y_n_1 = lowpass -> filtered_data;

}
void Reset_lowpass(LOWPASS* lowpass){
	lowpass -> filtered_data = 0.0;
}
