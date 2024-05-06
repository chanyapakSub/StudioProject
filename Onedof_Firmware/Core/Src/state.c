/*
 * state.c
 *
 *  Created on: May 3, 2024
 *      Author: naker
 */
#include "state.h"

void point_init(POINT* point){
	point -> goal = 0;
}



void homing_init(HOME* home){
	home -> homing_command = 0;
	home -> homing_state[0] = 0;
	home -> homing_state[1] = 0;
	home -> is_home = 0;
	home -> homing_first = 0;
	home -> homing_ts = 0;
	home -> pwm = 0;
}
void homing(HOME* home, GPIO_TypeDef* GPIO_Prox, uint16_t GPIO_Pin_Prox)
{
	// If we have homing command
	if(home -> homing_command == 1){
		if(home -> homing_state[0] == 0){
			// Homing state selection
			// Robot is not home but proximity is detecting
			if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && home -> is_home == 0){
				home -> homing_state[1] = 2;
			}
			// Robot is not home and proximity isn't detecting
			else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && home -> is_home == 0){
				home -> homing_state[1] = 3;
			}
			// Set homing state
			home -> homing_state[0] = 1;
		}
		if(home -> homing_state[0] == 1){
			// Homing
			if(home -> homing_state[1] == 2){
//				home -> state_check += 10;
				if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1 && home -> homing_ts > 14000){
					// Stop when proximity was detected
					home -> pwm = 0;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
					// Reset homing state and other
					home -> homing_ts = 0;
					home -> homing_state[0] = 0;
					home -> homing_state[1] = 0;
					home -> homing_command = 0;
					home -> is_home = 1;
					return;
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1 && home -> homing_ts >= 0 && home -> homing_ts < 4000){
					// Move upper
					home -> pwm = 200;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 200);
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_ts >= 4000 && home -> homing_ts < 14000){ // wait 0.5 secs
					// Stop
					home -> pwm = 0;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_ts >= 14000){ // wait 1.25 secs
					// Move lower
					home -> pwm = -120;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -120);
				}
				home -> homing_ts++;
			}
			else if(home -> homing_state[1] == 3){
				if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (home -> homing_first == 1)){
					// Stop when proximity was detected
					home -> pwm = 0;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
					// Reset homing state and other
					home -> homing_ts = 0;
					home -> homing_first = 0;
					home -> homing_state[0] = 0;
					home -> homing_state[1] = 0;
					home -> homing_command = 0;
					home -> is_home = 1;
					return;
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (home -> homing_first == 0)){
					// Stop when proximity was detected
					if(home -> homing_ts == 0){
						home -> pwm = 0;
//						Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
					}
					home -> homing_ts++;
					if(home -> homing_ts >= 2000 && home -> homing_ts < 6000){ // wait 0.25 secs
						 // Move upper
						home -> pwm = 200;
//						Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 200);
					}
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 1) && home -> homing_ts >= 6000 && home -> homing_ts < 16000){ // wait 0.5 secs
					// Stop
					home -> pwm = 0;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
					home -> homing_ts++;
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 1) && home -> homing_ts >= 16000){ // wait 1.25 secs
					// Move lower
					home -> pwm = -120;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -120);
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 1)){
					home -> homing_ts++;
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 0) && home -> homing_ts == 0){
					home -> pwm = -120;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -120); // Move lower
				}else{
					home -> homing_first = 1;
				}
			}
		}
	}
	// Nothing happen
	else{
		return;
	}
}

void emergency(EMER* emer, GPIO_TypeDef* GPIO_EmerLight, uint16_t GPIO_Pin_EmerLight){
	if(emer -> emer_state == 1){
		HAL_GPIO_WritePin(GPIO_EmerLight, GPIO_Pin_EmerLight, SET);
	}else{
		HAL_GPIO_WritePin(GPIO_EmerLight, GPIO_Pin_EmerLight, RESET);
	}
	return;
}
