/*
 * state.c
 *
 *  Created on: May 3, 2024
 *      Author: naker
 */
#include "state.h"

void point_init(POINT* point){
	point -> goal = 0.0;
}



void homing_init(HOME* home){
	home -> homing_command = 0;
	home -> homing_state[0] = 0;
	home -> homing_state[1] = 0;
	home -> is_home = 0;
	home -> homing_first = 0;
	home -> homing_sec = 0;
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
				home -> homing_state[1] = 1;
			}
			// Robot is not home and proximity isn't detecting
			else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && home -> is_home == 0){
				home -> homing_state[1] = 2;
			}
			if(home -> homing_state[1] != 0){
				// Set homing state
				home -> homing_state[0] = 1;
			}
		}
		if(home -> homing_state[0] == 1){
			// Homing
			if(home -> homing_state[1] == 1){
//				home -> state_check += 10;
				if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_first == 1 && home -> homing_sec == 1){
					if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0){
						// Check Proximity again
						home -> pwm = 7000;
						// Reset homing state and other
						home -> homing_ts = 0;
						home -> homing_state[0] = 0;
						home -> homing_state[1] = 0;
						home -> homing_command = 0;
						home -> homing_first = 0;
						home -> homing_sec = 0;
						home -> is_home = 1;
						return;
					}
				}
				else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1 && home -> homing_first == 1 && home -> homing_sec == 0 && home -> homing_ts >= 1500){
					// Stop when proximity was detected
					home -> pwm = 14000;
					home -> homing_sec = 1;
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1 && home -> homing_first == 0 && home -> homing_ts >= 0 && home -> homing_ts < 1000){
					// Move upper
					home -> pwm = 14000;
					home -> homing_first = 1;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 200);
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_ts >= 1000 && home -> homing_ts < 1500){ // wait 1.0 secs
					// Stop
					home -> pwm = 7000;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
				}else if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_ts >= 1500){ // wait 1.25 secs
					// Move lower
					home -> pwm = -6500;
//					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -120);
				}
				home -> homing_ts++;
			}
			else if(home -> homing_state[1] == 2){
				if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0 && home -> homing_first == 1 && home -> homing_sec == 1){
					if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0){
						// Check Proximity again
						home -> pwm = 7000;
						// Reset homing state and other
						home -> homing_ts = 0;
						home -> homing_state[0] = 0;
						home -> homing_state[1] = 0;
						home -> homing_command = 0;
						home -> homing_first = 0;
						home -> is_home = 1;
						return;
					}
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (home -> homing_first == 1) && (home -> homing_sec == 0)){
					// Stop when proximity was detected
					static uint8_t for_one = 1;
					if(home -> homing_sec == 0 && for_one == 1){
						home -> pwm = 7000;
						home -> homing_ts = 0;
						for_one = 0;
					}
					if(home -> homing_ts >= 1000){
						// Stop for 1 sec then move upper
						home -> pwm = 14000;
						home -> homing_sec = 1;
						for_one = 1;
					}
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (home -> homing_first == 0) && (home -> homing_sec == 0)){
					// Stop when proximity was detected
					if(home -> homing_ts == 0){
						home -> pwm = 7000;
					}
					if(home -> homing_ts >= 1000){ // stop 1 secs
						 // Move upper
						home -> pwm = 14000;
					}
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 0) && (home -> homing_sec == 0) && (home -> homing_ts >= 1000)){
					home -> homing_first = 1;
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 1) && (home -> homing_sec == 0) && home -> homing_ts >= 1500 && home -> homing_ts < 2500){
					// Move upper for 0.5 sec then stop
					home -> pwm = 7000;
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 1) && (home -> homing_sec == 0) && home -> homing_ts >= 2500){ // wait 1.25 secs
					// After stop for 1 sec Move lower
					home -> pwm = -6500;
				}
				else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && (home -> homing_first == 0) && (home -> homing_sec == 0) && home -> homing_ts == 0){
					home -> pwm = -6500;
					return;
				}
				home -> homing_ts++;
			}
		}
	}
	// Nothing happen
	else{
		return;
	}
}
void Reset_homing(HOME* home){
	home -> homing_state[0] = 0;
	home -> homing_state[1] = 0;
	home -> homing_command = 0;
	home -> homing_first = 0;
	home -> homing_sec = 0;
	home -> homing_ts = 0;
	home -> is_home = 0;
	home -> pwm = 0;
	home -> state_check = 0;
}

void emer_init(EMER* emer){
	emer -> emer_state = 0;
}

void emergency(EMER* emer, GPIO_TypeDef* GPIO_EmerLight, uint16_t GPIO_Pin_EmerLight){
	if(emer -> emer_state == 1){
		HAL_GPIO_WritePin(GPIO_EmerLight, GPIO_Pin_EmerLight, SET);
	}else{
		HAL_GPIO_WritePin(GPIO_EmerLight, GPIO_Pin_EmerLight, RESET);
	}
	return;
}
