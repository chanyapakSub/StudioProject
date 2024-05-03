/*
 * state.c
 *
 *  Created on: May 3, 2024
 *      Author: naker
 */
#include "state.h"

void homing(HOME* home, GPIO_TypeDef* GPIO_Prox, uint16_t GPIO_Pin_Prox, TIM_HandleTypeDef* htim, uint16_t htim_channel
		, GPIO_TypeDef* GPIO_PWM, uint16_t GPIO_Pin_PWM, u16u8_t* registerFrame[200]
		, GPIO_TypeDef* GPIO_HomingButton, uint16_t GPIO_Pin_HomingButton, QEI* encoder)
{
	static uint64_t is_home_ts = 0;
	// Clear is_home variable if is_home_ts = 1 mins is_home will clear
	if(is_home_ts == 480000){
		home -> is_home = 0;
		is_home_ts = 0;
	}
	// Check homing command from Homing button and Base system command
	if(state == 0b0010){
		home -> homing_command = 1;
		registerFrame[0x01] -> U16 = 0b0000;
		registerFrame[0x10] -> U16 = 0b0010;
	}
	else if(HAL_GPIO_ReadPin(GPIO_HomingButton, GPIO_Pin_HomingButton)){
		home -> homing_command = 1;
		registerFrame[0x10] -> U16 = 0b0010;
	}
	// If we have homing command
	else if(home -> homing_command){
		if(home -> homing_state[0] == 0){
			// Homing state selection
			// Robot is actually home
			if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && home -> is_home){
				home -> homing_state[1] = 1;
			}
			// Robot is not home but proximity is detecting
			else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && !(home -> is_home)){
				home -> homing_state[1] = 2;
			}
			// Robot is not home and proximity isn't detecting
			else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 0) && !(home -> is_home)){
				home -> homing_state[1] = 3;
			}
			// Set homing state
			home -> homing_state[0] = 1;
		}
		else if(home -> homing_state[0] == 1){
			static uint64_t homing_ts = 0; // Start homing time stamp
			// Homing
			if(home -> homing_state[1] == 1){
				Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0);
				home -> homing_command = 0;
				home -> homing_state[0] = 0;
				Reset_qei(encoder); // Reset encoder parameter
				home -> is_home = 1;
				registerFrame[0x10] -> U16 = 0b0000;
				return;
			}
			else if(home -> homing_state[1] == 2){
				home -> homing_state[1] = 1;
				if(HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1){
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0); // Stop when proximity was detected
					// Reset homing state and other
					homing_ts = 0;
					home -> homing_state[0] = 0;
					home -> homing_state[1] = 0;
					home -> homing_command = 0;
					Reset_qei(encoder); // Reset encoder parameter
					home -> is_home = 1;
					registerFrame[0x10] -> U16 = 0b0000;
					return;
				}else if(homing_ts == 0){
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 200); // Move upper
				}else if(homing_ts == 4000){ // wait 0.5 secs
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0); // Stop
				}else if(homing_ts == 14000){ // wait 1.25 secs
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -100); // Move lower
				}
				homing_ts++;
			}
			else if(home -> homing_state[1] == 3){
				static uint8_t homing_first = 0;
				if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (homing_first == 1)){
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0); // Stop when proximity was detected
					// Reset homing state and other
					homing_ts = 0;
					homing_first = 0;
					home -> homing_state[0] = 0;
					home -> homing_state[1] = 0;
					home -> homing_command = 0;
					Reset_qei(encoder); // Reset encoder parameter
					home -> is_home = 1;
					registerFrame[0x10] -> U16 = 0b0000;
					return;
				}else if((HAL_GPIO_ReadPin(GPIO_Prox, GPIO_Pin_Prox) == 1) && (homing_first == 0)){
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0); // Stop when proximity was detected
					homing_ts++;
					if(homing_ts == 2000){ // wait 0.25 secs
						Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 200); // Move upper
						homing_first = 1;
					}
				}else if(homing_ts == 6000){ // wait 0.5 secs
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, 0); // Stop
					homing_ts++;
				}else if(homing_ts == 16000){ // wait 1.25 secs
					Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -100); // Move lower
				}else{
					if(homing_first == 1){
						homing_ts++;
					}else if(homing_first == 0){
						homing_ts = 0;
						Update_pwm(htim, htim_channel, GPIO_PWM, GPIO_Pin_PWM, -100); // Move lower
					}
				}
			}
		}
	}
	// Nothing happen
	else{
		is_home_ts++;
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
