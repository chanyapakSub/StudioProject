/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PWM_signal_Pin GPIO_PIN_0
#define PWM_signal_GPIO_Port GPIOC
#define Direaction_motor_Pin GPIO_PIN_1
#define Direaction_motor_GPIO_Port GPIOC
#define Current_sensor_Pin GPIO_PIN_0
#define Current_sensor_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define joy_switch_3_Pin GPIO_PIN_4
#define joy_switch_3_GPIO_Port GPIOC
#define joy_switch_4_Pin GPIO_PIN_5
#define joy_switch_4_GPIO_Port GPIOC
#define Pilot_controller_error_Pin GPIO_PIN_1
#define Pilot_controller_error_GPIO_Port GPIOB
#define Pilot_motor_error_Pin GPIO_PIN_2
#define Pilot_motor_error_GPIO_Port GPIOB
#define Solenoid_valve_pull_Pin GPIO_PIN_10
#define Solenoid_valve_pull_GPIO_Port GPIOB
#define Proximity_Pin GPIO_PIN_12
#define Proximity_GPIO_Port GPIOB
#define Proximity_EXTI_IRQn EXTI15_10_IRQn
#define Emergency_switch_Pin GPIO_PIN_15
#define Emergency_switch_GPIO_Port GPIOB
#define Emergency_switch_EXTI_IRQn EXTI15_10_IRQn
#define Set_home_Pin GPIO_PIN_6
#define Set_home_GPIO_Port GPIOC
#define Reed_switch_pull_Pin GPIO_PIN_7
#define Reed_switch_pull_GPIO_Port GPIOC
#define Emergency_light_Pin GPIO_PIN_8
#define Emergency_light_GPIO_Port GPIOC
#define Solenoid_valve_push_Pin GPIO_PIN_8
#define Solenoid_valve_push_GPIO_Port GPIOA
#define Reed_switch_push_Pin GPIO_PIN_9
#define Reed_switch_push_GPIO_Port GPIOA
#define joy_switch_2_Pin GPIO_PIN_10
#define joy_switch_2_GPIO_Port GPIOA
#define Encoder_A_Pin GPIO_PIN_11
#define Encoder_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_12
#define Encoder_B_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define Solenoid_valve_vacuum_Pin GPIO_PIN_4
#define Solenoid_valve_vacuum_GPIO_Port GPIOB
#define joy_switch_1_Pin GPIO_PIN_5
#define joy_switch_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
