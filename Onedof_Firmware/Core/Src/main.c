/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "pid.h"
#include "qei.h"
#include "lowpass.h"
#include "pwm.h"
#include "adc.h"
#include "joy.h"
#include "eff.h"
#include "ModBusRTU.h"
#include "Basesystem.h"
#include "state.h"
#include "Scurve.h"
#include "Trapezoidal.h"
#include "kalman_saifah.h"
#include "kalman.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define encoder_tim				htim4
#define encoder_channel			TIM_CHANNEL_ALL
#define encoder_ppr				((uint16_t)8192)
#define encoder_frequency		((uint16_t)1000)
#define encoder_cnt_period		((uint32_t)65536)

#define	proximity_gpio			GPIOB
#define proximity_pin			GPIO_PIN_12

#define reed_push_gpio			GPIOA
#define reed_push_pin			GPIO_PIN_9
#define reed_pull_gpio			GPIOC
#define reed_pull_pin			GPIO_PIN_7
#define solenoid_push_gpio		GPIOA
#define solenoid_push_pin		GPIO_PIN_8
#define solenoid_pull_gpio		GPIOB
#define solenoid_pull_pin		GPIO_PIN_10
#define vacuum_gpio				GPIOB
#define vacuum_pin				GPIO_PIN_4

#define current_gpio			GPIOA
#define current_pin				GPIO_PIN_0
#define current_adc				hadc1

#define pwm_tim					htim1
#define pwm_channel				TIM_CHANNEL_1
#define dir_gpio				GPIOC
#define dir_pin					GPIO_PIN_1

#define controller_error_gpio	GPIOB
#define controller_error_pin	GPIO_PIN_1
#define motor_error_gpio		GPIOB
#define motor_error_pin			GPIO_PIN_2
#define emer_light_gpio			GPIOC
#define emer_light_pin			GPIO_PIN_8
#define emer_gpio				GPIOB
#define emer_pin				GPIO_PIN_15
#define home_gpio				GPIOC
#define home_pin				GPIO_PIN_6

#define sw_1_gpio				GPIOB
#define sw_1_pin				GPIO_PIN_5
#define sw_2_gpio				GPIOA
#define sw_2_pin				GPIO_PIN_10
#define sw_3_gpio				GPIOC
#define sw_3_pin				GPIO_PIN_4
#define sw_4_gpio				GPIOC
#define sw_4_pin				GPIO_PIN_5

#define main_loop_tim			htim3
#define modbus_tim				htim5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

// Temporary variable
uint64_t repeat_cheack = 0;
uint32_t limitswitch_test = 0;
float64_t test = 0.0;
float min_error = 1000;
uint64_t pid_time[2] = {0};
uint64_t sensor[6] = {0};
uint8_t first_check = 1;


// Set point
double setpoint = 0.0;

// State
enum{
	WAIT, RUNNING, HOMING, EMERGENCY
};
uint8_t jog_status[2] = {0}; // For jog mode only 0 for pick 1 for place
uint16_t mode = WAIT; /* For select mode from base system mode == 0 for stand by ,mode == 1 run point mode,
 	 	 	 	 	 mode == 2 run set shelve ,mode == 3 run jog mode(Full automatic pick & place)*/
uint8_t ready = 0;
POINT point;
HOME home;
EMER emer;
uint8_t testing = 0; // Testing mode or Not testing mode
uint64_t skip = 0;
uint8_t is_finish_pick = 0;
uint8_t is_finish_place = 0;
uint8_t pick_index = 0;
uint8_t place_index = 0;

// Modbus
u16u8_t registerFrame[200] = {0};

// End effector
EFF eff;

// Joy
JOY joy;
int32_t jog = 0;

// Current sensor
ADC current_sensor;

// Update_motor variables
int32_t pwm_signal = 0;

// QEI variables
uint8_t is_update_encoder = 0;
QEI encoder;

// Low pass variable
LOWPASS lowpass;

// Torque pid
//PID t_pid;

// Velocity pid
PID v_pid;
float v_kp_u = 0.00435;
float v_ki_u = 0.00015;
float v_kd_u = 0.0;
float v_kp_d = 0.00435; // 15
float v_ki_d = 0.00015; //3
float v_kd_d = 0.0;
double v_e = 0.0;
float v_output = 0;

// Position pid
PID p_pid;
float p_kp_u = 0.00075;  //0.1
float p_ki_u = 0.00006;	// 0.07
float p_kd_u = 0.0;
float p_kp_d = 0.00075; //0.1
float p_ki_d = 0.00006; // 0.07
float p_kd_d = 0.0;
double p_e = 0.0;
double p_output = 0;
uint8_t start_position_control = 0;
uint8_t is_finish_position = 0;

// Kalman filter
Kalman kalman;
float kalman_velocity = 0.0;
float kalman_velocity_z = 0.0;

// Kalman Saifa

//float32_t dt = 0.001; // 1,000 Hz
//float32_t A_data[9] = {1, 0.001, 0.0000005, 0, 1, 0.001, 0, 0, 1};
//float32_t B_data[3] = {0, 0, 0};
//float32_t C_data[3] = {1, 0, 0};
//float32_t G_data[3] = {0.0000001667, 0.0000005, 0.001};
//float32_t Q_data[1] = {1}; //3
//float32_t R_data[1] = {9}; //7
//float32_t P_data[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
//float32_t x_data[3] = {0, 0, 0};
//float32_t K_data[3]; // Kalman gain data
//float32_t S_data[1]; // Innovation covariance data
//float32_t temp1_data[9], temp2_data[9], temp3_data[3], temp4_data[1], temp5_data[3], temp6_data[9];
//
//float32_t estimated_state[3];
//
//float32_t u_data[1] = {0};
//float32_t w_data[1] = {0.207}; // Example noise, should be random in practice
//
//KalmanFilter saifah;

//struct robot_data{
//	float z_axis_position;
//	float x_axis_position;
//};

float x_axis_position = 0.0;

// Trajectory
volatile Scurve_GenStruct genScurveData;
volatile Scurve_EvaStruct evaScurveData;
trapezoidalGen genTrapezoidalData;
trapezoidalCompute computeTrapezoidalData;
double initial_position = 0.0;
double target_position = 0.0;
double max_velocity = 650.0;
double max_acceleration = 1000.0;
double max_jerk = 3500.0;
double setpoint_pos = 0.0;
double setpoint_vel = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Update_torque_control(double s);
void Update_velocity_control(double s);
void Update_position_control(double s);
void Reset_main_variable();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Modbus setting
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize = 200;
  Modbus_init(&hmodbus, registerFrame);
  registerFrame[0x00].U16 = 22881; // Set default heart beat to "Ya"

  // Update command timer
  HAL_TIM_Base_Start_IT(&main_loop_tim);

  // PWM generator
  HAL_TIM_Base_Start(&pwm_tim);
  HAL_TIM_PWM_Start(&pwm_tim, pwm_channel);

  // Encoder reader
  HAL_TIM_Encoder_Start(&encoder_tim, encoder_channel);
  QEI_init(&encoder, encoder_ppr, encoder_frequency, encoder_cnt_period);

  // Current reader
  ADC_init(&current_adc, &current_sensor);

  // Kalman filter
//  KalmanFilter_Init(&saifah, A_data, B_data, C_data, G_data, Q_data, R_data, P_data, x_data
//		  , K_data, temp1_data, temp2_data, temp3_data, temp4_data, temp5_data, temp6_data, S_data);
  kalman_init(&kalman);


  // State initialize
  homing_init(&home);
  point_init(&point);
  emer_init(&emer);


  // PID initialize
  PID_init(&p_pid, p_kp_u, p_ki_u, p_kd_u, p_kp_d, p_ki_d, p_kd_d);
  PID_init(&v_pid, v_kp_u, v_ki_u, v_kd_u, v_kp_d, v_ki_d, v_kd_d);

  //Set point
  setpoint = 0.0;

  HAL_GPIO_WritePin(emer_light_gpio, emer_light_pin, RESET);
  HAL_GPIO_WritePin(vacuum_gpio, vacuum_pin, RESET);
  HAL_GPIO_WritePin(solenoid_pull_gpio, solenoid_pull_pin, RESET);
  HAL_GPIO_WritePin(solenoid_push_gpio, solenoid_push_pin, RESET);
  HAL_GPIO_WritePin(controller_error_gpio, controller_error_pin, RESET);
  HAL_GPIO_WritePin(motor_error_gpio, motor_error_pin, RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Update modbus
		Modbus_Protocal_Worker();
		// Update reed switch
		Update_actual_eff(&eff, reed_pull_gpio, reed_pull_pin, reed_push_gpio, reed_push_pin);
		if(HAL_GPIO_ReadPin(emer_gpio, emer_pin) == 0){
			static uint64_t timestamp = 0;
			static uint8_t first = 1;
			if(first){
				timestamp = HAL_GetTick() + 800;
				first = 0;
			}
			if(HAL_GetTick() > timestamp){
				if(HAL_GPIO_ReadPin(emer_gpio, emer_pin) == 0){
					if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
						static uint64_t timestamp_in = 0;
						static uint8_t first_in = 1;
						if(first_in){
							timestamp = HAL_GetTick() + 800;
							first_in = 0;
						}
						if(HAL_GetTick() > timestamp_in){
							Reset_main_variable();
							// Stop motor
							Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, 0);
							// Emergency light enable
							HAL_GPIO_WritePin(emer_light_gpio, emer_light_pin, SET);
							// Deactivate end effector
							eff.solenoid_command[0] = 0;
							eff.solenoid_command[1] = 0;
							eff.solenoid_command[2] = 0;
							Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
							mode = EMERGENCY;
						}
						first_in = 1;
						first = 1;
					}
				}
			}
		}
//		if(HAL_GPIO_ReadPin(emer_gpio, emer_pin) == 0){
//			static uint64_t timestamp = 0;
//			static uint8_t first = 1;
//			if(first == 1){
//				timestamp = HAL_GetTick() + 1000; // Delay time in ms
//				first = 0;
//			}
//			if(HAL_GetTick() > timestamp){
//				if(HAL_GPIO_ReadPin(emer_gpio, emer_pin) == 0){
//					Reset_main_variable();
//					// Stop motor
//					Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, 0);
//					// Emergency light enable
//					HAL_GPIO_WritePin(emer_light_gpio, emer_light_pin, SET);
//					// Deactivate end effector
//					eff.solenoid_command[0] = 0;
//					eff.solenoid_command[1] = 0;
//					eff.solenoid_command[2] = 0;
//					Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
//					mode = EMERGENCY;
//				}
//				first = 1;
//			}
//		}
		if(mode == WAIT){

			// Update peripheral
			Vacuum_Status(&eff); // Vacuum
			Gripper_Movement_Status(&eff); // End effector
			// End effector update
			Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);

			//  homing command from Homing button and Base system Check command
			if(home.homing_command == 0){
				Set_Home(); // Refresh homing command from base system
				if(state == 0b0010){
					// Have command from base system
					home.homing_command = 1;
					registerFrame[0x01].U16 = 0b0000; // Reset data of base system status
					registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
					state = registerFrame[0x10].U16;
					mode = HOMING;
				}
				else if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
					static uint64_t timestamp = 0;
					static uint8_t first = 1;
					if(first){
						timestamp = HAL_GetTick() + 100; // Delay time in ms
						first = 0;
					}
					if(HAL_GetTick() > timestamp){
						if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
							// Have command from home switch
							home.homing_command = 1;
							registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
							state = registerFrame[0x10].U16;
							mode = HOMING;
//							timestamp = HAL_GetTick() + 500;
						}
						first = 1;
					}
				}
			}
			else{
				// Nothing happen
				pwm_signal = 0;
				Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal); // Update main PWM signal
			}
		}
		if(mode == RUNNING){
			if(testing == 1){
				Update_joy(&joy);
				Update_qei(&encoder, &htim4);
				Update_lowpass(&lowpass, encoder.mmps);
				sensor[0] = __HAL_TIM_GET_COUNTER(&encoder_tim); // Encoder
				sensor[1] = HAL_GPIO_ReadPin(proximity_gpio, proximity_pin); // Proximity
				sensor[2] = HAL_GPIO_ReadPin(reed_pull_gpio, reed_pull_pin); // Reed switch pull
				sensor[3] = HAL_GPIO_ReadPin(reed_push_gpio, reed_push_pin); // Reed switch push
				sensor[4] = HAL_GPIO_ReadPin(emer_gpio, emer_pin); // Emergency button
				sensor[5] = HAL_GPIO_ReadPin(home_gpio, home_pin); // Home button
				Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
			}
			else if(testing == 0){
				// End effector update
				Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
				// Main controller loop
				if(is_update_encoder == 1){
					Update_qei(&encoder, &htim4); // Update encoder
					Update_lowpass(&lowpass, encoder.mmps);
//					compute_kalman(&kalman, 0.0, encoder.mm);

//					KalmanFilter_Predict(saifah, u_data[0], w_data[0]);
//					KalmanFilter_Update(saifah, );
//					Trajectory_Generator(&genScurveData, initial_position, target_position, max_velocity, max_acceleration, max_jerk); // Generate trajectory
//					Trajectory_Evaluated(&genScurveData, &evaScurveData, initial_position, target_position, max_velocity, max_acceleration, max_jerk); // Evaluate trajectory
					if(state != 1){
						trapezoidalGeneration(&genTrapezoidalData, initial_position, target_position, max_velocity, max_acceleration);
						trapezoidalComputation(&computeTrapezoidalData, &genTrapezoidalData, max_velocity, max_acceleration);
	//
	//					setpoint_pos = evaScurveData.setposition; // Position set point
	//					setpoint_vel = evaScurveData.setvelocity; // Feed forward velocity
						setpoint_pos = computeTrapezoidalData.set_pos;
						setpoint_vel = computeTrapezoidalData.set_vel;
//						if(start_position_control == 1){
//							start_position_control = 0;
//						}
						Update_position_control(setpoint_pos);
						Update_velocity_control(setpoint_vel + p_output);
						if(is_finish_position == 0){
							pwm_signal = -65535 + (((v_output - (-24)) * (65535 - (-65535))) / (24 - (-24)));
							Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal); // Update main PWM signal
						}
						else{
							pwm_signal = 8000;
							Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal); // Update main PWM signal
						}
						is_update_encoder = 0;
					}
				}
				if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
					static uint64_t timestamp = 0;
					static uint8_t first = 1;
					if(first){
						timestamp = HAL_GetTick() + 100; // Delay time in ms
						first = 0;
					}
					if(HAL_GetTick() > timestamp){
						if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
							// Have command from home switch
							home.homing_command = 1;
							registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
							state = registerFrame[0x10].U16;
							mode = HOMING;
						}
						first = 1;
					}
				}
				if(registerFrame[0x01].U16 != 0){

					// Check command from base system status
					//  homing command from Homing button and Base system Check command
					Set_Home(); // Refresh homing command from base system
					if(state == 0b0010){
						// Have command from base system
						mode = HOMING; // Go to wait mode for Deactivate end effector
						home.homing_command = 1;
						registerFrame[0x01].U16 = 0b0000; // Reset data of base system status
						registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
						state = registerFrame[0x10].U16 = 0b0010;
					}
					// Go point command from base system
					else if(Run_Point_Mode() == 1){
						registerFrame[0x10].U16 = 0b00010000;
						state = registerFrame[0x10].U16;
//						x_axis_position = registerFrame[0x40].U16 / 10.0;
						setpoint = Set_Goal_Point();
						ready = 1;
					}
					// Set shelves command from base system
					else if(Set_Shelves() == 1){
						joy.shelves_position[0] = 0;
						joy.shelves_position[1] = 0;
						joy.shelves_position[2] = 0;
						joy.shelves_position[3] = 0;
						joy.shelves_position[4] = 0;
						ready = 1;
					}
					// Run jog mode from base system
					else if(Run_Jog_Mode() == 1){
						SetPick_PlaceOrder();
						jog_status[0] = 1; // Go pick first
						jog_status[1] = 0;
						registerFrame[0x10].U16 = 4;
						state = registerFrame[0x10].U16;
						strcpy(Jogmode, "Go to Pick...");
						ready = 1;
						skip = 0;
					}
				}
				else{
					// Check state from z moving status
					// Set shelve
					if(state == 1){
						if(ready == 1){
							static uint8_t i = 0;
							static uint8_t repeat = 0;
							static int32_t speed = 14000;
	//						static float step = 5.0;
	//						static float position = 10.0;
							// Update joy stick command
							Update_joy(&joy);
							if(joy.is_place == 0){
								repeat = 0;
							}
							if(i > 5){
								registerFrame[0x23].U16 = joy.shelves_position[0];  //1st Shelve Position
								registerFrame[0x24].U16 = joy.shelves_position[1];  //2nd Shelve Position
								registerFrame[0x25].U16 = joy.shelves_position[2];  //3rd Shelve Position
								registerFrame[0x26].U16 = joy.shelves_position[3];  //4th Shelve Position
								registerFrame[0x27].U16 = joy.shelves_position[4];  //5th Shelve Position
								state = 0b0000;
								registerFrame[0x10].U16 = state;
								ready = 0;
								i = 0;
								initial_position = encoder.mm;
								target_position = encoder.mm;

							}
							else if (!joy.s_1 && joy.s_2 && joy.s_3 && joy.s_4){
								if(joy.is_place == 1){
									if(repeat != 1){
	//									position += 5.0;
	//									setpoint = position;
	//									computeTrapezoidalData.t = 0;
										pwm_signal = speed;
										Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal);
										repeat = 1;
									}
								}
							}
							else if (!joy.s_1 && !joy.s_2 && joy.s_3 && joy.s_4){
								// switch 2 has pushed
								if(joy.is_place == 1){
									if(repeat != 1){
//										x_axis_position = x_axis_position - 5.0;
										x_axis_position = x_axis_position + 5.0;
										repeat = 1;
									}
								}
//								if(repeat != 1){
//									static uint8_t choose = 0;
//									if(choose == 2){
//										speed = 8000;
//										choose = 0;
//	//									step = 5.0;
//									}
//									else{
//										speed = 14000;
//	//									step = 10.0;
//										choose++;
//									}
//	//								initial_position = encoder.mm;
//	//								setpoint = 10;
//	//								target_position = setpoint;
//									repeat = 1;
//								}
							}
							else if(joy.s_1 && !joy.s_2 && joy.s_3 && !joy.s_4){
								if(joy.is_place == 1){
									if(repeat != 1){
										x_axis_position = x_axis_position - 5.0;
										repeat = 1;
									}
								}
							}
							else if (joy.s_1 && joy.s_2 && !joy.s_3 && joy.s_4){
								// switch 3 has pushed
								// save data for base system
								if(joy.is_place == 1){
									if(repeat != 1){
										joy.shelves_position[i] = (uint16_t)(encoder.mm * 10);
										i++;
										repeat = 1;
									}
								}
							}
							else if (joy.s_1 && joy.s_2 && joy.s_3 && !joy.s_4){
								// switch 4 has pushed
								if(joy.is_place == 1){
									if(repeat == 0){
	//									initial_position = encoder.mm;
	//									position -= 5.0;
	//									setpoint = position;
	//									target_position = setpoint;
	//									computeTrapezoidalData.t = 0;
										pwm_signal = -speed;
										Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal);
										repeat = 1;
									}
								}
							}
							else{
								pwm_signal = 5000;
								Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal);
							}
						}
					}
					// Go pick
					else if(state == 4){
						static uint8_t pass = 0;
						if(ready == 1){
							if(pick_index == 5){
								pick_index = 0;
								skip = 1;
							}
							else if(pick_index >= 0 && pick_index < 5){
								if(pick_index){
									pass = 0;
								}
								// Set up trajectory
								initial_position = encoder.mm;
								target_position = (float)(joy.shelves_position[(Pick[pick_index] - 1)]) / 10.0;
								computeTrapezoidalData.t = 0;
								computeTrapezoidalData.is_finish = 0;
								x_axis_position = 10.0;
								p_e = 0.0;
								is_finish_position = 0;
								ready = 0;
								skip = 0;
							}
						}
//						&& is_finish_position == 1
						if(computeTrapezoidalData.is_finish == 1 && ready == 0 && skip == 0){
							if(pass == 0){
								eff.solenoid_command[0] = 1;
								eff.solenoid_command[1] = 1; // Push forward
								eff.solenoid_command[2] = 0;
								pass = 1;
							}
							if(pass == 1){
								if((eff.actual_status[0] == 0) && (eff.actual_status[1] == 1)){
									static uint64_t timestamp = 0;
									static uint8_t first = 1;
									if(first == 1){
										timestamp = HAL_GetTick() + 800;
										first = 0;

									}
									if(HAL_GetTick() > timestamp){
										// End effector is push
										eff.solenoid_command[0] = 1;
										eff.solenoid_command[1] = 0;
										eff.solenoid_command[2] = 1; // Pull back
										pass = 2;
										first = 1;
									}
								}
							}
							if(pass == 2){
								if((eff.actual_status[0] == 1) && (eff.actual_status[1] == 0)){
//									static uint64_t timestamp = 0;
//									if(timestamp > 300){
									// Deactivate solenoid valve
									eff.solenoid_command[1] = 0;
									eff.solenoid_command[2] = 0;
									pick_index++;
									ready = 1;
									registerFrame[0x10].U16 = 8;
									state = registerFrame[0x10].U16; // Then go place
//									timestamp = 0;
									pass = 0;
//									}
//									else{
//										timestamp++;
//									}
								}
							}
						}
					}
					// Go place
					else if(state == 8){
						static uint8_t pass = 0;
						if(ready == 1){
							if(place_index >= 0 && place_index < 5){
								if(place_index == 0){
									pass = 0;
								}
								// Set up trajectory
								initial_position = encoder.mm;
								target_position = ((float)(joy.shelves_position[(Place[place_index] - 1)]) / 10.0) + 10.0;
								computeTrapezoidalData.t = 0;
								computeTrapezoidalData.is_finish = 0;
								x_axis_position = -10.0;
								p_e = 0.0;
								is_finish_position = 0;
								ready = 0;
								skip = 0;
							}
						}
//						&& is_finish_position == 1
						if(computeTrapezoidalData.is_finish == 1 && ready == 0 && skip == 0){
							if(pass == 0){
								eff.solenoid_command[1] = 1; // Push forward
								eff.solenoid_command[2] = 0;
								pass = 1;
							}
							if(pass == 1){
								if((eff.actual_status[0] == 0) && (eff.actual_status[1] == 1)){
									static uint64_t timestamp = 0;
									static uint8_t first = 1;
									if(first == 1){
										timestamp = HAL_GetTick() + 800;
										first = 0;
									}
									if(HAL_GetTick() > timestamp){
										// End effector is push
										eff.solenoid_command[0] = 0;
										eff.solenoid_command[1] = 0;
										eff.solenoid_command[2] = 1; // Pull back
										pass = 2;
//										timestamp = 0;
										first = 1;
									}
								}
							}
							if(pass == 2){
								if((eff.actual_status[0] == 1) && (eff.actual_status[1] == 0)){
									// Deactivate solenoid valve
									eff.solenoid_command[0] = 0;
									eff.solenoid_command[1] = 0;
									eff.solenoid_command[2] = 0;
									pass = 0;
									place_index++;
									ready = 1;
									registerFrame[0x10].U16 = 4;
									state = registerFrame[0x10].U16; // Then go place
//									timestamp = 0;
									if(place_index == 5){
										place_index = 0;
										state = 0b0000;
										registerFrame[0x10].U16 = state;
										ready = 0;
										skip = 1;
									}
								}
							}
						}
					}
					// Run point mode
					else if(state == 16){
						if(ready == 1){
							static uint64_t timestamp = 0;
							static uint8_t first = 1;
							if(first){
								timestamp = HAL_GetTick() + 500;
								first = 0;
							}
							if(HAL_GetTick() > timestamp){
								initial_position = encoder.mm;
								target_position = setpoint;
								computeTrapezoidalData.t = 0;
								p_e = 0;
								ready = 0;
								first = 1;
							}
						}
						else if((computeTrapezoidalData.is_finish == 1) && (ready == 0)){
							static uint64_t timestamp = 0;
							static uint8_t first = 1;
							if(first){
								timestamp = HAL_GetTick() + 500;
								first = 0;
							}
							if(HAL_GetTick() > timestamp){
								state = 0b0000;
								registerFrame[0x10].U16 = 0b0000;
//								timestamp = 0;
								first = 1;
							}
//							timestamp++;
						}
					}
					else {
						// Update peripheral
						Vacuum_Status(&eff); // Vacuum
						Gripper_Movement_Status(&eff); // End effector
					}

				}

			}
		}
		if(mode == HOMING){
			if(home.is_home == 1){
				home.is_home = 0;
				registerFrame[0x10].U16 = 0b0000; // Reset data of moving status
				// Finish homing state at 0 mm
				state = registerFrame[0x10].U16;

				Reset_main_variable();
				// Change Mode
				mode = RUNNING;
			}
		}
		if(mode == EMERGENCY){
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 1){
				HAL_GPIO_WritePin(emer_light_gpio, emer_light_pin, RESET);
				mode = WAIT;
			}
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1145;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Direaction_motor_Pin|Emergency_light_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Solenoid_valve_push_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Pilot_controller_error_Pin|Pilot_motor_error_Pin|Solenoid_valve_pull_Pin|Solenoid_valve_vacuum_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Direaction_motor_Pin */
  GPIO_InitStruct.Pin = Direaction_motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Direaction_motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Solenoid_valve_push_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Solenoid_valve_push_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : joy_switch_3_Pin joy_switch_4_Pin Set_home_Pin Reed_switch_pull_Pin */
  GPIO_InitStruct.Pin = joy_switch_3_Pin|joy_switch_4_Pin|Set_home_Pin|Reed_switch_pull_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Pilot_controller_error_Pin Pilot_motor_error_Pin Solenoid_valve_pull_Pin Solenoid_valve_vacuum_Pin */
  GPIO_InitStruct.Pin = Pilot_controller_error_Pin|Pilot_motor_error_Pin|Solenoid_valve_pull_Pin|Solenoid_valve_vacuum_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Proximity_Pin Emergency_switch_Pin joy_switch_1_Pin */
  GPIO_InitStruct.Pin = Proximity_Pin|Emergency_switch_Pin|joy_switch_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergency_light_Pin */
  GPIO_InitStruct.Pin = Emergency_light_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Emergency_light_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Reed_switch_push_Pin joy_switch_2_Pin */
  GPIO_InitStruct.Pin = Reed_switch_push_Pin|joy_switch_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Main timer interrupt for run program with accuracy time
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// Run with 1000 Hz
	if(htim == &htim3){
		// Routine
		if(registerFrame[0x00].U16 == 18537){
			registerFrame[0x00].U16 = 22881;
		}
		registerFrame[0x04].U16 = eff.update_actual_status[0x04].U16;	// Gripper Movement Actual Status(0x10)
		registerFrame[0x10].U16 = state;							// Z-axis Moving Status(0x10)
		registerFrame[0x11].U16 = (uint16_t)(encoder.mm * 10);		// Z-axis Actual Position(0x11)
		registerFrame[0x12].U16 = (int16_t)(encoder.mmps * 10);		// Z-axis Actual Speed (0x12)
		registerFrame[0x13].U16 = (int16_t)(encoder.mmpss * 10);	// Z-axis Acceleration(0x13)
		registerFrame[0x40].U16 = (int16_t)(x_axis_position * 10);	// X-axis Actual Position(0x40)
		// Update encoder
		if(is_update_encoder == 0){
			is_update_encoder = 1;
		}
		if(mode == RUNNING){
//			static uint8_t timestamp = 0;
//			if(start_position_control == 0 && timestamp == 10){
//				start_position_control = 1;
//				timestamp = 0;
//			}
//			timestamp++;
		}
		if(mode == HOMING){
//			// End effector position check
//			if(home.homing_command == 1 && eff.actual_status[0] == 0 && eff.actual_status[1] == 1){
//				eff.solenoid_command[0] = 0;
//				eff.solenoid_command[1] = 0;
//				eff.solenoid_command[2] = 1;
//				Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
//				return;
//			}
//			if(home.homing_command == 1 && eff.actual_status[0] == 1 && eff.actual_status[1] == 0){
//				eff.solenoid_command[0] = 0;
//				eff.solenoid_command[1] = 0;
//				eff.solenoid_command[2] = 0;
//				Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);
//			}
			// Homing state
			homing(&home, GPIOB, GPIO_PIN_12); // Homing function
			pwm_signal = home.pwm;
			Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal); // Update main PWM signal
		}
	}
}
// GPIO interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Activate and Deactivate testing mode
	if(GPIO_Pin == GPIO_PIN_13){
		static uint8_t test = 0;
		if(test == 0){
			target_position = 10.0;
			initial_position = encoder.mm;
			evaScurveData.t = 0;
			computeTrapezoidalData.t = 0.0;
			setpoint_pos = 0.0;
			setpoint_vel = 0.0;
			test = 1;
//			testing = 1;
		}
		else if(test == 1){
			initial_position = encoder.mm;
			target_position = 10.0;
			evaScurveData.t = 0.0;
			computeTrapezoidalData.t = 0.0;
			setpoint_pos = 0.0;
			setpoint_vel = 0.0;
			test = 0;
//			testing = 0;
		}
		mode = RUNNING;
	}

//	if(GPIO_Pin == GPIO_PIN_15){
//		// Emergency switch interrupted
//		static uint64_t timestamp = 0;
//		static uint8_t first_time = 1;
//		if(first_time == 1){
//			timestamp = HAL_GetTick() + 2000;
//			first_time = 0;
//		}
//		if(HAL_GetTick() > timestamp){
//			if(HAL_GPIO_ReadPin(emer_gpio, emer_pin) == 0){
//				first_time = 1;
//	//			test = 0;
//	//			setpoint = 0.0;
//	//			Reset_homing(&home);
//	//			Reset_qei(&encoder, &htim4);
//	//			Reset_pid(&p_pid);
//	//			p_output = 0.0;
//	//			p_e = 0.0;
//	//			Reset_pid(&v_pid);
//	//			v_output = 0;
//	//			v_e = 0.0;
//	//			initial_position = 0.0;
//	//			target_position = 0.0;
//	//			evaScurveData.t = 0.0;
//	//
//	//			state = 0;
//	//			registerFrame[0x10].U16 = 0;
//	//			registerFrame[0x01].U16 = 0;
//

//			}
//		}
//		else{return;}
//	}
}

// Torque control update
void Update_torque_control(double s){

}
// Velocity control update
void Update_velocity_control(double s){
	// input is millimeter unit
//	v_e = s - lowpass.filtered_data;
//	if(target_position - initial_position >= 0){
//		v_e = s - Get_mmps(&encoder);
//		v_output = (int32_t)floor((Update_pid(&v_pid, v_e, 24.0, 24.0)));
//	}
//	else{
//		v_e = s - Get_mmps(&encoder);
//		v_output = (int32_t)floor((Update_pid(&v_pid, v_e, 24.0, 24.0)));
//	}
	v_e = s - Get_mmps(&encoder);
	v_output = floor((Update_pid(&v_pid, v_e, 24.0, 24.0)));
//	if(v_e >= -0.2 && v_e <= 0.2){
//		v_output = 2.0;
//	}
//	else{
//	}
}
// Position control update
void Update_position_control(double s){
	//input is pulse unit
	p_e = s - Get_mm(&encoder);
	if(p_e >= -0.1 && p_e <= 0.1){
		static uint64_t timestamp = 0;
		static uint8_t first = 1;
		if(first == 1){
			timestamp = HAL_GetTick() + 100;
			first = 0;
		}
		if(HAL_GetTick() > timestamp){
			if(p_e >= -0.1 && p_e <= 0.1){
				p_pid.y_n = 0.0;
				p_pid.y_n_1 = 0.0;
				p_output = 0.0;
				is_finish_position = 1;
			}
			first = 1;
		}
	}
	else{
		p_output = Update_pid(&p_pid, p_e, 650.0, 650.0);
		is_finish_position = 0;
	}
}
// Reset variable function
void Reset_main_variable(){
	// Reset point reset
	setpoint = 0.0;
	x_axis_position = 0.0;
	// Reset PWM signal
	pwm_signal = 0;
	// Encoder compute enable
	is_update_encoder = 0;
	// Reset trajectory
	initial_position = 0.0;
	target_position = 0.0;
	evaScurveData.setposition = 0.0;
	evaScurveData.setvelocity = 0.0;
	evaScurveData.setacceleration = 0.0;
	evaScurveData.t = 0.0;
	computeTrapezoidalData.set_pos = 0.0;
	computeTrapezoidalData.set_vel = 0.0;
	computeTrapezoidalData.t = 0.0;
	// Reset homing data
	Reset_homing(&home);
	// Reset state enable
	pick_index = 0;
	place_index = 0;
	state = 0;
	ready = 0;
	test = 0;
	// Reset MODBUS
	registerFrame[0x10].U16 = 0;
	registerFrame[0x01].U16 = 0;
	// Reset encoder
	Reset_qei(&encoder, &htim4);
	// Reset lowpass
	Reset_lowpass(&lowpass);
	// Reset PID
	is_finish_position = 0;
	p_e = 0.0;
	p_output = 0.0;
	v_e = 0.0;
	v_output = 0;
	Reset_pid(&p_pid);
	Reset_pid(&v_pid);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
