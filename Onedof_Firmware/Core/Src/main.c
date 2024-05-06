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
#include "kalman.h"
#include "pid.h"
#include "qei.h"
#include "pwm.h"
#include "adc.h"
#include "joy.h"
#include "eff.h"
#include "ModBusRTU.h"
#include "Basesystem.h"
#include "state.h"
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

#define modbus_tim				htim5
#define main_loop_tim			htim3

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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

// Temporary variable
uint8_t tuning = 1;
uint64_t repeat_cheack = 0;
uint32_t limitswitch_test = 0;
float32_t test = 700;
float32_t min_error = 500;
uint64_t pid_time[2] = {0};
uint64_t sensor[6] = {0};

// Set point
float32_t set_point = 0;

// State
uint8_t finish_job = 0;
uint16_t mode = 0; /* For select mode from base system mode == 0 for stand by ,mode == 1 run point mode,
 	 	 	 	 	 mode == 2 run set shelve ,mode == 3 run jog mode(Full automatic pick & place)*/
uint8_t ready = 1;
POINT point;
HOME home;
EMER emer;

// Modbus
u16u8_t registerFrame[200];
uint8_t heartbeat_status = 0;

// End effector
EFF eff;

// Joy
JOY joy;
int32_t jog = 0;

// Homing
uint64_t homing_ts = 0; // delay time for 2nd homing
uint8_t homing_first = 1; // Homing first status
uint8_t homing_second = 0; // Homing 2nd status
//uint8_t homing = 0; // Homing status
uint8_t is_home = 0; // Is robot home

// Current sensor
ADC current_sensor;

// Update_motor variables
int16_t pwm_signal = 0;

// QEI variables
QEI encoder;

// Torque pid
//PID t_pid;

// Velocity pid
PID v_pid;
float32_t v_kp = 10;
float32_t v_ki = 0;
float32_t v_kd = 0;
float32_t v_e = 0;
int32_t v_output = 0; // for tuning only int32_t

// Position pid
PID p_pid;
float32_t p_kp = 2;
float32_t p_ki = 0;
float32_t p_kd = 0;
float32_t p_e = 0;
int32_t p_output = 0; // for tuning only int32_t

// Kalman filter
KalmanFilter kalman;
float32_t kalman_velocity = 0.0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Update_torque_control(float32_t s);
void Update_velocity_control(float32_t s);
void Update_position_control(float32_t s);
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
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Modbus setting
  hmodbus.huart = &huart2;
  hmodbus.htim = &htim16;
  hmodbus.slaveAddress = 0x15;
  hmodbus.RegisterSize =200;
  Modbus_init(&hmodbus, registerFrame);
  registerFrame[0x00].U16 = 22881; // Set default heart beat to "Ya"
  // Update MODBUS timer
  HAL_TIM_Base_Start_IT(&modbus_tim);

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

  // Position PID
  PID_init(&p_pid, p_kp, p_ki, p_kd, 5.0/(float32_t)encoder_frequency);
  PID_init(&v_pid, v_kp, v_ki, v_kd, 1.0/(float32_t)encoder_frequency);

  // Kalman filter
  Kalman_Start(&kalman);

  // State initialize
  homing_init(&home);
  point_init(&point);

  //Set point
  set_point = 0.0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  Modbus_Protocal_Worker();
//	  Set_Shelves();
//	  Gripper_Movement_Status();
//	  Vacuum_Status();
//	  Run_Jog_Mode();
//	  SetPick_PlaceOrder();
//	  Set_Home();
//	  Set_Goal_Point();

//	  sensor[0] = __HAL_TIM_GET_COUNTER(&htim4);
//	  sensor[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
//	  sensor[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
//	  sensor[3] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
//	  sensor[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
//	  sensor[5] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
//	  while(mode == 1){
//		  Update_joy(&joy);
//		  if (!joy.s_1 && joy.s_2 && joy.s_3 && joy.s_4){
//			  // switch 1 has pushed
//			  jog += 10; // Move up 10 mm.
//		  }else if (joy.s_1 && !joy.s_2 && joy.s_3 && joy.s_4){
//			  // switch 2 has pushed
//			  jog -= 10; // Move down 10 mm.
//		  }else if (joy.s_1 && joy.s_2 && !joy.s_3 && joy.s_4){
//			  // switch 3 has pushed
//			  mode = 0; // Change mode to Automatic
//		  }else if (joy.s_1 && joy.s_2 && joy.s_3 && !joy.s_4){
//			  // switch 4 has pushed
//			  // save data for base system
//		  }
//	  }
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
  htim1.Init.Prescaler = 169;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  /*Configure GPIO pins : Proximity_Pin joy_switch_1_Pin */
  GPIO_InitStruct.Pin = Proximity_Pin|joy_switch_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergency_switch_Pin */
  GPIO_InitStruct.Pin = Emergency_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Emergency_switch_GPIO_Port, &GPIO_InitStruct);

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
	if(htim == &htim5){
		//Update modbus
		if(registerFrame[0x00].U16 == 18537){
			registerFrame[0x00].U16 = 22881;
			heartbeat_status = 1;
		}
		else if(heartbeat_status == 0){
			registerFrame[0x00].U16 = 22881;
		}
		else if(heartbeat_status == 1){
		  	//Routine
			registerFrame[0x04].U16 = eff.update_actual_status[0x04].U16;	// Gripper Movement Actual Status(0x10)
			registerFrame[0x10].U16 = state;							// Z-axis Moving Status(0x10)
			registerFrame[0x11].U16 = encoder.mm;						// Z-axis Actual Position(0x11)
			registerFrame[0x12].U16 = encoder.mmps;						// Z-axis Actual Speed (0x12)
			registerFrame[0x13].U16 = encoder.mmpss;					// Z-axis Acceleration(0x13)    //////ความเร่งต้องเปลี่ยน/////
			registerFrame[0x40].U16 = encoder.rpm;						// X-axis Actual Position(0x40)
		  heartbeat_status = 0;
		}
	}
	// Run with 1000 Hz
	if(htim == &htim3){
		// Update main

		// wait for start command same as home switch
		if(ready == 1){
			if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
				ready = 0;
			}else{return;}
		}

		// Update encoder
		Update_qei(&encoder, &htim4);

		// Update current sensor
//		Update_adc(&current_sensor);

		// Update reed switch status
		Update_actual_eff(&eff, reed_pull_gpio, reed_pull_pin, reed_push_gpio, reed_push_pin);

		// Update gripper command from base system
		Gripper_Movement_Status(&eff);
//		// Enable gripper with command
//		Update_eff(&eff, solenoid_pull_gpio, solenoid_pull_pin, solenoid_push_gpio, solenoid_push_pin, vacuum_gpio, vacuum_pin);

		// Update point mode (set goal point)
		point.goal = (float32_t)Set_Goal_Point();
		mode = Run_Point_Mode();

//		kalman_velocity = SteadyStateKalmanFilter(&kalman, test, encoder.mmps);

		//  homing command from Homing button and Base system Check command
		Set_Home(); // Refresh homing command from base system
		if(home.homing_command == 0){
			if(state == 0b0010){
				// Have command from base system
				home.homing_command = 1;
				registerFrame[0x01].U16 = 0b0000; // Reset data of base system status
				registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
			}
			else if(HAL_GPIO_ReadPin(home_gpio, home_pin) == 1){
				// Have command from home switch
				home.homing_command = 1;
//				registerFrame[0x10].U16 = 0b0010; // Set data of moving status to Home
			}
		}

		// Homing state
		homing(&home, GPIOB, GPIO_PIN_12); // Homing function
		pwm_signal = home.pwm; // Set PWM from homing function

		if(home.is_home == 1 && encoder.pulse != 0){
			// Finish homing state
			Reset_qei(&encoder); // Reset encoder data
			registerFrame[0x10].U16 = 0b0000; // Reset data of moving status
		}

		// Emergency break state
		emergency(&emer, emer_light_gpio, emer_light_pin); // Activate emergency light
		if(emer.emer_state == 1){
			if(home.is_home == 1 && encoder.pulse == 0){
				emer.emer_state = 0; // Reset emergency state
				home.is_home = 0; // Reset is_home state
			}
			else{
				// No reset from homing state
				return;
			}
		// Finish homing while normal run not in emergency break state
		}else if(emer.emer_state == 0){
			if(home.is_home == 1 && encoder.pulse == 0){
				home.is_home = 0; // Reset is_home state
			}
		}

//		Software limit
//		else if(encoder.mm >= 685 || encoder.mm <= 5){
//			Update_pwm(&htim1, TIM_CHANNEL_1, GPIOC, GPIO_PIN_1, 0);
//		}

//		// Tuning and test mode(PID & Trajectory)
		else if(tuning == 1){
			static uint64_t control_loop_ts = 0;
			if(control_loop_ts == 5){
				// Position control loop update
//				Update_position_control(v_output);
				control_loop_ts = 0;
			}
			else{control_loop_ts++;}
			// Velocity control loop update
			Update_velocity_control(test);
			// PWM signal update
			pwm_signal = v_output;
		}
		// Tuning and test mode(Joy control)
		else if(tuning == 2){
			Update_joy(&joy);
		}
		// Tuning and test mode(Sensor or other check)
		else if(tuning == 3){
		  sensor[0] = __HAL_TIM_GET_COUNTER(&encoder_tim);
		  sensor[1] = HAL_GPIO_ReadPin(proximity_gpio, proximity_pin);
		  sensor[2] = HAL_GPIO_ReadPin(reed_pull_gpio, reed_pull_pin);
		  sensor[3] = HAL_GPIO_ReadPin(reed_push_gpio, reed_push_pin);
		  sensor[4] = HAL_GPIO_ReadPin(emer_gpio, emer_pin);
		  sensor[5] = HAL_GPIO_ReadPin(home_gpio, home_pin);
		}

		// Run point mode
		else if(mode == 1 && tuning == 0 && emer.emer_state == 0 && home.is_home == 0){
			// Check for set shelves command
			set_point = point.goal;
		}

		// Set shelve mode
		else if(mode == 2 && tuning == 0 && emer.emer_state == 0 && home.is_home == 0){

		}
		// Run jog mode
		else if(mode == 3 && tuning == 0 && emer.emer_state == 0 && home.is_home == 0){

		}

		// Main Controller loop ,Check don't have home command
		if(home.homing_command == 0 && tuning == 0){
//			static uint64_t control_loop_ts = 0;
//			if(control_loop_ts == 5){
//				// Position control loop update
//				Update_position_control(v_output);
//				control_loop_ts = 0;
//			}
//			else{control_loop_ts++;}
//			// Velocity control loop update
//			Update_velocity_control(test);
//			// PWM signal update
//			pwm_signal = v_output;
//			if((encoder.mm >= (set_point-0.1) && encoder.mm <= (set_point+0.1)) && (mode == 1 || mode == 2 || mode == 3)){
//				finish_job = 1;
//			}
		}
		Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, pwm_signal); // Update main PWM signal
	}
}

// GPIO interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	// Activate and Deactivate tuning mode
	if(GPIO_Pin == GPIO_PIN_13){
		tuning++;
		if(tuning == 4){
			tuning = 0;
		}
	}

	if(GPIO_Pin == GPIO_PIN_15){
		// Emergency switch interrupted
		// Stop motor
		Update_pwm(&pwm_tim, pwm_channel, dir_gpio, dir_pin, 0);
		// Emergency light enable
		HAL_GPIO_WritePin(emer_light_gpio, emer_light_pin, SET);
		emer.emer_state = 1;
	}
}

// Torque control update
void Update_torque_control(float32_t s){

}
// Velocity control update
void Update_velocity_control(float32_t s){
	//input is pulse unit
	v_e = s - Get_mmps(&encoder);
	pid_time[0]++;
	if(v_e <= min_error){
		min_error = v_e;
//		pid_time[1] = pid_time[0];
	}

	v_output = Update_pid(&v_pid, v_e, 900.0, 1000.0);
}
// Position control update
void Update_position_control(float32_t s){
	//input is pulse unit
	p_e = s - Get_mm(&encoder);
	p_output = Update_pid(&p_pid, p_e, 900.0, 1000.0);
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
