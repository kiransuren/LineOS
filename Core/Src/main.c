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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Only one address is possible for the APDS9960, no alternates are available
static const uint16_t _APDS9960_I2C_ADDRESS = 0x39<<1; //use 8-bit address
//#define _DEVICE_IDS = (const(0xAB), const(0xA8))
#define _APDS9960_ENABLE 0x80
#define _APDS9960_ATIME 0x81
#define _APDS9960_PILT 0x89
#define _APDS9960_PIHT 0x8B
#define _APDS9960_PERS 0x8C
#define _APDS9960_CONTROL 0x8F
#define _APDS9960_ID 0x92
#define _APDS9960_STATUS 0x93
#define _APDS9960_CDATAL 0x94
#define _APDS9960_RDATAL 0x96
#define _APDS9960_GDATAL 0x98
#define _APDS9960_BDATAL 0x9A
#define _APDS9960_PDATA 0x9C
#define _APDS9960_GPENTH 0xA0
#define _APDS9960_GEXTH 0xA1
#define _APDS9960_GCONF1 0xA2
#define _APDS9960_GCONF2 0xA3
#define _APDS9960_GPULSE 0xA6
#define _APDS9960_GCONF4 0xAB
#define _APDS9960_GFLVL 0xAE
#define _APDS9960_GSTATUS 0xAF
#define _APDS9960_AICLEAR 0xE7
#define _APDS9960_GFIFO_U 0xFC

#define _BIT_MASK_GCONF4_GFIFO_CLR 0x04
#define _BIT_MASK_ENABLE_EN 0x01
#define _BIT_MASK_ENABLE_COLOR 0x02
#define _BIT_MASK_STATUS_AVALID 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for heartbeatTsk */
osThreadId_t heartbeatTskHandle;
const osThreadAttr_t heartbeatTsk_attributes = {
  .name = "heartbeatTsk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for grabberMotorTsk */
osThreadId_t grabberMotorTskHandle;
const osThreadAttr_t grabberMotorTsk_attributes = {
  .name = "grabberMotorTsk",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for colourSenseRead */
osThreadId_t colourSenseReadHandle;
const osThreadAttr_t colourSenseRead_attributes = {
  .name = "colourSenseRead",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wheelMotorTsk */
osThreadId_t wheelMotorTskHandle;
const osThreadAttr_t wheelMotorTsk_attributes = {
  .name = "wheelMotorTsk",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
void hearbeatTaskFunc(void *argument);
void grabberMotorTaskFunc(void *argument);
void colourSensorReadTsk(void *argument);
void wheelMotorTask(void *argument);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float red_ratio_R = 0;
float red_ratio_L = 0;

float p_term = 0;
float i_term = 0;
float d_term = 0;

bool enable_autonomy = false;
bool enable_motor_test = false;

float control_signal = 0;
float error_signal = 0;
uint32_t leftMotorDuty = 0;
uint32_t rightMotorDuty = 0;

uint16_t clear_data_L = 0;
uint16_t red_data_L = 0;
uint16_t green_data_L = 0;
uint16_t blue_data_L = 0;
uint16_t clear_data_R = 0;
uint16_t red_data_R = 0;
uint16_t green_data_R = 0;
uint16_t blue_data_R = 0;

volatile uint32_t elapsedTimeMs = 0;

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // Start timer for grabber servo
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //htim1.Instance->CCR2 = 50; // initialize pulse width to 1ms = 0 degree positon

  // Start timer for motor driver (left and right motors)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  htim3.Instance->CCR1 = 0; // initialize duty cycle to 0% - Motor off
  htim3.Instance->CCR2 = 0; // initialize duty cycle to 0% - Motor off

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of heartbeatTsk */
  heartbeatTskHandle = osThreadNew(hearbeatTaskFunc, NULL, &heartbeatTsk_attributes);

  /* creation of grabberMotorTsk */
  grabberMotorTskHandle = osThreadNew(grabberMotorTaskFunc, NULL, &grabberMotorTsk_attributes);

  /* creation of colourSenseRead */
  colourSenseReadHandle = osThreadNew(colourSensorReadTsk, NULL, &colourSenseRead_attributes);

  /* creation of wheelMotorTsk */
  wheelMotorTskHandle = osThreadNew(wheelMotorTask, NULL, &wheelMotorTsk_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 210-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 21-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 250-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Output_4_Pin|Output_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Output_1_GPIO_Port, Output_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Output_3_GPIO_Port, Output_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Output_1_Pin */
  GPIO_InitStruct.Pin = Output_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Output_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Output_4_Pin Output_2_Pin */
  GPIO_InitStruct.Pin = Output_4_Pin|Output_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Output_3_Pin */
  GPIO_InitStruct.Pin = Output_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Output_3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
typedef enum {
    FORWARD,
    BACKWARD
} motor_direction_t;

typedef enum {
    LEFT,
    RIGHT
} motor_side_t;

void setMotorDirection(motor_direction_t direction, motor_side_t side)
{
	// Resetting all
	HAL_GPIO_WritePin(GPIOC, Output_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Output_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Output_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Output_4_Pin, GPIO_PIN_RESET);
	uint8_t broken_message[] = "FIX MOTORS\r\n"; // message to send
	if (direction == FORWARD)
	{
		if(side == LEFT)
		{
			//Setting motor 1 to forward
			//HAL_GPIO_WritePin(GPIOB, Output_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Output_2_Pin, GPIO_PIN_SET);
		}
		else
		{
			//Setting motor 2 to forward
			//HAL_GPIO_WritePin(GPIOB, Output_4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, Output_4_Pin, GPIO_PIN_SET);
		}

	}
	else if (direction == BACKWARD)
	{
		if(side == LEFT)
		{
			//Setting motor 1 to backward
			HAL_GPIO_WritePin(GPIOC, Output_1_Pin, GPIO_PIN_SET);
		}
		else
		{
			//Setting motor 2 to backward
			HAL_GPIO_WritePin(GPIOB, Output_3_Pin, GPIO_PIN_SET);
		}
	}
	else
	{
		HAL_UART_Transmit(&huart2,broken_message,sizeof(broken_message),10);
	}
}

void setMotorsOff()
{
	// Resetting all motor input pins
	HAL_GPIO_WritePin(GPIOC, Output_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Output_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Output_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Output_4_Pin, GPIO_PIN_RESET);
}

void setRightMotorDutyCycle(uint16_t duty)
{
	htim3.Instance->CCR1=duty;
}

void setLeftMotorDutyCycle(uint16_t duty)
{
	htim3.Instance->CCR2=duty;
}

bool colourSensorSetup(I2C_HandleTypeDef *i2cHandle)
{
	printf("Colour Sensor Setup BEGIN\n");
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t buf[2];
	//Reset basic config registers to power-on defaults
	buf[0] = _APDS9960_GPENTH;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_GEXTH;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_GCONF1;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(&hi2c1, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_GCONF2;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_GCONF4;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_GPULSE;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_ATIME;
	buf[1] = 175;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_CONTROL;
	buf[1] = 3;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	buf[0] = _APDS9960_CONTROL;
	buf[1] = 3;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);

	//Clear all non-gesture interrupts
	buf[0] = _APDS9960_AICLEAR;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 1, HAL_MAX_DELAY);

	//Clear gesture FIFOs and interrupt
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_GCONF4, 1, buf, 1, HAL_MAX_DELAY);
	buf[1] = buf[0];	// move current GCONF4 value into buf[1]
	buf[1] |= _BIT_MASK_GCONF4_GFIFO_CLR;
	buf[0] = _APDS9960_GCONF4;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);

	//Disable sensor and all functions/interrupts
	buf[0] = _APDS9960_ENABLE;
	buf[1] = 0;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	osDelay(1); //Sleeping could take at ~2-25 ms if engines were looping

	//Re-enable sensor and wait 10ms for the power on delay to finish
	buf[0] = 0;
	buf[1] = 0;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_ENABLE, 1, buf, 1, HAL_MAX_DELAY);
	buf[1] = buf[0];	// move current _APDS9960_ENABLE value into buf[1]
	buf[1] |= _BIT_MASK_ENABLE_EN;
	buf[1] |= _BIT_MASK_ENABLE_COLOR;
	buf[0] = _APDS9960_ENABLE;
	ret = HAL_I2C_Master_Transmit(i2cHandle, _APDS9960_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
	osDelay(1);
	buf[0] = 100;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_ENABLE, 1, buf, 1, HAL_MAX_DELAY);

	if(ret != HAL_OK)
	{
		printf("Colour sensor enable FAILED");
		return false;
	}
	printf("Colour Sensor has been enabled\n");
	return true;
}

int colourSensorRead(I2C_HandleTypeDef *i2cHandle, uint16_t* red, uint16_t* green, uint16_t* blue, uint16_t* clear)
{

	int ret = 0;
	uint8_t buf[2];

	// Check if color data ready
	//ret = HAL_I2C_Mem_Read(&hi2c1, _APDS9960_I2C_ADDRESS, _APDS9960_STATUS, 1, buf, 1, HAL_MAX_DELAY);
	//uint8_t c_data_ready = buf[0] & _BIT_MASK_STATUS_AVALID;

	//read colour data from I2C
	buf[0] = 0;
	buf[1] = 0;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_CDATAL, 1, buf, 2, HAL_MAX_DELAY);
	*clear = buf[1] << 8 | buf[0];
	buf[0] = 0;
	buf[1] = 0;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_RDATAL, 1, buf, 2, HAL_MAX_DELAY);
	*red = buf[1] << 8 | buf[0];
	buf[0] = 0;
	buf[1] = 0;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_GDATAL, 1, buf, 2, HAL_MAX_DELAY);
	*green = buf[1] << 8 | buf[0];
	buf[0] = 0;
	buf[1] = 0;
	ret = HAL_I2C_Mem_Read(i2cHandle, _APDS9960_I2C_ADDRESS, _APDS9960_BDATAL, 1, buf, 2, HAL_MAX_DELAY);
	*blue = buf[1] << 8 | buf[0];

	return ret;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_hearbeatTaskFunc */
/**
  * @brief  Function implementing the heartbeatTsk thread. Sends out Hello World message over UART as a heartbeat at a rate of 1Hz
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_hearbeatTaskFunc */
void hearbeatTaskFunc(void *argument)
{
  /* USER CODE BEGIN 5 */
	bool debounce = false;
	/* Infinite loop */
	 for(;;)
	 {
		 if(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		 {
			 debounce = true;
			 osDelay(150);
		 }else if(debounce)
		 {
			 debounce = false;
			 enable_autonomy = !enable_autonomy;
			 printf("Button pressed, new autonomy state %d!", enable_autonomy);
		 }
	 }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_grabberMotorTaskFunc */
/**
* @brief Function implementing the grabberMotorTsk thread.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_grabberMotorTaskFunc */
void grabberMotorTaskFunc(void *argument)
{
  /* USER CODE BEGIN grabberMotorTaskFunc */
  /* This task moves the grabber servo motor from 0degrees to 180degrees then resets to 0degrees and repeats*/
  // 50 -> 100 : 0deg to 180deg linearly
  int i = 0;
  for(;;)
  {
	if (i==100) {
		i=50;
	}
	htim1.Instance->CCR2 = i;
	i+=10;
	osDelay(200);
  }
  /* USER CODE END grabberMotorTaskFunc */
}

/* USER CODE BEGIN Header_colourSensorReadTsk */
/**
* @brief Function implementing the colourSenseRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_colourSensorReadTsk */
void colourSensorReadTsk(void *argument)
{
  /* USER CODE BEGIN colourSensorReadTsk */

	// setup i2C interfaces
	colourSensorSetup(&hi2c3);
	colourSensorSetup(&hi2c1);

	// define buffer variables
	// NOTE: Moved to global variables

	// define calibration values for wavelengths
	uint16_t red_calib_L = 0;
	uint16_t green_calib_L = 0;
	uint16_t blue_calib_L = 0;

	const TickType_t taskPeriod = 2;
	TickType_t lastWakeTime = xTaskGetTickCount();
	 /* Infinite loop */
	for(;;)
	{
		vTaskDelayUntil(&lastWakeTime, taskPeriod);
		// read from sensors
		colourSensorRead(&hi2c1, &red_data_R, &green_data_R, &blue_data_R, &clear_data_R);
		colourSensorRead(&hi2c3, &red_data_L, &green_data_L, &blue_data_L, &clear_data_L);

		// calibrate left sensor
		red_data_L += red_calib_L;
		green_data_L += green_calib_L;
		blue_data_L += blue_calib_L;

		// Calculate ratio of red wavelength to total wavelength strength
		red_ratio_R = ((float)red_data_R / (float)(red_data_R+green_data_R+blue_data_R))*100;
		red_ratio_L = ((float)red_data_L / (float)(red_data_L+green_data_L+blue_data_L))*100;
	 }
  /* USER CODE END colourSensorReadTsk */
}

/* USER CODE BEGIN Header_wheelMotorTask */
/**
* @brief Function implementing the wheelMotorTsk thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_wheelMotorTask */
void wheelMotorTask(void *argument)
{
  /* USER CODE BEGIN wheelMotorTask */
	float buffer = 0;		//3, 6
	float max_pwm = 250;
	uint16_t h_speed = 0; //100, 125
	uint16_t l_speed = 100;	//50 is lowest possible
	float right_adjustment = 1;
	float left_adjustment = 0.82; //0.82 for 75 speed


	const float Kp = 0.8; //4, 10
	const float Ki = 0; //0
	const float Kd = 0; //0
	const float error_max = 30; //30

	float previous_error = 0;
	float integral = 0;

	setMotorDirection(FORWARD, LEFT);
	setMotorDirection(FORWARD, RIGHT);

	const TickType_t taskPeriod = 1;
	TickType_t lastWakeTime = xTaskGetTickCount();
	  /* Infinite loop */
	  for(;;)
	  {
		//vTaskDelayUntil(&lastWakeTime, taskPeriod);
		if(!enable_autonomy)
		{
			setRightMotorDutyCycle(0);
			setLeftMotorDutyCycle(0);
			continue;
		}


		float error = red_ratio_R-red_ratio_L;
		error_signal = error;

		float derivative = -(error-previous_error) / (float)taskPeriod;
		//integral += ((error+previous_error)/2) * taskPeriod; // trapezoidal estimation

		previous_error = error;
		p_term = Kp * error;
		d_term = derivative;
		i_term = Ki * integral;

		control_signal = Kp * error + Kd * derivative + Ki * integral;

		// Motor Control Test
		if(enable_motor_test)
		{
			// Move Forward
			setMotorDirection(FORWARD, LEFT);
			setMotorDirection(FORWARD, RIGHT);
			setLeftMotorDutyCycle(l_speed);
			setRightMotorDutyCycle(l_speed);
			//osDelay(1000);
			continue;

		}
		//control_signal=0;

		if(control_signal > 0)
		{
			//turn right
			//leftMotorDuty = h_speed*control_signal;
			//rightMotorDuty = h_speed-(h_speed*control_signal);
			setMotorDirection(FORWARD, LEFT);
			rightMotorDuty = l_speed; //control_signal;
			if(error > error_max)
			{
				rightMotorDuty = control_signal+l_speed; //control_signal;
				setMotorDirection(BACKWARD, RIGHT);
			}else{
				setMotorDirection(FORWARD, RIGHT);
			}
			leftMotorDuty = control_signal+l_speed; //buffer;
		}
		else if (control_signal < 0)
		{
			//turn left
			//leftMotorDuty = h_speed-(h_speed*-control_signal);
			//rightMotorDuty = h_speed*-control_signal;
			setMotorDirection(FORWARD, RIGHT);
			leftMotorDuty = l_speed;
			if(error < -error_max)
			{
				setMotorDirection(BACKWARD, LEFT);
				leftMotorDuty = -control_signal+l_speed;
			}else{
				setMotorDirection(FORWARD, LEFT);
			}
			rightMotorDuty = -control_signal+l_speed;
		}
		else{
			leftMotorDuty = h_speed;
			rightMotorDuty = h_speed;
		}

		setLeftMotorDutyCycle(leftMotorDuty*left_adjustment);
		setRightMotorDutyCycle(rightMotorDuty);

	  }

  /* USER CODE END wheelMotorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
