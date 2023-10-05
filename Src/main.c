/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CAM_ADDRESS				((uint8_t)(51))
#define BYTES16(x, y)			((uint16_t)(x)) + (((uint16_t)(y)) << 8)

#define STATE_CATCH_BALL		1
#define STATE_ROTATE_BACK 	2
#define STATE_STOP_ROBOT		3

struct TrackingCAMBlobInfo_t
{
	uint8_t type,
					dummy;
	uint16_t cx,
					cy;
	uint32_t area;
	uint16_t left,
					right,
					top,
					bottom;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
struct TrackingCAMBlobInfo_t cam_blobs[10];

uint8_t irseeker_received[11];
uint8_t is_ball_exist, irseeker_status;
float ball_angle, ball_distance;

volatile float wt_z_angle = 0.0f;
uint8_t wt901_recieved;
struct PIDUnion wt901_pid;

float optical_pos_x = 0.0f,
			optical_pos_y = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t CAM_Update(void);
uint8_t irseeker_parser(uint8_t buffer[11], uint8_t* is_exist, float* angle, float* distance);

void wt901_serialcopy(uint8_t ucData);
void bytes2float(uint8_t *a, float* val);

void play(uint8_t *state);
uint8_t ball_in_mouth(void);

uint8_t	pickup_position_opt(float yx, float yy, float bx, float by, float *x, float *y);
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
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	
	// start timer for motors PWM
	// motor A
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	// motor B
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	// motor C
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	
	// motor D
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	
	// dribbler
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	TIM1->CCR2 = 965;
	HAL_Delay(6000);

	wt901_pid.kp = 2.8f;
	wt901_pid.ki = 0.00125f;
	wt901_pid.kd = 15.0f;
	wt901_pid.angle_wrap = 1;
	wt901_pid.out_min = -160.0f;
	wt901_pid.out_max = 160.0f;
	wt901_pid.setpoint = 0.0f;
	
	uint8_t game_state = STATE_CATCH_BALL;
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)irseeker_received, 11);
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)&wt901_recieved, 1);
	while (1)
  {
		uint8_t n = CAM_Update();
		if(n > 0)
		{
			float yellow_goal_x = 0,
						yellow_goal_y = 0,
						blue_goal_x = 0,
						blue_goal_y = 0;
			uint8_t yellow_exist = 0,
					 blue_exist = 0;
			
			for(uint8_t i = 0; i < n; ++i)
			{
					struct TrackingCAMBlobInfo_t info = cam_blobs[i];
				
					if(info.type == 0 && yellow_exist == 0)
					{
						// yellow goal
						yellow_goal_x = (float)info.cx;
						yellow_goal_y = (float)info.cy;
						yellow_exist = 1;
					}
					else if(info.type == 0)
					{
						// blue goal
						blue_goal_x = (float)info.cx;
						blue_goal_y = (float)info.cy;
						blue_exist = 1;
					}
			}
			
			if(blue_exist && yellow_exist)
			{
				if(blue_goal_x > yellow_goal_x)
				{
					float tmp = yellow_goal_x;
					yellow_goal_x = blue_goal_x;
					blue_goal_x = tmp;
					
					tmp = yellow_goal_y;
					yellow_goal_y = blue_goal_y;
					blue_goal_y = tmp;
				}
				
				pickup_position_opt(yellow_goal_x, yellow_goal_y, blue_goal_x, blue_goal_y, &optical_pos_x, &optical_pos_y);
			}
		}

		play(&game_state);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim1.Init.Prescaler = 320-1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 255;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 255;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint8_t CAM_Update()
{
	const static uint8_t len = 16;
	uint8_t resp[len];
	uint8_t n = 0,
					idx = 0;
	uint8_t lines = 10;
	
	for(uint8_t i = 0; i < lines; ++i)
	{
		// trackingcam read blobs
		uint8_t addr = len + i * len;
		if(HAL_I2C_Master_Transmit(&hi2c1, (CAM_ADDRESS << 1), &addr, 1,  10) == HAL_ERROR)
		{
			return n;
		}
		if(HAL_I2C_Master_Receive(&hi2c1, (CAM_ADDRESS << 1), &resp[0], len,  10) == HAL_ERROR)
		{
			return n;
		}
		// 
		
		idx = 0;
		cam_blobs[i].type = resp[idx++];
		if(cam_blobs[i].type == 0xFF)
		{
			return n;
		}
		else
		{
			n++;
		}
		
		cam_blobs[i].dummy = resp[idx++];
		cam_blobs[i].cx = BYTES16(resp[idx], resp[idx + 1]);
		cam_blobs[i].cy = BYTES16(resp[idx + 2], resp[idx + 3]);
		cam_blobs[i].area = (uint32_t)BYTES16(resp[idx + 4], resp[idx + 5]) * 4;
		cam_blobs[i].left = BYTES16(resp[idx + 6], resp[idx + 7]);
		cam_blobs[i].right = BYTES16(resp[idx + 8], resp[idx + 9]);
		cam_blobs[i].top = BYTES16(resp[idx + 10], resp[idx + 11]);
		cam_blobs[i].bottom = BYTES16(resp[idx + 12], resp[idx + 13]);
	}
	
	return n;
}

uint8_t irseeker_parser(uint8_t buffer[11], uint8_t* is_exist, float* angle, float* distance)
{
	uint8_t angle_code = buffer[0],
					distance_code = buffer[6];
	
	if(angle_code != 97 || distance_code != 100)
	{
		return 0;
	}
	
	(*is_exist) = buffer[1];
	bytes2float(buffer + 2, angle);
	bytes2float(buffer + 7, distance);
	
	if((*angle) < -180.0f || (*angle) > 180.0f)
	{
		return 0;
	}
	
	return 1;
}

void bytes2float(uint8_t *a, float* val)
{
	memcpy(val, a, 4);
}

void wt901_serialcopy(uint8_t ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	struct SAngle
	{
		short Angle[3];
		short T;
	};
	
	ucRxBuffer[ucRxCnt++] = ucData;
	if(ucRxBuffer[0] != 0x55)
	{
		ucRxCnt = 0;
		return;
	}
	
	if (ucRxCnt < 11) { return; }
	else
	{
		switch(ucRxBuffer[1])
		{
			// RPY
			case 0x53:	
			{
				struct SAngle stcAngle;
				memcpy(&stcAngle,&ucRxBuffer[2],8);
				wt_z_angle = (float)stcAngle.Angle[2]/ 32768.0f * 180.0f;
				break;
			}
		}
		ucRxCnt=0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if(huart == &huart3) 
	{	
		wt901_serialcopy(wt901_recieved);
	}
	
	if(huart == &huart1)
	{
		irseeker_status = irseeker_parser(irseeker_received, &is_ball_exist, &ball_angle, &ball_distance);
	}
}

void play(uint8_t *state)
{
	static uint32_t dribler_timer = 0;
	static uint8_t dribler_st = 0;
	static int16_t wt901_angular_vel = 0;

	if((*state) == STATE_CATCH_BALL)
	{
		uint8_t ball_vel = 230;
		
		if(ball_distance < 110.0f && dribler_st == 0)
		{
			dribler_st = 1;
			dribler_timer = HAL_GetTick();
		}
		
		if(ball_distance < 110.0f && dribler_st == 1 && (HAL_GetTick() - dribler_timer) >= 1000)
		{
			// off dribler
			TIM1->CCR2 = 965;
		}
		
		if(ball_distance >= 110.0f)
		{
			// on dribler
			TIM1->CCR2 = 935;
			ball_vel = 160;
			
			dribler_st = 0;
			dribler_timer = 0;
		}
    
		wt901_pid.input = wt_z_angle;
		if(compute(&wt901_pid, 3))
		{
			wt901_angular_vel = (int16_t)wt901_pid.output;
			
			float dir = 0;
			if(abs(optical_pos_x) > 0.14f || abs(optical_pos_y) > 0.55f)
			{
				dir = atan2f(optical_pos_y, optical_pos_x) * (180.0f / PI);
			}
			else if(irseeker_status != 0 && is_ball_exist != 0)
			{
				dir = fabs(ball_angle) * 1.16666666667f + 40.0f;
				dir = (ball_angle < 0 ? -dir : dir) * (ball_distance / 95.0f);
					
				if(fabs(ball_angle) <= 10.0f)
				{
					dir = 0.0f;
				}
			}
			else
			{
				ball_vel = 0;
			}
			move(ball_vel, dir, wt901_angular_vel);
		}
		
		if(ball_in_mouth())
		{
			(*state) = STATE_ROTATE_BACK;
		}
	}
	else if((*state) == STATE_ROTATE_BACK)
	{
		if(ball_in_mouth() == 0)
		{
			(*state) = STATE_CATCH_BALL;
			return;
		}
		
		wt901_pid.input = wt_z_angle;
		wt901_pid.setpoint = 180.0f;
		if(compute(&wt901_pid, 3))
		{
			wt901_angular_vel = (int16_t)wt901_pid.output;
			
			if(fabs(180.0f - fabs(wt_z_angle)) <= 10.0f)
			{
				// robot state is not back
				(*state) = STATE_STOP_ROBOT;
				return;
			}
			
			move(160.0f, 180.0f, wt901_angular_vel);
		}
	}
	else if((*state) == STATE_STOP_ROBOT)
	{
		move(0, 0.0f, 0);
	}
}

uint8_t ball_in_mouth(void)
{
	static uint8_t in_mouth = 0;
	static uint32_t in_mouth_timer = 0;
	
	if(ball_distance >= 140.0f && in_mouth == 0)
	{
		in_mouth = 1;
		in_mouth_timer = HAL_GetTick();
	}
	
	if(ball_distance >= 140.0f && in_mouth && (HAL_GetTick() - in_mouth_timer) >= 600)
	{
		return 1;
	}
	
	if(ball_distance < 120.0f)
	{
		in_mouth = 0;
		in_mouth_timer = 0;
	}
	
	return 0;
}

void interpolation(float *x, float *y)
{
	static const int n = 4;
	static const float k[n] = 
	{
		-252.0f,
		17.7f,
		-0.248f,
		0.00152f
	};
	
	float cx = (*x) - 157.0f,
				cy = 131.0f - (*y);
	float r = sqrtf(cx * cx + cy * cy),
				r_interpolation = 0;
	float alpha = atan2f(cy, cx);
	
	for(uint8_t i = 0; i < n; ++i)
	{
		r_interpolation += k[i] * powf(r, (float)i);
	}
	
	r_interpolation /= 2627.0f;
	(*x) = r_interpolation * cosf(alpha);
	(*y) = r_interpolation * sinf(alpha);
}

uint8_t	pickup_position_opt(float yx, float yy, float bx, float by, float *x, float *y)
{
	float yellow_radius = sqrtf(powf(yx - 152.0f, 2) + powf(yy - 134.5f, 2));
	float blue_radius = sqrtf(powf(bx - 152.0f, 2) + powf(by - 134.5f, 2));
	float gate_distance = sqrtf(powf(yx - bx, 2) + powf(yy - by, 2));
				
	float gate_k = (yy - by) / (yx - bx);
	float gate_b = yy - gate_k * yx;
				
	yellow_radius = (yellow_radius / gate_distance) * 2.0f;
	blue_radius = (blue_radius / gate_distance) * 2.0f;
				
	if((yellow_radius + blue_radius) < 2.0f)
	{
		return 0;
	}
	
	float cos_alpha = (powf(yellow_radius, 2) - powf(blue_radius, 2) + 4.0f) / (4.0f * yellow_radius);
	float sin_alpha = sqrtf(1 - powf(cos_alpha, 2));
					
	(*x) = 1 - cos_alpha * yellow_radius;
	(*y) = sin_alpha * yellow_radius;			
	if((gate_k * 152.0f + gate_b) > 134.5f)
	{
		(*y) *= -1;
	}
	
	return 1;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
