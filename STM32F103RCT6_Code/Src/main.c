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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define true 1
#define false 0
#define LEFT_MOTOR_PWM 	TIM_CHANNEL_1

#define RIGHT_MOTOR_PWM	TIM_CHANNEL_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t Rx_data[2];
uint8_t data[20];
uint8_t motor_speed, motor_mode;
uint8_t begin, end, data_idx;

uint8_t change = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void handle_data(void);
void stop_motor(void);
void turn_left(void);
void turn_right(void);
void run_forward(void);
void run_backward(void);
void detect_change(void);
//{
//	if(end)//end = true
//	{
//		uint8_t mode = atoi(&data[0]);
//		
//		end=false;
//	}
//}
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
	uint8_t pwm = 90;
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
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); 
	
	stop_motor();
	HAL_UART_Receive_IT(&huart4,Rx_data,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (true)
  {
		detect_change();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN2_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN2_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void clear_data(void)
{
	for(int i=0;i<20;i++)	{data[i]=0;}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  
{
	uint8_t i;
	if( huart->Instance==UART4)
	{
		switch(Rx_data[0])
		{
			case '*': 
				clear_data();
				begin=true; 
				break;
			case '#': 
				end = true;
				begin = false;
				data_idx = 0;
				HAL_UART_Transmit(&huart4,data,20,1000);
				handle_data();
				//end = false;
				break;
			default:
			{
				if(begin && !end)
				{
					data[data_idx]= Rx_data[0];
					data_idx++;
				}
			}
		}
		HAL_UART_Receive_IT(&huart4,Rx_data,1);
	}
}
uint8_t* substring(const uint8_t* str, int start, int end){
    int length = end - start;
    uint8_t* substr = (uint8_t*)malloc((length+1)*sizeof(uint8_t));
    strncpy(substr, str + start,length);
    substr[length]='\0';
    return substr;
}

void stop_motor(void)
{
	__HAL_TIM_SetCompare(&htim1, LEFT_MOTOR_PWM, 0);
	__HAL_TIM_SetCompare(&htim1, RIGHT_MOTOR_PWM, 0);
	HAL_GPIO_WritePin(GPIOA,IN2_Pin,0);
	HAL_GPIO_WritePin(GPIOA,IN4_Pin,0);
}
void turn_left(void)
{
	__HAL_TIM_SetCompare(&htim1, LEFT_MOTOR_PWM, motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN2_Pin,0);
	HAL_UART_Transmit(&huart4,"\nMotor turn left",16,1000);
}
void turn_right(void)
{
	__HAL_TIM_SetCompare(&htim1, RIGHT_MOTOR_PWM, motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN4_Pin,0);
	HAL_UART_Transmit(&huart4,"\nMotor turn right",17,1000);
}
void run_forward(void)
{
	__HAL_TIM_SetCompare(&htim1, LEFT_MOTOR_PWM, motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN2_Pin,0);
	__HAL_TIM_SetCompare(&htim1, RIGHT_MOTOR_PWM,motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN4_Pin,0);
	HAL_UART_Transmit(&huart4,"\nMotor run fw",13,1000);
}
void run_backward(void)
{
	__HAL_TIM_SetCompare(&htim1, LEFT_MOTOR_PWM, 100-motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN2_Pin,1);
	__HAL_TIM_SetCompare(&htim1, RIGHT_MOTOR_PWM, 100-motor_speed);
	HAL_GPIO_WritePin(GPIOA,IN4_Pin,1);
	HAL_UART_Transmit(&huart4,"\nMotor run bw",13,1000);
}

void handle_data(void)
{
	if(end)//detect end trig
	{
		end = false;
		motor_mode = data[0];
		if(motor_mode=='5')
		{
			HAL_UART_Transmit(&huart4,"speed",5,1000); 		
			uint8_t* speed_str = substring(data,2,5);
			motor_speed = atoi(speed_str);
			free(speed_str);
		}
		change = true;
	}else return;
}
void run_motor(void)
{
	switch(motor_mode)
		{
			case '0':
				stop_motor();
				break;
			case '1': 
				//HAL_UART_Transmit(&huart4,"forward",7,1000); 
				run_forward();
				break;
			case '2': 
				//HAL_UART_Transmit(&huart4,"backward",8,1000); 
				run_backward();
				break;
			case '3': 
				//HAL_UART_Transmit(&huart4,"left",4,1000); 		
				turn_left();
				break;
			case '4': 
				//HAL_UART_Transmit(&huart4,"right",5,1000); 		
				turn_right();
				break;
			default	: HAL_UART_Transmit(&huart4,"invalid",7,1000); 	break;
		}
}
void detect_change(void)
{
	if(change)
	{
		run_motor();
		change = false;
	}
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
