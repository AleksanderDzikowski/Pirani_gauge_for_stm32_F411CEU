/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE 2048
#define ADC1_DR_Address 0x4001204C;
#define REFERENCE_RESISTOR 22.0
//	!!DO NOT CHANGE!!
#define VOLATGE_DIVIDER 3.15f ///Constant value from voltage divider; DO NOT CHANGE
#define CALIBRE_OPAMP 1.1657f
#define CALIBRE_REF  0.9427f
//	!!DO NOT CHANGE!!

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t first_message[] =
		"Nr\t Time[s]\t U_Out[V]\t U_Resis[V]\t P[mW]\t I[mA]\t PWM[%]\n";

uint8_t first_info[] =
		{
				"Type code to set value in per cent to PWM. Correct code format look's like this: $SET_xxxxx\nwhere xxx is number between 0 and 10000. You should remembered type three digits for all case \nfor instance: 081125 for 81,125 %.\n" };
uint8_t second_info[] =
		{
				"If you want to start measure type '$SET_START' and type '$SET_STOP_' for finish.\n" };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t time; // Current time in milisecond
char data_rx[10] = { 0 }; //Table with received message
char data_tx[100] = { 0 }; //Table with message to send
char first[6], second[6]; // Tables for checking receive string
volatile int16_t value = 0; // Variable using in receiving iterrupt
volatile uint8_t controling = 0; //Variable to seting function
volatile uint16_t set_pwm = 50 * 9999 / 100;
volatile uint16_t *ptr_pwm = &set_pwm;
volatile uint32_t number = 0;
// Joistic Res_ref LowPass OpAmp_OUT
volatile uint16_t Measure[2] = { 0 }; // Table contain measure
volatile uint16_t prescaler = 999;
volatile uint16_t *ptr_prescaler = &prescaler;
volatile uint16_t arr = 9999;
volatile uint16_t *ptr_arr = &arr;
volatile float resistance;
volatile float Voltage[2] = { 0 };
volatile float Power = 0;
volatile float Current = 0;
volatile float Pwm_per_cent = 0;
// [ PWM, Rmax, Ropt, Rmin ]
const uint16_t resistance_map[7][4] = { { 1733, 26, 24, 22 },
		{ 2533, 29, 26, 23 }, { 3333, 32, 28, 24 }, { 5000, 36, 31, 26 }, {
				6667, 40, 34, 27 }, { 8333, 44, 37, 30 }, { 9999, 47, 40, 32 } };
//Variable used in FFT
volatile uint16_t buffADC[2 * SIZE] = { 0 };
uint8_t State = 0; // 0 - wait, 1 - first half, 2 - second half

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_UART_Transmit(&huart2, first_info,
			(uint16_t) strlen(first_info), 1000);
	HAL_UART_Transmit(&huart2, second_info,
			(uint16_t) strlen(second_info), 1000);
	HAL_UART_Receive_IT(&huart2, data_rx, 11); //Turning on receiving
	//HAL_UART_Receive_IT(&huart2, data_rx, 11); //Turning on receiving

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	HAL_ADC_Start(&hadc1);

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Voltage[0] = (Measure[0]) * (3.3f / 4096.0f) * VOLATGE_DIVIDER * CALIBRE_REF; //
		Voltage[1] = (Measure[1]) * (3.3f / 4096.0f) * CALIBRE_OPAMP; //

		Current = Voltage[1] / 10.0f;
		Power = Voltage[0] * Current;
		//Pwm_per_cent = (float) set_pwm / 9999 * 100.0f;
		if (Current <= 0) {
			resistance = 0;
		} else {
			resistance = Voltage[0] / Current;
		}


		//sprintf(data_tx, "REF: %f\t HEAD: %f\n", Voltage[0], Voltage[1]);
		//sprintf(data_tx, "REF: %d\t HEAD: %d\n", Measure[0], Measure[1]);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 49;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		number++;
		//uart_tx_it(&huart2, data_tx);
		if (controling == 2 && prescaler > 0 && arr != 0) {
			__HAL_TIM_SET_PRESCALER(&htim2, *ptr_prescaler);
			*ptr_prescaler -= 1;

		} else if (controling == 2 && prescaler == 0) {
			*ptr_prescaler = 49;
			__HAL_TIM_SET_PRESCALER(&htim2, *ptr_prescaler);
		}
		sprintf(data_tx,
						"PWM:%d\t ADC_OpAmp:%d\t V_OpAmp:%.4f\t ADC_Ref:%d\t V_Ref:%.4f\t I:%.4f\t P:%.4f\t R:%.4f\n",
						*ptr_pwm, Measure[0], Voltage[0], Measure[1], Voltage[1],
						Current, Power, resistance);
		uart_tx_it(&huart2, data_tx);
	}
}
void HAL_SYSTICK_Callback(void) {
	time++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
	HAL_UART_Receive_IT(&huart2, (uint8_t*) data_rx, 11); //Turning on receiving
	if (uart == &huart2) {
		strncpy(first, data_rx, 5);
		if (strncmp(first, "$SET_", 5) == 0) {
			strncpy(second, data_rx + 5, 5);
			if (strncmp(second, "START", 5) == 0 && controling != 1) {
				time = 0;
				uart_tx_it(&huart2, first_message);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
				//HAL_ADC_Start(&hadc1);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &Measure, 2);
				controling = 1;
				prescaler = 999;
				HAL_TIM_Base_Start_IT(&htim10);
				uart_tx_it(&huart2, "Start measure\n");
			} else if (strncmp(second, "STOP_", 5) == 0 && controling != 0) {
				if (controling == 1) {
					controling = 0;
					prescaler = 999;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
					//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
					HAL_ADC_Stop_DMA(&hadc1);
					//HAL_ADC_Stop(&hadc1);
					HAL_TIM_Base_Stop_IT(&htim10);
				} else if (controling == 2) {
					controling = 0;
					prescaler = 999;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
					HAL_ADC_Stop_DMA(&hadc1);
					HAL_ADC_Stop(&hadc1);
					HAL_TIM_Base_Stop_IT(&htim10);
					__HAL_TIM_SET_PRESCALER(&htim2, 19);
				}
			} else if (strncmp(second, "10000", 5) == 0) {
				*ptr_pwm = atoi(second) - 1;
				if (controling == 1) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
				}
			} else if (second[0] == '0') {
				*ptr_pwm = atoi(second);
				if (controling == 1) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
				}
			} else if (strncmp(second, "RAKE_", 5) == 0) {
				*ptr_pwm = 50 * 9999 / 100;
				if (controling == 1) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
					controling = 2;
				} else if (controling == 0) {
					time = 0;
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
					HAL_ADC_Start(&hadc1);
					HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &Measure, 2);
					HAL_TIM_Base_Start_IT(&htim10); // Starting timer 10
					controling = 2;
					__HAL_TIM_SET_AUTORELOAD(&htim2, 5000);
				}
			} else if (strncmp(second, "FFT__", 5) == 0) {
				HAL_ADC_Start(&hadc1);
				//HAL_ADC_Start_DMA(&hadc1, buffADC, 2 * SIZE);
			} else {
				uart_tx_it(&huart2, "Wrong format. Try again.\n");
			}

		} else if (strncmp(first, "$RLC_", 5) == 0) {
			strncpy(second, data_rx + 5, 5);
			if (strncmp(second, ".5KHZ", 5) == 0) {
				__HAL_TIM_SET_PRESCALER(&htim2, 19);
			} else if (strncmp(second, "1KHZ_", 5) == 0) {
				__HAL_TIM_SET_PRESCALER(&htim2, 9);
			} else if (strncmp(second, "2KHZ_", 5) == 0) {
				__HAL_TIM_SET_PRESCALER(&htim2, 4);
			} else if (strncmp(second, "5KHZ_", 5) == 0) {
				__HAL_TIM_SET_PRESCALER(&htim2, 1);
			} else if (strncmp(second, "10KHZ", 5) == 0) {
				__HAL_TIM_SET_PRESCALER(&htim2, 0);
			} else {
				uart_tx_it(&huart2, "Wrong format. Try again.\n");
			}
		} else {
			uart_tx_it(&huart2,
					"Wrong format. Code should started with $SET_xxxxx.\n");
		}
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
