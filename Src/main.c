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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uart.h"
#include "gauge.h"
#include "fatfs_sd.h"

#include "fonts.h"
#include "ssd1306.h"
#include "test.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim2;
extern volatile uint32_t time;
extern volatile enum DISK_STATUS card_status;
extern volatile enum AUTO_MEASURE algorithm_status;

extern char data_for_disp[12];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t counter_of_data = 0;
float power_sum = 0;
float resistance_sum = 0;
float pressure_sum = 0;
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM11_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uart_init();
	adc_init();
	//pwm_init();

	//Mount SD Card
	fresult = f_mount(&fs, "", 1);

	if (fresult != FR_OK) {
		uart_tx(&huart2, (uint8_t *)"error in mounting SD CARD...\n");
	}
	else {
		card_status = DISK_OK;
		uart_tx(&huart2, (uint8_t *)"SD CARD mounted successfully...\n");
		//Check card capacity
		f_getfree("", &fre_clust, &pfs);
		total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
		sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
		uart_tx(&huart2, (uint8_t *)buffer);
		bufclear();
		free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
		sprintf(buffer, "SD CARD Free space:\t%lu\n", free_space);
		uart_tx(&huart2, (uint8_t *)buffer);
		bufclear();
		while( ( fresult = f_open(&file,"measure.csv",FA_OPEN_ALWAYS | FA_READ | FA_WRITE )  ) != FR_OK );
		uart_tx(&huart2, (uint8_t *)"measure.csv created\n");
		f_close(&file);
	}

	algorithm_status = CURRENT_100;
	SSD1306_Init();
	dataStruct.pressure = 0;
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		calc_data(&dataStruct, Measure[NUMBERS_ADC_CHANNELS]);
		calc_pressure(&dataStruct);

		sprintf(data_for_disp, "p:%.4fmba", dataStruct.pressure);
		disp_pressure(data_for_disp);
		sprintf(data_for_disp, "R:%.2fOhm", dataStruct.resistanceLoad);
		disp_resistance(data_for_disp);
		sprintf(data_for_disp, "P:%.2fW", dataStruct.powerLoad);
		disp_power(data_for_disp);
		SSD1306_UpdateScreen();
		HAL_Delay(50);
	/*
	if (algorithm_status != STOP_AUTO || measure_status != STOP) {
		calc_data(&dataStruct, Measure[NUMBERS_ADC_CHANNELS]);
		calc_pressure(&dataStruct);

		//pressure_array[counter_of_data] = dataStruct.pressure;
		pressure_sum += dataStruct.pressure;
		//resistance_array[counter_of_data] = dataStruct.resistanceLoad;
		resistance_sum += dataStruct.resistanceLoad;
		//power_array[counter_of_data] = dataStruct.powerLoad;
		power_sum += dataStruct.powerLoad;

		if (counter_of_data == 9) {

			//dataStruct.mean_resistance = resistance_sum / counter_of_data;
 			//dataStruct.mean_pressure = pressure_sum /counter_of_data;
			//dataStruct.mean_power = power_sum /counter_of_data;

			sprintf(data_for_disp, "p:%.3fmba", dataStruct.pressure);
			disp_pressure(data_for_disp);
			sprintf(data_for_disp, "R:%.2fOhm", dataStruct.resistanceLoad);
			disp_resistance(data_for_disp);
			sprintf(data_for_disp, "P:%.2fW", dataStruct.powerLoad);
			disp_power(data_for_disp);
			SSD1306_UpdateScreen();
			HAL_Delay(50);

			resistance_sum = 0;
			pressure_sum = 0;
			power_sum = 0;
			dataStruct.mean_resistance = 0;
			dataStruct.mean_pressure = 0;
			dataStruct.mean_power = 0;
			counter_of_data = 0;
		}
		counter_of_data++;
	} else {
		SSD1306_GotoXY(0,0);
		SSD1306_Puts("UP -> START", &Font_11x18, 1);
		SSD1306_GotoXY(0,25);
		SSD1306_Puts("DOWN ->  ", &Font_11x18, 1);
		SSD1306_GotoXY(0,45);
		SSD1306_Puts("   STOP ", &Font_11x18, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(50);
	}*/
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_480CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 10999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 9999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		number++;
		wobbuling(measure_status, prescaler, arr);
		prepare_message_data(dataStruct, Measure[NUMBERS_ADC_CHANNELS]);
		uart_tx_it(&huart2, data_tx);
		if (card_status == DISK_OK) {
			size_t len = strlen(data_tx);
			strncpy(buffer, data_tx, len);
			fresult = f_puts(buffer, &file);
			bufclear();
		}
	} else if (htim->Instance == TIM11) {
		if (algorithm_status != STOP_AUTO) {
			uart_tx(&huart2, (uint8_t *)"TIM11");
			if (dataStruct.resistanceLoad > 0) {
				if (dataStruct.resistanceLoad >= resistance_map[0][0]
						&& dataStruct.resistanceLoad <= resistance_map[0][2]) {
				} else if (dataStruct.resistanceLoad >= resistance_map[1][0]
						&& dataStruct.resistanceLoad <= resistance_map[1][2]) {
					*ptr_pwm = PWM_TO_CURRENT_100;
					algorithm_status = CURRENT_100;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
				} else if (dataStruct.resistanceLoad >= resistance_map[2][0]
						&& dataStruct.resistanceLoad <= resistance_map[2][2]) {
					*ptr_pwm = PWM_TO_CURRENT_200;
					algorithm_status = CURRENT_200;
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
				}
			}
		}
	}
}

void HAL_SYSTICK_Callback(void) {
	time++;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
	if (recvd_data == '\n') {
		reception_complete = TRUE;
		strncpy(first, data_rx, LEN_FIRST);
		if (strncmp(first, "$SET_", LEN_FIRST) == 0) {
			strncpy(second, data_rx + LEN_FIRST, MAX_LEN_SECOND); //
			if (strncmp(second, "START", strlen("START")) == 0 && measure_status != RUN) {
				measure_status = RUN;

				prescaler = 999;
				pwm_init();
				start_measure_manual();
			} else if (strncmp(second, "STOP", MAX_LEN_SECOND) == 0) {

				measure_status = STOP;
				if (measure_status == RUN) {

					prescaler = 999;
					stop_measure_manual();

				} else if (measure_status == WOBBUL) {
					prescaler = 999;
					stop_wobbul();
				} //which STOP
				else if (algorithm_status != START_AUTO) {
					algorithm_status = STOP_AUTO;
					*ptr_pwm = 0;

					HAL_TIM_Base_Stop_IT(&htim11);
					stop_measure_manual();
				}
			// to STOP
			}else if (strncmp(second, "AUTO", MAX_LEN_SECOND) == 0) {
				algorithm_status = CURRENT_26;
				*ptr_pwm = PWM_TO_CURRENT_26;
				HAL_TIM_Base_Start_IT(&htim11);
				start_measure_manual();
			} else if (strncmp(second, "10000", MAX_LEN_SECOND) == 0) {
				*ptr_pwm = atoi(second) - 1;
				//set_pulse_width(*ptr_pwm);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
			} else if (second[0] == '0') {
				*ptr_pwm = atoi(second);
				//set_pulse_width(*ptr_pwm);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
			} else if (strncmp(second, "WOBB", MAX_LEN_SECOND) == 0) {
				*ptr_pwm = 50 * 9999 / 100; // set PWM at 50%
				if (measure_status == RUN) {
					start_wobbul_raw();
				} else if (measure_status == STOP) {
					time = 0;
					start_wobbul_raw();
				}
			} else {
				uart_tx_it(&huart2, (uint8_t *)"Wrong format. Try again.\n");
			}
		//strncmp to $SET_
		} else if (strncmp(first, (const char )"$RLC_", LEN_FIRST) == 0) {
			strncpy(second, (const char ) (data_rx + (uint8_t)LEN_FIRST), MAX_LEN_SECOND);
			freq_set(second);
		} else {
			uart_tx_it(&huart2, (uint8_t *)"Wrong format. Code should started with $SET_xxxxx.\n");
		}
		count = 0;
		memset(&data_rx, '\0', sizeof(data_rx));
	} else {
		data_rx[count++] = recvd_data;
	}
	HAL_UART_Receive_IT(&huart2, &recvd_data, 1); //Turning on receiving

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
