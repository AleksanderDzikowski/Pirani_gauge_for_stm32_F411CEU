/*
 * gauge.c
 *
 *  Created on: Aug 26, 2019
 *      Author: root
 */

#include "gauge.h"
#include <stdio.h>
#include "uart.h"
#include "main.h"

UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

uint8_t *first_message =
		"PWM\t ADC_OpAmp\t V_OpAmp\t V_Load\t ADC_Ref\t V_Ref\t I\t P\t R\n";

uint8_t *first_info =
		"Type code to set value in per cent to PWM. Correct code format look's like this: $SET_xxxxx where xxx is number between 0 and 10000. You should remembered type three digits for all case "
				"for instance: 081125 for 81,125 %. Default value is 50%\n"
				"In order to change PWM frequency type $RLC_xxxxx where xxxxx is '.5KHZ', '1KHZ_', '2KHZ_', '5KHZ_' and '10KHZ'.\n"
				"If you want to start measure type '$SET_START' and type '$SET_STOP_' for finish.\n";

uint8_t *adc_message = "Initialize ADC: ";
uint8_t *pwm_message = "Initialize PWM: ";

uint8_t *status_confirm = "Status Confirm!\n";
uint8_t *status_unconfirm = "Status Unconfirm!\n";

//For FFT usage
volatile uint16_t buffADC[2 * SIZE_FFT_TABLE] = { 0 };
uint8_t State = 0; // 0 - wait, 1 - first half, 2 - second half

const float referenceResistor = 10.0f;

volatile uint32_t time; // Current time in milisecond
char data_rx[10] = { 0 }; //Table with received message
char data_tx[150] = { 0 }; //Table with message to send
char first[6], second[6]; // Tables for checking receive string
volatile int16_t value = 0; // Variable using in receiving iterrupt
volatile uint8_t controling = 0; //Variable to seting function
volatile uint16_t set_pwm = 50 * 9999 / 100;
volatile uint16_t *ptr_pwm = &set_pwm;
volatile uint32_t number = 0;
// Joistic Res_ref LowPass OpAmp_OUT
uint16_t Measure[2] = { 0 }; // Table contain measure
volatile uint16_t prescaler = 999;
volatile uint16_t *ptr_prescaler = &prescaler;
volatile uint16_t arr = 9999;
volatile uint16_t *ptr_arr = &arr;

// [ PWM, Rmax, Ropt, Rmin ]
const uint16_t resistance_map[7][4] = { { 1733, 26, 24, 22 },
		{ 2533, 29, 26, 23 }, { 3333, 32, 28, 24 }, { 5000, 36, 31, 26 }, {
				6667, 40, 34, 27 }, { 8333, 44, 37, 30 }, { 9999, 47, 40, 32 } };
const float valueOfBit = 0.0008056640625;

void uart_init(void) {
	HAL_UART_Transmit(&huart2, first_info, (uint16_t) strlen(first_info), 1000);
	HAL_UART_Receive_IT(&huart2, data_rx, 11); //Turning on receiving
}

void pwm_init(void) {
	HAL_UART_Transmit(&huart2, pwm_message, (uint16_t) strlen(pwm_message),
			1000);
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) == HAL_OK) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		HAL_UART_Transmit(&huart2, status_confirm,
				(uint16_t) strlen(status_confirm), 1000);
	} else {
		HAL_UART_Transmit(&huart2, status_unconfirm,
				(uint16_t) strlen(status_unconfirm), 1000);
	}
}

void adc_init(void) {
	HAL_UART_Transmit(&huart2, adc_message, (uint16_t) strlen(adc_message),
			1000);
	if (HAL_ADC_Start(&hadc1) == HAL_OK) {
		HAL_UART_Transmit(&huart2, status_confirm,
				(uint16_t) strlen(status_confirm), 1000);
	} else {
		HAL_UART_Transmit(&huart2, status_unconfirm,
				(uint16_t) strlen(status_unconfirm), 1000);
	}
}

void calc_data(MeasureData *measure, uint16_t adc_value[2]) {
	measure->adcData[0] = Measure[0];
	measure->adcData[1] = Measure[1];

	measure->voltageLoad = measure->adcData[0] * valueOfBit * VOLATGE_DIVIDER
			* CALIBRE_OPAMP;
	measure->voltageRawOpamp = measure->adcData[1] * valueOfBit;
	measure->voltageReferenceResistor = measure->adcData[1] * valueOfBit
			* CALIBRE_REF;

	measure->current = measure->voltageReferenceResistor / referenceResistor;
	measure->powerLoad = measure->voltageLoad * measure->current;

	if (measure->current <= 0) {
		measure->resistanceLoad = 0;
	} else {
		measure->resistanceLoad = measure->voltageLoad / measure->current;
	}
}

void prepare_message_data(MeasureData measure, uint16_t adc_value[2]) {
	sprintf(data_tx, "%d\t %d\t %.4f\t %.4f\t %d\t %.4f\t %.4f\t %.4f\t %.4f\n",
			*ptr_pwm, Measure[0], measure.voltageRawOpamp, measure.voltageLoad,
			Measure[1], measure.voltageReferenceResistor, measure.current,
			measure.powerLoad, measure.resistanceLoad);
}

void start_measure_manual(void) {
	uart_tx_it(&huart2, "Start measure\n");
	uart_tx_it(&huart2, first_message);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &Measure, 2);
	HAL_TIM_Base_Start_IT(&htim10);
}

void stop_measure_manual(void) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop_IT(&htim10);
}

void freq_set(char *str_correct) {
	if (strncmp(str_correct, ".5KHZ", 5) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 19);
	} else if (strncmp(str_correct, "1KHZ_", 5) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 9);
	} else if (strncmp(str_correct, "2KHZ_", 5) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 4);
	} else if (strncmp(str_correct, "5KHZ_", 5) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 1);
	} else if (strncmp(str_correct, "10KHZ", 5) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 0);
	} else {
		uart_tx_it(&huart2, "Wrong format. Try again.\n");
	}
}
