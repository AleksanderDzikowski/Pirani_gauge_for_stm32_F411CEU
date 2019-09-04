/*
 * gauge.c
 *
 *  Created on: Aug 26, 2019
 *      Author: root
 */

#include "gauge.h"
#include <stdio.h>
#include <math.h>
#include "uart.h"
#include "main.h"
#include "fonts.h"
#include "ssd1306.h"

UART_HandleTypeDef huart2;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim10;

uint8_t *first_message =
	{"Start measure\nPWM;ADC_OpAmp;V_OpAmp;V_Load;ADC_Ref;V_Ref;I;P;R;p\n"};

uint8_t *first_info =
{"Type code to set value in per cent to PWM. Correct code format look's like this: $SET_xxxxx where xxx is number between 0 and 10000. You should remembered type three digits for all case "
				"for instance: 081125 for 81,125 %. Default value is 50%\n"
				"In order to change PWM frequency type $RLC_xxxxx where xxxxx is '.5KHZ', '1KHZ_', '2KHZ_', '5KHZ_' and '10KHZ'.\n"
				"If you want to start measure type '$SET_START' and type '$SET_STOP_' for finish.\n"};

uint8_t *adc_message = {"Initialize ADC: "};
uint8_t *pwm_message = {"Initialize PWM: "};

uint8_t *status_confirm = "Status confired!\n";
uint8_t *status_unconfirm = "Status Not confirmed!\n";

const float referenceResistor = 9.9899f;

volatile uint32_t time; // Current time in milisecond
uint8_t data_rx[10] = { 0 }; //Table with received message
uint8_t data_tx[150] = { 0 }; //Table with message to send
uint8_t first[6], second[6]; // Tables for checking receive string
volatile int16_t value = 0; // Variable using in receiving iterrupt
volatile uint16_t set_pwm = 5 * 9999 / 100;
volatile uint16_t *ptr_pwm = &set_pwm;
volatile uint32_t number = 0;
uint8_t reception_complete = FALSE;
uint32_t count = 0;
volatile enum MEASURE_STATUS measure_status = STOP;
volatile enum DISK_STATUS card_status = DISK_ERROR;
volatile int name_number = 1;
volatile char name_file[14] = "measure_1.csv";

size_t LEN_FIRST = 5;
size_t MAX_LEN_SECOND = 5; //max length of second word
// Joistic Res_ref LowPass OpAmp_OUT
uint16_t Measure[NUMBERS_ADC_CHANNELS] = { 0 }; // Table contain measure
volatile uint16_t prescaler = 999;
volatile uint16_t *ptr_prescaler = &prescaler;
volatile uint16_t arr = 9999;
volatile uint16_t *ptr_arr = &arr;

volatile enum AUTO_MEASURE algorithm_status = STOP;
const double pressure_multiplicative[] = {0.42372, 0.47720, 0.5137};
const double pressure_numerator_additive[] = {1055.9, 50.1689, 2.38111};
const double pressure_numerator_multiplicative[] = {51.9634, 2.12694, 0.0718438};
const double pressure_denominator[] = {28.2399, 42.5687, 53.4147};

const float resistance_map[3][3] =
		{
			{ 20.5, 24.25, 27.5 }, //26,6
			{ 24.5, 33.75, 42.5 }, //100
			{ 34.25, 43.75, 52.5 }, //200
		};

const float valueOfBit = 0.0008056640625;

//FATFS variable
FATFS fs; // file system
FIL file; // file
FRESULT fresult; // to store the result
char buffer[1024]; // to store data

UINT br, bw; //file read/write count

//capacity related variables
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void uart_init(void) {
	HAL_UART_Transmit(&huart2, first_info, (uint16_t) strlen(first_info), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, &recvd_data, 1); //Turning on receiving
}

void pwm_init(void) {
	HAL_UART_Transmit(&huart2, pwm_message, (uint16_t) strlen(pwm_message), HAL_MAX_DELAY);
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) == HAL_OK) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		HAL_UART_Transmit(&huart2, status_confirm,(uint16_t) strlen(status_confirm),HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart2, status_unconfirm,(uint16_t) strlen(status_unconfirm), HAL_MAX_DELAY);
	}
}

void adc_init(void) {
	HAL_UART_Transmit(&huart2, adc_message, (uint16_t) strlen(adc_message), HAL_MAX_DELAY);
	if (HAL_ADC_Start(&hadc1) == HAL_OK) {
		HAL_UART_Transmit(&huart2, status_confirm,
				(uint16_t) strlen(status_confirm), HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart2, status_unconfirm,
				(uint16_t) strlen(status_unconfirm), HAL_MAX_DELAY);
	}
}

void calc_data(MeasureData *measure, uint16_t adc_value[NUMBERS_ADC_CHANNELS]) {
	//measure->adcData[0] = Measure[OPAMP_LOCATION];
	//measure->adcData[1] = Measure[REFERENCE_LOCATION];

	measure->voltageRawOpamp = Measure[OPAMP_LOCATION] * valueOfBit * CALIBRE_OPAMP;


	if ( *ptr_pwm >= 8499)
		measure->voltageLoad = CALIBRE_LOAD_POLYNOMIAL_SQUARE *
					((measure->voltageRawOpamp * VOLATGE_DIVIDER)*(measure->voltageRawOpamp * VOLATGE_DIVIDER))
					+ (CALIBRE_LOAD_POLYNOMIAL_LINEAR * (measure->voltageRawOpamp * VOLATGE_DIVIDER));
	else if ( *ptr_pwm <= 1499 )
		measure->voltageLoad = CALIBRE_LOAD_POLYNOMIAL_SQUARE *
							( (measure->voltageRawOpamp * VOLATGE_DIVIDER)*(measure->voltageRawOpamp * VOLATGE_DIVIDER) -ERROR_VOLTAGE)
							+ (CALIBRE_LOAD_POLYNOMIAL_LINEAR * (measure->voltageRawOpamp * VOLATGE_DIVIDER) -ERROR_VOLTAGE);
	else if(*ptr_pwm >= 9599)
		measure->voltageLoad = (CALIBRE_LOAD_POLYNOMIAL_SQUARE *
							((measure->voltageRawOpamp * VOLATGE_DIVIDER)*(measure->voltageRawOpamp * VOLATGE_DIVIDER))
							+ (CALIBRE_LOAD_POLYNOMIAL_LINEAR * (measure->voltageRawOpamp * VOLATGE_DIVIDER))) * CALIBRE_LOAD_LAST;
	else
		measure->voltageLoad = measure->voltageRawOpamp * VOLATGE_DIVIDER * CALIBRE_LOAD_MEAN - ERROR_VOLTAGE;


	/*measure->voltageLoad = CALIBRE_LOAD_POLYNOMIAL_SQUARE *
			(( measure->voltageRawOpamp * VOLATGE_DIVIDER )*( (measure->voltageRawOpamp) * VOLATGE_DIVIDER))
			+ (CALIBRE_LOAD_POLYNOMIAL_LINEAR * ( (measure->voltageRawOpamp) * VOLATGE_DIVIDER));
	*/





	measure->voltageReferenceResistor = Measure[REFERENCE_LOCATION] * valueOfBit * CALIBRE_REF;

	measure->current = (measure->voltageReferenceResistor / referenceResistor) - ERROR_CURRENT;
	measure->powerLoad = measure->voltageLoad * measure->current;

	if (measure->current <= 0) {
		measure->resistanceLoad = 0;
	} else {
		measure->resistanceLoad = measure->voltageLoad / measure->current;
	}
}

void calc_pressure(MeasureData *measure) {
	switch(algorithm_status){
			case CURRENT_26:
				pressure_logarithm_argument = (pressure_numerator_additive[CURRENT_26] - pressure_numerator_multiplicative[CURRENT_26]
											*measure->resistanceLoad) / (measure->resistanceLoad - pressure_denominator[CURRENT_26]);
				measure->pressure = pressure_multiplicative[CURRENT_26] * log(-pressure_logarithm_argument);
				break;
			case CURRENT_100:
				pressure_logarithm_argument = (pressure_numerator_additive[CURRENT_100] - pressure_numerator_multiplicative[CURRENT_100]
											*measure->resistanceLoad) / (measure->resistanceLoad - pressure_denominator[CURRENT_100]);
				measure->pressure = pressure_multiplicative[CURRENT_100] * log(-pressure_logarithm_argument);
				break;
			case CURRENT_200:
				pressure_logarithm_argument = (pressure_numerator_additive[CURRENT_200] - pressure_numerator_multiplicative[CURRENT_200]
											*measure->resistanceLoad) / (measure->resistanceLoad - pressure_denominator[CURRENT_200]);
				measure->pressure = pressure_multiplicative[CURRENT_200] * log(-pressure_logarithm_argument);
				break;
	}

}

void prepare_message_data(MeasureData measure, uint16_t adc_value[NUMBERS_ADC_CHANNELS]) {
	sprintf(data_tx, "%d;%d;%.3f;%.3f;%d;%.3f;%.3f;%.3f;%.3f;%.3f;%d\n",
			*ptr_pwm, Measure[OPAMP_LOCATION], measure.voltageRawOpamp, measure.voltageLoad,
			Measure[REFERENCE_LOCATION], measure.voltageReferenceResistor, measure.current,
			measure.powerLoad, measure.resistanceLoad, measure.pressure, algorithm_status);

}

void start_measure_manual(void) {
	uart_tx_it(&huart2, first_message);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &Measure, NUMBERS_ADC_CHANNELS);
	HAL_TIM_Base_Start_IT(&htim10);
	if (card_status == DISK_OK)
		fresult = f_open(&file, "measure.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
}

void stop_measure_manual(void) {
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop_IT(&htim10);
	if (card_status == DISK_OK)
		fresult = f_close(&file);

}

void start_wobbul_raw(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &Measure, NUMBERS_ADC_CHANNELS);
	HAL_TIM_Base_Start_IT(&htim10); // Starting timer 10
	measure_status = WOBBUL;
	__HAL_TIM_SET_AUTORELOAD(&htim2, 5000);
}

void wobbuling(volatile enum MEASURE_STATUS STAT, volatile uint16_t PRESCALER, volatile uint16_t ARR){
	if (measure_status == WOBBUL && prescaler > 0 && arr != 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, *ptr_prescaler);
		*ptr_prescaler -= 1;

	} else if (measure_status == WOBBUL && prescaler == 0) {
		*ptr_prescaler = 49;
		__HAL_TIM_SET_PRESCALER(&htim2, *ptr_prescaler);
	}
}

void start_wobbul_run(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, *ptr_pwm);
	measure_status = WOBBUL;
}

void set_pulse_width(uint8_t value) {
	if (measure_status == RUN) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, value);
	}
}

void stop_wobbul(void){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_TIM_Base_Stop_IT(&htim10);
	__HAL_TIM_SET_PRESCALER(&htim2, 19);
}

void freq_set(uint8_t *str_correct) {
	if (strncmp(str_correct, ".5KHZ", MAX_LEN_SECOND) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 19);
	} else if (strncmp(str_correct, "1KHZ_", MAX_LEN_SECOND) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 9);
	} else if (strncmp(str_correct, "2KHZ_", MAX_LEN_SECOND) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 4);
	} else if (strncmp(str_correct, "5KHZ_", MAX_LEN_SECOND) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 1);
	} else if (strncmp(str_correct, "10KHZ", MAX_LEN_SECOND) == 0) {
		__HAL_TIM_SET_PRESCALER(&htim2, 0);
	} else {
		uart_tx_it(&huart2, "Wrong format. Try again.\n");
	}
}

uint8_t buff_size (uint8_t *buff) {
	int i = 0;
	while (*buff++ != '\0') i++;
	return i;
}

void bufclear(void) {
	for (uint16_t i = 0; i <1024; i++) {
		buffer[i] = '\0';
	}
}

char data_for_disp[12] = {0};

void disp_pressure(const char *data){
	SSD1306_GotoXY(0,0);
	SSD1306_Puts(data, &Font_11x18, 1);
}

void disp_resistance(const char *data){
	SSD1306_GotoXY(0, 25);
	SSD1306_Puts(data, &Font_11x18, 1);
}

void disp_power(const char *data){
	SSD1306_GotoXY(0, 45);
	SSD1306_Puts(data, &Font_11x18, 1);
}
