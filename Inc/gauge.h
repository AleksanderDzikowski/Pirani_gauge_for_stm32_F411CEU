/*
 * gauge.h
 *
 *  Created on: Aug 26, 2019
 *      Author: root
 */

#ifndef GAUGE_H_
#define GAUGE_H_

#include <stdio.h>
#include "main.h"

#define SIZE_FFT_TABLE 2048


//	!!DO NOT CHANGE!!
#define VOLATGE_DIVIDER 3.67f ///Constant value from voltage divider; DO NOT CHANGE
#define CALIBRE_OPAMP 1.0f
#define CALIBRE_REF  1.0f
//	!!DO NOT CHANGE!!

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim10;

typedef struct PIRANI_DATA {
	volatile uint16_t adcData[2];
	volatile float voltageLoad;
	volatile float voltageReferenceResistor;
	volatile float voltageRawOpamp;
	volatile float current;
	volatile float powerLoad;
	volatile float resistanceLoad;
} MeasureData;

MeasureData dataStruct;

uint8_t *first_message;
uint8_t *first_info;

const float referenceResistor;

//Variable used in FFT
volatile uint16_t buffADC[2 * SIZE_FFT_TABLE];
uint8_t State; // 0 - wait, 1 - first half, 2 - second half

volatile uint32_t time; // Current time in milisecond

char data_rx[10]; //Table with received message
char data_tx[150]; //Table with message to send
char first[6], second[6]; // Tables for checking receive string
volatile int16_t value; // Variable using in receiving iterrupt
volatile uint8_t controling; //Variable to seting function
volatile uint16_t set_pwm;
volatile uint16_t *ptr_pwm;
volatile uint32_t number;
// Joistic Res_ref LowPass OpAmp_OUT
uint16_t Measure[2]; // Table contain measure
volatile uint16_t prescaler;
volatile uint16_t *ptr_prescaler;
volatile uint16_t arr;
volatile uint16_t *ptr_arr;

// [ PWM, Rmax, Ropt, Rmin ]
const uint16_t resistance_map[7][4];

const float valueOfBit; //Voltage value of 1b -> 3.3/4096


void uart_init(void);
void pwm_init(void);
void adc_init(void);
void calc_data(MeasureData *measure, uint16_t adc_value[2]);
void prepare_message_data(MeasureData measure, uint16_t adc_value[2]);
void start_measure_manual(void);
void stop_measure_manual(void);
void freq_set(char *);

#endif /* GAUGE_H_ */
