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
#include "fatfs.h"
#include "fatfs_sd.h"

#define SIZE_FFT_TABLE 2048

#define TRUE 1
#define FALSE 0

//	!!DO NOT CHANGE!!
#define VOLATGE_DIVIDER 3.67f ///Constant value from voltage divider; DO NOT CHANGE
#define CALIBRE_LOAD 1.12805f
#define CALIBRE_REF  1.00119f
#define CALIBRE_OPAMP 0.9976f
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

enum MEASURE_STATUS { STOP=0, RUN=1, WOBBUL = 2};

MeasureData dataStruct;

uint8_t *first_message;
uint8_t *first_info;

const float referenceResistor;

//Variable used in FFT
volatile uint16_t buffADC[2 * SIZE_FFT_TABLE];
uint8_t State; // 0 - wait, 1 - first half, 2 - second half

volatile uint32_t time; // Current time in milisecond

uint8_t data_rx[10]; //Table with received message
uint8_t data_tx[150]; //Table with message to send
uint8_t recvd_data;
uint32_t count;
uint8_t reception_complete;
size_t LEN_FIRST;
size_t MAX_LEN_SECOND; //max length of second word
uint8_t first[6], second[6]; // Tables for checking receive string
uint8_t recv_comend[100];

volatile int16_t value; // Variable using in receiving iterrupt
volatile enum MEASURE_STATUS status; //Variable to seting function
volatile uint16_t set_pwm;
volatile uint16_t *ptr_pwm;
volatile uint32_t number;
// Joistic Res_ref LowPass OpAmp_OUT
uint16_t Measure[2]; // Table contain measure

//variables for wobbulating function. This function changing value of PSC and ARR in timer
volatile uint16_t prescaler;
volatile uint16_t *ptr_prescaler;
volatile uint16_t arr;
volatile uint16_t *ptr_arr;

// [ PWM, Rmax, Ropt, Rmin ]
const uint16_t resistance_map[7][4];

const float valueOfBit; //Voltage value of 1b -> 3.3/4096

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


void uart_init(void);
void pwm_init(void);
void adc_init(void);
void calc_data(MeasureData *measure, uint16_t adc_value[2]);
void prepare_message_data(MeasureData measure, uint16_t adc_value[2]);
void start_measure_manual(void);
void stop_measure_manual(void);
void start_wobbul_raw(void);
void start_wobbul_run(void);
void set_pulse_width(uint8_t );
void stop_wobbul(void);
void wobbuling(volatile enum MEASURE_STATUS, volatile uint16_t, volatile uint16_t );
void freq_set(uint8_t *);
uint8_t buff_size (uint8_t *);
void bufclear(void);


#endif /* GAUGE_H_ */
