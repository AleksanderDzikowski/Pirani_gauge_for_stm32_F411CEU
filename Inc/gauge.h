/*
 * gauge.h
 *
 *  Created on: Aug 26, 2019
 * Author: 		: Aleksander Dzikowski
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
#define VOLATGE_DIVIDER  (3.67f) ///Constant value from voltage divider; DO NOT CHANGE
#define CALIBRE_LOAD (1.0f)
#define CALIBRE_LOAD_MEAN (1.01924f)
#define CALIBRE_LOAD_LAST (1.02f)
#define CALIBRE_LOAD_POLYNOMIAL_SQUARE (0.0000035231f)
#define CALIBRE_LOAD_POLYNOMIAL_LINEAR (1.06661f)
#define CALIBRE_REF  (1.00119f)
#define CALIBRE_OPAMP (0.997659f)
#define ERROR_VOLTAGE (0.18f)
#define ERROR_CURRENT (0.00083f)

#define PWM_TO_CURRENT_26 1099
#define PWM_TO_CURRENT_100 3036
#define PWM_TO_CURRENT_200 6102
//	!!DO NOT CHANGE!!

#define NUMBERS_ADC_CHANNELS 2
#define OPAMP_LOCATION 0
#define REFERENCE_LOCATION 1

extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim10;

typedef struct PIRANI_DATA {
	volatile uint16_t adcData[NUMBERS_ADC_CHANNELS];
	volatile float voltageLoad;
	volatile float voltageReferenceResistor;
	volatile float voltageRawOpamp;
	volatile float current;
	volatile float powerLoad;
	volatile float resistanceLoad;
	volatile double pressure;
	volatile double mean_pressure;
	volatile double mean_resistance;
	volatile double mean_power;
} MeasureData;

enum MEASURE_STATUS { STOP=0, RUN=1, WOBBUL = 2};
enum DISK_STATUS {DISK_OK = 0, DISK_ERROR = 1};
enum AUTO_MEASURE {START_AUTO = -2, STOP_AUTO = -1, CURRENT_26 = 0, CURRENT_100 = 1, CURRENT_200 = 2};

MeasureData dataStruct;
const float resistance_map[3][3];

const double pressure_multiplicative_26;
const double pressure_multiplicative_100;
const double pressure_multiplicative_200;

const double pressure_numerator_additive_26;
const double pressure_numerator_additive_100;
const double pressure_numerator_additive_200;

const double pressure_numerator_multiplicative_26;
const double pressure_numerator_multiplicative_100;
const double pressure_numerator_multiplicative_200;

const double pressure_denominator_26;
const double pressure_denominator_100;
const double pressure_denominator_200;
volatile double pressure_logarithm_argument;

uint8_t *first_message;
uint8_t *first_info;

const float referenceResistor;

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
volatile enum MEASURE_STATUS measure_status; //Variable to seting function
volatile uint16_t set_pwm;
volatile uint16_t *ptr_pwm;
volatile uint32_t number;
// Joistic Res_ref LowPass OpAmp_OUT
uint16_t Measure[NUMBERS_ADC_CHANNELS]; // Table contain measure

//variables for wobbulating function. This function changing value of PSC and ARR in timer
volatile uint16_t prescaler;
volatile uint16_t *ptr_prescaler;
volatile uint16_t arr;
volatile uint16_t *ptr_arr;


const float valueOfBit; //Voltage value of 1b -> 3.3/4096

//FATFS variable
FATFS fs; // file system
FIL file; // file
FRESULT fresult; // to store the result
char buffer[1024]; // to store data

UINT br, bw; //file read/write count
volatile int name_number;
//capacity related variables
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char data_for_disp[12];

void uart_init(void);
void pwm_init(void);
void adc_init(void);
void calc_data(MeasureData *measure, uint16_t adc_value[NUMBERS_ADC_CHANNELS]);
void calc_pressure(MeasureData *measure);
void prepare_message_data(MeasureData measure, uint16_t adc_value[NUMBERS_ADC_CHANNELS]);
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
void disp_pressure(const char *data);
void disp_resistance(const char *data);
void disp_power(const char *data);


#endif /* GAUGE_H_ */
