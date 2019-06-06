/*
 * uart.h
 *
 *  Created on: Jun 6, 2019
 *      Author: root
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "string.h"

extern uint32_t time_uart;

void uart_tx_it(UART_HandleTypeDef *str_uart, char *text);

#endif /* UART_H_ */
