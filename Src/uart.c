/*
 * uart.c
 *
 *  Created on: Jun 6, 2019
 *      Author: root
 */

#include "uart.h"


uint32_t time_uart = 500;

void uart_tx(UART_HandleTypeDef *str_uart, char *text) {
	HAL_UART_Transmit(str_uart, text, strlen(text), time_uart);
}

void uart_tx_it(UART_HandleTypeDef *str_uart, char *text) {
	HAL_UART_Transmit_IT(str_uart, text, strlen(text));
	HAL_UART_Transmit(str_uart, "\n", 2, time_uart);

}


