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

void uart_rx_it(UART_HandleTypeDef *str_uart, char *buff, char *buff_size) {
	HAL_StatusTypeDef status;
	char current_char;
	uint16_t char_counter = 0;

	while(char_counter < buff_size - 1) {
		status = HAL_UART_Receive_IT(str_uart, &current_char, 1);
		if (status == HAL_OK) {
			if (current_char == '\n') {
				if (char_counter == 0) continue;
				else break;
			*(buff + char_counter++) = current_char;
			}
		}
	}
	*(buff + char_counter) = '\0';
}

