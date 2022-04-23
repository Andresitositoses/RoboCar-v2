/*
 * print.c
 *
 *  Created on: Apr 23, 2022
 *      Author: andre
 */

#include "print.h"
#include "string"

void print(UART_HandleTypeDef *huart, char *str) {
	std::string cadena = str;
	for (unsigned int i = 0; i < cadena.length(); i++) {
		HAL_UART_Transmit(huart, (uint8_t*) &cadena.at(i), 1, HAL_MAX_DELAY);
	}
}

void print(UART_HandleTypeDef *huart, float num) {
	std::string cadena = std::to_string(num);
	for (unsigned int i = 0; i < cadena.length(); i++) {
		HAL_UART_Transmit(huart, (uint8_t*) &cadena.at(i), 1, HAL_MAX_DELAY);
	}
}

void print(UART_HandleTypeDef *huart, char *str, float num) {
	print(huart, str);
	print(huart, num);
	print(huart, "\n");
}
