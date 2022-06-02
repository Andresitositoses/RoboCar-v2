/*
 * print.c
 *
 *  Created on: Apr 23, 2022
 *      Author: andre
 */

#include "print.h"

// Char *
void print(UART_HandleTypeDef *huart, char *str) {
	string cadena = str;
	for (unsigned int i = 0; i < cadena.length(); i++) {
		HAL_UART_Transmit(huart, (uint8_t*) &cadena.at(i), 1, HAL_MAX_DELAY);
	}
}

// Float
void print(UART_HandleTypeDef *huart, float num) {
	string cadena = to_string(num);
	for (unsigned int i = 0; i < cadena.length(); i++) {
		HAL_UART_Transmit(huart, (uint8_t*) &cadena.at(i), 1, HAL_MAX_DELAY);
	}
}

// Int
void print(UART_HandleTypeDef *huart, int num) {
	string cadena = to_string(num);
	for (unsigned int i = 0; i < cadena.length(); i++) {
		HAL_UART_Transmit(huart, (uint8_t*) &cadena.at(i), 1, HAL_MAX_DELAY);
	}
}

// Char * + Float
void print(UART_HandleTypeDef *huart, char *str, float num) {
	print(huart, str);
	print(huart, num);
	print(huart, (char *)"\n");
}

// Char * + Int
void print(UART_HandleTypeDef *huart, char *str, int num) {
	print(huart, str);
	print(huart, num);
	print(huart, (char *)"\n");
}

// Float + Char * + Float
void print(UART_HandleTypeDef *huart, float value1, char *str, float value2){
	print(huart, value1);
	print(huart, str);
	print(huart, value2);
	print(huart, (char *)"\n");
}

// Int + Char * + Int
void print(UART_HandleTypeDef *huart, int value1, char *str, int value2){
	print(huart, value1);
	print(huart, str);
	print(huart, value2);
	print(huart, (char *)"\n");
}
