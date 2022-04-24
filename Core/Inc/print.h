/*
 * print.h
 *
 *  Created on: Apr 23, 2022
 *      Author: andre
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_

#include "stm32u5xx_hal.h"
#include "string"

using namespace std;

// Char *
void print(UART_HandleTypeDef *huart, char *str);

// Float
void print(UART_HandleTypeDef *huart, float value);

// Int
void print(UART_HandleTypeDef *huart, int value);

// Char * + Float
void print(UART_HandleTypeDef *huart, char *str, float value);

// Char * + Int
void print(UART_HandleTypeDef *huart, char *str, int value);

// Float + Char * + Float
void print(UART_HandleTypeDef *huart, float value1, char *str, float value2);

// Int + Char * + Int
void print(UART_HandleTypeDef *huart, int value1, char *str, int value2);


#endif /* INC_PRINT_H_ */
