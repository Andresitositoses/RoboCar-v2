/*
 * print.h
 *
 *  Created on: Apr 23, 2022
 *      Author: andre
 */

#ifndef INC_PRINT_H_
#define INC_PRINT_H_

#include "stm32u5xx_hal.h"

void print(UART_HandleTypeDef *huart, char *str);

void print(UART_HandleTypeDef *huart, float value);

void print(UART_HandleTypeDef *huart, char *str, float value);


#endif /* INC_PRINT_H_ */
