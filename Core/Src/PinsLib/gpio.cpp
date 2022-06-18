/*
 * gpio.cpp
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#include "PinsLib/gpio.h"

gpio::gpio(uint32_t number, GPIO_TypeDef *port) {
	this->number = number;
	this->port = port;
}

gpio::~gpio() {
	free(port);
}


void gpio::write(int state) {
	HAL_GPIO_WritePin(port, number, (GPIO_PinState) state);
}

int gpio::read() {
	return HAL_GPIO_ReadPin(port, number);
}
