/*
 * gpio.h
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#ifndef INC_PINSLIB_GPIO_H_
#define INC_PINSLIB_GPIO_H_

#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_gpio.h"

class gpio {
	private:
		uint32_t number;
		GPIO_TypeDef *port;

	public:
		gpio(uint32_t number, GPIO_TypeDef *port);
		~gpio();

		void write(int state);
		int read(int state);
};

#endif /* INC_PINSLIB_GPIO_H_ */
