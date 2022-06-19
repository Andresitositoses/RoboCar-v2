/*
 * pwm.h
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#ifndef INC_PINSLIB_PWM_H_
#define INC_PINSLIB_PWM_H_

#include "stm32u5xx_hal.h"

class pwm {
	private:
		TIM_HandleTypeDef htim;
		uint32_t channel;

	public:
		pwm(TIM_HandleTypeDef htim, uint32_t channel);
		~pwm();

		void setPulse(int pulse);
};

#endif /* INC_PINSLIB_PWM_H_ */
