/*
 * pwm.cpp
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#include "PinsLib/pwm.h"

pwm::pwm(TIM_HandleTypeDef htim, uint32_t channel) {
	this->htim = htim;
	this->channel = channel;

	HAL_TIM_PWM_Start(&this->htim, this->channel);
}

pwm::~pwm() {
	HAL_TIM_PWM_Stop(&htim, channel);
}

void pwm::setPulse(int pulse) {
	htim.Instance->CCR1 = pulse;
}
