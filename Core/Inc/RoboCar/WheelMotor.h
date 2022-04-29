/*
 * WheelMotor.h
 *
 *  Created on: 20 abr. 2022
 *      Author: andre
 */

#ifndef INC_ROBOCAR_WHEELMOTOR_H_
#define INC_ROBOCAR_WHEELMOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_ll_ucpd.h"
#include "stm32u5xx_ll_bus.h"
#include "stm32u5xx_ll_cortex.h"
#include "stm32u5xx_ll_rcc.h"
#include "stm32u5xx_ll_system.h"
#include "stm32u5xx_ll_utils.h"
#include "stm32u5xx_ll_pwr.h"
#include "stm32u5xx_ll_gpio.h"
#include "stm32u5xx_ll_dma.h"
#include "stm32u5xx_ll_exti.h"

#include "string"
#include "vector"

// Wheels defines
#define LEFT_WHEEL_FORWARD_PIN GPIO_PIN_15
#define LEFT_WHEEL_FORWARD_PORT GPIOD
#define LEFT_WHEEL_BACKWARD_PIN GPIO_PIN_7
#define LEFT_WHEEL_BACKWARD_PORT GPIOE
#define LEFT_WHEEL_SPEED_CHANNEL TIM_CHANNEL_1

#define RIGHT_WHEEL_FORWARD_PIN GPIO_PIN_1
#define RIGHT_WHEEL_FORWARD_PORT GPIOC
#define RIGHT_WHEEL_BACKWARD_PIN GPIO_PIN_13
#define RIGHT_WHEEL_BACKWARD_PORT GPIOF
#define RIGHT_WHEEL_SPEED_CHANNEL TIM_CHANNEL_1

// Encoders
#define LEFT_ENCODER_PIN GPIO_PIN_14
#define LEFT_ENCODER_PORT GPIOE

#define RIGHT_ENCODER_PIN GPIO_PIN_13
#define RIGHT_ENCODER_PORT GPIOE

// UART communication handler
extern UART_HandleTypeDef huart1;

// Timers for PWMs
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;

// Macros for pair<int,int> vector accesses
#define minimum first
#define maximum second

using std::string;
using std::pair;

namespace RoboCar {

	struct GPIO {
		uint32_t number;
		GPIO_TypeDef *port;
	};

	struct PWM {
		TIM_HandleTypeDef htim;
		uint32_t channel;
	};

	// Types of wheels
	enum Wheel {
		LEFT = 1, RIGHT = 0
	};

	class WheelMotor {
	private:
		// GPIO pins used for wheel direction
		GPIO *forwardPin;
		GPIO *backwardPin;

		// PWM pin used for wheel speed
		PWM *speedPin;

		// GPIO pin for wheel encoder
		GPIO *encoderPin;

		// Speed configuration parameters
		std::vector<std::pair<int, float>> *speeds;
		float minSpeed;
		float maxSpeed;

		// General variables for wheel operation
		bool moving;
		bool calibrated;
		int pulse;

	public:
		// Initializes the specificated wheel (LEFT or RIGHT)
		WheelMotor(Wheel wheel);

		// Unlink all wheel pins
		~WheelMotor();

		// Functions related to the movement of the wheel
		void goForward();
		void goBackward();
		void stop();

		// Functions related to the speed regulation
		bool setSpeed(int speed);
		float getCurrentSpeed();
		void updateSpeed(int referenceSpeed, int currentSpeed);

		// Calibration functions
		void calibrate();
		void showCalibrationValues();

	private:
		// Useful function to change PWM pulse
		void setPulse(int pulse);
	};
}

#endif /* INC_ROBOCAR_WHEELMOTOR_H_ */
