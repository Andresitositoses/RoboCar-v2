/*
 * WheelMotor.cpp
 *
 *  Created on: 20 abr. 2022
 *      Author: andre
 */

#include "stm32u5xx_hal.h"

#include "RoboCar/WheelMotor.h"
#include "print.h"
#include "tx_api.h"

// Period and pulse parameters
#define PERIOD                      1000
#define DEFAULT_PULSE 				0

// Proportional PID constant
#define PULSE_CONSTANT 				5

// Measure parameters
#define MEASURES_FOR_SPEED          6
#define MAX_ATTEMPTS_TO_READ        450000
#define CYCLES_PER_SECOND			160000000

#define MINIMUN_VARIATION			10

namespace RoboCar {

	WheelMotor::WheelMotor(Wheel wheel) {

		// GPIOs and PWM configurations
		switch (wheel) {
		case LEFT:
			// Forward
			this->forwardPin = new gpio(LEFT_WHEEL_FORWARD_PIN, LEFT_WHEEL_FORWARD_PORT);
			// Backward
			this->backwardPin = new gpio(LEFT_WHEEL_BACKWARD_PIN, LEFT_WHEEL_BACKWARD_PORT);
			// Speed
			this->speedPin = new pwm(htim16, LEFT_WHEEL_SPEED_CHANNEL);
			// Encoder
			this->encoderPin = new gpio(LEFT_ENCODER_PIN, LEFT_ENCODER_PORT);
			print(&huart1, (char*) "Left wheel created\n");
			break;
		case RIGHT:
			// Forward
			this->forwardPin = new gpio(RIGHT_WHEEL_FORWARD_PIN, RIGHT_WHEEL_FORWARD_PORT);
			// Backward
			this->backwardPin = new gpio(RIGHT_WHEEL_BACKWARD_PIN, RIGHT_WHEEL_BACKWARD_PORT);
			// Speed
			this->speedPin = new pwm(htim4, RIGHT_WHEEL_SPEED_CHANNEL);
			// Encoder
			this->encoderPin = new gpio(RIGHT_ENCODER_PIN, RIGHT_ENCODER_PORT);
			print(&huart1, (char*) "Right wheel created\n");
			break;
		default: // Error
			print(&huart1, (char*) "Wheel number is not valid\n");
			exit(1);
		}

		// Default speed
		this->speedPulse = 0;
		this->pulse = 0;
		this->speedPin->setPulse(pulse);

		// General paramaters initialization
		movingForward = false;
		movingBackward = false;
		calibrated = 0;
		minSpeed = 9999999;
		maxSpeed = 0;
		speeds = new std::vector<std::pair<int, float>>();
	}

	/**
	 * @brief Remove all wheel pins
	 */
	WheelMotor::~WheelMotor() {
		delete forwardPin;
		delete backwardPin;
		delete speedPin;
		delete encoderPin;
	}

	/**
	 * @brief Makes the wheel go forward
	 */
	void WheelMotor::goForward() {
		// Activate forward pin and deactivate backward pin
		forwardPin->write(1);
		backwardPin->write(0);
		// The wheel is moving
		movingForward = true;
	}

	/**
	 * @brief Makes the wheel go forward
	 */
	void WheelMotor::goBackward() {
		// Activate backward pin and deactivate forward pin
		forwardPin->write(0);
		backwardPin->write(1);
		// The wheel is moving
		movingBackward = true;
	}

	/**
	 * @brief The wheel stops
	 */
	void WheelMotor::stop() {
		// Deactivate forward and backward pins
		forwardPin->write(0);
		backwardPin->write(0);
		// The wheel is stopped
		movingForward = false;
		movingBackward = false;
	}

	/*
	 * @return Minimum speed from speeds vector
	 */
	float WheelMotor::getMinSpeed(){
		return minSpeed;
	}

	/*
	 * @return Maximum speed from speeds vector
	 */
	float WheelMotor::getMaxSpeed(){
		return maxSpeed;
	}

	/**
	 * @brief Updates wheel speed using the calibrations vector
	 * @param speed Speed value to set. This value must be between minSpeed and maxSpeed
	 * @return true if speed has been changed, false otherwise
	 */
	bool WheelMotor::setSpeed(float speed) {
		// The wheel must be calibrated
		if (!calibrated) {
			print(&huart1,
					(char*) "The wheel is not calibrated. It's not possible to set a valid speed value.\n");
			return false;
		}

		// Speed must be between minSpeed and maxSpeed
		if (!(speed >= minSpeed && speed <= maxSpeed)) {
			print(&huart1, speed);
			print(&huart1, (char*) " speed value is not valid.\n");
			print(&huart1, (char*) "Min=");
			print(&huart1, minSpeed);
			print(&huart1, (char*) " Max=", maxSpeed);
			return false;
		}

		// Searching for the most suitable speed value
		for (int i = 0; i < (int) speeds->size() - 1; i++) {
			if (speeds->at(i).second <= speed && speeds->at(i + 1).second >= speed) {
				setPulse(speeds->at(i).first);
				speedPulse = speeds->at(i).first; // Useful to know start reference point
				print(&huart1, (char *)"Setting pulse to ", speeds->at(i).first);
				return true;
			}
		}

		print(&huart1,
				(char*) "Error: an unexpected error has occurred setting speed value: ",
				speed);
		return false;
	}

	/*
	 * @brief Get the real wheel speed through the encoder
	 * @return Current wheel speed value
	 */
	float WheelMotor::getCurrentSpeed(){
		// Initial measure
		ULONG startTime = DWT->CYCCNT;

		// Reset counter after reboot
		if (startTime == 0) {
			CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
			DWT->CYCCNT = 0;
			DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
			startTime = DWT->CYCCNT;
		}

		int cont = 0;

		// Gets the time it takes to change between 0's and 1's MEASURES_FOR_SPEED times
		for (int i = 0; i < MEASURES_FOR_SPEED; i++) {
			// Exit the loop in case the wheel is stopped
			while (this->encoderPin->read() != 1 && cont++ < MAX_ATTEMPTS_TO_READ);
			if (cont >= MAX_ATTEMPTS_TO_READ) {
				return 0;
			} else {
				cont = 0;
			}
			while (this->encoderPin->read() != 0 && cont++ < MAX_ATTEMPTS_TO_READ);
			if (cont >= MAX_ATTEMPTS_TO_READ) {
				return 0;
			} else {
				cont = 0;
			}
		}

		// End measure
		ULONG stopTime = DWT->CYCCNT;

		return CYCLES_PER_SECOND * MEASURES_FOR_SPEED / (float) abs((int) stopTime - (int) startTime);
	}

	/**
	 * @brief Updates wheel speed based on a reference speed using a proportional PID
	 * @param referenceSpeed Reference speed on which to regulate the speed
	 * @param currentSpeed Current wheel speed (determined by its corresponding encoder)
	 */
	void WheelMotor::updateSpeed(float referenceSpeed, float currentSpeed) {
		if (!(movingForward || movingBackward))
			return;

		float pulse_change = (referenceSpeed - currentSpeed) * PULSE_CONSTANT;
		pulse += pulse_change;
		if (pulse > PERIOD)
			pulse = PERIOD;
		else if (pulse < 0){
			pulse = 0;
		}
		/*
		print(&huart1, (char *)"Reference speed: ", referenceSpeed);
		print(&huart1, (char *)"Current speed: ", currentSpeed);
		print(&huart1, (char *)"Pulse change: ", pulse_change);
		print(&huart1, (char *)"Setting pulse to ", pulse);
		*/
		setPulse(pulse);
	}

	void WheelMotor::updateSpeed(float factorX, int limit){

		if (!(movingForward || movingBackward))
			return;

		/*
		print(&huart1, (char*)"factorX: ", factorX);
		print(&huart1, (char*)"limit: ", limit);
		print(&huart1, (char*)"speedPulse: ", speedPulse);
		*/

		float direction = movingForward ? 1 : -1;

		if (factorX > limit) {
			pulse = speedPulse + direction * (MINIMUN_VARIATION + limit * PULSE_CONSTANT);
		}
		else if (factorX < -limit) {
			pulse = speedPulse + direction * (- MINIMUN_VARIATION - limit * PULSE_CONSTANT);
		}
		else {
			pulse = speedPulse + direction * factorX * PULSE_CONSTANT;
		}

		if (pulse > PERIOD)
			pulse = PERIOD;
		else if (pulse < 0){
			pulse = 0;
		}

		//print(&huart1, (char*)"Nuevo pulso: ", pulse);

		setPulse(pulse);
	}

	/**
	 * @brief Start a calibration process. The wheel begins to accelerate.
	 */
	void WheelMotor::calibrate() {

		// Parameters initialization
		speeds->clear();
		minSpeed = INT32_MAX;
		maxSpeed = 0;
		setPulse(DEFAULT_PULSE);

		// Wheel stops before start calibration
		stop();
		tx_thread_sleep(200); // 2 seconds

		// Calibrates for 10 different pulses
		goForward();
		for (int pulse = 0; pulse <= PERIOD; pulse += 50) {
			// Set incremented pulse
			setPulse(pulse);
			tx_thread_sleep(100); // 2 seconds

			// Takes a few measures
			float speed = 0;
			for (int i = 0; i < MEASURES_FOR_SPEED; i++){
				speed += getCurrentSpeed();
			}
			speed /= MEASURES_FOR_SPEED;
			print(&huart1, (char *)"Velocidad actual: ", speed);

			// Adds a new pulse-speed pair to the the vector (only if the wheel is moving)
			if (speed > 0) {
				std::pair<int, float> *par = new std::pair<int, float>;
				par->first = pulse;
				par->second = speed;
				speeds->push_back(*par);
				free(par);
				minSpeed = std::min(minSpeed, speed);
				maxSpeed = std::max(maxSpeed, speed);
			}
		}

		// Stops the wheel
		stop();
		calibrated = true;
	}

	bool WheelMotor::isMoving() {
		return movingForward || movingBackward;
	}

	/*
	 * Loads pulse-speed pairs from an external vector
	 */
	void WheelMotor::loadCalibration(std::vector<std::pair<int, float>> *speeds){
		for (int i = 0; i < (int) speeds->size(); i++){
			this->speeds->push_back({speeds->at(i).first, speeds->at(i).second});
			if (speeds->at(i).second > maxSpeed){
				maxSpeed = speeds->at(i).second;
			}
			if (speeds->at(i).second < minSpeed){
				minSpeed = speeds->at(i).second;
			}
		}
		calibrated = true;
	}

	/*
	 * Print functions of vector's speeds
	 */
	void WheelMotor::showCalibrationValues() {
		for (int i = 0; i < (int) speeds->size(); i++) {
			print(&huart1, speeds->at(i).first); print(&huart1, (char*) " --- ", speeds->at(i).second);
		}
	}

	/**
	 * @brief Private function to set a pulse value to modifies the wheel speed
	 * @param pulse Pulse values must be between 0 and PERIOD
	 */
	void WheelMotor::setPulse(int pulse) {
		if (pulse < 0 || pulse > PERIOD) {
			print(&huart1, (char*) "Pulse value not valid: ", pulse);
			return;
		}
		this->pulse = pulse;
		this->speedPin->setPulse(pulse);
	}

} /* namespace RoboCar */
