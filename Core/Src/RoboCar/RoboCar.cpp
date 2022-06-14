/*
 * RoboCar.cpp
 *
 *  Created on: 30 abr. 2022
 *      Author: andre
 */

#include "RoboCar/RoboCar.h"
#include "print.h"

namespace RoboCar {

	/*
	 *
	 */
	RoboCar::RoboCar(){
		leftWheel = new WheelMotor(Wheel::LEFT);
		rightWheel = new WheelMotor(Wheel::RIGHT);

		speed = 0;
		minSpeed = 99999999;
		maxSpeed = 0;
		moving = false;
	}

	/*
	 *
	 */
	RoboCar::~RoboCar(){
		delete leftWheel;
		delete rightWheel;
	}

	void RoboCar::goForward(){
		leftWheel->goForward();
		rightWheel->goForward();
		moving = true;
	}

	void RoboCar::goBackward(){
		leftWheel->goBackward();
		rightWheel->goBackward();
		moving = true;
	}

	void RoboCar::rotateLeft(){
		leftWheel->goBackward();
		rightWheel->goForward();
		moving = true;
	}

	void RoboCar::rotateRight(){
		leftWheel->goForward();
		rightWheel->goBackward();
		moving = true;
	}

	void RoboCar::stop(){
		leftWheel->stop();
		rightWheel->stop();
		moving = false;
	}

	void RoboCar::setSpeed(float speed){
		print(&huart1, (char *)"Setting speed to: ", speed);
		leftWheel->setSpeed(speed);
		rightWheel->setSpeed(speed);
		this->speed = speed;
	}

	void RoboCar::setMinSpeed(){
		print(&huart1, (char *)"Setting speed to: ", minSpeed);
		leftWheel->setSpeed(minSpeed);
		rightWheel->setSpeed(minSpeed);
		speed = minSpeed;
	}

	void RoboCar::setMaxSpeed(){
		print(&huart1, (char *)"Setting speed to: ", maxSpeed);
		leftWheel->setSpeed(maxSpeed);
		rightWheel->setSpeed(maxSpeed);
		speed = maxSpeed;
	}

	void RoboCar::setMeanSpeed(){
		print(&huart1, (char *)"Setting speed to: ", (minSpeed + maxSpeed) / 2);
		leftWheel->setSpeed((minSpeed + maxSpeed) / 2);
		rightWheel->setSpeed((minSpeed + maxSpeed) / 2);
		speed = (minSpeed + maxSpeed) / 2;
	}

	float RoboCar::getSpeed(){
		return speed;
	}

	float RoboCar::getMinSpeed(){
		return minSpeed;
	}

	float RoboCar::getMaxSpeed(){
		return maxSpeed;
	}

	void RoboCar::updateSpeed(){
		float leftSpeed = leftWheel->getCurrentSpeed();
		float rightSpeed = rightWheel->getCurrentSpeed();
		leftWheel->updateSpeed(speed, leftSpeed);
		rightWheel->updateSpeed(speed, rightSpeed);
	}

	void RoboCar::updateSpeed(float factorX){
		leftWheel->updateSpeed(speed, leftSpeed, -factorX);
		rightWheel->updateSpeed(speed, rightSpeed, factorX);
	}

	bool RoboCar::isMoving(){
		return moving;
	}

	void RoboCar::calibrate(){

		print(&huart1, (char *)"Starting calibration...\n");

		leftWheel->calibrate();
		rightWheel->calibrate();
		minSpeed = max(leftWheel->getMinSpeed(), rightWheel->getMinSpeed());
		maxSpeed = min(leftWheel->getMaxSpeed(), rightWheel->getMaxSpeed());
	}

	void RoboCar::loadCalibration(){
		vector<pair<int, float>> *leftSpeeds = new vector<pair<int, float>>();
		vector<pair<int, float>> *rightSpeeds = new vector<pair<int, float>>();

		*leftSpeeds = { { 200, 4.88 }, { 250, 8.53 }, { 300, 11.95 },
				{ 350, 15.24 }, { 400, 17.89 }, { 450, 20.09 }, { 500, 22.25 }, {
						550, 24.47 }, { 600, 25.79 }, { 650, 26.25 },
				{ 700, 27.35 }, { 750, 28.57 }, { 800, 29.09 }, { 850, 29.09 }, {
						900, 29.23 }, { 950, 30.84 }, { 1000, 31.19 } };
		*rightSpeeds = { { 200, 5.14 }, { 250, 8.64 }, { 300, 12.2 },
				{ 350, 15.25 }, { 400, 17.6 }, { 450, 19.7 }, { 500, 21.2 }, {
						550, 22.14 }, { 600, 23.02 }, { 650, 24.77 },
				{ 700, 26.0 }, { 750, 27.31 }, { 800, 28.0 }, { 850, 28.3 }, {
						900, 28.31 }, { 950, 29.18 }, { 1000, 31.19 } };

		leftWheel->loadCalibration(leftSpeeds);
		rightWheel->loadCalibration(rightSpeeds);
		minSpeed = max(leftWheel->getMinSpeed(), rightWheel->getMinSpeed());
		maxSpeed = min(leftWheel->getMaxSpeed(), rightWheel->getMaxSpeed());

		free(leftSpeeds);
		free(rightSpeeds);
	}

	void RoboCar::showCalibrations(){
		print(&huart1, (char*) "Left wheel calibration values:\n");
		leftWheel->showCalibrationValues();
		print(&huart1, (char*) "Right wheel calibration values:\n");
		rightWheel->showCalibrationValues();
	}

}


