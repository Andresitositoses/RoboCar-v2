/*
 * algorithms.cpp
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#include "algorithms.h"

void goAway(RoboCar::RoboCar *roboCar, bool *bottom, float *current_dir, float *objective_dir) {
	while(1) {

		roboCar->goForward();

		if (*bottom) {
			*objective_dir = *current_dir;
		}

		tx_thread_sleep(1); // 1cs -> 10ms
	}
}

void goBack(RoboCar::RoboCar *roboCar, bool *bottom, float *current_dir, float *objective_dir) {

	tx_thread_sleep(200); // 200cs -> 2000ms

	while(1) {

		roboCar->goBackward();

		if (*bottom) {
			*objective_dir = *current_dir;
		}

		tx_thread_sleep(1); // 1cs -> 10ms
	}
}

void makingSquares(RoboCar::RoboCar *roboCar, int *encoders_flag, float *current_dir, float *objective_dir) {

	tx_thread_sleep(200); // 200cs -> 2000ms

	int delay = 1; // 1 cs -> 10 ms
	float distance = 0; // m
	float velocity = 16; // 16 cm/s -> 0.16 m/s
	roboCar->setSpeed(velocity);

	while(1) {

		roboCar->goForward();
		distance += (velocity / 10000); // (velocity / 100) m/s -> (velocity / 10000) m/cs
		print(&huart1, (char*)"Distance: ", distance);

		if (distance > 0.2) { // every 20 cm
			*encoders_flag = 1;
			roboCar->rotateRight(current_dir, objective_dir, 90);
			*encoders_flag = 0;
			distance = 0;
		}

		tx_thread_sleep(delay); // 10ms
	}
}
