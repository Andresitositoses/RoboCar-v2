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

void makingSquares(RoboCar::RoboCar *roboCar, int *centimeters, float *current_dir, float *objective_dir) {

	tx_thread_sleep(200); // 200cs -> 2000ms

	int delay = 1; // 1 cs -> 10 ms
	*centimeters = 0;
	roboCar->setSpeed(10);

	while(1) {

		roboCar->goForward();

		if (*centimeters == 60) { // every 60 cm
			roboCar->turnRight(current_dir, objective_dir, 90);
			*centimeters = 0;
		}

		tx_thread_sleep(delay); // 10ms
	}
}

void makingUnmakingSquares(RoboCar::RoboCar *roboCar, int *centimeters, float *current_dir, float *objective_dir) {

	tx_thread_sleep(200); // 200cs -> 2000ms

	int delay = 1; // 1 cs -> 10 ms
	*centimeters = 0;
	int distance = 50; // 50 cm
	roboCar->setSpeed(10);

	enum Modes { FORWARD, BACKWARD, ROTLEFT, ROTRIGHT };

	int numSteps = 14;
	int steps[numSteps] = {
			FORWARD, ROTRIGHT,
			FORWARD, ROTRIGHT,
			FORWARD, ROTRIGHT,
			FORWARD,
			BACKWARD, ROTLEFT,
			BACKWARD, ROTLEFT,
			BACKWARD, ROTLEFT,
			BACKWARD
	};

	int cont = 0;
	while(1) {

		switch (steps[cont]) {
			case FORWARD:
				*centimeters = 0;
				roboCar->goForward();
				print(&huart1, (char*)"Avanzando...\n");
				while(*centimeters < distance);
				break;
			case BACKWARD:
				*centimeters = 0;
				roboCar->goBackward();
				print(&huart1, (char*)"Retrocediendo...\n");
				while(*centimeters < distance);
				break;
			case ROTLEFT:
				print(&huart1, (char*)"Rotando a la izquierda...\n");
				roboCar->rotateLeft(current_dir, objective_dir, 90);
				break;
			case ROTRIGHT:
				print(&huart1, (char*)"Rotando a la derecha...\n");
				roboCar->rotateRight(current_dir, objective_dir, 90);
				break;
			default:
				break;
		}

		cont = (cont + 1) % numSteps;
		tx_thread_sleep(delay); // 10ms
	}
}



