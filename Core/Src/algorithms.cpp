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
	roboCar->setSpeed(10);

	bool making = true;
	int cont = 0;

	while(1) {

		if (making) {

			roboCar->goForward();

			print(&huart1, (char*)"Avanzando...\n");

			if (*centimeters == 60) { // every 60 cm
				if (cont < 4)
					roboCar->turnRight(current_dir, objective_dir, 90);
				*centimeters = 0;
				cont++;
				if (cont == 4) {
					making = false;
					cont = 0;

					while (*centimeters != 60) {
						roboCar->goForward();

						print(&huart1, (char*)"Avanzando...\n");
					}
					*centimeters = 0;
				}
			}

		} else {

			roboCar->goBackward();

			print(&huart1, (char*)"Retrocediendo...\n");

			if (*centimeters == 60) { // every 60 cm
				if (cont < 4)
					roboCar->turnLeft(current_dir, objective_dir, 90);
				*centimeters = 0;
				cont++;
				if (cont == 4) {
					making = true;
					cont = 0;

					while (*centimeters != 60) {
						roboCar->goBackward();

						print(&huart1, (char*)"Retrocediendo...\n");
					}
					*centimeters = 0;
				}
			}
		}

		tx_thread_sleep(delay); // 10ms
	}
}



