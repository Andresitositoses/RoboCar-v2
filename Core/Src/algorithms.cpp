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

	while(1) {

		roboCar->goBackward();

		if (*bottom) {
			*objective_dir = *current_dir;
		}

		tx_thread_sleep(1); // 1cs -> 10ms
	}

}
