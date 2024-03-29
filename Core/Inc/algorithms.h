/*
 * algorithms.h
 *
 *  Created on: 18 jun. 2022
 *      Author: andre
 */

#ifndef INC_ALGORITHMS_H_
#define INC_ALGORITHMS_H_

#include "tx_api.h"

#include "RoboCar/RoboCar.h"
#include "print.h"

// UART communication handler
extern UART_HandleTypeDef huart1;

void goAway(RoboCar::RoboCar *roboCar, bool *button, float *current_dir, float *objective_dir);

void goBack(RoboCar::RoboCar *roboCar, bool *button, float *current_dir, float *objective_dir);

void makingSquares(RoboCar::RoboCar *roboCar, int *centimeters, float *current_dir, float *objective_dir);

void makingSquaresReverse(RoboCar::RoboCar *roboCar, int *centimeters, float *current_dir, float *objective_dir);

void makingUnmakingSquares(RoboCar::RoboCar *roboCar, int *centimeters, float *current_dir, float *objective_dir);

#endif /* INC_ALGORITHMS_H_ */
