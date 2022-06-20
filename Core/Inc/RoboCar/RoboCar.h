/*
 * RoboCar.h
 *
 *  Created on: 30 abr. 2022
 *      Author: andre
 */

#ifndef INC_ROBOCAR_ROBOCAR_H_
#define INC_ROBOCAR_ROBOCAR_H_

#include "RoboCar/WheelMotor.h"

using namespace std;

namespace RoboCar {

	class RoboCar {
	private:
		// ...
		WheelMotor *leftWheel;
		WheelMotor *rightWheel;

		float speed;
		float minSpeed;
		float maxSpeed;

	public:
		// ...
		RoboCar();

		~RoboCar();

		void goForward();
		void goBackward();
		void rotateLeft();
		void rotateRight();
		void turnLeft(float *current_dir, float *objective_dir, float gyro_degrees);
		void turnRight(float *current_dir, float *objective_dir, float gyro_degrees);
		void stop();

		void setSpeed(float speed);
		void setMinSpeed();
		void setMaxSpeed();
		void setMeanSpeed();
		float getSpeed();
		float getMinSpeed();
		float getMaxSpeed();
		float getLeftWheelSpeed();
		float getRightWheelSpeed();
		void updateSpeed(float leftSpeed, float rightSpeed);
		void updateSpeed(float factorX, int limit);
		bool isMoving();

		//TODO: obtención de la distancia en un sentido

		void calibrate();
		void loadCalibration();
		void showCalibrations();

	};

}


#endif /* INC_ROBOCAR_ROBOCAR_H_ */
