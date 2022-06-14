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
		bool moving;

	public:
		// ...
		RoboCar();

		~RoboCar();

		void goForward();
		void goBackward();
		void rotateLeft();
		void rotateRight();
		void stop();

		void setSpeed(float speed);
		void setMinSpeed();
		void setMaxSpeed();
		void setMeanSpeed();
		float getSpeed();
		float getMinSpeed();
		float getMaxSpeed();
		void updateSpeed(float factorX);
		bool isMoving();

		//TODO: obtención de la distancia en un sentido

		void calibrate();
		void loadCalibration();
		void showCalibrations();

	};

}


#endif /* INC_ROBOCAR_ROBOCAR_H_ */
