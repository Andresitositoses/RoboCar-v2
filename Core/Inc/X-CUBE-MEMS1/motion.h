/*
 * motion.h
 *
 *  Created on: 15 may. 2022
 *      Author: andre
 */

#ifndef INC_X_CUBE_MEMS1_MOTION_H_
#define INC_X_CUBE_MEMS1_MOTION_H_

void motionAC_init();
void motionAC_calibrate(bool print);
float acc_bias_to_mg(float acc_bias);

void motionGC_init();
void motionGC_calibrate(bool print);
float gyro_bias_to_mdps(float gyro_bias);

#endif /* INC_X_CUBE_MEMS1_MOTION_H_ */
