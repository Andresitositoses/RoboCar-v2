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

void motionAC2_init();
void motionAC2_calibrate(bool print);

void motionGC_init();
void motionGC_calibrate(bool print);

void motionMC_init();
void motionMC_calibrate(bool print);

float acc_bias_to_mg(float acc_bias);
float gyro_bias_to_mdps(float gyro_bias);
float mag_val_to_mGauss(float mag_val_uT);

#endif /* INC_X_CUBE_MEMS1_MOTION_H_ */
