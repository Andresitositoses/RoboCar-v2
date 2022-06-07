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
bool motionMC_calibrate(bool print);

void motionFX_init();
bool motionFX_calibrate(bool print);

void motionEC_init();
void motionEC_calibrate(bool print);

void motionEC_MC_init();
bool motionEC_MC_calibrate(bool print);

void Acc_GetOrientation(char *Orientation);
void Mag_GetOrientation(char *Orientation);
void calc_matrix(char orientation[], float matrix[][3]);
void q_conjug(float q_conj[], float q_src[]);
void q_multiply(float q_res[], float q_a[], float q_b[]);
void v_rotate(float v_new[], float q_rot[], float v_old[]);
int calc_heading(float *heading, float v_head[]);
void MotionEC_manager_calc_heading(float quaternion[], float *heading, int *heading_valid);
void transform_orientation(float *input, float output[], float matrix[][3]);

float acc_bias_to_mg(float acc_bias);
float gyro_bias_to_mdps(float gyro_bias);
float mag_val_to_mGauss(float mag_val_uT);

float motion_getDegrees();
#endif /* INC_X_CUBE_MEMS1_MOTION_H_ */
