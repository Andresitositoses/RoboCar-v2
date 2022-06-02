/*
 * sensors.h
 *
 *  Created on: 25 abr. 2022
 *      Author: andre
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

// Para sensores
#include "BSP/env_sensors/b_u585i_iot02a_env_sensors.h"
#include "BSP/motion_sensors/b_u585i_iot02a_motion_sensors.h"

// UART communication handler
extern UART_HandleTypeDef huart1;

void initSensors();

BSP_MOTION_SENSOR_Axes_t getAxesAccelerometer();
BSP_MOTION_SENSOR_Axes_t getAxesGyroscope();
BSP_MOTION_SENSOR_Axes_t getAxesMagnetometer();
float getMagnetoAzimuth(float mag_x, float mag_y);

void showAccelerometerValues();
void showGyroscopeValues();
void showAccelGyroValues();
void showMagnetometerValues();

void MEMS_Read_AccValue(float *data_x, float *data_y, float *data_z);
void MEMS_Read_Acc2Value(float *data_x, float *data_y);
void MEMS_Read_GyroValue(float *data_x, float *data_y, float *data_z);
void MEMS_Read_MagValue(float *data_x, float *data_y, float *data_z);

#endif /* INC_SENSORS_H_ */
