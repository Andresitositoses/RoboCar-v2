/*
 * sensors.c
 *
 *  Created on: 25 abr. 2022
 *      Author: andre
 */

#include "sensors.h"
#include "print.h"

void initSensors() {

	// LPS22HH module
	BSP_ENV_SENSOR_Init(1, ENV_TEMPERATURE); // Temperature
	BSP_ENV_SENSOR_Enable(1, ENV_TEMPERATURE);
	BSP_ENV_SENSOR_Init(1, ENV_PRESSURE); // Pressure
	BSP_ENV_SENSOR_Enable(1, ENV_PRESSURE);

	// ISM330DLC module
	BSP_MOTION_SENSOR_Init(0, MOTION_ACCELERO); // Accelerometer
	BSP_MOTION_SENSOR_Init(0, MOTION_GYRO); // Gyroscope
	BSP_MOTION_SENSOR_Enable(0, MOTION_ACCELERO);
	BSP_MOTION_SENSOR_Enable(0, MOTION_GYRO);

	// IIS2MDC module
	BSP_MOTION_SENSOR_Init(1, MOTION_MAGNETO); // Magnetometer
	BSP_MOTION_SENSOR_Enable(1, MOTION_MAGNETO);

	HAL_Delay(100);
}

BSP_MOTION_SENSOR_Axes_t getAxesAccelerometer() {
	BSP_MOTION_SENSOR_Axes_t axes;
	BSP_MOTION_SENSOR_GetAxes(0, MOTION_ACCELERO, &axes);
	return axes;
}

BSP_MOTION_SENSOR_Axes_t getAxesGyroscope() {
	BSP_MOTION_SENSOR_Axes_t axes;
	BSP_MOTION_SENSOR_GetAxes(0, MOTION_GYRO, &axes);
	return axes;
}

BSP_MOTION_SENSOR_Axes_t getAxesMagnetometer() {
	BSP_MOTION_SENSOR_Axes_t axes;
	BSP_MOTION_SENSOR_GetAxes(1, MOTION_MAGNETO, &axes);
	return axes;
}

void showAccelerometerValues() {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesAccelerometer();
	print(&huart1, (char*) "x, y, z: ");
	print(&huart1, (int) ejesAcel.x);
	print(&huart1, (char*) ", ");
	print(&huart1, (int) ejesAcel.y);
	print(&huart1, (char*) ", ");
	print(&huart1, (char*) "", (int) ejesAcel.z);
}

void showGyroscopeValues() {
	BSP_MOTION_SENSOR_Axes_t ejesGiro = getAxesGyroscope();
	print(&huart1, (char*) "x, y, z: ");
	print(&huart1, (int) ejesGiro.x);
	print(&huart1, (char*) ", ");
	print(&huart1, (int) ejesGiro.y);
	print(&huart1, (char*) ", ");
	print(&huart1, (char*) "", (int) ejesGiro.z);
}

void showAccelGyroValues() {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesAccelerometer();
	BSP_MOTION_SENSOR_Axes_t ejesGiro = getAxesGyroscope();
	print(&huart1, (char*) "accelX=");
	print(&huart1, (int) ejesAcel.x);
	print(&huart1, (char*) ", accelY=");
	print(&huart1, (int) ejesAcel.y);
	print(&huart1, (char*) ", accelZ=");
	print(&huart1, (int) ejesAcel.z);
	print(&huart1, (char*) " ----- ");
	print(&huart1, (char*) "gyroX=");
	print(&huart1, (int) ejesGiro.x);
	print(&huart1, (char*) ", gyroY=");
	print(&huart1, (int) ejesGiro.y);
	print(&huart1, (char*) ", gyroZ=");
	print(&huart1, (char*) "", (int) ejesGiro.z);
}

void showMagnetometerValues() {
	BSP_MOTION_SENSOR_Axes_t ejesMagn = getAxesMagnetometer();
	print(&huart1, (char*) "x, y, z: ");
	print(&huart1, (int) ejesMagn.x);
	print(&huart1, (char*) ", ");
	print(&huart1, (int) ejesMagn.y);
	print(&huart1, (char*) ", ");
	print(&huart1, (char*) "", (int) ejesMagn.z);
}

/*
 * MEMS functions
 */
void MEMS_Read_AccValue(float *data_x, float *data_y, float *data_z) {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesAccelerometer();
	*data_x = (float) ejesAcel.x;
	*data_y = (float) ejesAcel.y;
	*data_z = (float) ejesAcel.z;
}

void MEMS_Read_Acc2Value(float *data_x, float *data_y) {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesAccelerometer();
	*data_x = (float) ejesAcel.x;
	*data_y = (float) ejesAcel.y;
}

void MEMS_Read_GyroValue(float *data_x, float *data_y, float *data_z) {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesGyroscope();
	*data_x = (float) ejesAcel.x;
	*data_y = (float) ejesAcel.y;
	*data_z = (float) ejesAcel.z;
}

void MEMS_Read_MagValue(float *data_x, float *data_y, float *data_z) {
	BSP_MOTION_SENSOR_Axes_t ejesAcel = getAxesMagnetometer();
	*data_x = (float) ejesAcel.x;
	*data_y = (float) ejesAcel.y;
	*data_z = (float) ejesAcel.z;
}

