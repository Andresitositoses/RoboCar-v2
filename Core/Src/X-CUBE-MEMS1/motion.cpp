/*
 * motion.cpp
 *
 *  Created on: 15 may. 2022
 *      Author: andre
 */

#include "X-CUBE-MEMS1/motion.h"
#include "main.h"
#include "print.h"
#include "sensors.h"

#include "X-CUBE-MEMS1/motion_ac.h"

#define VERSION_STR_LENG 35
#define REPORT_INTERVAL 20

// Calibration modes
#define DYNAMIC_CALIBRATION 0
#define SIX_POINT_CALIBRATION 1

// Handlers
extern UART_HandleTypeDef huart1;

void motionAC_init() {

	// Necesario para el algoritmo de inicialización
	__CRC_CLK_ENABLE()
	;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	MAC_knobs_t Knobs;

	// Accelerometer calibration API initialization function
	MotionAC_Initialize(1);
	MotionAC_GetKnobs(&Knobs);
	Knobs.MoveThresh_g = 0.2f;
	Knobs.Run6PointCal = DYNAMIC_CALIBRATION;
	Knobs.Sample_ms = REPORT_INTERVAL;
	MotionAC_SetKnobs(&Knobs);

	// Optional: Get version
	MotionAC_GetLibVersion(lib_version);
	print(&huart1, (char*) "Versión de MotionAC: ");
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");

	// Initialize DWT register for counting clock cycles purpose
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; // Disable counter
}

void motionAC_calibrate() {
	float acc_cal_x, acc_cal_y, acc_cal_z;

	// Accelerometer Calibration Algorithm
	uint8_t is_calibrated = 0;
	MAC_input_t data_in = { data_in.Acc[0] = 0.0f, data_in.Acc[1] = 0.0f,
			data_in.Acc[2] = 0.0f };
	MAC_output_t data_out;

	// Get acceleration X/Y/Z in g
	MEMS_Read_AccValue(&data_in.Acc[0], &data_in.Acc[1], &data_in.Acc[2]);

	// Convert acceleration from [mg] to [g]
	data_in.Acc[0] = (float) data_in.Acc[0] / 1000.0f;
	print(&huart1, (float) data_in.Acc[0], (char*) " --- ", (float) 1000.0f);
	data_in.Acc[1] = (float) data_in.Acc[1] / 1000.0f;
	data_in.Acc[2] = (float) data_in.Acc[2] / 1000.0f;
	data_in.TimeStamp = 0;

	// Run Accelerometer Calibration algorithm
	DWT->CYCCNT = 0; /* Clear count of clock cycles */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Enable counter */

	MotionAC_Update(&data_in, &is_calibrated);

	print(&huart1, (char*) "is calibrated: ", is_calibrated);

	// Get Calibration coeficients
	MotionAC_GetCalParams(&data_out);

	// Do offset & scale factor calibration

	// Apply correction
	acc_cal_x = ((data_in.Acc[0] - data_out.AccBias[0])
			* data_out.SF_Matrix[0][0]);
	acc_cal_y = ((data_in.Acc[1] - data_out.AccBias[1])
			* data_out.SF_Matrix[1][1]);
	acc_cal_z = ((data_in.Acc[2] - data_out.AccBias[2])
			* data_out.SF_Matrix[2][2]);

	// Offset coefficients
	print(&huart1, (char*) "data_out.AccBias[0]: ", data_out.AccBias[0]);
	print(&huart1, (char*) "data_out.AccBias[1]: ", data_out.AccBias[1]);
	print(&huart1, (char*) "data_out.AccBias[2]: ", data_out.AccBias[2]);

	// Scale factor coefficients
	print(&huart1, (char*) "data_out.SF_Matrix[0][0]: ",
			data_out.SF_Matrix[0][0]);
	print(&huart1, (char*) "data_out.SF_Matrix[0][1]: ",
			data_out.SF_Matrix[0][1]);
	print(&huart1, (char*) "data_out.SF_Matrix[0][2]: ",
			data_out.SF_Matrix[0][2]);

	print(&huart1, (char*) "data_out.SF_Matrix[1][0]: ",
			data_out.SF_Matrix[1][0]);
	print(&huart1, (char*) "data_out.SF_Matrix[1][1]: ",
			data_out.SF_Matrix[1][1]);
	print(&huart1, (char*) "data_out.SF_Matrix[1][2]: ",
			data_out.SF_Matrix[1][2]);

	print(&huart1, (char*) "data_out.SF_Matrix[2][0]: ",
			data_out.SF_Matrix[2][0]);
	print(&huart1, (char*) "data_out.SF_Matrix[2][1]: ",
			data_out.SF_Matrix[2][1]);
	print(&huart1, (char*) "data_out.SF_Matrix[2][2]: ",
			data_out.SF_Matrix[2][2]);

	// Calibrated data
	print(&huart1, (char*) "nuevo valor: ", acc_cal_x);
	print(&huart1, (char*) "nuevo valor: ", acc_cal_y);
	print(&huart1, (char*) "nuevo valor: ", acc_cal_z);

	// Calibration quality
	print(&huart1, (char*) "data_out.CalQuality: ", data_out.CalQuality);
}