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
#include "X-CUBE-MEMS1/motion_gc.h"

#define VERSION_STR_LENG 35
#define REPORT_INTERVAL 20

// Calibration modes
#define DYNAMIC_CALIBRATION 0
#define SIX_POINT_CALIBRATION 1

// Handlers
extern UART_HandleTypeDef huart1;

float acc_cal_x, acc_cal_y, acc_cal_z;
static volatile uint32_t TimeStamp = 0;

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
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionGC_init() {

}

//TODO Habrá que devolver los valores obtenidos, no solamente imprimirlos
void motionAC_calibrate(bool print_values) {

	uint8_t is_calibrated = 0;
	uint32_t time_stamp_uint32;
	float acc_x_mg, acc_y_mg, acc_z_mg;
	MAC_input_t data_in = { data_in.Acc[0] = 0.0f, data_in.Acc[1] = 0.0f,
			data_in.Acc[2] = 0.0f };
	MAC_output_t data_out;

	// Read acceleration X/Y/Z values in mg
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	// Convert acceleration from [mg] to [g]
	data_in.Acc[0] = (float) acc_x_mg / 1000.0f;
	data_in.Acc[1] = (float) acc_y_mg / 1000.0f;
	data_in.Acc[2] = (float) acc_z_mg / 1000.0f;
	time_stamp_uint32 = TimeStamp * REPORT_INTERVAL;
	data_in.TimeStamp = (int) time_stamp_uint32;

	// data_in -> acceleration [g] and timestamp values [ms]
	MotionAC_Update(&data_in, &is_calibrated);

	// Get Calibration coeficients
	MotionAC_GetCalParams(&data_out);

	// Do offset & scale factor calibration (acceleration [mg] and bias values [mg])
	acc_cal_x = ((acc_x_mg - acc_bias_to_mg(data_out.AccBias[0]))
			* data_out.SF_Matrix[0][0]);
	acc_cal_y = ((acc_y_mg - acc_bias_to_mg(data_out.AccBias[1]))
			* data_out.SF_Matrix[1][1]);
	acc_cal_z = ((acc_z_mg - acc_bias_to_mg(data_out.AccBias[2]))
			* data_out.SF_Matrix[2][2]);

	// Offset coefficients
	if (print_values) {
		print(&huart1, (char*) "data_out.AccBias[0]: ",
				acc_bias_to_mg(data_out.AccBias[0]));
		print(&huart1, (char*) "data_out.AccBias[1]: ",
				acc_bias_to_mg(data_out.AccBias[1]));
		print(&huart1, (char*) "data_out.AccBias[2]: ",
				acc_bias_to_mg(data_out.AccBias[2]));

		// Scale factor coefficients
		print(&huart1, (char*) "data_out.SF_Matrix[0]: ");
		print(&huart1, data_out.SF_Matrix[0][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out.SF_Matrix[0][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out.SF_Matrix[0][2]);

		print(&huart1, (char*) "data_out.SF_Matrix[1]: ");
		print(&huart1, data_out.SF_Matrix[1][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out.SF_Matrix[1][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out.SF_Matrix[1][2]);

		print(&huart1, (char*) "data_out.SF_Matrix[2]: ");
		print(&huart1, data_out.SF_Matrix[2][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out.SF_Matrix[2][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out.SF_Matrix[2][2]);

		// Calibrated data
		print(&huart1, (char*) "x's value: ");
		print(&huart1, data_in.Acc[0], (char*) " --> ", acc_cal_x);
		print(&huart1, (char*) "y's value: ");
		print(&huart1, data_in.Acc[1], (char*) " --> ", acc_cal_y);
		print(&huart1, (char*) "z's value: ");
		print(&huart1, data_in.Acc[2], (char*) " --> ", acc_cal_z);

		// Calibration quality
		print(&huart1, (char*) "data_out.CalQuality: ", data_out.CalQuality);
		print(&huart1, (char*) (char*) "TimeStamp (ms): ", data_in.TimeStamp);
		print(&huart1, (char*) "is calibrated: ", is_calibrated);
	}

	TimeStamp++;
}

float acc_bias_to_mg(float acc_bias) {
	float ans_float;

	if (acc_bias >= 0.0f) {
		/* To be MISRA C-2012 compliant the original calculation:
		 return (int16_t)(acc_bias * 1000.0f + 0.5f);
		 has been split to separate expressions */
		ans_float = acc_bias * 1000.0f + 0.5f;
		return ans_float;
	} else {
		/* To be MISRA C-2012 compliant the original calculation:
		 return (int16_t)(acc_bias * 1000.0f - 0.5f);
		 has been split to separate expressions */
		ans_float = acc_bias * 1000.0f - 0.5f;
		return ans_float;
	}
}

void motionGC_calibrate() {

}

