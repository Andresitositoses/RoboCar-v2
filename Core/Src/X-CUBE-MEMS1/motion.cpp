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
#include "X-CUBE-MEMS1/motion_ac2.h"
#include "X-CUBE-MEMS1/motion_mc.h"

#define VERSION_STR_LENG 		35
#define REPORT_INTERVAL 		20
#define SAMPLE_FREQUENCY 		50.0f

// Calibration modes
#define DYNAMIC_CALIBRATION 	0
#define SIX_POINT_CALIBRATION 	1

// Handlers
extern UART_HandleTypeDef huart1;

float acc_cal_x, acc_cal_y, acc_cal_z;
float gyr_cal_x, gyr_cal_y, gyr_cal_z;
float mag_cal_x, mag_cal_y, mag_cal_z;
static volatile uint32_t TimeStamp = 0;

// Init functions

void motionAC_init() {

	// Initialization
	__CRC_CLK_ENABLE()
	;
	char lib_version[VERSION_STR_LENG];
	MAC_knobs_t Knobs;

	// Accelerometer calibration API initialization function
	MotionAC_Initialize(1);
	MotionAC_GetKnobs(&Knobs);
	Knobs.MoveThresh_g = 0.2f;
	Knobs.Run6PointCal = DYNAMIC_CALIBRATION;
	Knobs.Sample_ms = REPORT_INTERVAL;
	MotionAC_SetKnobs(&Knobs);

	// Get version
	MotionAC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionAC2_init() {

	float sample_frequency = SAMPLE_FREQUENCY;

	// Initialization
	__CRC_CLK_ENABLE()
	;
	char lib_version[VERSION_STR_LENG];
	MAC2_knobs_t Knobs;

	// Accelerometer calibration API initialization function
	MotionAC2_Init(MAC2_MCU_STM32, &sample_frequency);
	MotionAC2_GetKnobs(&Knobs);
	Knobs.FullScale = (float) 2000; // 2 g value recommended to perform a successful calibration
	Knobs.CalDuration_s = 120; // Duration of calibration in seconds
	Knobs.XlNoiseScale = 1.0f;
	MotionAC2_SetKnobs(&Knobs);

	// Get version
	MotionAC2_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionGC_init() {

	float sample_frequency = SAMPLE_FREQUENCY;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	MGC_knobs_t Knobs;

	// Gyroscope calibration API initialization function
	MotionGC_Initialize(MGC_MCU_STM32, &sample_frequency);

	// Get current settings and set desired ones
	MotionGC_GetKnobs(&Knobs);
	Knobs.AccThr = 0.008f;
	Knobs.GyroThr = 0.15f;
	MotionGC_SetKnobs(&Knobs);

	// Get version
	MotionGC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionMC_init() {

	// Initialization
	char lib_version[VERSION_STR_LENG];

	MotionMC_Initialize(REPORT_INTERVAL, 1);

	MotionMC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

// Calibration functions

void motionAC_calibrate(bool print_values) {

	uint8_t is_calibrated = 0;
	uint32_t time_stamp_uint32;
	float acc_x_mg, acc_y_mg, acc_z_mg;
	MAC_input_t data_in = { data_in.Acc[0] = 0.0f, data_in.Acc[1] = 0.0f,
			data_in.Acc[2] = 0.0f };
	MAC_output_t data_out;

	// Read acceleration X/Y/Z values in mg
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	// Convert acceleration data from [mg] to [g]
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

	if (print_values) {

		// Bias values
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
		print(&huart1, (char*) "Acc_x's value: ");
		print(&huart1, acc_x_mg, (char*) " --> ", acc_cal_x);
		print(&huart1, (char*) "Acc_y's value: ");
		print(&huart1, acc_y_mg, (char*) " --> ", acc_cal_y);
		print(&huart1, (char*) "Acc_z's value: ");
		print(&huart1, acc_z_mg, (char*) " --> ", acc_cal_z);

		print(&huart1, (char*) "data_out.CalQuality: ", data_out.CalQuality);
		print(&huart1, (char*) (char*) "TimeStamp (ms): ", data_in.TimeStamp);
		print(&huart1, (char*) "is calibrated: ", is_calibrated);
	}

	TimeStamp++;
}

void motionAC2_calibrate(bool print_values) {

	uint8_t is_calibrated = 0;
	uint64_t time_stamp_uint64;
	float acc_x_mg, acc_y_mg;
	MAC2_input_t data_in = { data_in.Acc_X = 0.0f, data_in.Acc_Y = 0.0f };
	MAC2_cal_params_t data_out;

	// Read acceleration X/Y values in mg
	MEMS_Read_Acc2Value(&acc_x_mg, &acc_y_mg);

	// Convert acceleration data from [mg] to [g]
	data_in.Acc_X = (float) acc_x_mg / 1000.0f;
	data_in.Acc_Y = (float) acc_y_mg / 1000.0f;
	time_stamp_uint64 = TimeStamp * REPORT_INTERVAL;

	// data_in -> acceleration [g] and timestamp values [ms]
	is_calibrated = MotionAC2_Update(&data_in, time_stamp_uint64);

	// Get Calibration coeficients
	MotionAC2_GetCalParams(&data_out);

	// Do offset & scale factor calibration (acceleration [mg] and bias values [mg])
	acc_cal_x =
			((acc_x_mg - acc_bias_to_mg(data_out.Bias[0])) * data_out.SF[0]);
	acc_cal_y =
			((acc_y_mg - acc_bias_to_mg(data_out.Bias[1])) * data_out.SF[1]);

	if (print_values) {

		// Bias values
		print(&huart1, (char*) "data_out.AccBias[0]: ",
				acc_bias_to_mg(data_out.Bias[0]));
		print(&huart1, (char*) "data_out.AccBias[1]: ",
				acc_bias_to_mg(data_out.Bias[1]));

		// Scale factor coefficients
		print(&huart1, (char*) "data_out.SF_Matrix[0]: ", data_out.SF[0]);
		print(&huart1, (char*) "data_out.SF_Matrix[1]: ", data_out.SF[1]);

		// Calibrated data
		print(&huart1, (char*) "Acc_x's value: ");
		print(&huart1, acc_x_mg, (char*) " --> ", acc_cal_x);
		print(&huart1, (char*) "Acc_y's value: ");
		print(&huart1, acc_y_mg, (char*) " --> ", acc_cal_y);

		print(&huart1, (char*) "data_out.CalStatus: ", data_out.CalStatus);
		print(&huart1, (char*) "TimeStamp (ms): ", (int) time_stamp_uint64);
		print(&huart1, (char*) "is calibrated: ", is_calibrated);
	}

	TimeStamp++;
}

void motionGC_calibrate(bool print_values) {

	// Read accelerometer and gyroscope values
	int bias_update = 0;
	float acc_x_mg, acc_y_mg, acc_z_mg;
	float gyr_x_mpds, gyr_y_mpds, gyr_z_mpds;
	MGC_input_t data_in = { data_in.Acc[0] = 0.0f, data_in.Acc[1] = 0.0f,
			data_in.Acc[2] = 0.0f, data_in.Gyro[0] = 0.0f, data_in.Gyro[1] =
					0.0f, data_in.Gyro[2] = 0.0f };
	MGC_output_t data_out;

	// Get acceleration X/Y/Z in mg
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	// Get angular rate X/Y/Z in mdps
	MEMS_Read_GyroValue(&gyr_x_mpds, &gyr_y_mpds, &gyr_z_mpds);

	// Convert acceleration from [mg] to [g]
	data_in.Acc[0] = (float) acc_x_mg / 1000.0f;
	data_in.Acc[1] = (float) acc_y_mg / 1000.0f;
	data_in.Acc[2] = (float) acc_z_mg / 1000.0f;

	// Convert angular velocity from [mdps] to [dps]
	data_in.Gyro[0] = (float) gyr_x_mpds / 1000.0f;
	data_in.Gyro[1] = (float) gyr_y_mpds / 1000.0f;
	data_in.Gyro[2] = (float) gyr_z_mpds / 1000.0f;

	// Gyroscope calibration algorithm update
	MotionGC_Update(&data_in, &data_out, &bias_update);

	// Get Calibration coeficients
	MotionGC_GetCalParams(&data_out);

	// Do offset & scale factor calibration (bias values [mdps])
	gyr_cal_x = gyr_x_mpds - gyro_bias_to_mdps(data_out.GyroBiasX);
	gyr_cal_y = gyr_y_mpds - gyro_bias_to_mdps(data_out.GyroBiasY);
	gyr_cal_z = gyr_z_mpds - gyro_bias_to_mdps(data_out.GyroBiasZ);

	if (print_values) {

		// Bias values
		print(&huart1, (char*) "data_out.GyroBiasX: ",
				gyro_bias_to_mdps(data_out.GyroBiasX));
		print(&huart1, (char*) "data_out.GyroBiasY: ",
				gyro_bias_to_mdps(data_out.GyroBiasY));
		print(&huart1, (char*) "data_out.GyroBiasZ: ",
				gyro_bias_to_mdps(data_out.GyroBiasZ));

		// Data convertion
		print(&huart1, (char*) "Gyr_x's value: ");
		print(&huart1, gyr_x_mpds, (char*) " --> ", gyr_cal_x);
		print(&huart1, (char*) "Gyr_y's value: ");
		print(&huart1, gyr_y_mpds, (char*) " --> ", gyr_cal_y);
		print(&huart1, (char*) "Gyr_z's value: ");
		print(&huart1, gyr_z_mpds, (char*) " --> ", gyr_cal_z);

		print(&huart1, (char*) "bias_update: ", bias_update);
	}
}

void motionMC_calibrate(bool print_values) {

	float mag_x_mG, mag_y_mG, mag_z_mG;
	MMC_Input_t *data_in = new MMC_Input_t;
	MMC_Output_t *data_out = new MMC_Output_t;

	// Read magnetometer X/Y/Z values in mGauss
	MEMS_Read_MagValue(&mag_x_mG, &mag_y_mG, &mag_z_mG);

	// Convert magnetometer data from [mGauss] to [uT]
	data_in->Mag[0] = (float) mag_x_mG / 10.0f;
	data_in->Mag[1] = (float) mag_y_mG / 10.0f;
	data_in->Mag[2] = (float) mag_z_mG / 10.0f;
	data_in->TimeStamp = (int) HAL_GetTick();

	// data_in -> magnetometer data [uT]
	MotionMC_Update(&*data_in);

	// Get Calibration coeficients
	MotionMC_GetCalParams(&*data_out);

	// Do hard & soft iron calibration
	float *mag_raw_mG = (float*) malloc(sizeof(float) * 3);
	float *mag_comp_mG = (float*) malloc(sizeof(float) * 3);

	mag_raw_mG[0] = (float) mag_x_mG;
	mag_raw_mG[1] = (float) mag_y_mG;
	mag_raw_mG[2] = (float) mag_z_mG;

	// Compensate magnetometer data (coefficients in [mGauss])
	for (int i = 0; i < 3; i++) {
		mag_comp_mG[i] = 0.0f;
		for (int j = 0; j < 3; j++) {
			mag_comp_mG[i] += (mag_raw_mG[j] - data_out->HI_Bias[j] * 10.0f)
					* data_out->SF_Matrix[i][j];
		}

		mag_comp_mG[i] += (mag_comp_mG[i] >= 0.0f) ? 0.5f : -0.5f;
	}

	mag_cal_x = mag_comp_mG[0];
	mag_cal_y = mag_comp_mG[1];
	mag_cal_z = mag_comp_mG[2];

	free(mag_raw_mG);
	free(mag_comp_mG);

	if (print_values) {

		// Bias values
		print(&huart1, (char*) "data_out.HI_Bias[0]: ",
				mag_val_to_mGauss(data_out->HI_Bias[0]));
		print(&huart1, (char*) "data_out.HI_Bias[1]: ",
				mag_val_to_mGauss(data_out->HI_Bias[1]));
		print(&huart1, (char*) "data_out.HI_Bias[2]: ",
				mag_val_to_mGauss(data_out->HI_Bias[2]));

		// Scale factor coefficients
		print(&huart1, (char*) "data_out.SF_Matrix[0]: ");
		print(&huart1, data_out->SF_Matrix[0][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out->SF_Matrix[0][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out->SF_Matrix[0][2]);

		print(&huart1, (char*) "data_out.SF_Matrix[1]: ");
		print(&huart1, data_out->SF_Matrix[1][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out->SF_Matrix[1][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out->SF_Matrix[1][2]);

		print(&huart1, (char*) "data_out.SF_Matrix[2]: ");
		print(&huart1, data_out->SF_Matrix[2][0]);
		print(&huart1, (char*) " ");
		print(&huart1, data_out->SF_Matrix[2][1]);
		print(&huart1, (char*) " ");
		print(&huart1, (char*) "", data_out->SF_Matrix[2][2]);

		// Calibrated data
		print(&huart1, (char*) "Mag_x's value: ");
		print(&huart1, mag_x_mG, (char*) " --> ", mag_cal_x);
		print(&huart1, (char*) "Mag_y's value: ");
		print(&huart1, mag_y_mG, (char*) " --> ", mag_cal_y);
		print(&huart1, (char*) "Mag_z's value: ");
		print(&huart1, mag_z_mG, (char*) " --> ", mag_cal_z);

		print(&huart1, (char*) "data_out.CalQuality: ", data_out->CalQuality);
		print(&huart1, (char*) (char*) "TimeStamp (ms): ", data_in->TimeStamp);

		free(data_in);
		free(data_out);
	}
}

// Convertion functions

float acc_bias_to_mg(float acc_bias) {
	float ans_float;

	if (acc_bias >= 0.0f) {
		ans_float = acc_bias * 1000.0f + 0.5f;
		return ans_float;
	} else {
		ans_float = acc_bias * 1000.0f - 0.5f;
		return ans_float;
	}
}

float gyro_bias_to_mdps(float gyro_bias) {
	float mdps_float;

	if (gyro_bias >= 0.0f) {
		mdps_float = gyro_bias * 1000.0f + 0.5f;
		return mdps_float;
	} else {
		mdps_float = gyro_bias * 1000.0f - 0.5f;
		return mdps_float;
	}
}

float mag_val_to_mGauss(float mag_val_uT) {
	float mGauss_float;

	if (mag_val_uT >= 0.0f) {
		mGauss_float = mag_val_uT * 10.0f + 0.5f;
		return mGauss_float;
	} else {
		mGauss_float = mag_val_uT * 10.0f - 0.5f;
		return mGauss_float;
	}
}
