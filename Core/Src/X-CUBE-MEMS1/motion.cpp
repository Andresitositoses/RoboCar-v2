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
#include "X-CUBE-MEMS1/motion_fx.h"
#include "X-CUBE-MEMS1/motion_ec.h"

#define VERSION_STR_LENG 		35
#define REPORT_INTERVAL 		20
#define SAMPLE_FREQUENCY 		25.0f

// For MotionFX
#define STATE_SIZE (size_t)(2450)
static uint8_t mfxstate[STATE_SIZE ];
#define CAL_FREQ  25U /* Algorithm frequency 25 Hz */
#define ALGO_FREQ  100U /* Algorithm frequency 100 Hz */
#define FROM_UT50_TO_MGAUSS  500.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define GBIAS_MAG_TH_SC                 (2.0f*0.001500f)
#define DECIMATION                      1U
float MagCalStatus = 0;
float mag_offset_x;
float mag_offset_y;
float mag_offset_z;

// For MotionEC and MotionFX
float AccMatrix[3][3];
float MagMatrix[3][3];
float GyrMatrix[3][3];

// Handlers
extern UART_HandleTypeDef huart1;

// General variables
float acc_cal_x, acc_cal_y, acc_cal_z;
float gyr_cal_x, gyr_cal_y, gyr_cal_z;
float mag_cal_x, mag_cal_y, mag_cal_z;
uint64_t time_stamp_uint64 = 0;
uint32_t prevTick, currTick;
float degrees;

//////////////////////////////////////////////////////////////
//////////////////// Motion init functions ///////////////////
//////////////////////////////////////////////////////////////

void motionAC_init() {

	__CRC_CLK_ENABLE()
	;

	prevTick = HAL_GetTick();
	currTick = prevTick;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	MAC_knobs_t Knobs;

	// Accelerometer calibration API initialization function
	MotionAC_Initialize(1);
	MotionAC_GetKnobs(&Knobs);
	Knobs.MoveThresh_g = 0.2f;
	Knobs.Run6PointCal = 0; // 0 -> DYNAMIC_CALIBRATION
	Knobs.Sample_ms = REPORT_INTERVAL;
	MotionAC_SetKnobs(&Knobs);

	// Get version
	MotionAC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionAC2_init() {

	__CRC_CLK_ENABLE()
	;

	float sample_frequency = SAMPLE_FREQUENCY;
	prevTick = HAL_GetTick();
	currTick = prevTick;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	MAC2_knobs_t Knobs;

	// Accelerometer calibration API initialization function
	MotionAC2_Init(MAC2_MCU_STM32, &sample_frequency);
	MotionAC2_GetKnobs(&Knobs);
	Knobs.FullScale = (float) 2000; // 2 g value recommended to perform a successful calibration
	Knobs.CalDuration_s = 20; // Duration of calibration in seconds
	Knobs.XlNoiseScale = 1.0f;
	MotionAC2_SetKnobs(&Knobs);

	// Get version
	MotionAC2_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionGC_init() {

	__CRC_CLK_ENABLE()
		;

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

	__CRC_CLK_ENABLE()
	;

	// Initialization
	char lib_version[VERSION_STR_LENG];

	MotionMC_Initialize(ALGO_FREQ / 10, 1); // 10 -> time[ms] between update function calls

	MotionMC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionFX_init() {

	__CRC_CLK_ENABLE()
	;

	prevTick = HAL_GetTick();
	currTick = prevTick;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	MFX_knobs_t Knobs;

	// Check if statically allocated memory size is sufficient
	// to store MotionFX algorithm state and resize if necessary
	if (STATE_SIZE < MotionFX_GetStateSize())
		Error_Handler();

	// Sensor Fusion API initialization function
	MotionFX_initialize((MFXState_t*) mfxstate);
	MotionFX_getKnobs((MFXState_t*) mfxstate, &Knobs);

	Acc_GetOrientation(Knobs.acc_orientation);
	Gyr_GetOrientation(Knobs.gyro_orientation);
	Mag_GetOrientation(Knobs.mag_orientation);

	calc_matrix(Knobs.acc_orientation, AccMatrix);
	calc_matrix(Knobs.gyro_orientation, GyrMatrix);
	calc_matrix(Knobs.mag_orientation, MagMatrix);

	Knobs.gbias_acc_th_sc = GBIAS_ACC_TH_SC;
	Knobs.gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;
	Knobs.gbias_mag_th_sc = GBIAS_MAG_TH_SC;

	Knobs.output_type = MFX_ENGINE_OUTPUT_ENU; // east, north, up coordinates
	Knobs.LMode = 1; // 1-static learning, 2-dynamic learning
	Knobs.modx = DECIMATION;

	MotionFX_setKnobs((MFXState_t*) mfxstate, &Knobs);

	// Enable 9-axis sensor fusion (ACC + GYRO + MAG)
	MotionFX_enable_9X((MFXState_t*) mfxstate, MFX_ENGINE_ENABLE);

	// Starts magnetometer calibration
	MotionFX_MagCal_init(10, 1);

	MagCalStatus = 0;

	// Get version
	MotionFX_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionEC_init() {

	__CRC_CLK_ENABLE()
	;

	prevTick = HAL_GetTick();
	currTick = prevTick;

	// Initialization
	char lib_version[VERSION_STR_LENG];
	float freq = ALGO_FREQ;

	char acc_orientation[4];
	char mag_orientation[4];

	Acc_GetOrientation(acc_orientation);
	Mag_GetOrientation(mag_orientation);

	calc_matrix(acc_orientation, AccMatrix);
	calc_matrix(mag_orientation, MagMatrix);

	// eCompass calibration API initialization function
	MotionEC_Initialize(MEC_MCU_STM32, &freq);

	MotionEC_SetOrientationEnable(MEC_ENABLE);
	MotionEC_SetVirtualGyroEnable(MEC_ENABLE);
	MotionEC_SetGravityEnable(MEC_ENABLE);
	MotionEC_SetLinearAccEnable(MEC_ENABLE);

	// Get version
	MotionEC_GetLibVersion(lib_version);
	print(&huart1, (char*) lib_version);
	print(&huart1, (char*) "\n");
}

void motionEC_MC_init(){
	motionMC_init();
	motionEC_init();
}

//////////////////////////////////////////////////////////////
///////////// Motion calibration functions ///////////////////
//////////////////////////////////////////////////////////////

void motionAC_calibrate(bool print_values) {

	uint8_t is_calibrated = 0;
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
	currTick = HAL_GetTick();
	time_stamp_uint64 += currTick - prevTick;
	data_in.TimeStamp = time_stamp_uint64;
	prevTick = currTick;

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
}

void motionAC2_calibrate(bool print_values) {

	uint8_t is_calibrated = 0;
	float acc_x_mg, acc_y_mg;
	MAC2_input_t data_in = { data_in.Acc_X = 0.0f, data_in.Acc_Y = 0.0f };
	MAC2_cal_params_t data_out;

	// Read acceleration X/Y values in mg
	MEMS_Read_Acc2Value(&acc_x_mg, &acc_y_mg);

	// Convert acceleration data from [mg] to [g]
	data_in.Acc_X = (float) acc_x_mg / 1000.0f;
	data_in.Acc_Y = (float) acc_y_mg / 1000.0f;
	currTick = HAL_GetTick();
	time_stamp_uint64 += currTick - prevTick;
	prevTick = currTick;

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
}

void motionGC_calibrate(bool print_values) {

	// Read accelerometer and gyroscope values
	int bias_update = 0;
	float acc_x_mg, acc_y_mg, acc_z_mg;
	float gyr_x_mdps, gyr_y_mdps, gyr_z_mdps;
	MGC_input_t data_in = { data_in.Acc[0] = 0.0f, data_in.Acc[1] = 0.0f,
			data_in.Acc[2] = 0.0f, data_in.Gyro[0] = 0.0f, data_in.Gyro[1] =
					0.0f, data_in.Gyro[2] = 0.0f };
	MGC_output_t data_out;

	// Get acceleration X/Y/Z in mg
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	// Get angular rate X/Y/Z in mdps
	MEMS_Read_GyroValue(&gyr_x_mdps, &gyr_y_mdps, &gyr_z_mdps);

	// Convert acceleration from [mg] to [g]
	data_in.Acc[0] = (float) acc_x_mg / 1000.0f;
	data_in.Acc[1] = (float) acc_y_mg / 1000.0f;
	data_in.Acc[2] = (float) acc_z_mg / 1000.0f;

	// Convert angular velocity from [mdps] to [dps]
	data_in.Gyro[0] = (float) gyr_x_mdps / 1000.0f;
	data_in.Gyro[1] = (float) gyr_y_mdps / 1000.0f;
	data_in.Gyro[2] = (float) gyr_z_mdps / 1000.0f;

	// Gyroscope calibration algorithm update
	MotionGC_Update(&data_in, &data_out, &bias_update);

	// Get Calibration coeficients
	MotionGC_GetCalParams(&data_out);

	// Do offset & scale factor calibration (bias values [mdps])
	gyr_cal_x = gyr_x_mdps - gyro_bias_to_mdps(data_out.GyroBiasX);
	gyr_cal_y = gyr_y_mdps - gyro_bias_to_mdps(data_out.GyroBiasY);
	gyr_cal_z = gyr_z_mdps - gyro_bias_to_mdps(data_out.GyroBiasZ);

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
		print(&huart1, gyr_x_mdps, (char*) " --> ", gyr_cal_x);
		print(&huart1, (char*) "Gyr_y's value: ");
		print(&huart1, gyr_y_mdps, (char*) " --> ", gyr_cal_y);
		print(&huart1, (char*) "Gyr_z's value: ");
		print(&huart1, gyr_z_mdps, (char*) " --> ", gyr_cal_z);

		print(&huart1, (char*) "bias_update: ", bias_update);
	}
}

bool motionMC_calibrate(bool print_values) {

	float mag_x_mG, mag_y_mG, mag_z_mG;
	MMC_Input_t *data_in = new MMC_Input_t;
	MMC_Output_t *data_out = new MMC_Output_t;

	// Read magnetometer X/Y/Z values in mGauss
	MEMS_Read_MagValue(&mag_x_mG, &mag_y_mG, &mag_z_mG);

	// Convert magnetometer data from [mGauss] to [uT]
	data_in->Mag[0] = (float) mag_x_mG / 10.0f;
	data_in->Mag[1] = (float) mag_y_mG / 10.0f;
	data_in->Mag[2] = (float) mag_z_mG / 10.0f;

	currTick = HAL_GetTick();
	time_stamp_uint64 += currTick - prevTick;
	data_in->TimeStamp = time_stamp_uint64;
	prevTick = currTick;

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
	}

	free(data_in);
	free(data_out);

	return (data_out->CalQuality == MMC_CALQSTATUSGOOD);
}

bool motionFX_calibrate(bool print_values) {

	currTick = HAL_GetTick();
	time_stamp_uint64 += currTick - prevTick;
	float delta_time = time_stamp_uint64 / 1000;
	prevTick = currTick;

	float mag_x_mG, mag_y_mG, mag_z_mG;

	// Read magnetometer X/Y/Z values in mGauss
	MEMS_Read_MagValue(&mag_x_mG, &mag_y_mG, &mag_z_mG);

	// If it is not calibrated yet
	if (MagCalStatus == 0){
		MFX_MagCal_input_t mag_data_in;
		mag_data_in.mag[0] = mag_x_mG * FROM_MGAUSS_TO_UT50;
		mag_data_in.mag[1] = mag_y_mG * FROM_MGAUSS_TO_UT50;
		mag_data_in.mag[2] = mag_z_mG * FROM_MGAUSS_TO_UT50;
		mag_data_in.time_stamp = time_stamp_uint64;
		MotionFX_MagCal_run(&mag_data_in);

		// Test if calibration data are available and get bias values
		MFX_MagCal_output_t mag_cal_test;
		MotionFX_MagCal_getParams(&mag_cal_test);

		if (mag_cal_test.cal_quality == MFX_MAGCALGOOD) {

			mag_offset_x = mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS;
			mag_offset_y = mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS;
			mag_offset_z = mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS;

			// Stops calibration process
			MotionFX_MagCal_init(10, 0);

			MagCalStatus = 1;
		}
	}

	mag_cal_x = mag_x_mG - mag_offset_x;
	mag_cal_y = mag_y_mG - mag_offset_y;
	mag_cal_z = mag_z_mG - mag_offset_z;

	if (MagCalStatus) {

		float acc_x_mg, acc_y_mg, acc_z_mg;
		float gyr_x_mdps, gyr_y_mdps, gyr_z_mdps;
		MFX_input_t data_in;
		MFX_output_t data_out;

		// Read acceleration X/Y/Z values in mg
		MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

		// Get angular rate X/Y/Z in mdps
		MEMS_Read_GyroValue(&gyr_x_mdps, &gyr_y_mdps, &gyr_z_mdps);

		// Do sensor orientation transformation: &AccValue, &GyrValue &MagValueComp, data_in.acc, data_in.mag
		float acc[3] = {acc_x_mg, acc_y_mg, acc_z_mg};
		float gyr[3] = {gyr_x_mdps, gyr_y_mdps, gyr_z_mdps};
		float mag[3] = {mag_cal_x, mag_cal_y, mag_cal_z};
		transform_orientation(acc, data_in.acc, AccMatrix);
		transform_orientation(gyr, data_in.gyro, GyrMatrix);
		transform_orientation(mag, data_in.mag, MagMatrix);

		// Convert acceleration from [mg] to [g]
		data_in.acc[0] = data_in.acc[0] / 1000.0f;
		data_in.acc[1] = data_in.acc[1] / 1000.0f;
		data_in.acc[2] = data_in.acc[2] / 1000.0f;

		// Convert angular velocity from [mdps] to [dps]
		data_in.gyro[0] = data_in.gyro[0] / 1000.0f;
		data_in.gyro[1] = data_in.gyro[1] / 1000.0f;
		data_in.gyro[2] = data_in.gyro[2] / 1000.0f;

		// Apply calibration results [uT/50]
		data_in.mag[0] = data_in.mag[0] * FROM_MGAUSS_TO_UT50;
		data_in.mag[1] = data_in.mag[1] * FROM_MGAUSS_TO_UT50;
		data_in.mag[2] = data_in.mag[2] * FROM_MGAUSS_TO_UT50;

		// Run Sensor Fusion algorithm
		// propagate: estimates the orientation in 3D space by giving more weight to gyroscope data
		// update: adjusts the predicted value by giving more weight to accelerometer and magnetometer data
		MotionFX_propagate(&mfxstate, &data_out, &data_in,
				&delta_time);
		MotionFX_update(&mfxstate, &data_out, &data_in,
				&delta_time,
				NULL);

		print(&huart1, (char*) "heading: ", data_out.headingErr); // yaw
		//print(&huart1, (char*) "rotation[1]: ", data_out.rotation[1]); // yaw
		//print(&huart1, (char*) "rotation[2]: ", data_out.rotation[2]); // yaw
		//print(&huart1, (char*) "rotation[3]: ", data_out.rotation[3]); // yaw

		if (print_values) {

			// Bias
			print(&huart1, (char*) "mag_offset_x: ",
					(float) (mag_offset_x * FROM_UT50_TO_MGAUSS));
			print(&huart1, (char*) "mag_offset_y: ",
					(float) (mag_offset_y * FROM_UT50_TO_MGAUSS));
			print(&huart1, (char*) "mag_offset_z: ",
					(float) (mag_offset_y * FROM_UT50_TO_MGAUSS));

			// Quaternion
			print(&huart1, (char*) "quaternion[0]: ",
					data_out.quaternion[0]);
			print(&huart1, (char*) "quaternion[1]: ",
					data_out.quaternion[1]);
			print(&huart1, (char*) "quaternion[2]: ",
					data_out.quaternion[2]);
			print(&huart1, (char*) "quaternion[3]: ",
					data_out.quaternion[3]);

			// Rotation
			print(&huart1, (char*) "rotation[0]: ", data_out.rotation[0]); // yaw
			print(&huart1, (char*) "rotation[1]: ", data_out.rotation[1]); // pitch
			print(&huart1, (char*) "rotation[2]: ", data_out.rotation[2]); // roll

			// Gravity
			print(&huart1, (char*) "gravity.x: ", data_out.gravity[0]);
			print(&huart1, (char*) "gravity.y: ", data_out.gravity[1]);
			print(&huart1, (char*) "gravity.z: ", data_out.gravity[2]);

			// Linear acceleration
			print(&huart1, (char*) "acc.x: ",
					data_out.linear_acceleration[0]);
			print(&huart1, (char*) "acc.y: ",
					data_out.linear_acceleration[1]);
			print(&huart1, (char*) "acc.z: ",
					data_out.linear_acceleration[2]);

			// Heading and headingErr
			print(&huart1, (char*) "heading: ", data_out.heading);
			print(&huart1, (char*) "headingErr: ", data_out.headingErr);

			// TimeStamp
			print(&huart1, (char*) "time_stamp: ", (int) time_stamp_uint64);
		}

		return true;

	}
	else {
		return false;
	}
}

void motionEC_calibrate(bool print_values) {

	float acc_x_mg, acc_y_mg, acc_z_mg;
	float mag_x_mG, mag_y_mG, mag_z_mG;
	MEC_input_t data_in;
	MEC_output_t data_out;

	currTick = HAL_GetTick();
	time_stamp_uint64 += currTick - prevTick;
	float delta_time = time_stamp_uint64 / 1000;
	prevTick = currTick;

	// Read acceleration X/Y/Z values in mg
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	// Read magnetometer X/Y/Z values in mGauss
	MEMS_Read_MagValue(&mag_x_mG, &mag_y_mG, &mag_z_mG);

	// Convert acceleration from [mg] to [g]
	data_in.acc[0] = (float) acc_x_mg / 1000.0f;
	data_in.acc[1] = (float) acc_y_mg / 1000.0f;
	data_in.acc[2] = (float) acc_z_mg / 1000.0f;

	// Magnetometer data in [uT/50]
	data_in.mag[0] = mag_x_mG * FROM_MGAUSS_TO_UT50;
	data_in.mag[1] = mag_y_mG * FROM_MGAUSS_TO_UT50;
	data_in.mag[2] = mag_z_mG * FROM_MGAUSS_TO_UT50;

	// Delta time [s]
	data_in.deltatime_s = delta_time;

	// Run E-Compass algorithm
	MotionEC_Run(&data_in, &data_out);

	if (print_values) {
		// Rotation values
		print(&huart1, (char*) "euler[0]: ", data_out.euler[0]);
		print(&huart1, (char*) "euler[1]: ", data_out.euler[1]);
		print(&huart1, (char*) "euler[2]: ", data_out.euler[2]);

		// Gravity values
		print(&huart1, (char*) "gravity[0]: ", data_out.gravity[0]);
		print(&huart1, (char*) "gravity[1]: ", data_out.gravity[1]);
		print(&huart1, (char*) "gravity[2]: ", data_out.gravity[2]);
	}
}

bool motionEC_MC_calibrate(bool print_values) {

	// Calculate mag_cal values
	if (!motionMC_calibrate(0)) {
		return false;
	}

	// Read acceleration X/Y/Z values in mg
	float acc_x_mg, acc_y_mg, acc_z_mg;
	MEMS_Read_AccValue(&acc_x_mg, &acc_y_mg, &acc_z_mg);

	MEC_input_t data_in;
	MEC_output_t data_out;

	// Do sensor orientation transformation: &AccValue, &MagValueComp, data_in.acc, data_in.mag
	float acc[3] = {acc_x_mg, acc_y_mg, acc_z_mg};
	float mag[3] = {mag_cal_x, mag_cal_y, mag_cal_z};
	transform_orientation(acc, data_in.acc, AccMatrix);
	transform_orientation(mag, data_in.mag, MagMatrix);

	// Convert raw accelerometer data from [mg] to [g]
	data_in.acc[0] = data_in.acc[0] / 1000.0f; /* East */
	data_in.acc[1] = data_in.acc[1] / 1000.0f; /* North */
	data_in.acc[2] = data_in.acc[2] / 1000.0f; /* Up */

	// Convert compensated magnetometer data from [mGauss] to [uT / 50], [mGauss / 5]
	data_in.mag[0] = data_in.mag[0] / 5.0f; /* East */
	data_in.mag[1] = data_in.mag[1] / 5.0f; /* North */
	data_in.mag[2] = data_in.mag[2] / 5.0f; /* Up */

	// Delta time [s]
	data_in.deltatime_s = time_stamp_uint64 / 1000;

	// Run E-Compass algorithm
	MotionEC_Run(&data_in, &data_out);

	degrees = data_out.euler[0];

	if (print_values) {
		// Rotation values
		print(&huart1, (char*) "euler[0]: ", data_out.euler[0]);
		print(&huart1, (char*) "euler[1]: ", data_out.euler[1]);
		print(&huart1, (char*) "euler[2]: ", data_out.euler[2]);

		// Gravity values
		print(&huart1, (char*) "gravity[0]: ", data_out.gravity[0]);
		print(&huart1, (char*) "gravity[1]: ", data_out.gravity[1]);
		print(&huart1, (char*) "gravity[2]: ", data_out.gravity[2]);
	}

	return true;
}

//////////////////////////////////////////////////////////////
//////////////////// Auxiliary functions /////////////////////
//////////////////////////////////////////////////////////////

void Acc_GetOrientation(char *Orientation)
{
	Orientation[0] = 'n';
	Orientation[1] = 'w';
	Orientation[2] = 'u';
}

void Gyr_GetOrientation(char *Orientation)
{
	Orientation[0] = 'n';
	Orientation[1] = 'w';
	Orientation[2] = 'u';
}

void Mag_GetOrientation(char *Orientation)
{
	Orientation[0] = 'n';
	Orientation[1] = 'e';
	Orientation[2] = 'u';
}

void calc_matrix(char orientation[], float matrix[][3])
{
	matrix[0][0] = (orientation[0] == 'e') ?  1
			   : (orientation[0] == 'w') ? -1
			   :                            0;

	matrix[0][1] = (orientation[1] == 'e') ?  1
			   : (orientation[1] == 'w') ? -1
			   :                            0;

	matrix[0][2] = (orientation[2] == 'e') ?  1
			   : (orientation[2] == 'w') ? -1
			   :                            0;

	matrix[1][0] = (orientation[0] == 'n') ?  1
			   : (orientation[0] == 's') ? -1
			   :                            0;

	matrix[1][1] = (orientation[1] == 'n') ?  1
			   : (orientation[1] == 's') ? -1
			   :                            0;

	matrix[1][2] = (orientation[2] == 'n') ?  1
			   : (orientation[2] == 's') ? -1
			   :                            0;

	matrix[2][0] = (orientation[0] == 'u') ?  1
			   : (orientation[0] == 'd') ? -1
			   :                            0;

	matrix[2][1] = (orientation[1] == 'u') ?  1
			   : (orientation[1] == 'd') ? -1
			   :                            0;

	matrix[2][2] = (orientation[2] == 'u') ?  1
			   : (orientation[2] == 'd') ? -1
			   :                            0;
}

void q_conjug(float q_conj[], float q_src[])
{
	q_conj[0] = (-1.0f) * q_src[0];
	q_conj[1] = (-1.0f) * q_src[1];
	q_conj[2] = (-1.0f) * q_src[2];
	q_conj[3] =           q_src[3];
}

void q_multiply(float q_res[], float q_a[], float q_b[])
{
	q_res[0] =
	q_a[3] * q_b[0]
	+ q_a[0] * q_b[3]
	+ q_a[1] * q_b[2]
	- q_a[2] * q_b[1]
	;

	q_res[1] =
	q_a[3] * q_b[1]
	+ q_a[1] * q_b[3]
	+ q_a[2] * q_b[0]
	- q_a[0] * q_b[2]
	;

	q_res[2] =
	q_a[3] * q_b[2]
	+ q_a[2] * q_b[3]
	+ q_a[0] * q_b[1]
	- q_a[1] * q_b[0]
	;

	q_res[3] =
	q_a[3] * q_b[3]
	- q_a[0] * q_b[0]
	- q_a[1] * q_b[1]
	- q_a[2] * q_b[2]
	;
}

void v_rotate(float v_new[], float q_rot[], float v_old[])
{
	float q_old[4];
	float q_new[4];
	float q_rot_inv[4];
	float q_temp[4];

	/* Create quaternion from old position vector */
	q_old[0] = v_old[0];
	q_old[1] = v_old[1];
	q_old[2] = v_old[2];
	q_old[3] = 0.0f;

	q_conjug(q_rot_inv, q_rot);
	q_multiply(q_temp, q_old, q_rot_inv);
	q_multiply(q_new, q_rot, q_temp);

	v_new[0] = q_new[0];
	v_new[1] = q_new[1];
	v_new[2] = q_new[2];
}

int calc_heading(float *heading, float v_head[])
{
	const float tol_deg = 5.0f; /* Tolerance [deg] */
	float tolerance = sinf(tol_deg * M_PI / 180.0f);

	if (v_head[0] > (-1.0f) * tolerance && v_head[0] < tolerance
		&& v_head[1] > (-1.0f) * tolerance && v_head[1] < tolerance)
	{
		*heading = 0.0f;
		return 0; /* Device is pointing up or down - it is impossible to evaluate heading */
	}

	else
	{
		*heading = atan2f(v_head[0], v_head[1]) * 180.0f / M_PI;
		*heading = floorf(*heading * 100.0f + 0.5f) / 100.0f;          /* Rounds number to two decimal digits */
		*heading = (*heading < 0.0f) ? (*heading + 360.0f) : *heading; /* Change negative value to be in range <0,360) */
		return 1;
	}
}

void MotionEC_manager_calc_heading(float quaternion[], float *heading, int *heading_valid){
	float v_base[3] = {0.0, 1.0, 0.0};
	float v_head[3];

	v_rotate(v_head, quaternion, v_base);
	*heading_valid = calc_heading(heading, v_head);
}

void transform_orientation(float *input, float output[], float matrix[][3])
{
	output[0] = matrix[0][0] * input[0]  +  matrix[0][1] * input[1]  +  matrix[0][2] * input[2];
	output[1] = matrix[1][0] * input[0]  +  matrix[1][1] * input[1]  +  matrix[1][2] * input[2];
	output[2] = matrix[2][0] * input[0]  +  matrix[2][1] * input[1]  +  matrix[2][2] * input[2];
}

//////////////////////////////////////////////////////////////
//////////////////// Convertion functions ////////////////////
//////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////
/////////////////////////// Getters //////////////////////////
//////////////////////////////////////////////////////////////

float motion_getDegrees(){
	return degrees;
}
