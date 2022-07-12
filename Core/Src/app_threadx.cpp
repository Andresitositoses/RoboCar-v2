/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include "main.h"
#include "print.h"
#include "sensors.h"
#include "RoboCar/RoboCar.h"
#include "algorithms.h"
#include "X-CUBE-MEMS1/motion.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Stacks sizes
#define MAINTHREAD_STACK_SIZE 4096
#define ENCODERS_STACK_SIZE 2048
#define SENSORS_STACK_SIZE 8192

bool calibrated = false;
extern float degrees;
float objective_dir = -1;
float deviation_dir;
float deviation_threshold = 1.5;
float factorX = 0; // Indicates how wrong the direction is
int centimeters = 0; // 1 switch between 0 and 1 is equals to 1cm

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Handlers
extern UART_HandleTypeDef huart1;
/////////////////////////
/// THREAD VARIABLES ////
/////////////////////////

// Main thread
uint8_t mainThread_stack[MAINTHREAD_STACK_SIZE];
TX_THREAD mainThread_ptr;
RoboCar::RoboCar *coche;
bool button = false;

// Encoder thread
uint8_t encondersThread_stack[ENCODERS_STACK_SIZE];
TX_THREAD encodersThread_ptr;

// Sensors thread
uint8_t sensorsThread_stack[SENSORS_STACK_SIZE];
TX_THREAD sensorsThread_ptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID mainThread_entry(ULONG initial_input);
VOID encodersThread_entry(ULONG initial_input);
VOID sensorsThread_entry(ULONG initial_input);

/* USER CODE END PFP */

/**
 * @brief  Application ThreadX Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT App_ThreadX_Init(VOID *memory_ptr) {
	UINT ret = TX_SUCCESS;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*) memory_ptr;

	/* USER CODE BEGIN App_ThreadX_MEM_POOL */
	(void) byte_pool;
	coche = new RoboCar::RoboCar();
	/* USER CODE END App_ThreadX_MEM_POOL */

	/* USER CODE BEGIN App_ThreadX_Init */
	tx_thread_create(&mainThread_ptr, (char* )"mainThread", mainThread_entry, 0,
			mainThread_stack, MAINTHREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&encodersThread_ptr, (char* )"encodersThread",
			encodersThread_entry, 0, encondersThread_stack, ENCODERS_STACK_SIZE,
			15, 15, 1, TX_AUTO_START);
	tx_thread_create(&sensorsThread_ptr, (char* )"sensorsThread",
			sensorsThread_entry, 0, sensorsThread_stack, SENSORS_STACK_SIZE, 15,
			15, 1, TX_AUTO_START);
	/* USER CODE END App_ThreadX_Init */

	return ret;
}

/**
 * @brief  MX_ThreadX_Init
 * @param  None
 * @retval None
 */
void MX_ThreadX_Init(void) {
	/* USER CODE BEGIN  Before_Kernel_Start */

	/* USER CODE END  Before_Kernel_Start */

	tx_kernel_enter();

	/* USER CODE BEGIN  Kernel_Start_Error */

	/* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

// Main thread
VOID mainThread_entry(ULONG initial_input) {

	coche->calibrate();
	coche->showCalibrations();
	coche->setSpeed(16);

	while (!calibrated){
		print(&huart1, (char*)"Calibrating...\n");
		tx_thread_sleep(100); // 1s
	}

	print(&huart1, (char*)"Calibrated successfully.\n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	// Inicialmente, el LED parpadeará cuando no esté orientado hacia el grado 0
	objective_dir = 0;

	print(&huart1, (char*)"Press the botton to start.\n");

	while (!button){
		tx_thread_sleep(10); // 0.1s
	}

	objective_dir = degrees;

	print(&huart1, (char*)"Initial orientation: ", objective_dir);

	tx_thread_sleep(200); // 200cs -> 2000ms

	while (1) {

		//goBack(coche, &button, &degrees, &objective_dir);
		makingUnmakingSquares(coche, &centimeters, &degrees, &objective_dir);

		tx_thread_sleep(100); // 1s
	}
}

// Encoders thread
VOID encodersThread_entry(ULONG initial_input) {

	int cont = 0;
	float leftSpeed, rightSpeed;

	while (1) {

		if (coche->isMoving()) {
			// Get wheels speeds
			leftSpeed = coche->getLeftWheelSpeed();
			if (leftSpeed != 0)
				centimeters += 10; // 5 cog teeth -> 10cm
			rightSpeed = coche->getRightWheelSpeed();
			if (rightSpeed != 0)
				centimeters += 10; // 5 cog teeth -> 10cm

			// Car is moving in a straight line
			if (objective_dir != -1) {
				// Car is oriented correctly
				if (abs(deviation_dir) < deviation_threshold){
					coche->updateSpeed(leftSpeed, rightSpeed);
				}
			}
			else {
				centimeters = 0;
			}
		}

		cont = (cont + 1) % 100;
		if (!cont)
			HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
		tx_thread_sleep(1); // 10 ms
	}
}

// Sensors thread
VOID sensorsThread_entry(ULONG initial_input) {

	initSensors();

	motionEC_MC_init();

	int blink_counter = 0;

	while (1) {

		if (motionEC_MC_calibrate(0)) {
			calibrated = true;

			button = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == 1);

			if (objective_dir != -1) {
				// Calculate deviation from objective direction
				float dist_left = (360 - degrees) + objective_dir;
				float dist_right = objective_dir - degrees;
				deviation_dir = (abs(dist_left) < abs(dist_right)) ? -dist_left : -dist_right;

				// (factorX < 0) -> left
				// (factorX > 0) -> right
				if (abs(deviation_dir) > deviation_threshold){

					factorX = deviation_dir / 7.5;

					coche->updateSpeed(factorX, 3);

					blink_counter = (blink_counter + 1) % 10;
					if (!blink_counter) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
				}
				else {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				}

			}
			else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			}

		}

		HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
		tx_thread_sleep(1); // Algorithm frequency -> 100 Hz
	}
}
/* USER CODE END 1 */
