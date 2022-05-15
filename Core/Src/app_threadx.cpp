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
#define SENSORS_STACK_SIZE 4096
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

	coche->loadCalibration();
	coche->showCalibrations();

	int cont = 0;
	while (1) {

		cont++;
		//print(&huart1, (char*) "Escribo, luego existo. ", cont);
		tx_thread_sleep(100); // 1s
	}
}

// Encoders thread
VOID encodersThread_entry(ULONG initial_input) {

	while (1) {
		// Speed control
		if (coche->isMoving()) {
			// Update car speed
			coche->updateSpeed();
		}

		HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
		tx_thread_sleep(10); // 100 ms
	}
}

// Sensors thread
VOID sensorsThread_entry(ULONG initial_input) {

	initSensors();
	motionAC_init();
	print(&huart1, (char*) "Sensors initialized\n");

	while (1) {

		motionAC_calibrate(1);

		HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
		tx_thread_sleep(20); // 200 ms
	}
}
/* USER CODE END 1 */
