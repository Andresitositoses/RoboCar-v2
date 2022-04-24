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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Stacks sizes
#define MAINTHREAD_STACK_SIZE 1024
#define ENCODERS_STACK_SIZE 1024
// Measure parameters
#define MEASURES_FOR_SPEED          3
#define MAX_ATTEMPTS_TO_READ        450000
#define CYCLES_PER_SECOND			160000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Handlers
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;

/////////////////////////
/// THREAD VARIABLES ///
/////////////////////////

// Main thread
uint8_t mainThread_stack[MAINTHREAD_STACK_SIZE];
TX_THREAD mainThread_ptr;
int moving = 0;

// Encoder threads
uint8_t encondersThread_stack[ENCODERS_STACK_SIZE];
TX_THREAD encodersThread_ptr;
float left_wheel_speed = 0, right_wheel_speed = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID mainThread_entry(ULONG initial_input);
VOID encodersThread_entry(ULONG initial_input);
float getSpeed(GPIO_TypeDef *port, uint32_t pin);

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
	/* USER CODE END App_ThreadX_MEM_POOL */

	/* USER CODE BEGIN App_ThreadX_Init */
	tx_thread_create(&mainThread_ptr, (char* )"mainThread", mainThread_entry, 0,
			mainThread_stack, MAINTHREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&encodersThread_ptr, (char* )"encodersThread",
			encodersThread_entry, 0, encondersThread_stack, ENCODERS_STACK_SIZE,
			15, 15, 1, TX_AUTO_START);
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
VOID mainThread_entry(ULONG initial_input) {
	while (1) {
		print(&huart1, "hilo 1: ", 1.0);
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		tx_thread_sleep(100); // 1s
	}
}

// Encoders thread
VOID encodersThread_entry(ULONG initial_input) {
	while (1) {
		// Speed calculation
		if (moving) {
			left_wheel_speed = getSpeed(LEFT_ENCODER_PORT, LEFT_ENCODER_PIN);
			right_wheel_speed = getSpeed(RIGHT_ENCODER_PORT, RIGHT_ENCODER_PIN);
		} else {
			left_wheel_speed = 0;
			right_wheel_speed = 0;
		}

		// Thread delay
		HAL_GPIO_TogglePin(BLUE_LED_PORT, BLUE_LED_PIN);
		tx_thread_sleep(1); // 10 ms
	}
}

float getSpeed(GPIO_TypeDef *port, uint32_t pin) {

	// Initial measure
	ULONG startTime = DWT->CYCCNT;
	int cont = 0;

	// Gets the time it takes to change between 0's and 1's MEASURES_FOR_SPEED times
	for (int i = 0; i < MEASURES_FOR_SPEED; i++) {
		// Exit the loop in case the wheel is stopped
		while (HAL_GPIO_ReadPin(port, pin) != 1 && cont++ < MAX_ATTEMPTS_TO_READ);
		if (cont >= MAX_ATTEMPTS_TO_READ) {
			return 0;
		} else {
			cont = 0;
		}
		while (HAL_GPIO_ReadPin(port, pin) != 0 && cont++ < MAX_ATTEMPTS_TO_READ);
		if (cont >= MAX_ATTEMPTS_TO_READ) {
			return 0;
		} else {
			cont = 0;
		}
	}

	// End measure
	ULONG stopTime = DWT->CYCCNT;

	// Reset counter after reboot
	if (startTime == 0){
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}

	return CYCLES_PER_SECOND * MEASURES_FOR_SPEED
			/ (float) abs((int) stopTime - (int) startTime);
}
/* USER CODE END 1 */
