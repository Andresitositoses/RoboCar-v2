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

/* USER CODE END PD */
#define MAINTHREAD_STACK_SIZE 1024
#define ENCODERS_STACK_SIZE 1024

// LEDs
#define BLUE_LED_PIN GPIO_PIN_13
#define BLUE_LED_PORT GPIOE
#define GREEN_LED_PIN GPIO_PIN_7
#define GREEN_LED_PORT GPIOH
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Handlers
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;

// Main thread
uint8_t mainThread_stack[MAINTHREAD_STACK_SIZE];
TX_THREAD mainThread_ptr;
// Encoders thread
uint8_t encondersThread_stack[ENCODERS_STACK_SIZE];
TX_THREAD encodersThread_ptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

VOID mainThread_entry(ULONG initial_input);
VOID encodersThread_entry(ULONG initial_input);

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
	tx_thread_create(&mainThread_ptr, "mainThread", mainThread_entry, 0,
			mainThread_stack, MAINTHREAD_STACK_SIZE, 15, 15, 1, TX_AUTO_START);
	tx_thread_create(&encodersThread_ptr, "encodersThread",
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
		tx_thread_sleep(100); // 10 cs -> 0.1 s -> 100 ms
	}
}

VOID encodersThread_entry(ULONG initial_input) {
	while (1) {
		print(&huart1, "hilo 2: ", 2.0);
		HAL_GPIO_TogglePin(BLUE_LED_PORT, BLUE_LED_PIN);
		tx_thread_sleep(10); // 10 cs -> 0.1 s -> 100 ms
	}
}
