/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   ThreadX applicative header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

// LEDs
#define BLUE_LED_PIN GPIO_PIN_13
#define BLUE_LED_PORT GPIOE
#define GREEN_LED_PIN GPIO_PIN_7
#define GREEN_LED_PORT GPIOH

// Wheels
#define LEFT_WHEEL_FORWARD_PIN GPIO_PIN_15
#define LEFT_WHEEL_FORWARD_PORT GPIOD
#define LEFT_WHEEL_BACKWARD_PIN GPIO_PIN_7
#define LEFT_WHEEL_BACKWARD_PORT GPIOE
#define LEFT_WHEEL_SPEED_CHANNEL TIM_CHANNEL_1

#define RIGHT_WHEEL_FORWARD_PIN GPIO_PIN_1
#define RIGHT_WHEEL_FORWARD_PORT GPIOC
#define RIGHT_WHEEL_BACKWARD_PIN GPIO_PIN_13
#define RIGHT_WHEEL_BACKWARD_PORT GPIOF
#define RIGHT_WHEEL_SPEED_CHANNEL TIM_CHANNEL_1

// Encoders
#define LEFT_ENCODER_PIN GPIO_PIN_14
#define LEFT_ENCODER_PORT GPIOE

#define RIGHT_ENCODER_PIN GPIO_PIN_8
#define RIGHT_ENCODER_PORT GPIOD

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
