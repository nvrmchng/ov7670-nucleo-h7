/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"

#include "stm32h7xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "cmsis_os.h"
#include "OV7670.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Delay_us_Rough(uint16_t delay_us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOF
#define VSYNC_Pin GPIO_PIN_0
#define VSYNC_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_7
#define D2_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_8
#define D3_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_9
#define D7_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_10
#define D1_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_11
#define D6_GPIO_Port GPIOE
#define D0_Pin GPIO_PIN_12
#define D0_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_13
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_14
#define D5_GPIO_Port GPIOE
#define HREF_Pin GPIO_PIN_10
#define HREF_GPIO_Port GPIOB
#define XCLK_Pin GPIO_PIN_11
#define XCLK_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define PCLK_Pin GPIO_PIN_15
#define PCLK_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
enum USER_BUTTON_STATE {
    USR_BTN_UNPRESSED = 0, DEBOUNCE_UNPRESSED = 1, USR_BTN_PRESSED = 2, DEBOUNCE_PRESSED = 3
};

extern enum USER_BUTTON_STATE User_Button_State;

extern osSemaphoreId_t myBinarySem01Handle;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
