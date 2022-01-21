/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EngineR1_Pin GPIO_PIN_13
#define EngineR1_GPIO_Port GPIOC
#define EngineR2_Pin GPIO_PIN_14
#define EngineR2_GPIO_Port GPIOC
#define EngineR3_Pin GPIO_PIN_15
#define EngineR3_GPIO_Port GPIOC
#define EngineR4_Pin GPIO_PIN_0
#define EngineR4_GPIO_Port GPIOF
#define NCS_MEMS_SPI_Pin GPIO_PIN_0
#define NCS_MEMS_SPI_GPIO_Port GPIOC
#define MEMS_INT1_Pin GPIO_PIN_1
#define MEMS_INT1_GPIO_Port GPIOC
#define MEMS_INT2_Pin GPIO_PIN_2
#define MEMS_INT2_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define ECHO_BACK_Pin GPIO_PIN_1
#define ECHO_BACK_GPIO_Port GPIOA
#define ECHO_LEFT_Pin GPIO_PIN_5
#define ECHO_LEFT_GPIO_Port GPIOA
#define EXT_RESET_Pin GPIO_PIN_5
#define EXT_RESET_GPIO_Port GPIOC
#define ECHO_FRONT_Pin GPIO_PIN_10
#define ECHO_FRONT_GPIO_Port GPIOB
#define ECHO_RIGHT_Pin GPIO_PIN_11
#define ECHO_RIGHT_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_6
#define LD3_GPIO_Port GPIOC
#define LD6_Pin GPIO_PIN_7
#define LD6_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOC
#define TRIG_Pin GPIO_PIN_8
#define TRIG_GPIO_Port GPIOA
#define USBF4_DM_Pin GPIO_PIN_11
#define USBF4_DM_GPIO_Port GPIOA
#define USBF4_DP_Pin GPIO_PIN_12
#define USBF4_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BT_State_Pin GPIO_PIN_15
#define BT_State_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOC
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOC
#define EngineL1_Pin GPIO_PIN_3
#define EngineL1_GPIO_Port GPIOB
#define EngineL2_Pin GPIO_PIN_4
#define EngineL2_GPIO_Port GPIOB
#define EngineL3_Pin GPIO_PIN_5
#define EngineL3_GPIO_Port GPIOB
#define EngineL4_Pin GPIO_PIN_6
#define EngineL4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define SYS_FREQ  1000000 // Hz
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
