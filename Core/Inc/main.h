/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stepper.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define USE_INTERRUPT_US_DELAY
//#define USE_BLOCKING_US_DELAY

#define RXBUFFSIZE	100U

//command byte defines for extension shield
#define	PrintFilenameTXT_cmd		0x01
#define	PrintCNCcmd_cmd				0x02
#define PrintInfo_cmd				0x03
#define	PrintCNCcmdAndLineNum_cmd	0x04

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void usDelay(uint32_t val);
void msDelay(uint32_t val);

int32_t mypow10(int32_t exponent);
void eval_and_execute_plot_cmd(uint8_t *cmdstr, uint32_t len, CP *currentpos);
uint8_t eval_arg_in_cmd(uint8_t* cmdstr, uint8_t startindx, char delimiter, uint8_t maxlen, int32_t* numarg);
void ResetCDCrxBuffer(void);
uint8_t CDCsend(uint8_t *str, uint32_t len, uint8_t retries);
void ext_brd_transmit_string(uint8_t printCmd, uint8_t* strbuff, uint8_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_EXT_CTR_BSY_Pin LL_GPIO_PIN_14
#define SPI2_EXT_CTR_BSY_GPIO_Port GPIOC
#define DIR_X_Pin LL_GPIO_PIN_1
#define DIR_X_GPIO_Port GPIOA
#define STEP_X_Pin LL_GPIO_PIN_2
#define STEP_X_GPIO_Port GPIOA
#define _SLEEP_Pin LL_GPIO_PIN_3
#define _SLEEP_GPIO_Port GPIOA
#define _RST_Pin LL_GPIO_PIN_4
#define _RST_GPIO_Port GPIOA
#define MS3_Pin LL_GPIO_PIN_5
#define MS3_GPIO_Port GPIOA
#define MS2_Pin LL_GPIO_PIN_6
#define MS2_GPIO_Port GPIOA
#define MS1_Pin LL_GPIO_PIN_7
#define MS1_GPIO_Port GPIOA
#define _EN_Pin LL_GPIO_PIN_0
#define _EN_GPIO_Port GPIOB
#define DIR_Y_Pin LL_GPIO_PIN_1
#define DIR_Y_GPIO_Port GPIOB
#define STEP_Y_Pin LL_GPIO_PIN_2
#define STEP_Y_GPIO_Port GPIOB
#define DIR_Z_Pin LL_GPIO_PIN_10
#define DIR_Z_GPIO_Port GPIOB
#define STEP_Z_Pin LL_GPIO_PIN_11
#define STEP_Z_GPIO_Port GPIOB
#define SPI2_CS_Pin LL_GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED1_Pin LL_GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin LL_GPIO_PIN_9
#define LED2_GPIO_Port GPIOA
#define USB_REENUM_Pin LL_GPIO_PIN_10
#define USB_REENUM_GPIO_Port GPIOA
#define LED3_Pin LL_GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define LED4_Pin LL_GPIO_PIN_3
#define LED4_GPIO_Port GPIOB
#define SW_ZH_Pin LL_GPIO_PIN_4
#define SW_ZH_GPIO_Port GPIOB
#define SW_ZH_EXTI_IRQn EXTI4_IRQn
#define SW_ZE_Pin LL_GPIO_PIN_5
#define SW_ZE_GPIO_Port GPIOB
#define SW_ZE_EXTI_IRQn EXTI9_5_IRQn
#define SW_YH_Pin LL_GPIO_PIN_6
#define SW_YH_GPIO_Port GPIOB
#define SW_YH_EXTI_IRQn EXTI9_5_IRQn
#define SW_YE_Pin LL_GPIO_PIN_7
#define SW_YE_GPIO_Port GPIOB
#define SW_YE_EXTI_IRQn EXTI9_5_IRQn
#define SW_XH_Pin LL_GPIO_PIN_8
#define SW_XH_GPIO_Port GPIOB
#define SW_XH_EXTI_IRQn EXTI9_5_IRQn
#define SW_XE_Pin LL_GPIO_PIN_9
#define SW_XE_GPIO_Port GPIOB
#define SW_XE_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
