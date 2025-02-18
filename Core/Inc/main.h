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
void ResetCDCrxBuffer(void);
uint8_t CDCsend(uint8_t *str, uint32_t len, uint8_t retries);
void ext_brd_transmit_string(uint8_t printCmd, uint8_t* strbuff, uint8_t len);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MODE1_1_Pin LL_GPIO_PIN_13
#define MODE1_1_GPIO_Port GPIOC
#define MODE2_1_Pin LL_GPIO_PIN_14
#define MODE2_1_GPIO_Port GPIOC
#define USB_REENUM_Pin LL_GPIO_PIN_15
#define USB_REENUM_GPIO_Port GPIOC
#define STCK_1_Pin LL_GPIO_PIN_0
#define STCK_1_GPIO_Port GPIOA
#define SERVO_PWM_Pin LL_GPIO_PIN_1
#define SERVO_PWM_GPIO_Port GPIOA
#define STCK_3_Pin LL_GPIO_PIN_2
#define STCK_3_GPIO_Port GPIOA
#define STCK_4_Pin LL_GPIO_PIN_3
#define STCK_4_GPIO_Port GPIOA
#define DIR_1_Pin LL_GPIO_PIN_4
#define DIR_1_GPIO_Port GPIOA
#define DIR_3_Pin LL_GPIO_PIN_6
#define DIR_3_GPIO_Port GPIOA
#define DIR_4_Pin LL_GPIO_PIN_7
#define DIR_4_GPIO_Port GPIOA
#define EN_FAULT_1_Pin LL_GPIO_PIN_0
#define EN_FAULT_1_GPIO_Port GPIOB
#define MODE1_3_Pin LL_GPIO_PIN_1
#define MODE1_3_GPIO_Port GPIOB
#define EN_FAULT_3_Pin LL_GPIO_PIN_2
#define EN_FAULT_3_GPIO_Port GPIOB
#define SW1_4_Pin LL_GPIO_PIN_10
#define SW1_4_GPIO_Port GPIOB
#define SW1_4_EXTI_IRQn EXTI15_10_IRQn
#define SW2_4_Pin LL_GPIO_PIN_11
#define SW2_4_GPIO_Port GPIOB
#define SW2_4_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_Pin LL_GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define MODE1_4_Pin LL_GPIO_PIN_8
#define MODE1_4_GPIO_Port GPIOA
#define MODE2_4_Pin LL_GPIO_PIN_9
#define MODE2_4_GPIO_Port GPIOA
#define EN_FAULT_4_Pin LL_GPIO_PIN_10
#define EN_FAULT_4_GPIO_Port GPIOA
#define MODE2_3_Pin LL_GPIO_PIN_15
#define MODE2_3_GPIO_Port GPIOA
#define STBY_RESET_ALL_Pin LL_GPIO_PIN_3
#define STBY_RESET_ALL_GPIO_Port GPIOB
#define SW1_1_Pin LL_GPIO_PIN_4
#define SW1_1_GPIO_Port GPIOB
#define SW1_1_EXTI_IRQn EXTI4_IRQn
#define SW2_1_Pin LL_GPIO_PIN_5
#define SW2_1_GPIO_Port GPIOB
#define SW2_1_EXTI_IRQn EXTI9_5_IRQn
#define EXTENSION_BRD_BSY_Pin LL_GPIO_PIN_6
#define EXTENSION_BRD_BSY_GPIO_Port GPIOB
#define SW1_3_Pin LL_GPIO_PIN_8
#define SW1_3_GPIO_Port GPIOB
#define SW1_3_EXTI_IRQn EXTI9_5_IRQn
#define SW2_3_Pin LL_GPIO_PIN_9
#define SW2_3_GPIO_Port GPIOB
#define SW2_3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
