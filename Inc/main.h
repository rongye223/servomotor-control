/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "motorcontrol.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GoHome_Pin GPIO_PIN_1
#define GoHome_GPIO_Port GPIOE
#define GoHome_EXTI_IRQn EXTI1_IRQn
#define Start_Stop_Pin GPIO_PIN_0
#define Start_Stop_GPIO_Port GPIOE
#define Start_Stop_EXTI_IRQn EXTI0_IRQn
#define M1_ENCODER_B_Pin GPIO_PIN_3
#define M1_ENCODER_B_GPIO_Port GPIOB
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define M1_ENCODER_A_Pin GPIO_PIN_15
#define M1_ENCODER_A_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_6
#define UART_TX_GPIO_Port GPIOB
#define M1_PWM_WH_Pin GPIO_PIN_7
#define M1_PWM_WH_GPIO_Port GPIOI
#define M1_PWM_VH_Pin GPIO_PIN_6
#define M1_PWM_VH_GPIO_Port GPIOI
#define M1_PWM_UH_Pin GPIO_PIN_5
#define M1_PWM_UH_GPIO_Port GPIOI
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOH
#define M1_PWM_UL_Pin GPIO_PIN_13
#define M1_PWM_UL_GPIO_Port GPIOH
#define M1_PWM_VL_Pin GPIO_PIN_14
#define M1_PWM_VL_GPIO_Port GPIOH
#define M1_RULER_B_Pin GPIO_PIN_7
#define M1_RULER_B_GPIO_Port GPIOC
#define M1_RULER_A_Pin GPIO_PIN_6
#define M1_RULER_A_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_0
#define M1_BUS_VOLTAGE_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin GPIO_PIN_3
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define LIM_P_Pin GPIO_PIN_1
#define LIM_P_GPIO_Port GPIOG
#define DBG_DAC_CH1_Pin GPIO_PIN_4
#define DBG_DAC_CH1_GPIO_Port GPIOA
#define LIM_N_Pin GPIO_PIN_0
#define LIM_N_GPIO_Port GPIOG
#define M1_CURR_AMPL_U_Pin GPIO_PIN_6
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define DBG_DAC_CH2_Pin GPIO_PIN_5
#define DBG_DAC_CH2_GPIO_Port GPIOA
#define M1_CURR_AMPL_W_Pin GPIO_PIN_1
#define M1_CURR_AMPL_W_GPIO_Port GPIOB
#define M1_CURR_AMPL_V_Pin GPIO_PIN_0
#define M1_CURR_AMPL_V_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
