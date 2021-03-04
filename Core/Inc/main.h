/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char      bool_t;
typedef float              fp32;
typedef double             fp64;
typedef signed char        int8_t;
typedef signed short int   int16_t;
typedef signed int         int32_t;
typedef signed long long   int64_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifndef PI
#define PI 3.141592653589793238462643383279f
#endif
#ifndef ARES_HEX
#define ARES_HEX 0x41524553
#endif
#ifndef SUCCESS
#define SUCCESS 1
#endif
#ifndef FAIL
#define FAIL 0
#endif
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.01745329251994329576923690768489f)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.295779513082320876798154814105f)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IST_DRDY_Pin GPIO_PIN_3
#define IST_DRDY_GPIO_Port GPIOE
#define IST_RST_Pin GPIO_PIN_2
#define IST_RST_GPIO_Port GPIOE
#define MPU_DRDY_Pin GPIO_PIN_8
#define MPU_DRDY_GPIO_Port GPIOB
#define MPU_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define PWR0_Pin GPIO_PIN_2
#define PWR0_GPIO_Port GPIOH
#define PWR1_Pin GPIO_PIN_3
#define PWR1_GPIO_Port GPIOH
#define PWR2_Pin GPIO_PIN_4
#define PWR2_GPIO_Port GPIOH
#define PWR3_Pin GPIO_PIN_5
#define PWR3_GPIO_Port GPIOH
#define MPU_NSS_Pin GPIO_PIN_6
#define MPU_NSS_GPIO_Port GPIOF
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
