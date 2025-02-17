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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define TMP117_ENABLE
#define BME280_ENABLE
//#define DISPLAY_1306
#define DISPLAY_ST7735S
#define COLUMN0 0
#define COLUMN1 54
#define COLUMN2 97
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dns.h"
#include "sntp.h"
#include "socket.h"
#include "stdarg.h"
#ifdef TMP117_ENABLE
#include "tmp117.h"
#endif
#ifdef BME280_ENABLE
#include "BME280.h"
#endif
#ifdef DISPLAY_ST7735S
#include "st7735.h"
#include "GFX_FUNCTIONS.h"
#include <ST7735_fonts.h>
#endif
#ifdef DISPLAY_1306
#include "ssd1306.h"
#endif

#include "SCD41.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define TRUE 1
#define FALSE 0
#define LED_PULSE LED_GPIO_Port->BSRR = (uint32_t)LED_Pin; LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16u;
#define TP_PULSE GPIOA->BSRR = (uint32_t)GPIO_PIN_15; GPIOA->BSRR = (uint32_t)GPIO_PIN_15 << 16u;
#define POWER_PULSE POWER_PULSE_GPIO_Port->BSRR = (uint32_t)POWER_PULSE_Pin; POWER_PULSE_GPIO_Port->BSRR = (uint32_t)POWER_PULSE_Pin << 16u;
#define POWER_VOLTAGE 960
#define POWER_CONVERT 0.416f
#define TRUST_INTERVAL 900 /* 3 sigma - 10%*/
#define GM_CPS2URh 9.6
#define GM_SELF_FONE 0.5	/* Собственный фон трубки */
#define MEAS_INTERVAL 1000
#define ANEM_INTERVAL 10000		/* Интервал опроса анемометра */
#define MEAS_CO2_INTERVAL1 1000
#define MEAS_CO2_INTERVAL2 5000
#define ZABB_MEAS_INTERVAL 10000
#define NTP_INTERVAL 60000 /* 0.1 час */
#define CO2_NOMINAL 800.0f
#define CO2_MAXIMUM 1000.0f
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
#define FB_Pin GPIO_PIN_5
#define FB_GPIO_Port GPIOA
#define PULSE_GM_Pin GPIO_PIN_6
#define PULSE_GM_GPIO_Port GPIOA
#define PULSE_GM_EXTI_IRQn EXTI9_5_IRQn
#define ST7735S_RESET_Pin GPIO_PIN_11
#define ST7735S_RESET_GPIO_Port GPIOB
#define Eth_CS_Pin GPIO_PIN_12
#define Eth_CS_GPIO_Port GPIOB
#define Eth_rst_Pin GPIO_PIN_8
#define Eth_rst_GPIO_Port GPIOA
#define ST7735S_CS_Pin GPIO_PIN_9
#define ST7735S_CS_GPIO_Port GPIOA
#define ST7735S_DC_Pin GPIO_PIN_10
#define ST7735S_DC_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define POWER_PULSE_Pin GPIO_PIN_15
#define POWER_PULSE_GPIO_Port GPIOA
#define BME280_SCL_Pin GPIO_PIN_8
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_9
#define BME280_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//uint16_t Z12, Z21, Z23, Z32, Z34, Z43, Z41, Z14;
/*
union {
	float f;
	uint32_t u;
} DX1;
union {
	float f;
	uint32_t u;
} DX2;
union {
	float f;
	uint32_t u;
} DY1;
union {
	float f;
	uint32_t u;
} DY2;
*/
#define INIT_FINISH_TEXT "Init finish.\r\n"
#define RESTART_TOUT "\r\nTime out, restart.\r\n"
#define START_TEXT "\r\nMeteostation start.\r\n"

/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define ANM_SOCKET		3
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG
#define DATA_BUF_SIZE   1024
#define ZABBIX_ENABLE
#define ZABBIXAGHOST	"Meteo"  // Default hostname.
#define ZABBIXPORT		10051
#define ANEMOMETERPORT	1600
#define ZABBIXMAXLEN	128
#define MAC_ADDRESS		0x00, 0x11, 0x22, 0x33, 0x44, 0xf7

//#define CO2_DEBUG


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
