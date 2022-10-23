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
#define DISPLAY_1306
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
#ifdef DISPLAY_1306
#include "ssd1306.h"
#endif
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
#define POWER_VOLTAGE 1004
#define TRUST_INTERVAL 900 /* 3 sigma*/
#define GM_CPS2URh 9.6
#define GM_SELF_FONE 1.5	/* Собственный фон трубки */
#define MEAS_INTERVAL 1000
#define MEAS_CO2_INTERVAL1 1000
#define MEAS_CO2_INTERVAL2 5000
#define NTP_INTERVAL 3600000 /* 1 час */
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
#define Eth_CS_Pin GPIO_PIN_12
#define Eth_CS_GPIO_Port GPIOB
#define Eth_rst_Pin GPIO_PIN_8
#define Eth_rst_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define POWER_PULSE_Pin GPIO_PIN_15
#define POWER_PULSE_GPIO_Port GPIOA
#define BME280_SCL_Pin GPIO_PIN_8
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_9
#define BME280_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
ADC_HandleTypeDef hadc1;
uint32_t fastCounter, CO2Counter, CO2Interval ;
uint16_t hvLevel;

//uint16_t Z12, Z21, Z23, Z32, Z34, Z43, Z41, Z14;
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
#define INIT_FINISH_TEXT "Init finish.\r\n"
#define RESTART_TOUT "\r\nTime out, restart.\r\n"
#define START_TEXT "\r\nMeteostation start.\r\n"
float temperature, pressure, humidity;

/* For W5500*/
#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define TCP_SOCKET		2
#define W5500_RST_Pin	GPIO_PIN_8
#define W5500_CS_Pin	GPIO_PIN_12
#define _DHCP_DEBUG_

//#define ZABBIX_DEBUG
#define DATA_BUF_SIZE   1024
#define ZABBIX_ENABLE
#define ZABBIXAGHOST	"Meteostation"  // Default hostname.
#define ZABBIXPORT		10051
#define ZABBIXMAXLEN	128
#define MAC_ADDRESS		0x00, 0x11, 0x22, 0x33, 0x44, 0xED
char ZabbixHostName[255];


#ifndef INC_SCD41_H_
#define INC_SCD41_H_

//#ifndef SCD4X_I2C_PORT
#define SCD4X_I2C_PORT	hi2c1
//#endif

#ifndef SCD4X_I2C_ADDR
#define SCD4X_I2C_ADDR   (0x62 << 1)   ///< SCD4X I2C address
#endif

#define SCD4X_SERIAL_NUMBER_WORD0   0xbe02   ///< SCD4X serial number 0
#define SCD4X_SERIAL_NUMBER_WORD1   0x7f07   ///< SCD4X serial number 1
#define SCD4X_SERIAL_NUMBER_WORD2   0x3bfb   ///< SCD4X serial number 2

#define SCD4X_CRC8_INIT             0xFF
#define SCD4X_CRC8_POLYNOMIAL       0x31

/* SCD4X Basic Commands */
#define SCD4X_START_PERIODIC_MEASURE    0x21b1   ///< start periodic measurement, signal update interval is 5 seconds.
#define SCD4X_READ_MEASUREMENT          0xec05   ///< read measurement
#define SCD4X_STOP_PERIODIC_MEASURE     0x3f86   ///< stop periodic measurement command

/* SCD4X On-chip output signal compensation */
#define SCD4X_SET_TEMPERATURE_OFFSET    0x241d   ///< set temperature offset
#define SCD4X_GET_TEMPERATURE_OFFSET    0x2318   ///< get temperature offset
#define SCD4X_SET_SENSOR_ALTITUDE       0x2427   ///< set sensor altitude
#define SCD4X_GET_SENSOR_ALTITUDE       0x2322   ///< get sensor altitude
#define SCD4X_SET_AMBIENT_PRESSURE      0xe000   ///< set ambient pressure

/* SCD4X Field calibration */
#define SCD4X_PERFORM_FORCED_RECALIB    0x362f   ///< perform forced recalibration
#define SCD4X_SET_AUTOMATIC_CALIB       (0x2416)   ///< set automatic self calibration enabled
#define SCD4X_GET_AUTOMATIC_CALIB       (0x2313)   ///< get automatic self calibration enabled

/* SCD4X Low power */
#define SCD4X_START_LOW_POWER_MEASURE   (0x21ac)   ///< start low power periodic measurement, signal update interval is approximately 30 seconds.
#define SCD4X_GET_DATA_READY_STATUS     (0xe4b8)   ///< get data ready status

/* SCD4X Advanced features */
#define SCD4X_PERSIST_SETTINGS          (0x3615)   ///< persist settings
#define SCD4X_GET_SERIAL_NUMBER         (0x3682)   ///< get serial number
#define SCD4X_PERFORM_SELF_TEST         (0x3639)   ///< perform self test
#define SCD4X_PERFORM_FACTORY_RESET     (0x3632)   ///< perform factory reset
#define SCD4X_REINIT                    (0x3646)   ///< reinit

/* SCD4X Low power single shot */
#define SCD4X_MEASURE_SINGLE_SHOT            (0x219d)   ///< measure single shot
#define SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY   (0x2196)   ///< measure single shot rht only
#define SCD4X_POWER_DOWN                     (0x36e0)   ///< Put the sensor from idle to sleep to reduce current consumption.
#define SCD4X_WAKE_UP                        (0x36f6)   ///< Wake up the sensor from sleep mode into idle mode.

/* Convenience Macro */
#define SCD4X_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data


  /**
   * @struct sSensorMeasurement_t
   * @brief Sensor readings, including CO2 concentration, temperature and humidity
   */
  typedef struct
  {
    uint16_t   CO2ppm;
    float   temp;
    float   humidity;
  } sSensorMeasurement_t;

/**************************** Init and reset ********************************/

  /**
   * @fn DFRobot_SCD4X
   * @brief Constructor
   * @param pWire - Wire object is defined in Wire.h, so just use &Wire and the methods in Wire can be pointed to and used
   * @param i2cAddr - SCD4X I2C address.
   * @return None
   */
  //DFRobot_SCD4X(TwoWire *pWire=&Wire, uint8_t i2cAddr=SCD4X_I2C_ADDR);

  /**
   * @fn begin
   * @brief Init function
   * @return bool type, true if successful, false if error
   */
  bool begin(void);

  /**
   * @fn setSleepMode
   * @brief Set the sensor as sleep or wake-up mode (SCD41 only)
   * @param mode - sleep and wake-up mode:
   * @n       SCD4X_POWER_DOWN : Put the sensor from idle to sleep to reduce current consumption.
   * @n       SCD4X_WAKE_UP : Wake up the sensor from sleep mode into idle mode.
   * @return None
   * @note Note that the SCD4x does not acknowledge the wake_up command. Command execution time : 20 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  void setSleepMode(uint16_t mode);

  /**
   * @fn performSelfTest
   * @brief perform self test
   * @details The perform_self_test feature can be used as an end-of-line test to check sensor
   * @n  functionality and the customer power supply to the sensor.
   * @return module status:
   * @n        0 : no malfunction detected
   * @n        other : malfunction detected
   * @note Command execution time : 10000 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  uint16_t performSelfTest(void);

  /**
   * @fn moduleReinit
   * @brief module reinit
   * @details  The reinit command reinitializes the sensor by reloading user settings from EEPROM.
   * @return None
   * @note Before sending the reinit command, the stop measurement command must be issued.
   * @n  If the reinit command does not trigger the desired re-initialization,
   * @n  a power-cycle should be applied to the SCD4x.
   * @n  Command execution time : 20 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  void moduleReinit(void);

  /**
   * @fn performFactoryReset
   * @brief perform factory reset
   * @details The perform_factory_reset command resets all configuration settings stored
   * @n  in the EEPROM and erases the FRC and ASC algorithm history.
   * @return None
   * @note Command execution time : 1200 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  void performFactoryReset(void);

/********************************* Measurement Function *************************************/

  /**
   * @fn measureSingleShot
   * @brief measure single shot（SCD41 only）
   * @details  On-demand measurement of CO2 concentration, relative humidity and temperature.
   * @n  Get the measured data through readMeasurement(sSensorMeasurement_t data) interface
   * @param mode - Single-measurement mode:
   * @n       SCD4X_MEASURE_SINGLE_SHOT : On-demand measurement of CO2 concentration, relative humidity and temperature.
   * @n                                   Max command duration 5000 [ms].
   * @n       SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY : On-demand measurement of relative humidity and temperature only.
   * @n                                            Max command duration 50 [ms].
   * @return None
   * @note In SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY mode, CO2 output is returned as 0 ppm.
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  void measureSingleShot(uint16_t mode);

  /**
   * @fn enablePeriodMeasure
   * @brief set periodic measurement mode
   * @param mode - periodic measurement mode:
   * @n       SCD4X_START_PERIODIC_MEASURE : start periodic measurement, signal update interval is 5 seconds.
   * @n       SCD4X_STOP_PERIODIC_MEASURE : stop periodic measurement command
   * @n       SCD4X_START_LOW_POWER_MEASURE :  start low power periodic measurement, signal update interval is approximately 30 seconds.
   * @return None
   * @note The measurement mode must be disabled when configuring the sensor; after giving the stop_periodic_measurement command, the sensor needs to wait 500ms before responding to other commands.
   */
  void enablePeriodMeasure(uint16_t mode);

  /**
   * @fn readMeasurement
   * @brief Read the measured data
   * @param data - sSensorMeasurement_t, sensor readings, including CO2 concentration (ppm), temperature (℃) and humidity (RH)
   * @return None
   * @note CO2 measurement range: 0~40000 ppm; temperature measurement range: -10~60 ℃; humidity measurement range: 0~100 %RH.
   */
  void readMeasurement(sSensorMeasurement_t * data);

  /**
   * @fn getDataReadyStatus
   * @brief get data ready status
   * @return data ready status:
   * @n        true : data ready
   * @n        false : data not ready
   */
  bool getDataReadyStatus(void);

/*************************** compensation and calibration ********************************/

  /**
   * @fn setTempComp
   * @brief set temperature offset
   * @details T(offset_actual) = T(SCD4X) - T(reference) + T(offset_previous)
   * @n T(offset_actual): the calculated actual temperature offset that is required
   * @n T(SCD4X): the temperature measured by the sensor (wait for a period of time to get steady readings)
   * @n T(reference): the standard reference value of the current ambient temperature
   * @n T(offset_previous): the previously set temperature offset
   * @n For example: 32(T(SCD4X)) - 30(T(reference)) + 2(T(offset_previous)) = 4(T(offset_actual))
   * @param tempComp - temperature offset value, unit ℃
   * @return None
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  void setTempComp(float tempComp);

  /**
   * @fn getTempComp
   * @brief get temperature offset
   * @return The current temp compensated value, unit ℃
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  float getTempComp(void);

  /**
   * @fn setSensorAltitude
   * @brief set sensor altitude
   * @param altitude - the current ambient altitude, unit m
   * @return None
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  void setSensorAltitude(uint16_t altitude);

  /**
   * @fn getSensorAltitude
   * @brief get sensor altitude
   * @return the current ambient altitude, unit m
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  uint16_t getSensorAltitude(void);

  /**
   * @fn setAmbientPressure
   * @brief set ambient pressure
   * @param ambientPressure - the current ambient pressure, unit Pa
   * @return None
   */
  void setAmbientPressure(uint32_t ambientPressure);

  /**
   * @fn performForcedRecalibration
   * @brief perform forced recalibration
   * @param CO2ppm - Target CO2 concentration, unit ppm
   * @return calibration amplitude, return (int16_t)0x7fff if the calibration failed
   * @note Command execution time : 400 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  int16_t performForcedRecalibration(uint16_t CO2ppm);

  /**
   * @fn setAutoCalibMode
   * @brief set automatic self calibration enabled
   * @param mode - automatic self-calibration mode:
   * @n       true : enable automatic self-calibration
   * @n       false : disable automatic self-calibration
   * @return None
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  void setAutoCalibMode(bool mode);

  /**
   * @fn getAutoCalibMode
   * @brief get automatic self calibration enabled
   * @return automatic self-calibration mode:
   * @n        true : enable automatic self-calibration
   * @n        false : disable automatic self-calibration
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  bool getAutoCalibMode(void);

  /**
   * @fn persistSettings
   * @brief Save the settings into EEPROM, default to be in RAM
   * @details Configuration settings such as the temperature offset, sensor altitude and the ASC enabled/disabled
   * @n  parameter are by default stored in the volatile memory (RAM) only and will be lost after a power-cycle.
   * @return None
   * @note To avoid unnecessary wear of the EEPROM, the persist_settings command should only be sent
   * @n  when persistence is required and if actual changes to the configuration have been made.
   * @n  The EEPROM is guaranteed to endure at least 2000 write cycles before failure.
   * @note Command execution time : 800 ms
   * @n When executing the command, the sensor can't be in period measurement mode
   */
  void persistSettings(void);


/*************************** get serial number *****************************/

  /**
   * @fn getSerialNumber
   * @brief Reading out the serial number can be used to identify the chip and to verify the presence of the sensor.
   * @n  Together, the 3 words constitute a unique serial number with a length of 48 bits (big endian format).
   * @param wordBuf - for buffering the obtained chip serial number
   * @return bool type, true if successful, false if error
   * @note When executing the command, the sensor can't be in period measurement mode
   */
  bool getSerialNumber(uint16_t * wordBuf);

/*********************** CRC Check & Sending Data Pack *************************/

  /**
   * @fn calcCRC
   * @brief Calculate the current crc check code to contrast it with the MISR read from the sensor
   * @param data - The measured data just obtained from the sensor
   * @return The current calculated crc check code
   */
  uint8_t calcCRC(uint16_t data);

  /**
   * @fn pack
   * @brief Pack the data to be sent
   * @param data - The data to be sent
   * @return The packed data to be sent
   */
  uint8_t * pack(uint16_t data);

/****************************** Read/Write Command Function ********************************/

  /**
   * @fn writeData
   * @brief Write register value through I2C bus
   * @param cmd - module commands 16bits
   * @param pBuf - Storage and buffer for data to be written
   * @param size - Length of data to be written
   * @return None
   */
  void SCDwriteData(uint16_t cmd, uint8_t *pBuf, size_t size);

  /**
   * @fn readData
   * @brief Read register value through I2C bus
   * @param cmd - module commands 16bits
   * @param pBuf - Storage and buffer for data to be read
   * @param size - Length of data to be read
   * @return Return the read length, returning 0 means reading failed
   */
  size_t SCDreadData(uint16_t cmd, uint8_t *pBuf, size_t size);



#endif /* INC_SCD41_H_ */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
