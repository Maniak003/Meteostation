/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef PUBLIC_VAR
#define PUBLIC_VAR

uint32_t fastCounter, CO2Counter, CO2Interval ;
uint16_t hvLevel;
bool gm_ready;
float temperature, pressure, humidity;
char ZabbixHostName[255];
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef	flash_ok;
char uart_buffer[10] = {0,};
char SndBuffer[200] = {0,};
char text1306[16];
uint16_t gm_counter = 0;
uint32_t gm_interval = 0, ntp_interval = 0, zabb_interval = 0;
float gm_cps = 0;
sSensorMeasurement_t messuremetData;
uint16_t CO2;

#ifdef ZABBIX_ENABLE
wiz_NetInfo net_info = {
	.mac  = { MAC_ADDRESS },
	.dhcp = NETINFO_DHCP
};
uint8_t ntp_server[4] = {192, 168, 1, 1};
uint8_t gDATABUF[DATA_BUF_SIZE];
datetime timeNTP;
#endif

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
char trans_str[64] = {0,};
bool gm_ready = FALSE;
bool first_start = TRUE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef ZABBIX_ENABLE

#ifdef ZABBIX_DEBUG
void UART_Printf(const char* fmt, ...) {
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&huart1, (uint8_t*) buff, strlen(buff), HAL_MAX_DELAY);
	va_end(args);
}
#endif


void W5500_Select(void) {
	HAL_GPIO_WritePin(Eth_CS_GPIO_Port, Eth_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
	HAL_GPIO_WritePin(Eth_CS_GPIO_Port, Eth_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
	HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
	HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
	uint8_t byte;
	W5500_ReadBuff(&byte, sizeof(byte));
	return byte;
}

void W5500_WriteByte(uint8_t byte) {
	W5500_WriteBuff(&byte, sizeof(byte));
}

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
#ifdef ZABBIX_DEBUG
	UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
#endif
    ip_assigned = true;
}

void Callback_IPConflict(void) {
#ifdef ZABBIX_DEBUG
    UART_Printf("Callback: IP conflict!\r\n");
#endif
}

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t dhcp_buffer[1024];
// 1K seems to be enough for this buffer as well
//uint8_t dns_buffer[1024];

/*
 * addr -- IP address zabbix server, dhcp option 224.
 * Settings for isc-dhcp-server:
 * 		option zabbix-server-ip code 224 = ip-address ;
 * 		option zabix-host-name code 225 = string ;
 * 		host anemometr {
 *      	hardware ethernet 00:11:22:33:44:xx;
 *      	fixed-address 192.168.1.24;
 *       	option zabbix-server-ip 192.168.1.6;
 *       	option zabix-host-name "Ed";
 * 		}
 *
 * key -- Zabbix key: {ALTIM_DIRECT, ALTIM_SPEED}
 *
 * value -- Float value of key.
*/
uint8_t sendToZabbix(uint8_t * addr, char * host, char * key, float value) {
#ifdef ZABBIX_DEBUG
    UART_Printf("Creating socket...\r\n");
#endif
    uint8_t tcp_socket = TCP_SOCKET;
    uint8_t code = socket(tcp_socket, Sn_MR_TCP, 10888, 0);
    if(code != tcp_socket) {
	#ifdef ZABBIX_DEBUG
        UART_Printf("socket() failed, code = %d\r\n", code);
	#endif
        return(-1);
    }
#ifdef ZABBIX_DEBUG
    UART_Printf("Socket created, connecting...\r\n");
#endif
    code = connect(tcp_socket, addr, ZABBIXPORT);
    if(code != SOCK_OK) {
	#ifdef ZABBIX_DEBUG
        UART_Printf("connect() failed, code = %d\r\n", code);
	#endif
        close(tcp_socket);
        return(-2);
    }
#ifdef ZABBIX_DEBUG
    UART_Printf("Connected, sending ZABBIX request...\r\n");
#endif
    {
    	char req[ZABBIXMAXLEN];
    	char str[ZABBIXMAXLEN - 13];
    	sprintf(str, "{\"request\":\"sender data\",\"data\":[{\"host\":\"%.20s\",\"key\":\"%s\",\"value\":\"%f\"}]}", host, key, value);
    	req[0] = 'Z';
    	req[1] = 'B';
		req[2] = 'X';
		req[3] = 'D';
		req[4] = 0x01;
		req[5] = strlen(str);
		req[6] = 0;
		req[7] = 0;
		req[8] = 0;
		req[9] = 0;
		req[10] = 0;
		req[11] = 0;
		req[12] = 0;
		strcpy(req + 13, str);
        //char req[] = "ZBXD\1\0\0\0\0\0\0\0\0{\"request\":\"sender data\",\"data\":[{\"host\":\"Ed\",\"key\":\"ALTIM_DIRECT\",\"value\":\"10\"}]}";
        uint8_t len = req[5] + 13;
        uint8_t* buff = (uint8_t*)&req;
        while(len > 0) {
		#ifdef ZABBIX_DEBUG
            UART_Printf("Sending %d bytes, data length %d bytes...\r\n", len, req[5]);
		#endif
            int32_t nbytes = send(tcp_socket, buff, len);
            if(nbytes <= 0) {
			#ifdef ZABBIX_DEBUG
                UART_Printf("send() failed, %d returned\r\n", nbytes);
			#endif
                close(tcp_socket);
                return(-3);
            }
			#ifdef ZABBIX_DEBUG
            UART_Printf("%d b sent!\r\n", nbytes);
			#endif
            len -= nbytes;
        }
    }
    /* Read data from Zabbix */
	#ifdef ZABBIX_DEBUG
		UART_Printf("Read.\r\n");
		{
			char buff[32];
			for(;;) {
				int32_t nbytes = recv(tcp_socket, (uint8_t*)&buff, sizeof(buff)-1);
				if(nbytes == SOCKERR_SOCKSTATUS) {
					UART_Printf("\r\nDisconnect.\r\n");
					break;
				}

				if(nbytes <= 0) {
					UART_Printf("\r\nrecv() failed, %d\r\n", nbytes);
					break;
				}

				buff[nbytes] = '\0';
				UART_Printf("%s", buff);
			}
		}

		UART_Printf("Closing socket.\r\n");
	#endif
    close(tcp_socket);
    return(0);
}



void init_w5500() {
	#ifdef ZABBIX_DEBUG
    UART_Printf("\r\ninit() called!\r\n");
    UART_Printf("W5500 callbacks...\r\n");
	#endif
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
	#ifdef ZABBIX_DEBUG
    UART_Printf("wizchip_init()...\r\n");
	#endif
    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
	#ifdef ZABBIX_DEBUG
    UART_Printf("DHCP_init()...\r\n");
	#endif

    // set MAC address before using DHCP
    setSHAR(net_info.mac);
    DHCP_init(DHCP_SOCKET, dhcp_buffer);
	#ifdef ZABBIX_DEBUG
    UART_Printf("DHCP callbacks...\r\n");
	#endif
    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );
	#ifdef ZABBIX_DEBUG
    UART_Printf("DHCP_run()...\r\n");
	#endif
    // actually should be called in a loop, e.g. by timer
    uint32_t ctr = 10000;
    while((!ip_assigned) && (ctr > 0)) {
        DHCP_run();
        ctr--;
        HAL_Delay(300);
    }
    if(!ip_assigned) {
		#ifdef ZABBIX_DEBUG
        UART_Printf("\r\nIP was not assigned :(\r\n");
		#endif
        return;
    }

    getIPfromDHCP(net_info.ip);
    getGWfromDHCP(net_info.gw);
    getSNfromDHCP(net_info.sn);
    getZABBIXfromDHCP(net_info.zabbix);
    getHostNamefromDHCP(net_info.hostname);
    getTimeSrvfromDHCP(net_info.tmsrv);
    if (net_info.hostname[0] == '\0') {
    	sprintf(ZabbixHostName, "%s", ZABBIXAGHOST);
    } else {
    	sprintf(ZabbixHostName, "%s", net_info.hostname);
    }

    //uint8_t dns[4];
    //getDNSfromDHCP(dns);
	#ifdef ZABBIX_DEBUG
    UART_Printf("IP:  %d.%d.%d.%d\r\nGW:  %d.%d.%d.%d\r\nNet: %d.%d.%d.%d\r\nZabbix: %d.%d.%d.%d\r\nHostName:%s\r\n",
        net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3],
        net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3],
        net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3],
        net_info.zabbix[0], net_info.zabbix[1], net_info.zabbix[2], net_info.zabbix[3],
		ZabbixHostName
    );
    UART_Printf("Calling wizchip_setnetinfo()...\r\n");
	#endif
    wizchip_setnetinfo(&net_info);

}
#endif

/*
 * Function for control SCD41
 */
void setSleepMode(uint16_t mode) {
  SCDwriteData(mode, NULL, 0);
  if(SCD4X_WAKE_UP == mode)
    HAL_Delay(20);   // Give it some time to switch mode
}

uint16_t performSelfTest(void) {
  SCDwriteData(SCD4X_PERFORM_SELF_TEST, NULL, 0);
  HAL_Delay(10000);
  uint8_t buf[3] = {0};
  return SCD4X_CONCAT_BYTES(buf[0], buf[1]);
}

void moduleReinit(void) {
  SCDwriteData(SCD4X_REINIT, NULL, 0);
  HAL_Delay(20);
}

void performFactoryReset(void) {
  SCDwriteData(SCD4X_PERFORM_FACTORY_RESET, NULL, 0);
  HAL_Delay(1200);
}

/*
 * Measurement Function for SCD41
 */

void measureSingleShot(uint16_t mode) {
  SCDwriteData(mode, NULL, 0);
  if(SCD4X_MEASURE_SINGLE_SHOT == mode) {
	  HAL_Delay(5000);
  } else if(SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY == mode) {
	  HAL_Delay(50);
  }
}

void enablePeriodMeasure(uint16_t mode) {
  SCDwriteData(mode, NULL, 0);
  if(SCD4X_STOP_PERIODIC_MEASURE == mode)
	HAL_Delay(500);   // Give it some time to switch mode
}

bool getDataReadyStatus(void) {
  uint8_t buf[3] = {0};
  SCDreadData(SCD4X_GET_DATA_READY_STATUS, buf, sizeof(buf));
  if(0x0000 == (SCD4X_CONCAT_BYTES(buf[0], buf[1]) & 0x7FF)){
    return false;
  }
  return true;
}

/*
void setTempComp(float tempComp) {
  uint16_t data = (uint16_t)(tempComp * ((uint32_t)1 << 16) / 175);
  uint8_t * sendPack  = pack(data);
  SCDwriteData(SCD4X_SET_TEMPERATURE_OFFSET, sendPack, 3);
  free(sendPack);
}

float getTempComp(void) {
  uint8_t buf[3] = {0};
  SCDreadData(SCD4X_GET_TEMPERATURE_OFFSET, buf, sizeof(buf));
  if(buf[2] != calcCRC(SCD4X_CONCAT_BYTES(buf[0], buf[1]))) {
    //DBG("The crc failed!");
  }
  return 175 * (float)( SCD4X_CONCAT_BYTES(buf[0], buf[1]) ) / ((uint32_t)1 << 16);
}

void setSensorAltitude(uint16_t altitude) {
  uint8_t * sendPack = pack(altitude);
  SCDwriteData(SCD4X_SET_SENSOR_ALTITUDE, sendPack, 3);
  free(sendPack);
}

uint16_t getSensorAltitude(void) {
  uint8_t buf[3] = {0};
  SCDreadData(SCD4X_GET_SENSOR_ALTITUDE, buf, sizeof(buf));
  return SCD4X_CONCAT_BYTES(buf[0], buf[1]);
}

void setAmbientPressure(uint32_t ambientPressure) {
  uint16_t data = (uint16_t)(ambientPressure / 100);
  uint8_t * sendPack = pack(data);
  SCDwriteData(SCD4X_SET_AMBIENT_PRESSURE, sendPack, 3);
  free(sendPack);
}

int16_t performForcedRecalibration(uint16_t CO2ppm) {
  uint8_t * sendPack = pack(CO2ppm);
  SCDwriteData(SCD4X_PERFORM_FORCED_RECALIB, sendPack, 3);
  free(sendPack);
  HAL_Delay(400);   // command execution time
  uint8_t buf[3] = {0};
  return (int16_t)(SCD4X_CONCAT_BYTES(buf[0], buf[1]) - 0x8000);
}

void setAutoCalibMode(bool mode) {
  uint16_t data = 0;
  if(mode) {
    data = 1;
  }
  uint8_t * sendPack = pack(data);
  SCDwriteData(SCD4X_SET_AUTOMATIC_CALIB, sendPack, 3);
  free(sendPack);
}

bool getAutoCalibMode(void) {
  uint8_t buf[3] = {0};
  SCDreadData(SCD4X_GET_AUTOMATIC_CALIB, buf, sizeof(buf));
  if(0x0000 == SCD4X_CONCAT_BYTES(buf[0], buf[1])){
    return false;
  }
  return true;
}
*/

void persistSettings(void) {
  SCDwriteData(SCD4X_PERSIST_SETTINGS, NULL, 0);
  HAL_Delay(800);
}

bool getSerialNumber(uint16_t * wordBuf) {
  bool ret = true;
  uint8_t buf[9] = {0};
  if(sizeof(buf) != SCDreadData(SCD4X_GET_SERIAL_NUMBER, buf, sizeof(buf))) {
    ret = false;
  }
  wordBuf[0] = SCD4X_CONCAT_BYTES(buf[0], buf[1]);
  wordBuf[1] = SCD4X_CONCAT_BYTES(buf[3], buf[4]);
  wordBuf[2] = SCD4X_CONCAT_BYTES(buf[6], buf[7]);
  return ret;
}

/*
uint8_t calcCRC(uint16_t data) {
  uint16_t current_byte;
  uint8_t crc = SCD4X_CRC8_INIT;
  uint8_t crc_bit;
  uint8_t buf[2] = {(uint8_t)((data >> 8) & 0xFF), (uint8_t)(data & 0xFF)};

  // calculates 8-Bit checksum with given polynomial
  for (current_byte = 0; current_byte < 2; ++current_byte) {
    crc ^= (buf[current_byte]);
    for (crc_bit = 8; crc_bit > 0; --crc_bit) {
      if (crc & 0x80)
        crc = (crc << 1) ^ SCD4X_CRC8_POLYNOMIAL;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

uint8_t * DFRobot_SCD4X_pack(uint16_t data) {
  uint8_t * pBuf = (uint8_t *)malloc(sizeof(uint8_t) * 3);
  memset(pBuf, 0, sizeof(uint8_t) * 3);
  pBuf[0] = (uint8_t)((data >> 8) & 0xFF);
  pBuf[1] = (uint8_t)(data & 0xFF);
  pBuf[2] = calcCRC(data);

  return pBuf;
}
*/

/****************************** Read/Write Command Function ********************************/

void SCDwriteData(uint16_t cmd, uint8_t *pBuf, size_t size) {
  HAL_I2C_Mem_Write(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, cmd, 2, NULL, 0, 100);
}

size_t SCDreadData(uint16_t cmd, uint8_t *pBuf, size_t size) {
  HAL_I2C_Mem_Write(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, cmd, 2, NULL, 0, 100);
  HAL_I2C_Master_Receive(&SCD4X_I2C_PORT, SCD4X_I2C_ADDR, pBuf, size, 100);
  return size;
}

void readMeasurement(sSensorMeasurement_t * data) {
  uint8_t buf[9] = {0};
  SCDreadData(SCD4X_READ_MEASUREMENT, buf, sizeof(buf));
  data->CO2ppm = SCD4X_CONCAT_BYTES(buf[0], buf[1]);
  data->temp = -45 + 175 * (float)(SCD4X_CONCAT_BYTES(buf[3], buf[4])) / ((uint32_t)1 << 16);
  data->humidity = 100 * (float)(SCD4X_CONCAT_BYTES(buf[6], buf[7])) / ((uint32_t)1 << 16);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  ST7735_Unselect();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  #ifdef DISPLAY_ST7735S
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);
  ST7735_WriteString(0, 0, "SCD41 init  ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  #endif
  #ifdef DISPLAY_1306
  ssd1306_Init();
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("SCD41 init  ", Font_6x8, 0x01);
  ssd1306_UpdateScreen();
  #endif

  uint16_t serialBufer[3];
  serialBufer[0] = 0;
  serialBufer[1] = 0;
  serialBufer[2] = 0;
  //LED_PULSE
  enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);  // Write: C4 0 D:3F 0 D:86 0
  HAL_Delay(500);
  //LED_PULSE
  if (getSerialNumber(serialBufer)) { // Write: C4 0 D:36 0 D:82 0 Read: C5 0 D:EA 0 D:AE 0 D:FA 0 D:B7 0 D:07 0 D:29 0 D:3B 0 D:BF Write: C4 0 D:21 0 D:B1 0
	#ifdef DISPLAY_ST7735S
	ST7735_WriteString(96, 0, "Ok", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	#endif
	#ifdef  DISPLAY_1306
	  ssd1306_WriteString("Ok", Font_6x8, 0x01);
	  ssd1306_UpdateScreen();
	  HAL_Delay(300);
	#endif
  } else {
	#ifdef DISPLAY_ST7735S
	ST7735_WriteString(96, 0, "Fail", Font_7x10, ST7735_RED, ST7735_BLACK);
	#endif
	#ifdef  DISPLAY_1306
	  ssd1306_WriteString("Failed.", Font_6x8, 0x01);
	  ssd1306_UpdateScreen();
	  HAL_Delay(300);
	#endif
  }

  #ifdef CO2_DEBUG
  sprintf(SndBuffer, "S0:%d S1:%d S2:%d   \r\n", serialBufer[0], serialBufer[1], serialBufer[2]);
  HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
  HAL_Delay(500);
  #endif
  enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

#ifdef BME280_ENABLE
	#ifdef DISPLAY_ST7735S
	ST7735_WriteString(0, 11, "BME280 init ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	#endif
	#ifdef DISPLAY_1306
	  ssd1306_SetCursor(0, 9);
	  ssd1306_WriteString("BME280 init ", Font_6x8, 0x01);
	  ssd1306_UpdateScreen();
	#endif
  BME280_Init();
	#ifdef DISPLAY_ST7735S
	ST7735_WriteString(96, 11, "Ok", Font_7x10, ST7735_GREEN, ST7735_BLACK);
	#endif
	#ifdef DISPLAY_1306
	ssd1306_WriteString("Ok", Font_6x8, 0x01);
	ssd1306_UpdateScreen();
	#endif
#endif

  /* Read MEassurment */ // Write: C4 0 D:EC 0 D:05 0 Read: C5 0 D:03 0 D:F4 0 D:EA 0 D:69 0 D:57 0 D:FE 0 D:5B 0 D:48 0 D:F8 0



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);


  #ifdef ZABBIX_ENABLE
	#ifdef DISPLAY_ST7735S
	ST7735_WriteString(0, 22, "Eth init", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	#endif
	#ifdef DISPLAY_1306
	//ssd1306_Init();
	ssd1306_SetCursor(0, 18);
	ssd1306_WriteString("Eth init    ", Font_6x8, 0x01);
	ssd1306_UpdateScreen();
	#endif
  HAL_GPIO_WritePin(Eth_rst_GPIO_Port, Eth_rst_Pin, GPIO_PIN_RESET);	// Reset W5500
  HAL_GPIO_WritePin(Eth_rst_GPIO_Port, Eth_rst_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Eth_CS_GPIO_Port, Eth_CS_Pin, GPIO_PIN_RESET);
  HAL_Delay(3000);
  init_w5500();
  if (net_info.tmsrv[0] == 0) {
	  SNTP_init(0, ntp_server, 28, gDATABUF);
  } else {
	  SNTP_init(0, net_info.tmsrv, 28, gDATABUF);
  }
#else
  HAL_GPIO_WritePin(Eth_rst_GPIO_Port, Eth_rst_Pin, GPIO_PIN_RESET);	// Reset W5500
  HAL_GPIO_WritePin(Eth_CS_GPIO_Port, Eth_CS_Pin, GPIO_PIN_SET);
#endif


  /* HV setting */
  HAL_TIM_Base_Start_IT(&htim1);
  gm_interval = HAL_GetTick();

  CO2Interval = MEAS_CO2_INTERVAL1;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  while (1)
  {
	  	  	if (HAL_GetTick() - CO2Counter > CO2Interval) {
	  	  		CO2Counter = HAL_GetTick();
	  	  		readMeasurement(&messuremetData);
				if (messuremetData.CO2ppm > 0) {
					CO2 = messuremetData.CO2ppm;
					CO2Interval = MEAS_CO2_INTERVAL2;
				} else {
					CO2Interval = MEAS_CO2_INTERVAL1;
				}
				#ifdef ZABBIX_ENABLE
				if (CO2 > 300) {
					sendToZabbix(net_info.zabbix, ZabbixHostName, "CO2", CO2);
				}
				#endif
				#ifdef CO2_DEBUG
				sprintf(SndBuffer, "CO2:%d H:%f T:%f   \r\n", messuremetData.CO2ppm, messuremetData.humidity, messuremetData.temp);
				HAL_UART_Transmit(&huart1, (uint8_t *) SndBuffer, sizeof(SndBuffer), 1000);
				#endif
	  	  	}
	  	  	if (HAL_GetTick() - fastCounter > MEAS_INTERVAL) {
	  	  		fastCounter = HAL_GetTick();

				#ifdef BME280_ENABLE
	  	  		temperature = BME280_ReadTemperature();
	  	  		pressure = BME280_ReadPressure() * 0.00750063755419211f; //0.00750063755419211
	  	  		humidity = BME280_ReadHumidity();
				#endif

				#ifdef DISPLAY_ST7735S
				ST7735_FillScreen(ST7735_BLACK);
				sprintf(text1306, "Temperature:%.1f", temperature);
				ST7735_WriteString(0, 0, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				/* Давление */
				sprintf(text1306, "Pressure:%.0f", pressure);
				ST7735_WriteString(0, 11, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				/* Влажность */
				sprintf(text1306, "Hummidity:%.1f", humidity);
				ST7735_WriteString(0, 22, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				/* CO2 */
				sprintf(text1306, "CO2:%d ppm", CO2);
				if (CO2 < CO2_NOMINAL) {
					ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_GREEN, ST7735_BLACK);
				} else if (CO2 < CO2_MAXIMUM) {
					ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
				} else {
					ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_RED, ST7735_BLACK);
				}

				/* Дозиметр */
				sprintf(text1306, "C:%d", gm_counter);
				ST7735_WriteString(0, 44, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				sprintf(text1306, "%.2fcps", gm_cps);
				ST7735_WriteString(0, 55, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				sprintf(text1306, "%.1fuRh", gm_cps * GM_CPS2URh);
				ST7735_WriteString(0, 66, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

				/* Высокое напряжение */
				sprintf(text1306, "%.0fv", hvLevel * POWER_CONVERT);
				ST7735_WriteString(0, 77, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
				#ifdef ZABBIX_ENABLE
				/* Часы */
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				sprintf(text1306, "%.2i:%.2i", sTime.Hours, sTime.Minutes);
				ST7735_WriteString(24, 88, text1306, Font_16x26, ST7735_BLUE, ST7735_BLACK);
				#endif
				#endif


				#ifdef DISPLAY_1306
				ssd1306_Fill(0);
				/* Температура */
				sprintf(text1306, "T:%.1f", temperature);
				ssd1306_SetCursor(COLUMN0, 0);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				/* Давление */
				sprintf(text1306, "P:%.0f", pressure);
				ssd1306_SetCursor(COLUMN0, 8);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				/* Влажность */
				sprintf(text1306, "H:%.1f", humidity);
				ssd1306_SetCursor(COLUMN0, 16);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				/* CO2 */
				sprintf(text1306, "CO2:%d", CO2);
				ssd1306_SetCursor(COLUMN0, 24);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				/* Дозиметр */
				sprintf(text1306, "C:%d", gm_counter);
				ssd1306_SetCursor(COLUMN1, 0);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				sprintf(text1306, "%.2fcps", gm_cps);
				ssd1306_SetCursor(COLUMN1, 8);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				sprintf(text1306, "%.1fuRh", gm_cps * GM_CPS2URh);
				ssd1306_SetCursor(COLUMN1, 16);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				/* Высокое напряжение */
				sprintf(text1306, "%.0fv", hvLevel * POWER_CONVERT);
				ssd1306_SetCursor(COLUMN1, 24);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);

				#ifdef ZABBIX_ENABLE
				/* Часы */
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				sprintf(text1306, "%.2i:%.2i", sTime.Hours, sTime.Minutes);
				ssd1306_SetCursor(COLUMN2, 0);
				ssd1306_WriteString(text1306, Font_6x8, 0x01);
				#endif

				ssd1306_UpdateScreen();
				#endif
				HAL_IWDG_Refresh(&hiwdg);
	  	  	}
		#ifdef ZABBIX_ENABLE
	  	/* Настройка времени через NTP  */
		if ((HAL_GetTick() - ntp_interval > NTP_INTERVAL) || first_start) {
			first_start = FALSE;
			ntp_interval = HAL_GetTick();
			do {} while (SNTP_run(&timeNTP) != 1);
			sTime.Hours = timeNTP.hh;
			sTime.Minutes = timeNTP.mm;
			sTime.Seconds = timeNTP.ss;
			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		}
		#if defined(TMP117_ENABLE) || defined(BME280_ENABLE)
		if (HAL_GetTick() - zabb_interval > ZABB_MEAS_INTERVAL) {
			zabb_interval = HAL_GetTick();
		  if ((temperature < 60.0) && (temperature > -40.0)) {
			  sendToZabbix(net_info.zabbix, ZabbixHostName, "TEMPERATURE", temperature);
			#ifdef BME280_ENABLE
			  sendToZabbix(net_info.zabbix, ZabbixHostName, "PRESSURE", pressure);
			  sendToZabbix(net_info.zabbix, ZabbixHostName, "HUMIDITY", humidity);
			#endif
		  } else {
			#ifdef BME280_ENABLE
			HAL_I2C_DeInit(&hi2c1);
			MX_I2C1_Init();
			BME280_Init();  // Сбой датчика bme280, пробуем исправить.
			#endif
		  }
		}
		#endif
		if (gm_ready) {
			gm_ready = FALSE;
			sendToZabbix(net_info.zabbix, ZabbixHostName, "GM_CPS", gm_cps);
		}
		#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 40000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 20;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7735S_RESET_Pin|Eth_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Eth_rst_Pin|ST7735S_CS_Pin|ST7735S_DC_Pin|LED_Pin
                          |POWER_PULSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PULSE_GM_Pin */
  GPIO_InitStruct.Pin = PULSE_GM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PULSE_GM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ST7735S_RESET_Pin */
  GPIO_InitStruct.Pin = ST7735S_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ST7735S_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Eth_CS_Pin */
  GPIO_InitStruct.Pin = Eth_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Eth_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Eth_rst_Pin ST7735S_CS_Pin ST7735S_DC_Pin POWER_PULSE_Pin */
  GPIO_InitStruct.Pin = Eth_rst_Pin|ST7735S_CS_Pin|ST7735S_DC_Pin|POWER_PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if(GPIO_Pin == PULSE_GM_Pin) // interrupt from PULSE_GM
   {
	   gm_counter++;
	   if (gm_counter >= TRUST_INTERVAL) {
		   gm_cps = (float) (gm_counter / (float) ((HAL_GetTick() - gm_interval) / 1000)) - GM_SELF_FONE;
		   gm_ready = TRUE;
		   if (gm_cps < 0) {
			   gm_cps = 0;
		   }
		   gm_counter = 0;
		   gm_interval = HAL_GetTick();
	   }
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim4);
   }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
