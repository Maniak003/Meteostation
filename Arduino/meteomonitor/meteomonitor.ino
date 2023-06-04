#include <Ethernet.h>
//#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_BME280.h>

//#define SERIAL_OUT
#define ZABBIXPORT 10051      // Zabbix erver Port
#define ZABBIXMAXLEN 128
#define ZABBIXAGHOST "Meteo_4"  // Zabbix item's host name
#define ZABBIXSENDPERIOD 300 // Period in secoonds
#define wdt_on false
#define MEASSURE_INTERVAL 10000
#define BMEADDR 0x76
#define SENSEAIR

#ifdef SENSEAIR
#include <SoftwareSerial.h>
#endif

#if wdt_on
#include <avr/wdt.h>
#endif

#if defined(WIZ550io_WITH_MACADDRESS) // Use assigned MAC address of WIZ550io
  ;
#else
  byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x46 };
#endif
#define LED A3      /* Моргалка */
#define ETHReset 7  /* Сброс ethernet интерфейса */

//#define OLED
#ifdef OLED
//#define USE_MICRO_WIRE
#include <GyverOLED.h>
GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
#endif

Adafruit_BME280 bme; // I2C
unsigned long ms_old, ms;
unsigned statBME280;

uint8_t res[ZABBIXMAXLEN];
EthernetClient client;
IPAddress server(192, 168, 1, 6); // Zabbix server IP.
String str;

/* SenseAir S8 
  RxD -- 8
  TxD -- 9
*/
#ifdef SENSEAIR
SoftwareSerial K_30_Serial(8, 9); // Программный порт
byte readCO2[] =  {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25}; //Команда для запроса показаний CO2 с датчика
//byte readTemp[] = {0xFE, 0X44, 0X00, 0X12, 0X02, 0X94, 0X45}; //Команда для запроса показаний температуры с датчика
//byte readRH[] =   {0xFE, 0x44, 0x00, 0x14, 0x02, 0x97, 0xE5}; //Команда для запроса показаний влажности с датчика
byte response[] = {0, 0, 0, 0, 0, 0, 0}; //массив для ответа от датчика
#endif
String valPress, valRH, valTemp, valCO2;
double val;

void sendToZabbix(String key, float val) {
  int len;
  str = "{\"request\":\"sender data\",\"data\":[{\"host\":\"";
  str = str + ZABBIXAGHOST;
  str = str + "\",\"key\":\""; 
  str = str + key;
  str = str + "\",\"value\":\"";
  str = str + val;
  str = str + "\"}]}";
  len = str.length();
  res[0] = 'Z';
  res[1] = 'B';
  res[2] = 'X';
  res[3] = 'D';
  res[4] = 0x01;
  res[5] = str.length();
  res[6] = 0;
  res[7] = 0;
  res[8] = 0;
  res[9] = 0;
  res[10] = 0;
  res[11] = 0;
  res[12] = 0;
  str.getBytes(&(res[13]), ZABBIXMAXLEN - 12);
  len = len + 13;
  #ifdef SERIAL_OUT
  for( int i = 13; i < len + 13; i++) {
    Serial.print((char)(res[i]));
  }
  Serial.println();
  #endif
  if (client.connect(server, ZABBIXPORT)) {
    client.write(res, len);
    #if wdt_on
    wdt_reset();
    #endif
  } else {
    #ifdef SERIAL_OUT
    Serial.println("Not conn.");
    #endif
  }      
  client.stop();
}

void setup() {
  #if wdt_on
    wdt_disable();
  #endif
#ifdef SERIAL_OUT
  Serial.begin(9600);
  while(!Serial);
  delay(1000);
#endif

#ifdef OLED
  oled.init();
#endif

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  #ifdef SENSEAIR
  K_30_Serial.begin(9600);
  #endif
  pinMode(ETHReset, OUTPUT);
  statBME280 = bme.begin(BMEADDR);
#ifdef SERIAL_OUT
  Serial.println();
  Serial.print("BME280-");
  if (!statBME280) {
    Serial.println("No");
  } else {
    Serial.println("Ok.");
  }
    Serial.print("Eth-");
#endif
  digitalWrite(ETHReset, LOW);
  digitalWrite(ETHReset, HIGH);
  ms_old = millis();
  #if wdt_on
    wdt_enable(WDTO_8S);
  #endif
  
    if (Ethernet.begin(mac) == 0) {
  #ifdef SERIAL_OUT
      Serial.println("FALSE.");
  #endif
    }
  #ifdef SERIAL_OUT
      Serial.println(Ethernet.localIP());
  #endif
    digitalWrite(LED, LOW);
}

void loop() {
  ms = millis();
  if (ms - ms_old > MEASSURE_INTERVAL) {
    digitalWrite(LED, HIGH);
    if (statBME280) {
      val = bme.readTemperature();
      sendToZabbix("TEMPERATURE", val);
      valTemp = "T:" + String(val);
      val = bme.readPressure() * 0.0075006156130264F; // From Pa to mmHg
      sendToZabbix("PRESSURE", val);
      valPress = "P:" + String(val);
      val = bme.readHumidity();
      sendToZabbix("HUMMIDITY", val);
      valRH = "H:" + String(val);
    }
    /* CO2 read */
    #ifdef SENSEAIR
    while (!K_30_Serial.available()) {
      K_30_Serial.write(readCO2, 7);
      delay(50);
    }
    int timeout = 0;                        // Счетчик таймаута
    while (K_30_Serial.available() < 7 ) {  // Ждем ответа 7 байт
      if (timeout++ > 10) {                 // Проверка таймаута
        while (K_30_Serial.available())
          K_30_Serial.read();
        break;
      }
      delay(50);
    }
    for (int i = 0; i < 7; i++)  {
      response[i] = K_30_Serial.read();
    }
    val = response[3] * 256 + response[4];
    if (val > 300) {
      sendToZabbix("CO2", val);
      valCO2 = "CO2:" + String((int) val);
    }
    #endif
    //sendRequest(readTemp);
    //valTemp = "Temp: " + String(val / 100) + " C";
    //sendRequest(readRH);
    //valRH = "RH: " + String(val / 100) + " %";
    #ifdef OLED
    oled.clear();
    oled.setCursor(0, 0);
    oled.print(valCO2);
    oled.setCursor(0, 1);
    oled.print(valTemp);
    oled.setCursor(0, 2);
    oled.print(valRH);
    oled.setCursor(0, 3);
    oled.print(valPress);
    oled.update();
    #endif
    #ifdef SERIAL_OUT
    Serial.println(valCO2);
    Serial.println(valTemp);
    Serial.println(valRH);
    Serial.println(valPress);
    #endif
   ms_old = ms;
   digitalWrite(LED, LOW);
  }
  
  #if wdt_on
    wdt_reset();
  #endif
}
