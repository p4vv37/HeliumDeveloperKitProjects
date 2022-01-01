/**
 * 
 * Based on: https://github.com/RAKWireless/WisBlock/blob/master/examples/RAK4630/solutions/Environment_Monitoring/Environment_Monitoring.ino
 * Updated to match https://www.arduino.cc/en/Reference/StyleGuide
 * This code reads tempreature, pressure etc. from an environment sensor and uploads it via Helium network.
 * Original comment:
 * @file Environment_Monitoring.ino
 * @author rakwireless.com
 * @brief This sketch demonstrate how to get environment data from BME680
 *        and send the data to lora gateway.
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#define DEBUG
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

#include "main.h"
#include "secrets.h"

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

void lorawanConfirmClassHandler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = PORT;
  lmh_send(&m_lora_app_data, CURRENT_CONFIRM);
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawanRxHandler(lmh_app_data_t *app_data)
{
  Serial.printf(
      "LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
      app_data->port, app_data->buffsize, app_data->rssi, app_data->snr,
      app_data->buffer);
}

/**@brief LoRa function for handling OTAA join failed
 */
static void lorawanJoinFailedHandler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");

  lmh_error_status ret = lmh_class_request(CURRENT_CLASS);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}

static lmh_callback_t G_LORA_CALLBACKS = {
    BoardGetBatteryLevel, BoardGetUniqueId,
    BoardGetRandomSeed, lorawanRxHandler,
    lorawan_has_joined_handler, lorawanConfirmClassHandler,
    lorawanJoinFailedHandler};

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t G_LORA_PARAM_INIT = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

Adafruit_BME680 bme;
void initBme680(void)
{
  Wire.begin();

  if (!bme.begin(0x76))
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize LoRa chip.
  lora_rak4630_init();
  initBme680();

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(9600);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
#ifdef OTAA
  Serial.println("Type: OTAA");
#else
  Serial.println("Type: ABP");
#endif // OTAA

  switch (CURRENT_REGION)
  {
  case LORAMAC_REGION_AS923:
    Serial.println("Region: AS923");
    break;
  case LORAMAC_REGION_AU915:
    Serial.println("Region: AU915");
    break;
  case LORAMAC_REGION_CN470:
    Serial.println("Region: CN470");
    break;
  case LORAMAC_REGION_EU433:
    Serial.println("Region: EU433");
    break;
  case LORAMAC_REGION_IN865:
    Serial.println("Region: IN865");
    break;
  case LORAMAC_REGION_EU868:
    Serial.println("Region: EU868");
    break;
  case LORAMAC_REGION_KR920:
    Serial.println("Region: KR920");
    break;
  case LORAMAC_REGION_US915:
    Serial.println("Region: US915");
    break;
  default:
    Serial.println("Region: -");
    break;
  }
  Serial.println("=====================================");

  // creat a user timer to send data to server period
  uint32_t err_code;
  err_code = timersInit();
  if (err_code != 0)
  {
    Serial.printf("timersInit failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  // Initialize LoRaWan
  err_code = lmh_init(&G_LORA_CALLBACKS, G_LORA_PARAM_INIT, OTAA, CURRENT_CLASS, CURRENT_REGION);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  // Start Join procedure
  lmh_join();
}

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/

static uint32_t COUNT = 0;
static uint32_t COUNT_FAIL = 0;

void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions.
}

/**@brief Function for handling user timerout event.
 */
void txLoraPeriodicHandler(void)
{
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
  Serial.println("Sending frame now...");
  sendLoraFrame();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timersInit(void)
{
  TimerInit(&appTimer, txLoraPeriodicHandler);
  return 0;
}

void bme680Get()
{
  Serial.print("result: ");
  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = PORT;

  double temp = bme.temperature;
  // Serial.printf("temp: %f\n", temp);
  double pres = bme.pressure / 100.0;
  // Serial.printf("pres: %f\n", pres);
  double hum = bme.humidity;
  // Serial.printf("humidity: %f\n", hum);
  uint32_t gas = bme.gas_resistance;

  uint16_t t = temp * 100;
  uint16_t h = hum * 100;
  uint32_t pre = pres * 100;
  // Serial.printf("buffsize: %d, \n", m_lora_app_data.buffsize);
  //result: T=28.25C, RH=50.00%, P=958.57hPa, G=100406 Ohms
  m_lora_app_data.buffer[i++] = 0x01;
  m_lora_app_data.buffer[i++] = (uint8_t)(t >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)t;
  m_lora_app_data.buffer[i++] = (uint8_t)(h >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)h;
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((pre & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(pre & 0x000000FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0xFF000000) >> 24);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x00FF0000) >> 16);
  m_lora_app_data.buffer[i++] = (uint8_t)((gas & 0x0000FF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(gas & 0x000000FF);
  m_lora_app_data.buffsize = i;
  Serial.printf("i: %d, buffsize: %d\n", i, m_lora_app_data.buffsize);
  // for (int i = 0; i < m_lora_app_data.buffsize - 1; ++i) {
  //     Serial.printf("%d, ", m_lora_app_data_buffer[i]);
  // }
  Serial.printf("sending..\n");
}

void sendLoraFrame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }
  if (!bme.performReading())
  {
    Serial.printf("sklipping!");
    return;
  }

  bme680Get();

  Serial.printf("size2: %d\n", m_lora_app_data.buffsize);
  // for (int i = 0; i < m_lora_app_data.buffsize - 1; ++i) {
  //     Serial.printf(": %d, ", m_lora_app_data_buffer[i]);
  // }
  Serial.printf("sending...");

  lmh_error_status error = lmh_send(&m_lora_app_data, CURRENT_CONFIRM);
  if (error == LMH_SUCCESS)
  {
    COUNT++;
    Serial.printf("lmh_send ok COUNT %d\n", COUNT);
  }
  else
  {
    COUNT_FAIL++;
    Serial.printf("lmh_send fail COUNT %d\n", COUNT_FAIL);
  }
}
