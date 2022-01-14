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

#include "main.h"

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

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  // Initialize LoRa chip.
  lora_rak4630_init();

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
      // turn the LED off by making the voltage LOW
      digitalWrite(LED_BUILTIN2, HIGH);
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!WORKING");
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
  Serial.println("Joining started..");
  lmh_join();

  // lis3dh init
  if (SensorTwo.begin() != 0)
  {
    Serial.println("Problem starting the sensor at 0x18.");
  }
  else
  {
    Serial.println("Sensor at 0x18 started.");
    // Set low power mode
    uint8_t data_to_write = 0;
    SensorTwo.readRegister(&data_to_write, LIS3DH_CTRL_REG1);
    data_to_write |= 0x08;
    SensorTwo.writeRegister(LIS3DH_CTRL_REG1, data_to_write);
    delay(100);

    data_to_write = 0;
    SensorTwo.readRegister(&data_to_write, 0x1E);
    data_to_write |= 0x90;
    SensorTwo.writeRegister(0x1E, data_to_write);
    delay(100);
  }
  // gps init

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);
  timersInit();
  sendLoraFrame();
}

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
 */

static uint32_t COUNT = 0;
static uint32_t COUNT_FAIL = 0;

void loop()
{
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

String data = "";

/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
  if (tmp.indexOf(",E,") != -1)
  {
    direction_E_W = 0;
  }
  else
  {
    direction_E_W = 1;
  }

  if (tmp.indexOf(",S,") != -1)
  {
    direction_S_N = 0;
  }
  else
  {
    direction_S_N = 1;
  }
}

void sendLoraFrame(void)
{
  // if (lmh_join_status_get() != LMH_SET)
  //{
  //   // Not joined, try again later
  //   return;
  // }

  float x = 0;
  float y = 0;
  float z = 0;

  bool newData = false;

  Serial.println("check acc!");
  x = SensorTwo.readFloatAccelX() * 1000;
  Serial.println("SensorTwo.readFloatAccelX()");
  y = SensorTwo.readFloatAccelY() * 1000;
  z = SensorTwo.readFloatAccelZ() * 1000;
  Serial.println("Reading finished.");
  data = "X = " + String(x) + "mg" + " Y = " + String(y) + "mg" + " Z =" + String(z) + "mg";
  Serial.println(data);
  data = "";
  if (true || abs(x - z) < 400)
  {
    // For one second we parse GPS data and report some key values
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (Serial1.available())
      {
        char c = Serial1.read();
        //         Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        tmp_data += c;
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    }
    direction_parse(tmp_data);
    tmp_data = "";
    float flat, flon;
    int32_t ilat, ilon;
    if (newData)
    {
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
      ilat = flat * 100000;
      flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
      ilon = flon * 100000;
      memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
      m_lora_app_data.port = PORT;
      m_lora_app_data.buffer[0] = 0x09;
      // lat data
      m_lora_app_data.buffer[1] = (ilat & 0xFF000000) >> 24;
      m_lora_app_data.buffer[2] = (ilat & 0x00FF0000) >> 16;
      m_lora_app_data.buffer[3] = (ilat & 0x0000FF00) >> 8;
      m_lora_app_data.buffer[4] = ilat & 0x000000FF;
      if (direction_S_N == 0)
      {
        m_lora_app_data.buffer[5] = 'S';
      }
      else
      {
        m_lora_app_data.buffer[5] = 'N';
      }
      // lon data
      m_lora_app_data.buffer[6] = (ilon & 0xFF000000) >> 24;
      m_lora_app_data.buffer[7] = (ilon & 0x00FF0000) >> 16;
      m_lora_app_data.buffer[8] = (ilon & 0x0000FF00) >> 8;
      m_lora_app_data.buffer[9] = ilon & 0x000000FF;
      if (direction_E_W == 0)
      {
        m_lora_app_data.buffer[10] = 'E';
      }
      else
      {
        m_lora_app_data.buffer[10] = 'W';
      }
      m_lora_app_data.buffsize = 11;
      // sendLoraFrame();
    }
    else
    {
      Serial.println("No Location Found");
      TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
      TimerStart(&appTimer);
    }
  }
  else
  {
    Serial.println("Turn WisBlock with USB pointing up to start location search");
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }

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