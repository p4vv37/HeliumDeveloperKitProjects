#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>

#include "secrets.h"

#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

const int OTAA = 1;
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
const int SCHED_QUEUE_SIZE = 60;                                  /**< Maximum number of events in the scheduler queue. */
const int LORAWAN_DATERATE = DR_0;                                /*LoRaMac datarates definition, from DR_0 to DR_5*/
const int LORAWAN_TX_POWER = TX_POWER_5;                          /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
const int JOINREQ_NBTRIALS = 3;                                   /**< Number of trials for the join request. */
const DeviceClass_t CURRENT_CLASS = CLASS_A;
const eLoRaMacRegion_t CURRENT_REGION = LORAMAC_REGION_EU868; /* Region:EU868*/
const lmh_confirm CURRENT_CONFIRM = LMH_UNCONFIRMED_MSG;      /* confirm/unconfirm packet definition*/
const int PORT = LORAWAN_APP_PORT;                            /* data port*/

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

void sendLoraFrame(void);
uint32_t timersInit(void);

const int LORAWAN_APP_DATA_BUFF_SIZE = 64; /**< buffer size of the data to be transmitted. */
const int LORAWAN_APP_INTERVAL = 20000;    /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */

TimerEvent_t appTimer;