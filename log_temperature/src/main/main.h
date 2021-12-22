#pragma once

#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

#define OTAA true                                               // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                                       /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0                                     /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5                               /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                                        /**< Number of trials for the join request. */
#define CURRENT_CLASS CLASS_A           
#define CURRENT_REGION LORAMAC_REGION_EU868           /* Region:EU868*/
#define CURRENT_CONFIRM LMH_UNCONFIRMED_MSG                 /* confirm/unconfirm packet definition*/
#define PORT LORAWAN_APP_PORT;                              /* data port*/
/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/

void send_lora_frame(void);
uint32_t timers_init(void);

#define LORAWAN_APP_DATA_BUFF_SIZE 64 /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 30000    /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */

TimerEvent_t appTimer;