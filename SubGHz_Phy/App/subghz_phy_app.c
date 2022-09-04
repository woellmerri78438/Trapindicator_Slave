/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_seq.h"
#include "utilities_def.h"
#include "stm32_timer.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "battery_read.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */
typedef enum
{
  NONE,
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Buffer for TX and RX is selected LoRa PAYLOAD Length in Bytes*/
#define MAX_APP_BUFFER_SIZE		PAYLOAD_LEN

#define RXBUF_LEN				20

#define LEDConfIdle_PERIOD_MS                 500 //1Hz

/* Battery Readout with voltage divider & adc */
#define R8 45800 //Spannungsteiler unten gg Ground
#define R9 15000 //Spannungsteiler oben gg V3.3

/*LoRa*/
#define TX_TIMEOUT_VALUE              3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */
static UTIL_TIMER_Object_t timerSleepWakeup;
static UTIL_TIMER_Object_t timerLedIdleConf;
static UTIL_TIMER_Object_t timerWakedup;

/* Slave ID of this device */
uint8_t ownID = 0x3;

/*Battery Readout*/
float Akkuspannung;
int Akku_Prozent;
uint8_t battery_OK = 0;

/*Time for cyclic wakeup in mins. */
uint32_t mins_to_WakeupSync = 2;
uint32_t mins_passed = 0;

uint8_t wakeuptimeover = 0;

/*LoRa Variables*/
/*Actual State */
static States_t State = NONE;
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/*Receive Buffer*/
uint8_t RxBuffer[RXBUF_LEN] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
static void OnledConfIdleEvent(void *context);
static void timerSleepWakeupEvent(void *context);
static void onwakeuptimeover(void *context);

/*Main Processes for Sequencer*/
static void ConfigModeIdle(void);
static void SyncedWakeup(void);

/*Other Subfunctions*/
static void answerMasterSync(void);
static void answerMasterSleep(void);
static void answerMasterTrapPoll(void);

/*Helper Functions*/
/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  /* Radio Set frequency */
   Radio.SetChannel(RF_FREQUENCY);

   /* Radio configuration */
 #if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
   APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
   APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);

   Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                     LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

   Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                     LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                     LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                     0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

   Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

 #elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
   APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
   APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

   Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                     FSK_DATARATE, 0,
                     FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                     true, 0, 0, 0, TX_TIMEOUT_VALUE);

   Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                     0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                     0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                     0, 0, false, true);

   Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

 #else
 #error "Please define a modulation in the subghz_phy_app.h file."
 #endif /* USE_MODEM_LORA | USE_MODEM_FSK */

   //send Radio to sleep in the beginning
   Radio.Sleep();

  /*task registration*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_ConfigMode_Idle), UTIL_SEQ_RFU, ConfigModeIdle);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SyncedWakeup), UTIL_SEQ_RFU, SyncedWakeup);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_ConfigMode_Idle), CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
	State = TX;
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	/*Only Messages with the correct identifier are relevant*/
	if(payload[0] != 0xFA)
		return;
	/*If size is not correct, its also not a correct message from master*/
	if (size != PAYLOAD_LEN)
		return;
	/*First Bit must be a 1, because if not, its a message from a slave*/
	if (((payload[0] & 0x80) >> 7) != 1)
		return;

	State = RX;
	RxBufferSize = size;
	/*Only Copy if Received Data is less than Buffer availible*/
	if (RxBufferSize < RXBUF_LEN){
		memcpy(RxBuffer, payload, size);
	}
	RssiValue = rssi;
	SnrValue = LoraSnr_FskCfo;
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	State = TX_TIMEOUT;
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	State = RX_TIMEOUT;
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	State = RX_ERROR;
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void ConfigModeIdle(void){
	//Disable Reed Contact Interrupt in Config Mode (in System-Init all IRQs are enabled)
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	//LED Red Blink 1 Hz start
	UTIL_TIMER_Create(&timerLedIdleConf, LEDConfIdle_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledConfIdleEvent, NULL);
	UTIL_TIMER_Start(&timerLedIdleConf);

	/*Start Reception*/
	State = NONE;
	Radio.Rx(0);

	/*Wait for commands from master*/
	while(1) {
		if (State == RX_ERROR || State == RX_TIMEOUT){
			/*Reset State for next receive*/
			State = NONE;
			Radio.Rx(0);
			}
		if (State == RX){
			State = NONE;
			/* Look what command (fc) from master arrived*/
			uint8_t mastercmd = RxBuffer[1] & 0x0F;
			uint8_t id = RxBuffer[2] >> 4;
			/*If sync and register command:*/
			if (mastercmd == 1){
				memcpy(&mins_to_WakeupSync, RxBuffer+3, 4);
				answerMasterSync();
			}
			/*If sleepmode start command and own id*/
			if (mastercmd == 8 && id == ownID){
				memcpy(&mins_to_WakeupSync, RxBuffer+3, 4);
				answerMasterSleep();
				/*End Config Mode*/
				Radio.Sleep();
				UTIL_TIMER_Stop(&timerLedIdleConf);
				HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
				/*Start timer for synced wakeup interval*/
				UTIL_TIMER_Create(&timerSleepWakeup, 60000, UTIL_TIMER_ONESHOT, timerSleepWakeupEvent, NULL); //every min, mins++
				UTIL_TIMER_Start(&timerSleepWakeup);
				return;
			}
			State = NONE;
			Radio.Rx(0);
		}
	}
}

static void SyncedWakeup(void){
	HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 0);
	/*Start Timer, after this time it goes back to sleep for sure*/
	UTIL_TIMER_Create(&timerWakedup, 30000, UTIL_TIMER_ONESHOT, onwakeuptimeover, NULL);
	wakeuptimeover = 0;
	UTIL_TIMER_Start(&timerWakedup);
	/*While time is not over, wait for master poll*/
	while(!wakeuptimeover){
		State = NONE;
		Radio.Rx(0);
		if (State == RX_ERROR || State == RX_TIMEOUT){
			/*If Error occurs, try fetching another*/
			State = NONE;
			Radio.Rx(0);
		}
		if (State == RX){
			State = NONE;
			/* Look what command (fc) from master arrived*/
			uint8_t mastercmd = RxBuffer[1] & 0x0F;
			uint8_t id = RxBuffer[2] >> 4;
			/*If id is the own and mastercmd is "polling"*/
			if (mastercmd == 4 && id == ownID){
				answerMasterTrapPoll();
				UTIL_TIMER_Stop(&timerWakedup);
				Radio.Sleep();
				HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 1);
				return;
			}
			else{
				/*IF it was for someone else, try fetching another*/
				Radio.Rx(0);
			}
		}
	}
	Radio.Sleep();
	HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, 1);
	return;


}

static void answerMasterSync(void){
	  uint8_t i = 0;
	  /*fills tx buffer*/
	  memset(BufferTx, 0x0, PAYLOAD_LEN);
	  BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	  BufferTx[1] = 0x00; //Function -> Message from Slave, Acknoledge
	  BufferTx[2] = ownID << 4; //Slave ID
	  /*Data*/
	  BufferTx[3] = (uint8_t)Ausgabe_Akkustand(Berechnung_Akkustand(R9, R8));
	  BufferTx[4] = 0x00;
	  BufferTx[5] = 0x00;
	  BufferTx[6] = 0x00;
	  BufferTx[7] = 0x00;

	  /*Send Answer 3 times with a random delay between 0 and 6 seconds in whole seconds, because on-air-time can be up to 1 second*/
	  for (i = 0; i<3; i++){
		  srand(HAL_GetTick());//Set random number seed
		  uint16_t rnddelay = (rand()%6)*1000;
		  HAL_Delay(rnddelay);
		  State = NONE;
		  Radio.Send(BufferTx, PAYLOAD_LEN);
		  /*Wait until LoRa module has done someting*/
		  while (State != TX && State != TX_TIMEOUT);
	  }

	  /*return in config mode idle*/
	  State = NONE;
	  Radio.Rx(0);
}

static void answerMasterSleep(void){
	  /*fills tx buffer*/
	  memset(BufferTx, 0x0, PAYLOAD_LEN);
	  BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	  BufferTx[1] = 0x02; //Function -> Message from Slave, Acknoledge for sleepmode
	  BufferTx[2] = ownID <<4; //Slave ID
	  /*Data*/
	  BufferTx[3] = 0x00;
	  BufferTx[4] = 0x00;
	  BufferTx[5] = 0x00;
	  BufferTx[6] = 0x00;
	  BufferTx[7] = 0x00;

	  State = NONE;
	  Radio.Send(BufferTx, PAYLOAD_LEN);
	  /*Wait until LoRa module has done someting*/
	  while (State != TX && State != TX_TIMEOUT);

	  /*return*/
	  State = NONE;
}

static void answerMasterTrapPoll(void){
	  /*fills tx buffer*/
	  memset(BufferTx, 0x0, PAYLOAD_LEN);
	  BufferTx[0] = 0xFA; //Identifier, Message belongs to Trap devices
	  BufferTx[1] = 0x00; //Function -> Message from Slave, Acknoledge
	  BufferTx[2] = ownID <<4; //Slave ID
	  /*Data*/
	  BufferTx[3] = (uint8_t)Ausgabe_Akkustand(Berechnung_Akkustand(R9, R8));
	  BufferTx[4] = (uint8_t)HAL_GPIO_ReadPin(REEDPin_GPIO_Port, REEDPin_Pin);
	  BufferTx[5] = 0x00;
	  BufferTx[6] = 0x00;
	  BufferTx[7] = 0x00;

	  State = NONE;
	  Radio.Send(BufferTx, PAYLOAD_LEN);
	  /*Wait until LoRa module has done someting*/
	  while (State != TX && State != TX_TIMEOUT);

	  /*return*/
	  State = NONE;
}


static void OnledConfIdleEvent(void *context){
  HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin); /* LED_Red*/
  UTIL_TIMER_Start(&timerLedIdleConf);
}

static void timerSleepWakeupEvent(void *context){
	mins_passed++;
	/*Start timer right again, same as in slaves.
	*because this way wakeup time interval will stay
	*the same most likely, independent from different amout of code
	*done during waked up process  */
	UTIL_TIMER_Start(&timerSleepWakeup);
	if (mins_passed >= mins_to_WakeupSync){
		mins_passed = 0;
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SyncedWakeup), CFG_SEQ_Prio_0);
	}
}

static void onwakeuptimeover(void *context){
	wakeuptimeover = true;
}

/* USER CODE END PrFD */
