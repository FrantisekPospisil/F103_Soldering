/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern volatile uint8_t		System;
extern volatile uint8_t		Menu;

extern volatile uint16_t		TimerBeep;
extern volatile uint16_t		TimerStart;
extern volatile uint16_t		TimerSec;

extern volatile uint32_t		TemperatureIn;
extern volatile uint32_t		TemperatureOut;
extern uint8_t		MeanPointer;
extern volatile uint16_t		MeanMax;
extern volatile uint32_t		TemperatureMeasure;

extern uint16_t		AdResult[8];
extern uint16_t		AdArray0[65];
extern uint16_t		AdArray1[65];
extern uint16_t		AdArray2[65];
extern uint16_t		AdArray3[65];
extern uint32_t		MeanAdd0;
extern uint32_t		MeanAdd1;
extern uint32_t		MeanAdd2;
extern uint32_t		MeanAdd3;
extern volatile uint16_t		AdResult0;
extern volatile uint16_t		AdResult1;
extern volatile uint16_t		AdResult2;
extern volatile uint16_t		AdResult3;

extern volatile uint16_t		TemperatureMin;
extern volatile uint16_t		TemperatureMax;
extern volatile uint16_t		TemperatureSet0;
extern volatile uint16_t		TemperatureSet;
extern volatile uint16_t		TemperatureSetStandby;

extern volatile uint16_t		StandbyTemperature;
extern volatile uint16_t		StandbyDifference;
extern volatile uint16_t		StandbyTimerMax;
extern volatile uint16_t		TimerStandby;

extern volatile uint16_t		Const1;
extern volatile uint16_t		Const2;
extern volatile int16_t		Difference;
extern volatile int32_t		DifferenceIntegral;
extern volatile int16_t		SolderPWM;

extern volatile uint16_t		CounterTemperature;

extern uint8_t		EepromBuffer[30];
extern uint8_t		EepromStatus;

/* --- Bezpecnost / watchdog (bod 1, P0) --- */
extern volatile uint8_t		Fault;			// bitove pole aktivnich poruch (FAULT_*), 0 = OK
extern volatile uint32_t	PidHeartbeat;	// inkrementuje regulacni smycka (SysTick) - dukaz, ze zije
extern volatile uint16_t	HeaterFaultSec;	// pocet sekund plneho vykonu bez narustu teploty
extern volatile uint32_t	AdcTicks;		// inkrementuje akvizicni (DMA) preruseni - dukaz, ze prevodnik bezi

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void Beep( uint32_t Tone, uint16_t Time);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* ---- Bezpecnostni limity (teplota v 0.1 C, tj. 4000 = 400.0 C) ---- */
#define TEMP_ABS_MAX_C01   4500u   /* tvrdy strop 450.0 C, nezavisly na nastavene teplote */
#define FAULT_ADC_OPEN     4000u   /* ADC u plneho rozsahu => odpojeny termoclanek / pero */
#define HEATER_FAULT_SEC   30u     /* sekund plneho vykonu bez ohrevu => porucha topeni/cidla */
#define PID_PERIOD_MS      25u     /* perioda regulacni smycky [ms] (40 Hz) */

/* ---- Bitove priznaky poruch (promenna Fault) ---- */
#define FAULT_NO_IRON      0x01u   /* odpojene pero / rozpojeny termoclanek */
#define FAULT_OVERTEMP     0x02u   /* prekrocen tvrdy teplotni strop */
#define FAULT_HEATER       0x04u   /* topi naplno, ale teplota neroste */
#define FAULT_ADC          0x08u   /* zastavil se AD prevodnik / DMA (stara data) */

/* ---- Tony bzucaku [Hz] ----
   BZ1 je pasivni piezomenic (R17 1k paralelne s nim). Piezo ma ostrou rezonanci
   zhruba 3-5 kHz a mimo ni je vyrazne tissi az neslysitelne. Tony jsou proto
   voleny nahoru; hodnoty odpovidaji tomu, co stanice skutecne hrala pred
   opravou prepoctu v Beep(). Pro hlasitejsi zvuk jdi blize k 4 kHz. */
#define TONE_START1        1100u   /* startovni melodie - oktavy 1100/2200/4400 */
#define TONE_START2        2200u
#define TONE_START3        4400u
#define TONE_CLICK         5100u   /* cvaknuti enkoderu a tlacitka */
#define TONE_ERROR          500u   /* chyba EEPROM - nizky ton (na piezu je slabsi) */
#define TONE_FAULT         4000u   /* opakovany alarm pri poruse */
#define TONE_ACK           3000u   /* potvrzeni poruchy tlacitkem */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* Okamzite vypnuti topeni. Budic Q3 invertuje, takze plna hodnota compare
   (= ARR 1000) drzi gate BUZ11 dole => MOSFET rozepnuty. Stejnou cestou ridi
   vykon i PID, takze jde o overene "vypnuto" (stav v klidu pri SolderPWM = 0). */
#define HEATER_OFF()       LL_TIM_OC_SetCompareCH1(TIM2, 1000)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin LL_GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define D4_Pin LL_GPIO_PIN_0
#define D4_GPIO_Port GPIOB
#define D5_Pin LL_GPIO_PIN_1
#define D5_GPIO_Port GPIOB
#define D6_Pin LL_GPIO_PIN_2
#define D6_GPIO_Port GPIOB
#define Coder1_Pin LL_GPIO_PIN_13
#define Coder1_GPIO_Port GPIOB
#define Coder0_Pin LL_GPIO_PIN_14
#define Coder0_GPIO_Port GPIOB
#define Coder0_EXTI_IRQn EXTI15_10_IRQn
#define Button_Pin LL_GPIO_PIN_15
#define Button_GPIO_Port GPIOB
#define Button_EXTI_IRQn EXTI15_10_IRQn
#define Buzzer_Pin LL_GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define D7_Pin LL_GPIO_PIN_3
#define D7_GPIO_Port GPIOB
#define LCD_RS_Pin LL_GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOB
#define LCD_E_Pin LL_GPIO_PIN_5
#define LCD_E_GPIO_Port GPIOB
#define LCD_LED_Pin LL_GPIO_PIN_6
#define LCD_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
