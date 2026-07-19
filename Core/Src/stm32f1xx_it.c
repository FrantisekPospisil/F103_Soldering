/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Rotacni enkoder a tlacitko (vzorkuji se v SysTicku, 1 kHz) --- */
#define ENCODER_STEPS_PER_DETENT	4	/* kvadraturnich prechodu na jeden zub (EC11 = 4) */
#define BUTTON_DEBOUNCE_MS			20	/* stabilnich ms pro potvrzeni stisku */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Kvadraturni dekoder enkoderu.
   Stav vodicu = (A << 1) | B, kde A = Coder1 (PB13) a B = Coder0 (PB14).
   Index do tabulky = (predchozi stav << 2) | novy stav.
   Tabulka vraci +1 / -1 jen pro platny prechod Grayova kodu; pro neplatny
   prechod (zmenily se oba vodice najednou = zakmit) vraci 0. Zakmity tam a
   zpet se navic v akumulatoru samy odectou, takze zub nepreskoci ani neujede. */
static const int8_t EncoderTable[16] = {
	 0, +1, -1,  0,
	-1,  0,  0, +1,
	+1,  0,  0, -1,
	 0, -1, +1,  0
};
static uint8_t	EncoderPrev		= 0;	/* posledni platny stav vodicu */
static int8_t	EncoderAccum	= 0;	/* nasbirane prechody v ramci jednoho zubu */
static uint8_t	ButtonState		= 1;	/* potvrzeny stav tlacitka (1 = nestisknuto, pull-up) */
static uint8_t	ButtonCount		= 0;	/* pocitadlo stabilnich vzorku */
static uint8_t	PidDivider		= 0;	/* delicka SysTicku na periodu regulacni smycky */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Encoder_Step(int8_t dir);
static void Button_Press(void);
static void Inputs_Poll(void);
static void Control_Loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * Obsluha rotacniho enkoderu a tlacitka             * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Jeden zub enkoderu. dir = +1 doprava, -1 doleva. */
static void Encoder_Step(int8_t dir)
{
	if(dir > 0) {
		switch(System) {
		case 0: {
			if (TemperatureSet < TemperatureMax) TemperatureSet += 10;		// pridava teplotu v normalnim rezimu
			CounterTemperature = 0;
			break;
		}
		case 1: {
			if (Menu < 13) Menu++;											// chodi v MENU
			else Menu = 0;
			break;
		}
		case 0x11: {														// meni hodnoty promennych v MENU
			switch(Menu) {
			case 0: { if (TemperatureMin < 4000) TemperatureMin++; break; }
			case 1: { if (TemperatureMax < 4000) TemperatureMax++; break; }
			case 2: { if (TemperatureSet0 < 4000) TemperatureSet0++; break; }
			case 3: { if (StandbyTemperature < 4000) StandbyTemperature++; break; }
			case 4: { if (StandbyDifference < 500) StandbyDifference++; break; }
			case 5: { if (StandbyTimerMax < 500) StandbyTimerMax++; break; }
			case 6: { if (MeanMax < 100) MeanMax++; break; }
			case 7: { if (Const1 < 5000) Const1++; break; }
			case 8: { if (Const2 < 5000) Const2++; break; }
			}
			break;
		}
		}
	} else {
		switch(System) {
		case 0: {
			if (TemperatureSet > TemperatureMin) TemperatureSet -= 10;
			CounterTemperature = 0;
			break;
		}
		case 1: {
			if (Menu > 0) Menu--;
			else Menu = 12;
			break;
		}
		case 0x11: {
			switch(Menu) {
			case 0: { if (TemperatureMin > 400) TemperatureMin--; break; }
			case 1: { if (TemperatureMax > 1000) TemperatureMax--; break; }
			case 2: { if (TemperatureSet0 > 400) TemperatureSet0--; break; }
			case 3: { if (StandbyTemperature > 200) StandbyTemperature--; break; }
			case 4: { if (StandbyDifference > 0) StandbyDifference--; break; }
			case 5: { if (StandbyTimerMax > 10) StandbyTimerMax--; break; }
			case 6: { if (MeanMax > 4) MeanMax--; break; }
			case 7: { if (Const1 > 0) Const1--; break; }
			case 8: { if (Const2 > 0) Const2--; break; }
			}
			break;
		}
		}
	}
	Beep( TONE_CLICK, 10 );
}

/* Stisk tlacitka enkoderu (uz odrusenny, vola se jen jednou za stisk). */
static void Button_Press(void)
{
	if(Fault != 0) {							// stisk knofliku nejprve potvrdi/smaze aktivni poruchu
		Fault = 0;
		HeaterFaultSec = 0;
		DifferenceIntegral = 0;					// zabrani prekmitu po obnoveni regulace
		Beep( TONE_ACK, 20 );
	} else if(TimerStart > 3000) {
		switch(System) {
		case 0: {
			System = 1;							// pri stisku z normalniho rezimu na rezim MENU
			break;
		}
		case 1: {
			if(Menu==13) System = 2;			// v MENU na konci, polozka 13 jde na rezim ukladani do EEPROM
			if(Menu < 9) System = 0x11;
			break;
		}
		case 2: {
			break;								// z rezimu ukladani do EEPROM vyjde sam do normalniho rezimu
		}
		case 3: {
			System = 0;
			TemperatureSet = TemperatureSetStandby;
			CounterTemperature = 0;
			break;
		}
		case 0x11: {							// rezim zmeny jednolivych hodnot v MENU
			System = 1;
			break;
		}
		}
		Beep( TONE_CLICK, 10 );
	}
}

/* Vzorkovani vstupu - vola se kazdou milisekundu ze SysTicku.
   Vzorkovani je spolehlivejsi nez hranove preruseni: mechanicke kontakty
   zakmitavaji a EXTI by na kazdy zakmit vygenerovalo dalsi krok. */
static void Inputs_Poll(void)
{
	uint8_t	curr;
	uint8_t	raw;
	int8_t	mv;

	/* --- enkoder: kvadraturni dekodovani --- */
	curr  = LL_GPIO_IsInputPinSet(Coder1_GPIO_Port, Coder1_Pin) ? 2u : 0u;		// A
	curr |= LL_GPIO_IsInputPinSet(Coder0_GPIO_Port, Coder0_Pin) ? 1u : 0u;		// B
	if(curr != EncoderPrev) {
		mv = EncoderTable[(EncoderPrev << 2) | curr];
		EncoderPrev = curr;
		if(mv != 0) {															// 0 = neplatny prechod (zakmit) -> ignoruj
			EncoderAccum += mv;
			if(EncoderAccum >= ENCODER_STEPS_PER_DETENT) {
				EncoderAccum = 0;
				Encoder_Step( +1 );
			} else if(EncoderAccum <= -ENCODER_STEPS_PER_DETENT) {
				EncoderAccum = 0;
				Encoder_Step( -1 );
			}
		}
	}

	/* --- tlacitko: potvrzeni az po BUTTON_DEBOUNCE_MS stabilnich vzorcich --- */
	raw = LL_GPIO_IsInputPinSet(Button_GPIO_Port, Button_Pin) ? 1u : 0u;
	if(raw != ButtonState) {
		if(++ButtonCount >= BUTTON_DEBOUNCE_MS) {
			ButtonCount = 0;
			ButtonState = raw;
			if(raw == 0u) Button_Press();										// sestupna hrana = stisk (spina na GND)
		}
	} else {
		ButtonCount = 0;
	}
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * Regulacni smycka (kazdych PID_PERIOD_MS)          * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Diskretni PI regulator s podminenou integraci (anti-windup).
   Drive bezel v DMA preruseni na ~8 kHz, coz je pro tepelnou soustavu s
   casovou konstantou v sekundach nesmyslne rychle - regulace jen reagovala na
   sum. Nyni se vola ze SysTicku po PID_PERIOD_MS.

   Meritka konstant:
     P slozka = odchylka * Const1 / 100        (Const1 = 500 -> 1000 pri 20 C odchylce)
     I slozka = integral  * Const2 / 10000     (integral se scita po PID_PERIOD_MS)
   Vystupem je SolderPWM v desetinach procenta (0..1000). */
static void Control_Loop(void)
{
	static uint32_t	adcLast = 0;
	int32_t	error, p, i, out, integ, intMax;
	int32_t	kp = (int32_t)Const1;
	int32_t	ki = (int32_t)Const2;

	/* --- 1. prepocet namerene teploty --- */
	TemperatureMeasure = (((uint32_t)AdResult0 + 82u) * 1380u) >> 10;	// 28.6.2024 kalibrovano teplomerem, odchylka do 5 stupnu

	/* --- 2. kontrola, ze prevodnik dodava nova data --- */
	/* Bez toho by se pri zaseknutem ADC/DMA regulovalo podle stare hodnoty. */
	if(AdcTicks == adcLast) Fault |=  FAULT_ADC;
	else                    Fault &= ~FAULT_ADC;
	adcLast = AdcTicks;

	/* --- 3. detekce poruch (fail-safe smer = vypnout topeni) --- */
	/* Odpojene pero / rozpojeny termoclanek: HW (R4 1M na +5V) vytahne vystup
	   zesilovace na plny rozsah, takze AdResult0 jde k maximu. */
	if(AdResult0 > FAULT_ADC_OPEN)            Fault |=  FAULT_NO_IRON;
	else                                      Fault &= ~FAULT_NO_IRON;
	/* Tvrdy strop teploty, nezavisly na nastavene teplote (s malou hysterezi). */
	if(TemperatureMeasure > TEMP_ABS_MAX_C01) Fault |=  FAULT_OVERTEMP;
	else if(TemperatureMeasure < (TEMP_ABS_MAX_C01 - 100u)) Fault &= ~FAULT_OVERTEMP;
	/* Topi naplno dlouho bez ohrevu (odpocet sekund probiha v SysTicku). */
	if(HeaterFaultSec >= HEATER_FAULT_SEC)    Fault |=  FAULT_HEATER;

	/* --- 4. PI regulator --- */
	error = (int32_t)TemperatureSet - (int32_t)TemperatureMeasure;		// odchylka [0.1 C]
	integ = DifferenceIntegral;

	p   = error * kp / 100;
	i   = (ki > 0) ? (integ * ki / 10000) : 0;
	out = p + i;

	/* Anti-windup podminenou integraci: integruje se jen tehdy, kdyz vystup
	   nesaturuje, nebo kdyz by integrace saturaci naopak zmensovala. Pri
	   studenem startu (velka odchylka, vystup na 100 %) integral nenaroste,
	   takze po dosazeni teploty nezpusobi prekmit. */
	if(ki > 0) {
		if( ((out > 0) && (out < 1000))     ||
		    ((out <= 0)   && (error > 0))   ||
		    ((out >= 1000) && (error < 0)) ) {
			intMax = 1000L * 10000L / ki;								// aby I slozka sama nepresahla 100 %
			integ += error;
			if(integ >  intMax) integ =  intMax;						// tvrdy strop integralu
			if(integ < -intMax) integ = -intMax;
			i   = integ * ki / 10000;
			out = p + i;
		}
	} else {
		integ = 0;														// bez integracni slozky
	}
	DifferenceIntegral = integ;

	if(out < 0)    out = 0;												// omezeni vystupu
	if(out > 1000) out = 1000;
	SolderPWM = (int16_t)out;

	/* Difference se jen vypisuje v MENU a LCD_PrintNumber neumi zaporna cisla,
	   proto se stejne jako driv orizne na nulu. */
	Difference = (error > 0) ? (int16_t)error : 0;

	/* --- 5. vystup na topeni --- */
	if(Fault == 0) LL_TIM_OC_SetCompareCH1(TIM2, 1000 - SolderPWM);		// normalni regulace
	else           HEATER_OFF();										// porucha => topeni vypnuto

	PidHeartbeat++;														// "tep" pro watchdog - regulace zije
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  HEATER_OFF();		/* fail-safe: vypni topeni; IWDG vzapeti resetuje MCU */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if(TimerStart<10000) TimerStart++;															// pocita milisekundy od startu
	if(TimerSec<1000) TimerSec++; else TimerSec = 0;											// pocita sekundy

	/* * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* * * program ovladani piezomenice              * * */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * */


	if(TimerBeep>0) {																			// pocita milisekundy kdy ma bezet PWM pro piezomenic
		TimerBeep--;
		if(TimerBeep==0) {
			LL_TIM_DisableCounter(TIM1);
			LL_TIM_DisableAllOutputs(TIM1);														// konec tonu, zastavi se timer
		}
	}

	/* * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* * * kontrola zdravi topeni (pocita sekundy)   * * */
	/* * * * * * * * * * * * * * * * * * * * * * * * * * */
	/* Plny vykon dlouho bez narustu teploty = porouchane topne teleso nebo
	   termoclanek odpadly od hrotu. Priznak FAULT_HEATER pak nastavi regulace. */
	if(TimerSec == 0) {																			// jednou za sekundu (pri preteceni)
		if((SolderPWM > 900) && (TemperatureMeasure < 800)) {									// >90 % vykonu, ale < 80.0 C
			if(HeaterFaultSec < 0xFFFF) HeaterFaultSec++;
		} else {
			HeaterFaultSec = 0;
		}
	}

	Inputs_Poll();																				// vzorkovani enkoderu a tlacitka (1 kHz)

	if(++PidDivider >= PID_PERIOD_MS) {															// regulacni smycka
		PidDivider = 0;
		Control_Loop();
	}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  AdArray0[MeanPointer] = AdResult[0];
  AdArray1[MeanPointer] = AdResult[2];
  AdArray2[MeanPointer] = AdResult[4];
  AdArray3[MeanPointer] = AdResult[6];

  MeanAdd0 += AdArray0[MeanPointer];
  MeanAdd1 += AdArray1[MeanPointer];
  MeanAdd2 += AdArray2[MeanPointer];
  MeanAdd3 += AdArray3[MeanPointer];
  if(MeanPointer < 64) {
	  MeanPointer++;
  } else {
	  MeanPointer = 0;
  }
  MeanAdd0 -= AdArray0[MeanPointer];
  MeanAdd1 -= AdArray1[MeanPointer];
  MeanAdd2 -= AdArray2[MeanPointer];
  MeanAdd3 -= AdArray3[MeanPointer];
  AdResult0 = (uint16_t) (MeanAdd0 >> 6);
  AdResult1 = (uint16_t) (MeanAdd1 >> 6);
  AdResult2 = (uint16_t) (MeanAdd2 >> 6);
  AdResult3 = (uint16_t) (MeanAdd3 >> 6);

  /* Toto preruseni uz jen sbira data (~8 kHz). Prepocet teploty, detekce poruch
     i PI regulator se presunuly do Control_Loop() volane ze SysTicku po
     PID_PERIOD_MS - tepelna soustava je pomala a regulace na 8 kHz jen reagovala
     na sum a zbytecne zatezovala procesor. */
  AdcTicks++;                               // dukaz, ze prevodnik a DMA bezi (hlida Control_Loop)

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14 */
	/* Otaceni enkoderu se nove vzorkuje v SysTicku (Inputs_Poll) - kvadraturni
	   dekoder je odolny proti zakmitum mechanickych kontaktu. Hranove preruseni
	   se pro vstupy uz nepouziva, je zamaskovane v main.c. */
    /* USER CODE END LL_EXTI_LINE_14 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    /* USER CODE BEGIN LL_EXTI_LINE_15 */
	/* Tlacitko se nove vzorkuje v SysTicku (Inputs_Poll) vcetne debounce. */
    /* USER CODE END LL_EXTI_LINE_15 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
