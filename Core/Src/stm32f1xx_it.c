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
/* Jedna nota kratke melodie. tone = kmitocet [Hz] (0 = pomlka), ms = delka.
   Tabulku vzdy ukoncuje polozka s ms = 0. */
typedef struct {
	uint16_t	tone;
	uint16_t	ms;
} Note;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Rotacni enkoder a tlacitko (vzorkuji se v SysTicku, 1 kHz) --- */
#define ENCODER_STEPS_PER_DETENT	4	/* kvadraturnich prechodu na jeden zub (EC11 = 4) */
#define BUTTON_DEBOUNCE_MS			20	/* stabilnich ms pro potvrzeni stisku */

/* --- Signalizace "teplota dosazena" --- */
/* Melodie zazni az kdyz se teplota udrzi v pasmu +-TEMP_OK_BAND kolem zadane
   po celou dobu TEMP_OK_STABLE_MS. Pouhy prvni prechod pres zadanou teplotu
   nestaci - pri nabehu ji hrot obvykle prestreli a chvili kolem ni kmita. */
#define TEMP_OK_BAND				30	/* +-3.0 C kolem zadane teploty */
#define TEMP_OK_STABLE_MS			2000u	/* jak dlouho musi v pasmu vydrzet */
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

/* Melodie v D moll, obe v pasmu 1175-1760 Hz (jako startovni sekvence).
   Mollova tercie a skok o kvintu davaji ten "symfonicky" nadech.

   Ready:   D - A - F - A   (1 - 5 - b3 - 5) ... skok nahoru, konec na kvinte,
            drzeny ton pusobi otevrene a povzbudive = pripraveno k praci
   Standby: A - F - E - D   (5 - b3 - 2 - 1) ... klesava kadence rozvedena
            do zakladniho tonu = uzavrene, usinajici                        */
static const Note MelodyReady[] = {
	{ TONE_D6, 90 }, { 0, 20 }, { TONE_A6, 90 }, { 0, 20 },
	{ TONE_F6, 90 }, { 0, 20 }, { TONE_A6, 220 }, { 0, 0 }
};
static const Note MelodyStandby[] = {
	{ TONE_A6, 90 }, { 0, 20 }, { TONE_F6, 90 }, { 0, 20 },
	{ TONE_E6, 90 }, { 0, 20 }, { TONE_D6, 220 }, { 0, 0 }
};
static const Note *	MelodyPtr	= 0;	/* prave prehravana nota, 0 = ticho */

static uint8_t	TempOkArmed		= 1;	/* 1 = melodie "teplota dosazena" jeste zazni */
static uint16_t	TempOkCount		= 0;	/* po sobe jdouci cykly v pasmu kolem zadane teploty */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Encoder_Step(int8_t dir);
static void Button_Press(void);
static void Inputs_Poll(void);
static void Control_Loop(void);
static void Melody_Start(const Note *melody);
static void Melody_Poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * Prehravac kratkych melodii                        * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Spusti melodii. Nesmi blokovat - vola se z preruseni, takze se jen zapamatuje
   ukazatel a o postup se stara Melody_Poll(). */
static void Melody_Start(const Note *melody)
{
	MelodyPtr = melody;
}

/* Posouva melodii po notach. Vola se kazdou milisekundu ze SysTicku.
   Beep() je neblokujici (delku tonu odpocitava TimerBeep), takze staci pockat,
   az predchozi nota dohraje, a spustit dalsi. */
static void Melody_Poll(void)
{
	if(MelodyPtr == 0)		return;					// nic se nehraje
	if(TimerBeep > 0)		return;					// predchozi nota jeste zni
	if(MelodyPtr->ms == 0) {							// konec tabulky
		MelodyPtr = 0;
		return;
	}
	if(MelodyPtr->tone == 0)	TimerBeep = MelodyPtr->ms;			// pomlka - jen se odpocita cas
	else						Beep( MelodyPtr->tone, MelodyPtr->ms );
	MelodyPtr++;
}

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
			TimerStandby = 0;												// zasah uzivatele = pero se pouziva
			break;
		}
		case 1: {
			if (Menu < 11) Menu++;											// chodi v MENU (0..11)
			else Menu = 0;
			break;
		}
		case 0x11: {														// meni hodnoty promennych v MENU
			switch(Menu) {
			case 0: { if (TemperatureMin < 4000) TemperatureMin++; break; }
			case 1: { if (TemperatureMax < 4000) TemperatureMax++; break; }
			case 2: { if (TemperatureStart < 4000) TemperatureStart++; break; }
			case 3: { if (StandbyTemperature < 4000) StandbyTemperature++; break; }
			case 4: { if (StandbyDifference < 500) StandbyDifference++; break; }
			case 5: { if (StandbyTimerMax < 500) StandbyTimerMax++; break; }
			case 6: { if (RegKp < 5000) RegKp++; break; }
			case 7: { if (RegKi < 5000) RegKi++; break; }
			}
			break;
		}
		}
	} else {
		switch(System) {
		case 0: {
			if (TemperatureSet > TemperatureMin) TemperatureSet -= 10;
			TimerStandby = 0;												// zasah uzivatele = pero se pouziva
			break;
		}
		case 1: {
			if (Menu > 0) Menu--;
			else Menu = 11;
			break;
		}
		case 0x11: {
			switch(Menu) {
			case 0: { if (TemperatureMin > 400) TemperatureMin--; break; }
			case 1: { if (TemperatureMax > 1000) TemperatureMax--; break; }
			case 2: { if (TemperatureStart > 400) TemperatureStart--; break; }
			case 3: { if (StandbyTemperature > 200) StandbyTemperature--; break; }
			case 4: { if (StandbyDifference > 0) StandbyDifference--; break; }
			case 5: { if (StandbyTimerMax > 0) StandbyTimerMax--; break; }
			case 6: { if (RegKp > 0) RegKp--; break; }
			case 7: { if (RegKi > 0) RegKi--; break; }
			}
			break;
		}
		}
	}
	Beep( TONE_CLICK, 10 );
}

/* Stisk tlacitka enkoderu (uz odruseny, vola se jen jednou za stisk). */
static void Button_Press(void)
{
	if(Fault != 0) {							// stisk knofliku nejprve potvrdi/smaze aktivni poruchu
		Fault = 0;
		HeaterFaultSec = 0;
		RegIntegral = 0;						// zabrani prekmitu po obnoveni regulace
		Beep( TONE_ACK, 20 );
	} else if(TimerStart > 3000) {				// prvni 3 s po startu se stisk ignoruje
		switch(System) {
		case 0: {
			System = 1;							// z normalniho rezimu do MENU
			break;
		}
		case 1: {
			if(Menu==11) System = 2;			// posledni polozka = konec MENU a ulozeni do EEPROM
			if(Menu < 8) System = 0x11;			// polozky 0-7 jdou editovat
			break;
		}
		case 2: {
			break;								// z rezimu ukladani do EEPROM vyjde sam do normalniho rezimu
		}
		case 3: {								// navrat ze Standby
			System = 0;
			TemperatureSet = TemperatureSetStandby;		// obnovi pracovni teplotu
			TimerStandby = 0;
			TempOkArmed  = 1;						// po nahrati zpet zase ohlas dosazeni teploty
			TempOkCount  = 0;
			break;
		}
		case 0x11: {							// konec editace hodnoty, zpet do MENU
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

   Meritka konstant (vystup HeaterPower je v desetinach procenta, 0..1000):

     P slozka = odchylka * RegKp / 100
                RegKp = 500 -> plny vykon uz pri odchylce 20 C
                pasmo proporcionality [0.1 C] = 100000 / RegKp

     I slozka = integral * RegKi * PID_PERIOD_MS / 250000
                integral se scita o odchylku kazdy beh smycky
                integracni casova konstanta Ti [s] = RegKp * 2.5 / RegKi

   Nasobeni periodou a deleni 250000 se navzajem vykrati, takze meritko konstant
   NEZAVISI na PID_PERIOD_MS - zmena periody smycky uz nerozladi regulator. */
static void Control_Loop(void)
{
	static uint32_t	adcLast    = 0;
	static uint16_t	runawayCnt = 0;
	static uint32_t	runawayRef = 0;
	int32_t	error, p, i, out, integ, intMax;
	int32_t	tmcu;
	int32_t	kp = (int32_t)RegKp;
	int32_t	ki = (int32_t)RegKi;

	/* --- 1. prepocet namerene teploty --- */
	TemperatureMeasure = (((uint32_t)AdResult0 + 82u) * 1380u) >> 10;	// 28.6.2024 kalibrovano teplomerem, odchylka do 5 stupnu

	if(AdResult3 > 0) {													// AdResult3 = VREFINT, deli se jim
		/* --- 1b. teplota cipu z interniho cidla MCU --- */
		/* RM0008:  T[C] = (V25 - Vsense) / Avg_Slope + 25,  V25 = 1.43 V, sklon 4.3 mV/C.
		   Vsense se dopocita pomerove pres VREFINT (1.20 V), takze vysledek nezavisi
		   na presne hodnote VDDA:   Vsense = 1.20 * AdResult2 / AdResult3
		   Po dosazeni a prevodu na 0.1 C:  T = 3576 - 2791 * AdResult2 / AdResult3

		   POZOR: je to teplota KRISTALU, ne okoli - uvnitr skrinky byva o 10-20 C vic.
		   Navic V25 i sklon maji u F103 velky vyrobni rozptyl a cip nema tovarni
		   kalibraci, takze absolutne to muze byt dost vedle. Offset se doladi zmenou
		   konstanty 3576 (o kolik desetin stupne pricist, o tolik ji zvys). */
		tmcu = 3576 - (2791 * (int32_t)AdResult2) / (int32_t)AdResult3;
		TemperatureIn = (tmcu > 0) ? (uint32_t)tmcu : 0u;

		/* --- 1c. napajeci napeti z delice R15/R16 (100k / 10k) --- */
		/* Delic deli 11x:  U_pin = U_nap * 10k / (100k + 10k).
		   Napeti na pinu se opet urci pomerove pres VREFINT (1.20 V):
		     U_pin [V] = 1.20 * AdResult1 / AdResult3
		   Po vynasobeni 11 (delic) a 100 (prevod na setiny voltu):
		     U_nap [0.01 V] = 1320 * AdResult1 / AdResult3          */
		SupplyVoltage = (uint16_t)((1320u * (uint32_t)AdResult1) / (uint32_t)AdResult3);
	}

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

	/* Standby: pokles teploty o vic nez StandbyDifference pod nastavenou znamena,
	   ze se perem pracuje (spoj odebira teplo) - odpocet do Standby zacina znovu.
	   Kontroluje se tady, aby se zachytil i kratky pokles pri pajeni. */
	if(error > (int32_t)StandbyDifference) {
		TimerStandby = 0;
	}

	integ = RegIntegral;

	p   = error * kp / 100;
	i   = (ki > 0) ? (integ * ki * (int32_t)PID_PERIOD_MS / 250000L) : 0;
	out = p + i;

	/* Anti-windup podminenou integraci: integruje se jen tehdy, kdyz vystup
	   nesaturuje, nebo kdyz by integrace saturaci naopak zmensovala. Pri
	   studenem startu (velka odchylka, vystup na 100 %) integral nenaroste,
	   takze po dosazeni teploty nezpusobi prekmit. */
	if(ki > 0) {
		if( ((out > 0) && (out < 1000))     ||
		    ((out <= 0)   && (error > 0))   ||
		    ((out >= 1000) && (error < 0)) ) {
			intMax = 1000L * 250000L / (ki * (int32_t)PID_PERIOD_MS);	// aby I slozka sama nepresahla 100 %
			integ += error;
			if(integ >  intMax) integ =  intMax;						// tvrdy strop integralu
			if(integ < -intMax) integ = -intMax;
			i   = integ * ki * (int32_t)PID_PERIOD_MS / 250000L;
			out = p + i;
		}
	} else {
		integ = 0;														// RegKi = 0 -> cisty P regulator
	}
	RegIntegral = integ;

	if(out < 0)    out = 0;												// omezeni vystupu
	if(out > 1000) out = 1000;
	HeaterPower = (int16_t)out;

	/* RegError se jen vypisuje v MENU a LCD_PrintNumber neumi zaporna cisla,
	   proto se orizne na nulu. */
	RegError = (error > 0) ? (int16_t)error : 0;

	/* --- 4c. signalizace "teplota spolehlive dosazena" --- */
	/* Nestaci prvni prechod pres zadanou teplotu - pri nabehu ji hrot prestreli
	   a chvili kolem ni kmita. Melodie proto zazni az kdyz odchylka vydrzi
	   v pasmu +-TEMP_OK_BAND po celou dobu TEMP_OK_STABLE_MS.
	   Ohlasi se jen JEDNOU: po zapnuti (TempOkArmed = 1) a po probuzeni ze
	   Standby (tam se znovu nastavi v Button_Press). Ve Standby se nehlida,
	   aby melodie nezazněla pri klesnuti na klidovou teplotu. */
	if(TempOkArmed && (Fault == 0) && (System != 3)) {
		int32_t absErr = (error < 0) ? -error : error;
		if(absErr <= TEMP_OK_BAND) {
			if(++TempOkCount >= (TEMP_OK_STABLE_MS / PID_PERIOD_MS)) {
				TempOkArmed = 0;
				TempOkCount = 0;
				Melody_Start( MelodyReady );
			}
		} else {
			TempOkCount = 0;											// vypadla z pasma - pocita se znovu
		}
	}

	/* --- 4b. detekce "topi, i kdyz nema" --- */
	/* Pri nulovem vykonu nesmi teplota vyrazne stoupat. Chrani proti pripadu,
	   kdy PWM neovlada MOSFET (nepovoleny kanal casovace, prosly BUZ11 nakratko)
	   - tedy proti tepelnemu ujeti, ktere by jinak nic nezachytilo. Prah je
	   zamerne velkorysy, aby ho nespustil bezny prekmit po dohrati. */
	if(HeaterPower == 0) {
		if(++runawayCnt >= (4000u / PID_PERIOD_MS)) {						// okno 4 s
			runawayCnt = 0;
			if(TemperatureMeasure > (runawayRef + 200u)) {					// +20 C bez topeni
				Fault |= FAULT_RUNAWAY;
			}
			runawayRef = TemperatureMeasure;
		}
	} else {
		runawayCnt = 0;
		runawayRef = TemperatureMeasure;
	}

	/* --- 5. vystup na topeni --- */
	if(Fault == 0) LL_TIM_OC_SetCompareCH1(TIM2, 1000 - HeaterPower);	// normalni regulace
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
	Melody_Poll();																				// posune melodii na dalsi notu

	if(TimerSec == 0) {																			// jednou za sekundu (pri preteceni)
		/* * * * * * * * * * * * * * * * * * * * * * * * * * */
		/* * * kontrola zdravi topeni                    * * */
		/* * * * * * * * * * * * * * * * * * * * * * * * * * */
		/* Plny vykon dlouho bez narustu teploty = porouchane topne teleso nebo
		   termoclanek odpadly od hrotu. Priznak FAULT_HEATER pak nastavi regulace. */
		if((HeaterPower > 900) && (TemperatureMeasure < 800)) {									// >90 % vykonu, ale < 80.0 C
			if(HeaterFaultSec < 0xFFFF) HeaterFaultSec++;
		} else {
			HeaterFaultSec = 0;
		}

		/* * * * * * * * * * * * * * * * * * * * * * * * * * */
		/* * * Standby - pero lezi ve stojanku           * * */
		/* * * * * * * * * * * * * * * * * * * * * * * * * * */
		/* Kdyz po dobu StandbyTimerMax sekund teplota ani jednou neklesne o
		   StandbyDifference (tj. nikdo nepajel), snizi se teplota na
		   StandbyTemperature. Zpet do prace se prejde stiskem knofliku.
		   Odpocet nuluje Control_Loop pri kazdem poklesu; StandbyTimerMax = 0
		   funkci vypina. */
		if((System == 0) && (Fault == 0) && (StandbyTimerMax > 0) &&
		   (StandbyTemperature < TemperatureSet)) {											// jinak by Standby teplotu zvysoval
			if(TimerStandby < 0xFFFF) TimerStandby++;
			if(TimerStandby >= StandbyTimerMax) {
				TemperatureSetStandby = TemperatureSet;											// zaloha pracovni teploty
				TemperatureSet        = StandbyTemperature;										// snizit na klidovou
				TimerStandby          = 0;
				System                = 3;														// obrazovka Standby
				Melody_Start( MelodyStandby );													// klesava melodie = usina
			}
		} else if(System != 3) {
			TimerStandby = 0;																	// v MENU a pri poruse se necita
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
  /* Klouzavy prumer 64 vzorku ze ctyr kanalu ADC. DMA plni AdResult[] po 32bit
     slovech, proto se cte kazdy druhy prvek uint16 pole (0, 2, 4, 6). */
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
  AdResult0 = (uint16_t) (MeanAdd0 >> 6);		// termoclanek
  AdResult1 = (uint16_t) (MeanAdd1 >> 6);		// delic napajeni R15/R16
  AdResult2 = (uint16_t) (MeanAdd2 >> 6);		// interni teplomer MCU
  AdResult3 = (uint16_t) (MeanAdd3 >> 6);		// VREFINT

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
