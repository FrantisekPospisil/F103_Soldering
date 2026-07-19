/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
#include "HD44780.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Takt citace TIM1: APB2 = 64 MHz, prescaler 24 => 64 MHz / 25 = 2.56 MHz.
   Musi odpovidat TIM_InitStruct.Prescaler v MX_TIM1_Init (tim.c) a nastaveni
   APB2 v SystemClock_Config. Pouziva se pro prepocet kmitoctu v Beep(). */
#define TIM1_TICK_HZ	2560000u

/* --- Rozlozeni bloku nastaveni v EEPROM ---
   [ 0- 1] TemperatureMin      [ 8- 9] StandbyDifference
   [ 2- 3] TemperatureMax      [10-11] StandbyTimerMax
   [ 4- 5] TemperatureStart    [12-13] RegKp
   [ 6- 7] StandbyTemperature  [14-15] RegKi        [16-17] magicka znacka
   POZOR: pri KAZDE zmene poradi nebo poctu polozek zmen EEPROM_MAGIC1, jinak by
   se stara data s jinym rozlozenim nacetla jako platna a nastaveni by se prehazelo. */
#define EEPROM_ADDR		0x0010u		/* adresa bloku nastaveni v EEPROM */
#define EEPROM_LEN		18u			/* delka bloku vcetne magicke znacky */
#define EEPROM_MAGIC0	0xA5u
#define EEPROM_MAGIC1	0x63u		/* v2 - bez nepouzivaneho MeanMax */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t		System				= 0;		// ridi zobrazeni na displeji 	0 - normalni,
											//								1 - MENU - prechazi tlacitkem rot koderu
											//								2 - EEPROM - uklada se do pameti po konci rezimu MENU
											//								3 - Standby - kdyz je dlouho stejna teplota
											//								(poruchy resi samostatna promenna Fault)

volatile uint8_t		Menu				= 0;		// pozice v MENU: 0-7 zmena hodnot, 8-10 jen zobrazeni, 11 = konec a ulozeni

volatile uint16_t	TimerBeep			= 0;		// doba trvani tonu, generovano PWM
volatile uint16_t	TimerStart			= 0;		// pocita milisekundy od startu, zastavi se na 10 sekund
volatile uint16_t	TimerSec			= 0;		// pocita cas v milisekundach, pretece pri dosazeni sekundy v preruseni SYSTICK

volatile uint32_t	TemperatureIn		= 0;		// teplota cipu MCU z interniho cidla [0.1 C]
uint8_t		MeanPointer			= 0;		// ukazatel v kruhovem bufferu prumerovani (jen DMA preruseni)

volatile uint32_t	TemperatureMeasure	= 0;		// namerena teplota hrotu po zprumerovani [0.1 C]
volatile uint16_t	SupplyVoltage		= 0;		// napajeci napeti z delice R15/R16 [0.01 V]

/* Cil DMA. Prevodnik dela 4 prevody za sebou, DMA je uklada po 32bitovych
   slovech, takze v poli uint16 lezi na indexech 0, 2, 4, 6 (liche jsou nulove). */
uint16_t	AdResult[8];					// 0 - termoclanek (PA0)
											// 2 - delic napajeni R15/R16 (PA1)
											// 4 - interni teplomer MCU
											// 6 - VREFINT (1.20 V)
/* Kruhove buffery a mezisoucty klouzaveho prumeru - pouzivaji se VYHRADNE
   v DMA preruseni, proto zamerne nejsou volatile (drzi se tim akvizice rychla). */
uint16_t	AdArray0[65];					// termoclanek
uint16_t	AdArray1[65];					// napajeci napeti
uint16_t	AdArray2[65];					// teplomer MCU
uint16_t	AdArray3[65];					// VREFINT
uint32_t	MeanAdd0;
uint32_t	MeanAdd1;
uint32_t	MeanAdd2;
uint32_t	MeanAdd3;
volatile uint16_t	AdResult0;
volatile uint16_t	AdResult1;
volatile uint16_t	AdResult2;
volatile uint16_t	AdResult3;

volatile uint16_t	TemperatureMin		= 1000;		// EEPROM - minimalni teplota
volatile uint16_t	TemperatureMax		= 4000;		// EEPROM - maximalni teplota
volatile uint16_t	TemperatureStart		= 1400;		// EEPROM - nastavena teplota
volatile uint16_t	TemperatureSet		= 1100;		// nastavena teplota v normalnim rezimu, kopiruje se z TemperatureStart
volatile uint16_t	TemperatureSetStandby = 0;		// zaloha teploty, ktera bude vracena po opusteni Standby rezimu

volatile uint16_t	StandbyTemperature	= 1000;		// EEPROM - pozadovana teplota v Standby rezimu
volatile uint16_t	StandbyDifference	= 80;		// EEPROM - pokles teploty, ktery se jeste povazuje za pajeni
volatile uint16_t	StandbyTimerMax		= 300;		// EEPROM - sekund necinnosti do Standby (0 = vypnuto)
volatile uint16_t	TimerStandby		= 0;		// odpocet necinnosti [s], nuluje se pri kazdem pajeni

volatile uint16_t	RegKp				= 500;		// EEPROM - zesileni P slozky (pasmo proporcionality)
volatile uint16_t	RegKi				= 150;		// EEPROM - zesileni I slozky (rychlost dorovnani odchylky)
volatile int16_t	RegError			= 0;		// PI - regulacni odchylka [0.1 C], jen pro zobrazeni v MENU
volatile int32_t	RegIntegral			= 0;		// PI - akumulator integralni slozky
volatile int16_t	HeaterPower			= 0;		// vypocteny vykon topeni v desetinach procenta (0..1000)

uint8_t		EepromBuffer[30];				// Buffer pro zapis a cteni z EEPROM
uint8_t		EepromStatus = 0;				// stav pameti: EEPROM_STATUS_COMPLETE (1) / _ERROR (2)

/* --- Bezpecnost / watchdog --- */
volatile uint8_t	Fault			= 0;	// bitove pole aktivnich poruch (FAULT_*), 0 = OK
volatile uint32_t	PidHeartbeat	= 0;	// inkrementuje regulacni smycka (SysTick) - dukaz, ze zije
volatile uint16_t	HeaterFaultSec	= 0;	// pocet sekund plneho vykonu bez narustu teploty
volatile uint32_t	AdcTicks		= 0;	// inkrementuje akvizicni (DMA) preruseni - dukaz, ze prevodnik bezi
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Watchdog_Init(void);
static void Watchdog_Refresh(void);
static void Delay_Wd(uint32_t ms);
static void Display_Fault(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LL_SYSTICK_EnableIT();

  /* Enkoder i tlacitko se vzorkuji v SysTicku (Inputs_Poll v stm32f1xx_it.c).
     Hranove preruseni se proto zamaskuje - u mechanickych kontaktu generovalo
     na kazdy zakmit dalsi krok. GPIO konfigurace z CubeMX zustava beze zmeny. */
  LL_EXTI_DisableIT_0_31( LL_EXTI_LINE_14 | LL_EXTI_LINE_15 );

  LL_mDelay( 300 );
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AdResult, 4);

  /* * * * * * * * * * * * * * * * * * * * * * * * * * */
  /* * * Precte hodnoty promennych z EEPROM        * * */
  /* * * * * * * * * * * * * * * * * * * * * * * * * * */

  EEPROM_SPI_INIT(&hspi1);
  EepromStatus = EEPROM_SPI_ReadBuffer(EepromBuffer, EEPROM_ADDR, EEPROM_LEN );
  if((EepromBuffer[16] == EEPROM_MAGIC0) && (EepromBuffer[17] == EEPROM_MAGIC1)){
	  TemperatureMin     = EepromBuffer[ 0] * 256 + EepromBuffer[ 1];
	  TemperatureMax     = EepromBuffer[ 2] * 256 + EepromBuffer[ 3];
	  TemperatureStart   = EepromBuffer[ 4] * 256 + EepromBuffer[ 5];
	  StandbyTemperature = EepromBuffer[ 6] * 256 + EepromBuffer[ 7];
	  StandbyDifference  = EepromBuffer[ 8] * 256 + EepromBuffer[ 9];
	  StandbyTimerMax    = EepromBuffer[10] * 256 + EepromBuffer[11];
	  RegKp              = EepromBuffer[12] * 256 + EepromBuffer[13];
	  RegKi              = EepromBuffer[14] * 256 + EepromBuffer[15];
  }

  TemperatureSet = TemperatureStart;

  LCD_Init();
  LL_GPIO_SetOutputPin(LCD_LED_GPIO_Port, LCD_LED_Pin);
  LL_mDelay( 250 );
  LCD_Init();
  LCD_Position( 0, 0 );
  LCD_WriteCString( " 03_Soldering AI" );
  LCD_Position( 1, 0 );
  LCD_WriteCString( " 20.7.2026      " );
  Beep( TONE_START1, 100 );
  LL_mDelay( 400 );
  Beep( TONE_START2, 100 );
  LL_mDelay( 400 );
  Beep( TONE_START3, 100 );
  LL_mDelay( 400 );
  LCD_Position( 0, 0 );
  LCD_PrHex( EepromBuffer[16] );					// magicka znacka pro kontrolu obsahu EEPROM
  LCD_Write( ' ' );
  LCD_PrHex( EepromBuffer[17] );
  LCD_WriteCString( "           " );
  LCD_Position( 1, 0 );
  if(EepromStatus==1) {
	  LCD_WriteCString( "EEPROM read OK  " );
  } else {
	  LCD_WriteCString( "EEPROM read ERR " );
	  Beep( TONE_ERROR, 500 );
  }
  LL_mDelay( 1000 );

  /* Poradi je zamerne: nejdriv se do stinovych registru dostane bezpecna hodnota
     (topeni VYPNUTO) a teprve pak se vystup kanalu pripoji na pin.

     POZOR: LL_TIM_OC_Init() v MX_TIM2_Init() nastavuje OCState = DISABLE, takze
     nechava CC1E = 0 a casovac by PA15 vubec nebudil. Pin by zustal v log. 0,
     Q3 by nesepnul, R18 by vytahl gate BUZ11 nahoru a topeni by jelo NATRVALO
     NAPLNO bez ohledu na vypocteny vykon. Kanal je proto nutne povolit rucne -
     stejne jako u TIM1 v Beep(). CubeMX tenhle radek pro LL negeneruje.
     (LL_TIM_EnableAllOutputs zde smysl nema - TIM2 je general-purpose, nema
     registr BDTR ani bit MOE, zapis by sel do rezervovane oblasti.) */
  LL_TIM_SetAutoReload(TIM2, 1000);
  HEATER_OFF();                                      // bezpecna hodnota jeste pred povolenim vystupu
  LL_TIM_GenerateEvent_UPDATE(TIM2);                 // ARR i CCR maji preload - prekopiruj je hned
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // CC1E - propoj vystup kanalu na PA15
  LL_TIM_EnableCounter(TIM2);

  /* Spust nezavisly watchdog az po startovni sekvenci s dlouhymi prodlevami.
     Kdyz se zasekne hlavni smycka NEBO regulacni preruseni, IWDG resetuje MCU;
     po resetu zustava topeni vypnute (PA15 -> pull-up -> Q3 sepnut -> gate dole). */
  Watchdog_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Watchdog_Refresh();				// nakrmi IWDG jen pokud regulacni preruseni stale bezi
	  LL_mDelay( 100 );

	  /* Porucha (vadne/odpojene cidlo, prehrati, vadne topeni) ma prednost pred
	     beznym zobrazenim. Topeni uz je vypnute v regulacnim preruseni. */
	  if( Fault != 0 ) {
		  Display_Fault();
		  continue;
	  }

	  switch( System & 0x0F ) {
	  case 0: {									// normalni zobrazeni (odpojene pero resi Display_Fault)
		  LCD_Position( 0, 0 );
		  LCD_WriteCString( "Tsold = " );
		  LCD_PrTemp( (uint16_t) TemperatureMeasure );
		  LCD_WriteCString( " " );
		  LCD_Position( 1, 0 );
		  LCD_PrTemp( TemperatureSet );
		  LCD_WriteCString( "P =" );
		  LCD_PrintNumber( HeaterPower / 10, 3, 0);
		  LCD_WriteCString( "% " );
		  break;
	  }
	  case 1: {									// MENU - polozky 0-7 lze editovat, 11 = konec a ulozeni
		  LCD_Position( 0, 0 );
		  switch( Menu ) {
		  case 0: {
			  LCD_WriteCString( "1 Teplota Tmin  " );
		  	  LCD_Position( 1, 0 );
		  	  LCD_PrTemp( TemperatureMin );				// minimalni teplota, kterou lze nastavit
		  	  LCD_WriteCString( "      " );
		  	  break;
		  }
		  case 1: {
			  LCD_WriteCString( "2 Teplota Tmax  " );
		  	  LCD_Position( 1, 0 );
		  	  LCD_PrTemp( TemperatureMax );				// maximalni teplota, kterou lze nastavit
		  	  LCD_WriteCString( "      " );
		  	  break;
		  }
		  case 2: {
			  LCD_WriteCString( "3 Teplota Tset  " );
			  LCD_Position( 1, 0 );
			  LCD_PrTemp( TemperatureStart );			// pozadovana teplota po zapnuti
			  LCD_WriteCString( "      " );
		  	  break;
		  }
		  case 3: {
		  	  LCD_WriteCString( "4 Teplota Standb" );
		  	  LCD_Position( 1, 0 );
		  	  LCD_PrTemp( StandbyTemperature );			// teplota ve Standby rezimu
		  	  LCD_WriteCString( "      " );
		  	  break;
		  }
		  case 4: {
		  	  LCD_WriteCString( "5 Standby Diff  " );
		  	  LCD_Position( 1, 0 );
		  	  LCD_PrTemp( StandbyDifference );			// pokles teploty, ktery se jeste povazuje za pajeni
		  	  LCD_WriteCString( "       " );
		  	  break;
		  }
		  case 5: {
		  	  LCD_WriteCString( "6 Standby timer " );
		  	  LCD_Position( 1, 0 );
		  	  LCD_PrintNumber( StandbyTimerMax, 4, 0 );	// sekund necinnosti do Standby (0 = vypnuto)
		  	  LCD_WriteCString( " sec      " );
		  	  break;
		  }
		  case 6: {
			  LCD_WriteCString( "7 Zesileni P    " );
			  LCD_Position( 1, 0 );
			  LCD_PrintNumber( RegKp, 4, 0 );			// zesileni proporcionalni slozky
			  LCD_WriteCString( "           " );
			  break;
		  }
		  case 7: {
			  LCD_WriteCString( "8 Zesileni I    " );
			  LCD_Position( 1, 0 );
			  LCD_PrintNumber( RegKi, 4, 0 );			// zesileni integralni slozky
			  LCD_WriteCString( "           " );
			  break;
		  }
		  case 8: {
			  LCD_WriteCString( "9 Odchylka + I  " );
			  LCD_Position( 1, 0 );
			  LCD_PrintNumber( RegError, 4, 0 );		// regulacni odchylka a stav integralu
			  LCD_WriteCString( "  " );
			  LCD_PrintNumber( RegIntegral, 6, 0 );
			  break;
		  }
		  case 9: {
			  LCD_WriteCString( "10 Teplota MCU  " );
			  LCD_Position( 1, 0 );
			  LCD_PrTemp( TemperatureIn );				// teplota cipu z interniho cidla
			  LCD_WriteCString( "       " );
			  break;
		  }
		  case 10: {
			  LCD_WriteCString( "11 Napajeni     " );
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "   " );
			  LCD_PrintNumber( SupplyVoltage, 4, 2 );	// napajeci napeti z delice R15/R16 [0.01 V]
			  LCD_WriteCString( " V      " );
			  break;
		  }
		  case 11: {
			  LCD_WriteCString( "Exit menu       " );	// konec MENU, System pujde na 2, aby se ulozilo do EEPROM
			  LCD_Position( 1, 0 );
			  LCD_WriteCString( "                " );
			  break;
		  }
		  }
		  if ((System == 0x11) && (Menu < 8)) {			// sipka signalizuje rezim zmeny hodnoty
			  LCD_WriteCString( "^" );
		  } else {
		  	  LCD_WriteCString( " " );
		  }
		  break;
	  }
	  case 2: {									// ulozeni nastaveni do EEPROM
		  LCD_Position( 0, 0 );
		  LCD_WriteCString( "   W R I T E    " );
		  LCD_Position( 1, 0 );
		  LCD_WriteCString( "   E E P R O M  " );
		  Delay_Wd( 500 );						// dlouhe cekani UI - prubezne krmi watchdog
		  LCD_Clear();
		  System = 0;
		  EepromBuffer[ 0] = TemperatureMin >> 8;										// zapise promenne do EEPROM
		  EepromBuffer[ 1] = (uint8_t) (TemperatureMin);
		  EepromBuffer[ 2] = TemperatureMax >> 8;
		  EepromBuffer[ 3] = (uint8_t) (TemperatureMax);
		  EepromBuffer[ 4] = TemperatureStart >> 8;
		  EepromBuffer[ 5] = (uint8_t) (TemperatureStart);
		  EepromBuffer[ 6] = StandbyTemperature >> 8;
		  EepromBuffer[ 7] = (uint8_t) (StandbyTemperature);
		  EepromBuffer[ 8] = StandbyDifference >> 8;
		  EepromBuffer[ 9] = (uint8_t) (StandbyDifference);
		  EepromBuffer[10] = StandbyTimerMax >> 8;
		  EepromBuffer[11] = (uint8_t) (StandbyTimerMax);
		  EepromBuffer[12] = RegKp >> 8;
		  EepromBuffer[13] = (uint8_t) (RegKp);
		  EepromBuffer[14] = RegKi >> 8;
		  EepromBuffer[15] = (uint8_t) (RegKi);
		  EepromBuffer[16] = EEPROM_MAGIC0;
		  EepromBuffer[17] = EEPROM_MAGIC1;
		  EEPROM_SPI_INIT(&hspi1);
		  EepromStatus = EEPROM_SPI_WriteBuffer(EepromBuffer, EEPROM_ADDR, EEPROM_LEN );
		  if(EepromStatus==1) {
			  LCD_WriteCString( "EEPROM write OK " );
		  } else {
			  LCD_WriteCString( "EEPROM write ERR" );
			  Beep( TONE_ERROR, 500 );
		  }
		  Delay_Wd( 1000 );						// dlouhe cekani UI - prubezne krmi watchdog
		  break;
	  }
	  case 3: {
		  LCD_Position( 0, 0 );							// Standby - pero lezi ve stojanku
		  LCD_WriteCString( "* * STANDBY * * " );
		  LCD_Position( 1, 0 );
		  LCD_WriteCString( "Tsold=" );
		  LCD_PrTemp( (uint16_t) TemperatureMeasure );	// teplota klesa k StandbyTemperature
		  LCD_WriteCString( " " );
		  break;
	  }
	  }

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);
}

/* USER CODE BEGIN 4 */

/* Pipnuti na piezomenici. 'Tone' je pozadovany kmitocet v Hz, 'Time' delka v ms.
   Perioda v ticich citace = TIM1_TICK_HZ / Tone, takze ARR = ticky - 1.
   Strida se drzi na ~50 %, aby hlasitost nezavisela na vysce tonu (pri pevne
   hodnote compare byly nizke tony temer neslysitelne). */
void Beep( uint32_t Tone, uint16_t Time) {
	uint32_t ticks;

	if( Tone == 0 ) return;						// ochrana proti deleni nulou

	ticks = TIM1_TICK_HZ / Tone;				// pocet ticku na jednu periodu
	if( ticks < 2 )     ticks = 2;				// horni mez kmitoctu (ARR musi zustat >= 1)
	if( ticks > 65536 ) ticks = 65536;			// dolni mez ~39 Hz (ARR je 16bitovy)

	LL_TIM_SetAutoReload(TIM1, ticks - 1);				// perioda: f = TIM1_TICK_HZ / (ARR+1)
	LL_TIM_OC_SetCompareCH1(TIM1, ticks / 2);			// strida ~50 %
	LL_TIM_GenerateEvent_UPDATE(TIM1);					// ARR i CCR maji preload - prepis stinove registry hned

	/* Cesta signalu na pin: OC1REF -> CC1P (polarita) -> CC1E -> MOE -> PA8.
	   POZOR: CubeMX generuje LL_TIM_OC_Init() s OCState = LL_TIM_OCSTATE_DISABLE,
	   coz nechava CC1E = 0, takze casovac pin vubec nebudi. Kanal je proto nutne
	   povolit rucne - bez tohoto radku bzucak mlci bez ohledu na ARR/CCR. */
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);	// CC1E - pripoj vystup kanalu na pin
	LL_TIM_EnableAllOutputs(TIM1);						// MOE - hlavni povoleni vystupu (jen TIM1/TIM8)
	LL_TIM_EnableCounter(TIM1);							// CEN - rozbeh citace
	TimerBeep = Time;
}

/* ====================================================================== */
/*  Bezpecnost: watchdog + signalizace poruch                             */
/* ====================================================================== */

/* Spusti nezavisly watchdog (IWDG) s prodlevou ~1 s. IWDG bezi z LSI (~40 kHz)
   a je nezavisly na jadre i na maskovani preruseni - jakmile prestane byt
   "krmen", tvrde resetuje MCU. */
static void Watchdog_Init(void)
{
	IWDG->KR  = 0x0000CCCCu;			// start IWDG (zaroven natvrdo zapne LSI)
	IWDG->KR  = 0x00005555u;			// odemkne zapis do PR/RLR
	IWDG->PR  = 0x04u;					// delicka /64  => 40 kHz / 64 = 625 Hz
	IWDG->RLR = 625u;					// 625 / 625 Hz = ~1.0 s
	while( IWDG->SR != 0u ) { }			// pocka na zapsani PR/RLR
	IWDG->KR  = 0x0000AAAAu;			// prvni nakrmeni (reload)
}

/* Nakrmi IWDG, ale JEN pokud se od minula posunul PidHeartbeat - tedy pokud
   regulacni smycka stale bezi. Diky tomu watchdog odhali nejen zaseknutou
   hlavni smycku, ale i "mrtvou" regulaci (kdyby zustala topit). */
static void Watchdog_Refresh(void)
{
	static uint32_t last  = 0u;
	static uint8_t  first = 1u;
	uint32_t now = PidHeartbeat;
	if( first || (now != last) ) {
		first = 0u;
		last  = now;
		IWDG->KR = 0x0000AAAAu;			// reload
	}
}

/* Blokujici prodleva, ktera prubezne krmi watchdog (pro dlouha cekani v UI). */
static void Delay_Wd(uint32_t ms)
{
	while( ms ) {
		uint32_t chunk = (ms > 50u) ? 50u : ms;
		LL_mDelay( chunk );
		ms -= chunk;
		Watchdog_Refresh();
	}
}

/* Zobrazi aktivni poruchu a obcas pipne. Topeni je v tu chvili uz vypnute
   (rozhodnuto v regulacni smycce podle promenne Fault). */
static void Display_Fault(void)
{
	static uint8_t beepDiv = 0u;
	LCD_Position( 0, 0 );
	if(      Fault & FAULT_NO_IRON )  LCD_WriteCString( "  CHYBA: cidlo  " );
	else if( Fault & FAULT_OVERTEMP ) LCD_WriteCString( " CHYBA: prehrati" );
	else if( Fault & FAULT_RUNAWAY )  LCD_WriteCString( "CHYBA: topi samo" );
	else if( Fault & FAULT_ADC )      LCD_WriteCString( "CHYBA: prevodnik" );
	else                              LCD_WriteCString( " CHYBA: topeni  " );
	LCD_Position( 1, 0 );
	if( Fault & FAULT_NO_IRON )       LCD_WriteCString( "pripoj  pero    " );
	else                              LCD_WriteCString( "stiskni  knoflik" );
	if( ++beepDiv >= 15u ) {			// kratke pipnuti ~1x za 1.5 s
		beepDiv = 0u;
		Beep( TONE_FAULT, 40 );
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
  HEATER_OFF();           /* fail-safe: vypni topeni jeste pred zastavenim */
  __disable_irq();
  while (1)
  {
    /* IWDG (pokud uz bezi) timto MCU za ~1 s resetuje -> cisty start, topeni OFF */
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

