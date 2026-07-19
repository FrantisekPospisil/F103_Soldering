# F103_Soldering — pájecí stanice na STM32F103

Firmware regulované pájecí stanice s K-termočlánkem, PI regulací, LCD 16×2, rotačním
enkodérem a nastavením v SPI EEPROM. Projekt STM32CubeIDE, jazyk C (gnu11).

- Schéma / PCB (KiCad): `C:\Google\Kicad\Pajeci_stanice_03\` (`Pajeci_stanice_03.kicad_sch`, `pajeci_stanice.pdf`)
- Firmware: tento adresář

---

## 1. Hardware (podle schématu Pajeci_stanice_03)

### Napájení
`J6 (AC)` → můstek **GBU8M** → **+24 V** (C9 1000 µF, C18 47 µF) → **LM2576HVS-5V** (buck) → **+5 V**
→ **AMS1117-3.3** → **+3,3 V**. Nábojová pumpa (D4/D5 1N4002) + **L79L12** → **−12 V** (jen pro OZ).
Topení běží přímo z **+24 V**.

### Měření teploty
K-termočlánek (~40 µV/°C) na `J1` → R8 100k + C2 1 µF (τ ≈ 90 ms) → **LM6361 (U2)**, zesílení
≈ **187,5×** (R12 100k / R13 560R), offset trim RV1 20k1 proti **+5 V**, napájení OZ +5 V / −12 V
→ R10 1k5 + C3 220nF → **PA0**.

> **Fail-safe:** `R4 1M` vytahuje vstup na +5 V. Při **rozpojeném termočlánku** jde výstup OZ na plný
> rozsah → ADC ≈ 4095 → firmware to vyhodnotí jako poruchu a vypne topení.

### Topení
`PA15 (TIM2_CH1, ~2,56 kHz)` → R20 10k → **Q3 BC817 (INVERTUJE)** → gate **BUZ11 (Q4)**,
pull-up R18 1k2 na **+5 V** → low-side spínání topení na `J5` (mezi +24 V a drainem).
Bez freewheel diody, bez měření proudu.

### Ostatní
- **LCD** HD44780 16×2 na `J2` (14 pin), **VDD = +5 V**, R/W = GND, 4bitový režim. Podsvit přes Q1.
- **Enkodér** na `J4`, filtrační C4/C5/C6 10 nF.
- **EEPROM** 25LCxxx (U1) na SPI1, 1 MHz, mód 0.
- **Bzučák** BZ1 přes Q2. Je to **pasivní piezoměnič** (R17 1k paralelně = vybíjecí cesta
  pro kapacitní piezo), nemá vlastní oscilátor. Má **ostrou rezonanci cca 3–5 kHz** a mimo ni
  je výrazně tišší až neslyšitelný → tóny volit blízko 4 kHz, viz `TONE_*` v `main.h`.
- **SWD** na `J3` (ST-LINK).

### Pinout
| Pin | Funkce | Poznámka |
|---|---|---|
| PA0 | ADC1_IN0 | termočlánek (přes LM6361) |
| PA1 | ADC1_IN1 | dělič napájení R15/R16 (100k/10k = 1:11), měří +24 V |
| PA4–PA7 | SPI1 NSS/SCK/MISO/MOSI | EEPROM 25LCxxx |
| PA8 | TIM1_CH1 | bzučák (přes Q2) |
| PA13/PA14 | SWDIO/SWCLK | ST-LINK |
| PA15 | TIM2_CH1 | **topení** (přes Q3 → BUZ11) |
| PB0–PB3 | LCD D4–D7 | |
| PB4 / PB5 / PB6 | LCD RS / E / podsvit | |
| PB13 / PB14 | enkodér Coder1 (B) / Coder0 (A) | PB14 má EXTI |
| PB15 | tlačítko enkodéru | EXTI |

### Invarianty HW ↔ firmware (neporušovat!)
0. ⚠️ **`LL_TIM_OC_Init()` NEPOVOLUJE výstup kanálu.** CubeMX generuje
   `OCState = LL_TIM_OCSTATE_DISABLE`, což nechá `CC1E = 0` a časovač pin vůbec nebudí.
   **U obou časovačů se proto musí ručně zavolat `LL_TIM_CC_EnableChannel()`** —
   pro TIM2 v `main.c` při startu, pro TIM1 v `Beep()`. Bez toho u TIM1 mlčí bzučák
   a **u TIM2 jede topení natrvalo naplno** (nebuzený pin = log. 0 → Q3 nesepne →
   R18 vytáhne gate BUZ11 nahoru). Po regeneraci z CubeMX tohle vždy zkontroluj.
1. **Q3 invertuje**, proto se střída zapisuje jako `1000 - SolderPWM`.
   **`compare = 1000` znamená topení VYPNUTO** → makro `HEATER_OFF()`.
2. **Po resetu MCU je topení vypnuté**: PA15 se vrátí na JTDI s pull-upem → Q3 sepne →
   gate BUZ11 dole. Na tom stojí bezpečnost resetu watchdogem — nerušit.
3. Rozpojený termočlánek = ADC na plném rozsahu (viz R4 výše), tedy bezpečný směr.

---

## 2. Architektura firmwaru

Mix **LL** (GPIO, TIM, RCC, EXTI) a **HAL** (ADC, DMA, SPI). Takt **64 MHz z HSI** (krystal se nepoužívá).

| Kontext | Perioda | Co dělá |
|---|---|---|
| `DMA1_Channel1_IRQHandler` | ~126 µs (~8 kHz) | **jen akvizice**: klouzavý průměr 64 vzorků ze 4 kanálů + `AdcTicks++` |
| `SysTick_Handler` | 1 ms | čítače (`TimerStart`, `TimerSec`, `TimerBeep`), ukončení tónu, sekundové počítadlo poruchy topení, **`Inputs_Poll()`** (enkodér + tlačítko) |
| `Control_Loop()` | **25 ms** (`PID_PERIOD_MS`) | volá se ze SysTicku: přepočet teploty, kontrola živosti ADC, detekce poruch, **PI regulátor**, zápis střídy, heartbeat |
| `EXTI15_10_IRQHandler` | — | **nepoužívá se**, přerušení je zamaskované v `main.c` (vstupy se vzorkují) |
| `main()` smyčka | 100 ms | LCD, menu, zápis EEPROM, obrazovka poruchy, krmení watchdogu |

**ADC1**: 4 kanály, scan + continuous, software start, DMA circular do `AdResult[8]`
(0 = termočlánek, 2 = napájení, 4 = interní teploměr, 6 = Vref). Vzorkování 239,5 cyklů, ADC clock 8 MHz.

### Jednotky (pozor při úpravách)
- **Teplota: 0,1 °C** → `4000` = 400,0 °C. Zobrazuje `LCD_PrTemp()`.
- **Výkon `SolderPWM`: 0,1 %** → `1000` = 100 %.
- Kalibrace (empirická, 28. 6. 2024): `TemperatureMeasure = ((AdResult0 + 82) * 1380) >> 10`

### Stavy
`System` — **pouze UI**: `0` normální, `1` MENU, `2` zápis EEPROM, `3` Standby, `0x11` editace hodnoty.

### Zvuková signalizace
Krátké melodie hraje neblokující přehrávač (`Melody_Start()` / `Melody_Poll()`
v `stm32f1xx_it.c`) — tabulka not `{Hz, ms}`, `{0, ms}` je pomlka, `{0, 0}` konec.
`Melody_Poll()` se volá každou ms ze SysTicku a spustí další notu, až dozní předchozí.

Obě melodie jsou v **D moll**, v pásmu 1175–1760 Hz (stejně jako startovní sekvence):

- **teplota dosažena**: D–A–F–A (1–5–♭3–5), skok o kvintu, drženo na kvintě
- **Standby**: A–F–E–D (5–♭3–2–1), klesavá kadence rozvedená do základního tónu

„Spolehlivě dosaženo" znamená, že odchylka vydrží v pásmu `±TEMP_OK_BAND`
po celou dobu `TEMP_OK_STABLE_MS` — samotné překročení žádané teploty nestačí,
protože ji hrot při náběhu přestřelí. Ohlásí se **jen jednou**: po zapnutí
a po probuzení ze Standby (`TempOkArmed`).

### Standby
Při pájení odebírá spoj teplo a teplota hrotu klesne. Když **po dobu `StandbyTimerMax`
sekund teplota ani jednou neklesne o `StandbyDifference`** pod nastavenou, pero leží
ve stojánku → setpoint se sníží na `StandbyTemperature`, pracovní teplota se zálohuje
do `TemperatureSetStandby` a displej ukáže `* * STANDBY * *`. Zpět se přejde **stiskem
knoflíku**, který teplotu obnoví.

- odpočet `TimerStandby` nuluje `Control_Loop()` při každém poklesu (zachytí i krátký),
  dále otočení enkodérem a návrat ze Standby
- počítá se jen v `System == 0` bez poruchy; v MENU a při poruše se nuluje
- během náběhu z čista se nespustí — dokud je hrot studený, podmínka poklesu trvale platí
- `StandbyTimerMax = 0` funkci **vypíná**; nespustí se ani když `StandbyTemperature`
  není nižší než pracovní teplota

`Fault` — **bitové pole poruch**, nezávislé na UI; nenulové = topení vypnuto:
`FAULT_NO_IRON 0x01`, `FAULT_OVERTEMP 0x02`, `FAULT_HEATER 0x04`, `FAULT_ADC 0x08`,
`FAULT_RUNAWAY 0x10` (teplota roste při nulovém výkonu — hlídá „topí mimo kontrolu").
Maže se stiskem knoflíku (nebo samo, pokud příčina pominula).
Zapisuje se do něj **výhradně z kontextu SysTicku** (`Control_Loop`, `Button_Press`),
takže tam nemůže vzniknout souběh při čtení-úpravě-zápisu.

### PI regulátor
Je to **PI, ne PID** — derivační složka se záměrně nepoužívá (šum termočlánku zesílený
187× a dopravní zpoždění mezi tělesem a čidlem by ji dělaly škodlivou).
Vzorkovací perioda `PID_PERIOD_MS` = 25 ms, výstup `HeaterPower` v 0,1 % (0–1000).

- `P = odchylka * RegKp / 100` → **pásmo proporcionality [0,1 °C] = 100000 / RegKp**
- `I = integrál * RegKi * PID_PERIOD_MS / 250000` → **`Ti [s] = RegKp * 2,5 / RegKi`**
  (perioda se ve vzorci vykrátí, takže její změna regulátor nerozladí)
- **anti-windup podmíněnou integrací**: integruje se jen když výstup nesaturuje,
  nebo když by integrace saturaci zmenšovala → při studeném startu integrál nenaroste
- integrál je navíc tvrdě omezen tak, aby jeho složka sama nepřesáhla 100 %
- `RegKi = 0` integrál úplně vypne (čistý P) — hodí se při ladění

Výchozí `RegKp = 500` (pásmo 20 °C), `RegKi = 150` (Ti ≈ 8,3 s).

### Watchdog
**IWDG ~1 s**, spouští se až po startovní sekvenci. `Watchdog_Refresh()` krmí IWDG **jen když
se posunul `PidHeartbeat`** — odhalí tedy zaseknutou hlavní smyčku i „mrtvou" regulaci.
Dlouhá čekání v UI musí používat `Delay_Wd()`.

### EEPROM
Blok nastavení na `EEPROM_ADDR` (0x10), `EEPROM_LEN` = 18 bajtů, bez CRC:

| offset | položka | offset | položka |
|---|---|---|---|
| 0–1 | `TemperatureMin` | 8–9 | `StandbyDifference` |
| 2–3 | `TemperatureMax` | 10–11 | `StandbyTimerMax` |
| 4–5 | `TemperatureStart` | 12–13 | `RegKp` |
| 6–7 | `StandbyTemperature` | 14–15 | `RegKi` |
| | | 16–17 | magie `EEPROM_MAGIC0/1` |

⚠️ **Při každé změně pořadí nebo počtu položek se musí změnit `EEPROM_MAGIC1`**,
jinak se stará data s jiným rozložením načtou jako platná a nastavení se přeházejí.
Aktuálně `0xA5 0x63` (v2 — bez nepoužívaného `MeanMax`).

---

## 3. Pravidla programování

1. **Veškerý vlastní kód patří mezi `/* USER CODE BEGIN x */` a `/* USER CODE END x */`.**
   Cokoli mimo tyto bloky regenerace z `.ioc` smaže.
2. **Odsazení tabulátory** ve vlastním kódu. Části generované CubeMX mají 2 mezery — zachovávej
   lokální styl daného místa, needituj generovaný kód zbytečně.
3. **Komentáře česky, ale BEZ diakritiky (čisté ASCII)** ve všech `.c`/`.h`. (Tento `.md` diakritiku mít může.)
4. **Řetězce na LCD mají přesně 16 znaků** (displej 16×2), bez diakritiky. Znak „°" je `0xDF`.
5. **`volatile` u každé proměnné sdílené mezi přerušením a hlavní smyčkou.** Bez toho může
   optimalizovaný build (Release) hodnoty cachovat a UI/regulace čte stará data.
6. **Bezpečnost topení má přednost.** Každá cesta, která zastaví běh programu
   (`Error_Handler`, `HardFault_Handler`, nekonečná smyčka), musí **nejdřív zavolat `HEATER_OFF()`**.
   Nikdy nenechávej TIM2 na neznámé střídě.
7. **Neblokuj hlavní smyčku déle než ~1 s** bez `Delay_Wd()` — IWDG má timeout ~1 s.
8. **`Error_Handler()` je fatální** (zastaví regulaci) — volat jen při opravdu neopravitelné chybě.
   Chyby periferií (EEPROM, SPI) hlásit návratovým kódem, ne zastavením systému.
9. **Pojmenování**: globální proměnné `PascalCase`, `#define` `UPPER_SNAKE_CASE`, lokální `camelCase`.
10. Po změně `.ioc` a regeneraci **zkontrolovat `gpio.c`, `tim.c`, `adc.c`, `dma.c`** a bezpečnostní
    kód v `stm32f1xx_it.c`.

---

## 4. Build

Toolchain je v instalaci STM32CubeIDE (není v PATH):

```bash
export PATH="/c/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin:/c/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506/tools/bin:$PATH"
cd Debug && make -j4 all
```

Debug build je `-O0`, `-Wall`. **Build musí být bez varování.**
Orientační velikost: text ≈ 23,8 kB, bss ≈ 2,4 kB (STM32F103C8 = 64 kB flash / 20 kB RAM).

---

## 5. Stav a roadmap

### Hotovo
- **P0 bezpečnost**: IWDG s heartbeatem, `HEATER_OFF()` v `Error_Handler`/`HardFault_Handler`,
  detekce poruch (odpojené čidlo, tvrdý strop 450 °C, „topí naplno bez ohřevu" 30 s),
  obrazovka poruchy + potvrzení knoflíkem.
- **`Beep()`**: parametr je nyní skutečný kmitočet v Hz (`ARR = TIM1_TICK_HZ / f - 1`,
  `TIM1_TICK_HZ = 2 560 000`). Střída držena na ~50 %, ochrana proti dělení nulou,
  ošetřený rozsah ~39 Hz až po 16bitový strop ARR. Volané tóny jsou v konstantách
  `TONE_*` (`main.h`) a leží v pásmu, kde piezo skutečně hraje.
- Odstraněn mrtvý kód: stará větev „Neni pripojeno / pajeci pero" v `case 0`
  (nahradila ji `Display_Fault()`), nepoužité pole `Temperature[4]`, zakomentované řádky.
- **`volatile`** doplněno u všech 28 proměnných sdílených mezi přerušeními a hlavní smyčkou
  (pole a mezisoučty používané výhradně v DMA přerušení ho záměrně nemají — kvůli rychlosti).
- **`Eeprom.c`** přepsán: každé čekání má timeout (`EEPROM_TIMEOUT_MS`), žádné volání
  `Error_Handler()`, chyby se hlásí návratovým kódem. Odstraněn nepoužitý kód
  (`EEPROM_SendByte`, `sEE_WriteStatusRegister`, `RxBuffer`, `EEPROM_StatusByte`).
- **PID** přesunut z 8kHz DMA přerušení do `Control_Loop()` (25 ms) s podmíněnou integrací;
  přibyla kontrola živosti ADC (`FAULT_ADC`) a hlídání „topí, i když nemá" (`FAULT_RUNAWAY`).
- **Přejmenování** na mluvící názvy: `Const1`→`RegKp`, `Const2`→`RegKi`,
  `Difference`→`RegError`, `DifferenceIntegral`→`RegIntegral`,
  `TemperatureSet0`→`TemperatureStart`, `SolderPWM`→`HeaterPower`.
  Měřítko I složky je nově nezávislé na `PID_PERIOD_MS`.
- **Odstraněno `MeanMax`** — šlo nastavit v menu i uložit do EEPROM, ale průměrování
  má natvrdo 64 vzorků, takže ta položka nic nedělala. Menu přečíslováno na 0–12.
- **Napájecí napětí** (menu 11) z děliče R15/R16 poměrově přes VREFINT:
  `SupplyVoltage [0,01 V] = 1320 · AdResult1 / AdResult3`. Rozsah děliče je do ~36 V.
- **Zrušena položka menu „Prumerovani"** (zobrazovala nikdy nepočítané `TemperatureOut`);
  proměnná odstraněna, menu má nyní indexy **0–11** (0–7 editovatelné, 11 = uložení).
- **Teplota čipu** (menu 10) se počítá z interního čidla MCU poměrově přes VREFINT:
  `TemperatureIn [0,1 °C] = 3576 − 2791 · AdResult2 / AdResult3`.
  Je to teplota křemíku, ne okolí; F103 nemá tovární kalibraci, takže absolutně
  může být o 10–20 °C vedle — offset se ladí konstantou `3576`.
- **Standby dokončen** (viz sekce výše) — dřív se `System = 3` nikdy nenastavilo,
  `TimerStandby` se nepočítal a obrazovka měla dvě chyby (text mimo displej,
  obrácené zálohování teploty). Spodní mez `StandbyTimerMax` v menu snížena na 0 (= vypnuto).
- **Odstraněn další mrtvý kód**: `CounterTemperature` (jen se do ní zapisovalo — nedokončená
  signalizace dosažení teploty), `TemperatureOut` a funkce `LCD_PrPwm()` (po opravě jednotek
  u „Standby Diff" ji nikdo nevolal).
- **Enkodér**: hranové EXTI nahrazeno vzorkováním v SysTicku (1 kHz) s kvadraturním
  dekodérem (`EncoderTable`) a tlačítko má 20ms debounce — viz `Inputs_Poll()`,
  `Encoder_Step()`, `Button_Press()` v `stm32f1xx_it.c`. Citlivost ladí
  `ENCODER_STEPS_PER_DETENT` (4 pro EC11), směr se otočí prohozením A/B v `Inputs_Poll()`.

### Zbývá (v pořadí priority)
- **P1** — periodický re-init LCD (ochrana proti rozsypání displeje šumem).
- **P2** — signalizace dosažení teploty (pípnutí, až hrot poprvé dosáhne žádané);
  podpěťová kontrola (napětí se už měří, chybí jen mez a hláška); kompenzace studené spáry.
- **HW** — LCD na 3,3 V nebo level-shifter; logic-level MOSFET místo BUZ11; freewheel dioda;
  pojistka/TVS na vstupu.

### Známé drobnosti
- LCD je napájený z +5 V, ale buzený 3,3 V logikou → `VIH` je mimo spec (0,7 × VDD = 3,5 V).
  Funguje, ale je to tenká šumová rezerva a možná příčina občasného rozsypání displeje.
- `TemperatureSet0` má default `1400` (= 140,0 °C), což je na pájení málo — reálně se přepisuje z EEPROM.
