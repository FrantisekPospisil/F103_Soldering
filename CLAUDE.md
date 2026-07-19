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
| PA1 | ADC1_IN1 | dělič napájení R15/R16 — **osazeno, firmwarem nevyužito** |
| PA4–PA7 | SPI1 NSS/SCK/MISO/MOSI | EEPROM 25LCxxx |
| PA8 | TIM1_CH1 | bzučák (přes Q2) |
| PA13/PA14 | SWDIO/SWCLK | ST-LINK |
| PA15 | TIM2_CH1 | **topení** (přes Q3 → BUZ11) |
| PB0–PB3 | LCD D4–D7 | |
| PB4 / PB5 / PB6 | LCD RS / E / podsvit | |
| PB13 / PB14 | enkodér Coder1 (B) / Coder0 (A) | PB14 má EXTI |
| PB15 | tlačítko enkodéru | EXTI |

### Invarianty HW ↔ firmware (neporušovat!)
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
| `DMA1_Channel1_IRQHandler` | ~126 µs (~8 kHz) | **regulační smyčka**: klouzavý průměr 64 vzorků, přepočet teploty, PI regulátor, detekce poruch, heartbeat, zápis střídy |
| `SysTick_Handler` | 1 ms | čítače (`TimerStart`, `TimerSec`, `TimerBeep`), ukončení tónu, sekundové počítadlo poruchy topení, **`Inputs_Poll()`** (enkodér + tlačítko) |
| `EXTI15_10_IRQHandler` | — | **nepoužívá se**, přerušení je zamaskované v `main.c` (vstupy se vzorkují) |
| `main()` smyčka | 100 ms | LCD, menu, zápis EEPROM, obrazovka poruchy, krmení watchdogu |

**ADC1**: 4 kanály, scan + continuous, software start, DMA circular do `AdResult[8]`
(0 = termočlánek, 2 = napájení, 4 = interní teploměr, 6 = Vref). Vzorkování 239,5 cyklů, ADC clock 8 MHz.

### Jednotky (pozor při úpravách)
- **Teplota: 0,1 °C** → `4000` = 400,0 °C. Zobrazuje `LCD_PrTemp()`.
- **Výkon `SolderPWM`: 0,1 %** → `1000` = 100 %.
- Kalibrace (empirická, 28. 6. 2024): `TemperatureMeasure = ((AdResult0 + 82) * 1380) >> 10`

### Stavy
`System` — **pouze UI**: `0` normální, `1` MENU, `2` zápis EEPROM, `3` Standby (nedokončeno), `0x11` editace hodnoty.

`Fault` — **bitové pole poruch**, nezávislé na UI; nenulové = topení vypnuto:
`FAULT_NO_IRON 0x01`, `FAULT_OVERTEMP 0x02`, `FAULT_HEATER 0x04`.
Maže se stiskem knoflíku (nebo samo, pokud příčina pominula).

### Watchdog
**IWDG ~1 s**, spouští se až po startovní sekvenci. `Watchdog_Refresh()` krmí IWDG **jen když
se posunul `PidHeartbeat`** — odhalí tedy zaseknutou hlavní smyčku i „mrtvou" regulaci.
Dlouhá čekání v UI musí používat `Delay_Wd()`.

### EEPROM
Nastavení na adrese `0x10`, 26 bajtů, magie `0xA5 0x62` na offsetu 18/19. Bez CRC.

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
- **Enkodér**: hranové EXTI nahrazeno vzorkováním v SysTicku (1 kHz) s kvadraturním
  dekodérem (`EncoderTable`) a tlačítko má 20ms debounce — viz `Inputs_Poll()`,
  `Encoder_Step()`, `Button_Press()` v `stm32f1xx_it.c`. Citlivost ladí
  `ENCODER_STEPS_PER_DETENT` (4 pro EC11), směr se otočí prohozením A/B v `Inputs_Poll()`.

### Zbývá (v pořadí priority)
- **P1** — `volatile` u všech zbylých sdílených proměnných; timeouty v `Eeprom.c`
  (nekonečné čekací smyčky + volání `Error_Handler`); periodický re-init LCD.
- **P2** — dokončit Standby (`System == 3` se nikdy nenastaví, `TimerStandby` se neinkrementuje);
  signalizace dosažení teploty (`CounterTemperature` je nedokončená); readout napájecího napětí
  (PA1 je osazený, výpis v menu zakomentovaný) + podpěťová kontrola; kompenzace studené spáry;
  přesun PID z 8 kHz ISR do ~20–50 ms smyčky s pořádným anti-windupem.
- **HW** — LCD na 3,3 V nebo level-shifter; logic-level MOSFET místo BUZ11; freewheel dioda;
  pojistka/TVS na vstupu.

### Známé drobnosti
- LCD je napájený z +5 V, ale buzený 3,3 V logikou → `VIH` je mimo spec (0,7 × VDD = 3,5 V).
  Funguje, ale je to tenká šumová rezerva a možná příčina občasného rozsypání displeje.
- `TemperatureSet0` má default `1400` (= 140,0 °C), což je na pájení málo — reálně se přepisuje z EEPROM.
