# F103_Soldering — STM32F103 Soldering Station

Firmware for a home-built temperature-controlled soldering station.
K-type thermocouple, PI control loop, 16×2 LCD, rotary encoder, settings stored
in an SPI EEPROM. Written in C for STM32CubeIDE using a mix of ST's LL and HAL drivers.

The station regulates the tip temperature, drops to a standby temperature when the
iron is left in its stand, and shuts the heater down on a range of fault conditions.

> ⚠️ **Safety notice** — this design switches a heating element from a ~24 V rail
> derived from a mains transformer and bridge rectifier. Mains wiring, fusing and
> enclosure are your responsibility. Treat an uncontrolled heater as a fire hazard
> and read the *Safety features* section before reusing any of this.

---

## Features

- **PI temperature control** at 40 Hz with conditional-integration anti-windup
- **Independent watchdog (IWDG)** that also verifies the control loop is alive
- **Five fault detectors**, each of which cuts the heater (see below)
- **Auto standby** — detects that the iron is idle and lowers the tip temperature
- **Quadrature-decoded rotary encoder**, sampled and debounced in software
- Supply-voltage and MCU-die temperature readouts
- All settings adjustable from the LCD menu and persisted to EEPROM
- Short musical cues for "tip ready" and "entering standby"

Footprint: **~24.9 kB flash** and **~2.5 kB RAM** (STM32F103C8 has 64 kB / 20 kB).

---

## Hardware

Schematic: [`pajeci_stanice.pdf`](pajeci_stanice.pdf) (KiCad sources are kept
outside this repository).

| Block | Implementation |
|---|---|
| MCU | STM32F103C8T6 @ 64 MHz (PLL from internal HSI — the crystal is unused) |
| Power | transformer → GBU8M bridge → **+24 V** → LM2576HVS-5V → **+5 V** → AMS1117 → **+3.3 V**; charge pump + L79L12 → **−12 V** for the op-amp |
| Temperature sensing | K-type thermocouple → LM6361, gain ≈ 187.5×, offset trim; RC filtered (τ ≈ 90 ms) |
| Heater drive | TIM2_CH1 PWM @ 2.56 kHz → BC817 (**inverting**) → BUZ11 low-side switch |
| Display | HD44780 16×2, 4-bit mode |
| Input | EC11-style rotary encoder with push button |
| Settings storage | 25LCxxx SPI EEPROM @ 1 MHz |
| Buzzer | passive piezo transducer driven from TIM1_CH1 |

### Pinout

| Pin | Function |
|---|---|
| PA0 | ADC1_IN0 — thermocouple amplifier output |
| PA1 | ADC1_IN1 — supply voltage divider (100k / 10k) |
| PA4–PA7 | SPI1 NSS / SCK / MISO / MOSI — EEPROM |
| PA8 | TIM1_CH1 — buzzer |
| PA13/PA14 | SWDIO / SWCLK |
| PA15 | TIM2_CH1 — **heater PWM** |
| PB0–PB3 | LCD D4–D7 |
| PB4 / PB5 / PB6 | LCD RS / E / backlight |
| PB13 / PB14 | encoder channels B / A |
| PB15 | encoder push button |

### Two hardware quirks worth knowing

**The heater driver inverts.** Q3 sits between the timer pin and the MOSFET gate,
so the duty cycle is written as `1000 - HeaterPower`, and a *full* compare value
means the heater is **off**. After a reset PA15 reverts to its JTDI pull-up, which
holds the BUZ11 gate low — so the heater is off whenever the MCU is not running.
The watchdog reset path depends on this.

**An open thermocouple fails safe.** A 1 MΩ pull-up drives the amplifier to full
scale when the iron is unplugged, so the reading saturates *high* and the firmware
cuts the heater rather than driving it.

---

## Safety features

The heater is only driven while `Fault == 0`. All faults are latched into a bit
field, displayed on the LCD, and acknowledged with the encoder button.

| Fault | Trigger |
|---|---|
| `FAULT_NO_IRON` | ADC near full scale — iron unplugged or thermocouple open |
| `FAULT_OVERTEMP` | hard 450 °C ceiling, independent of the setpoint |
| `FAULT_HEATER` | full power for 30 s without the temperature rising |
| `FAULT_ADC` | ADC/DMA stopped delivering samples (stale readings) |
| `FAULT_RUNAWAY` | temperature keeps rising while the commanded power is zero |

On top of that:

- **IWDG (~1 s)** is refreshed only when the control loop's heartbeat advances, so
  a hung main loop *and* a dead control loop both trigger a reset — after which the
  hardware default leaves the heater off.
- `Error_Handler()` and `HardFault_Handler()` force the heater off before halting.
- Long UI waits use a watchdog-aware delay so saving to EEPROM cannot trip the reset.

---

## Firmware architecture

| Context | Period | Responsibility |
|---|---|---|
| `DMA1_Channel1_IRQHandler` | ~126 µs | acquisition only: 64-sample moving average of 4 ADC channels |
| `SysTick_Handler` | 1 ms | timers, buzzer/melody sequencer, encoder + button sampling |
| `Control_Loop()` | 25 ms | temperature conversion, fault detection, PI controller, PWM output |
| `main()` loop | 100 ms | LCD, menu, EEPROM writes, fault screen, watchdog |

The PI controller originally ran inside the 8 kHz ADC interrupt; it was moved to a
25 ms loop because a thermal system with a multi-second time constant gains nothing
from that rate and mostly reacted to noise.

**Units:** temperature is in 0.1 °C (`4000` = 400.0 °C), heater power in 0.1 %
(`1000` = 100 %).

---

## Menu

Press the encoder button to enter the menu, turn to navigate, press to edit a value,
press again to confirm. Items 1–8 are editable; the rest are read-only diagnostics.
Selecting *Exit menu* writes everything to EEPROM.

| # | Item | # | Item |
|---|---|---|---|
| 1 | minimum settable temperature | 7 | P gain (`RegKp`) |
| 2 | maximum settable temperature | 8 | I gain (`RegKi`) |
| 3 | temperature after power-on | 9 | control error + integrator |
| 4 | standby temperature | 10 | MCU die temperature |
| 5 | standby drop threshold | 11 | supply voltage |
| 6 | standby timeout (0 = disabled) | — | *Exit menu* — save |

### Standby

While soldering, the joint pulls heat out of the tip and the temperature dips. If no
such dip larger than the *standby drop threshold* happens for the whole *standby
timeout*, the iron is assumed to be sitting in its stand and the setpoint drops to
the standby temperature. A button press restores the working temperature.

---

## Tuning the PI controller

Two constants, both adjustable from the menu:

```
proportional band [°C] = 10000 / RegKp
integral time  Ti [s]  = RegKp * 2.5 / RegKi
```

Defaults are `RegKp = 500` (20 °C band) and `RegKi = 150` (Ti ≈ 8.3 s).

Setting `RegKi = 0` disables the integrator, which is handy while tuning: raise
`RegKp` until the temperature starts to oscillate, halve it, then set
`RegKi ≈ RegKp / 4` for Ti ≈ 10 s.

Note that the integral scaling deliberately cancels the loop period, so changing
`PID_PERIOD_MS` does not detune the controller.

---

## Building

The project is a STM32CubeIDE project and can be imported directly. To build from
the command line you need the toolchain bundled with CubeIDE (it is not on `PATH`
by default):

```bash
export PATH="/c/ST/STM32CubeIDE_<version>/STM32CubeIDE/plugins/\
com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.<ver>/tools/bin:\
/c/ST/STM32CubeIDE_<version>/STM32CubeIDE/plugins/\
com.st.stm32cube.ide.mcu.externaltools.make.win32_<ver>/tools/bin:$PATH"

cd Debug && make -j4 all
```

Debug builds use `-O0 -Wall` and are expected to compile without warnings.
Flash with ST-LINK over SWD.

### If you regenerate from the `.ioc`

CubeMX generates `LL_TIM_OC_Init()` with `OCState = LL_TIM_OCSTATE_DISABLE`, which
leaves `CC1E` cleared — **the timer then never drives the pin**. Both timers must
have their channel enabled explicitly:

```c
LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
```

Without it the buzzer is silent and, far worse, the heater runs at full power
continuously (an undriven pin reads low, and the driver stage inverts). Code outside
the `/* USER CODE BEGIN|END */` markers is overwritten on regeneration, so check
this after every regenerate.

---

## Repository layout

```
Core/Src, Core/Inc   application code (main, interrupts, LCD, EEPROM driver)
Drivers              ST HAL/LL drivers (unmodified)
Debug                build output and generated makefiles
F103_Soldering.ioc   STM32CubeMX project
pajeci_stanice.pdf   schematic
CLAUDE.md            detailed design notes (in Czech)
```

Source comments and the design notes in `CLAUDE.md` are written in Czech; this
README is the English summary.

---

## Status and known limitations

Working: temperature control, safety shutdowns, standby, menu, EEPROM persistence,
encoder handling, audible cues.

Known limitations:

- **No cold-junction compensation.** A thermocouple measures the difference between
  tip and cold junction, so the reading drifts with ambient temperature. Proper
  compensation needs a sensor at the connector; the MCU die sensor is too inaccurate
  to substitute (no factory calibration on F103, easily ±10–20 °C off).
- Calibration is a single hard-coded linear fit, not a two-point user calibration.
- The LCD is powered from 5 V but driven by 3.3 V logic, which is outside the
  HD44780 `VIH` spec. It works, but the noise margin is thin.
- BUZ11 is driven with only 5 V on the gate; a logic-level MOSFET would dissipate less.
- No flyback diode or current sensing on the heater output.
- Undervoltage detection is not implemented, although the supply voltage is measured.

---

## License

No license has been chosen yet — if you intend to reuse this, please ask first.
The ST HAL/LL drivers under `Drivers/` retain their original ST license.
