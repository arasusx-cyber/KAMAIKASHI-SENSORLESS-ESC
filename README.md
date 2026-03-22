# KAMAIKASHI SENSORLESS ESC

Reverse-engineering and custom firmware work for a scooter-class BLDC ESC based on **STM32F103C8T6** and **FD2103S** gate drivers.

This repository documents the hardware mapping, true 6PWM bring-up, current-sense behavior, lighting IO discovery, and the path toward stable sensorless control.

## Current project status

### Working
- STM32F103C8T6 main MCU
- TIM1 true 6PWM bring-up
- Safe phase OFF state using GPIO:
  - `HIN = LOW`
  - `LIN = HIGH`
- Motor spin test working at sensible speed with current-limited supply
- Battery voltage measurement calibrated
- Low-side shunt current sensing understood
- Front light / back light / display control pins partially reverse-engineered

### In progress
- robust 6-step commutation cleanup
- current-sense integration with active sector logic
- open-loop startup tuning
- sensorless BEMF transition logic

## Key hardware findings

### Power stage
- 3-phase bridge
- FD2103S x3 gate drivers
- STM32 TIM1 used for complementary outputs

### Important driver behavior
There is **no confirmed global driver enable pin** for the gate drivers.

The practical safe solution found during testing is:
- active phases: driven by true TIM1 complementary outputs
- OFF phase: forced by GPIO, not by trusting timer disable alone

Safe OFF state per phase:
- `HIN = LOW`
- `LIN = HIGH`

This resolved the low-side "sticking" issue.

### Current sensing
The current shunts are located on the **low-side emitters**.

That means current measurement is only valid when the corresponding low-side path is actually conducting.
This is not full always-valid phase-current sensing; it is **sector/state dependent low-side current sensing**.

## MCU pin map

### Core ESC
#### Power stage
- `PA8`  -> `HIN_A`
- `PA9`  -> `HIN_B`
- `PA10` -> `HIN_C`

- `PB13` -> `LIN_A`
- `PB14` -> `LIN_B`
- `PB15` -> `LIN_C`

#### ADC / sensing
- `PA0` -> `BEMF_A`
- `PA1` -> `BEMF_B`
- `PA2` -> `BEMF_C`
- `PA3` -> `Throttle / SP`
- `PA4` -> `IA`
- `PA5` -> `IB`
- `PA6` -> `IC`
- `PA7` -> `VBAT`
- `PB0` -> `NTC`

#### Other GPIO
- `PB1` -> `Brake`
- `PB2` -> `Status LED`

#### UART
- `PB6` -> `USART TX`
- `PB7` -> `USART RX`

### Reverse-engineered extra outputs
- `PB9`  -> `Back Light`
- `PB11` -> `Front Light`
- `PB4`  -> `DisplayControl`

### Reserved debug
- `PA13` -> `SWDIO`
- `PA14` -> `SWCLK`

### Extra free GPIO
- `PA11`
- `PA12`
- `PB5`
- `PB8`
- `PB10`
- `PB12`

### Extra GPIO after JTAG disable
- `PA15`
- `PB3`

## VBAT calibration

Empirically tuned:
- `VBAT_SCALE = 31.1f`

Measured points used during tuning:
- ~5 V input
- ~24 V input

## Commutation model

6-step table:

- Step 0: `A+ B- C float`
- Step 1: `A+ C- B float`
- Step 2: `B+ C- A float`
- Step 3: `B+ A- C float`
- Step 4: `C+ A- B float`
- Step 5: `C+ B- A float`

## Notes on current sense interpretation

Because the shunts are only on low-side emitters, the most meaningful current sample in a given 6-step sector is usually the phase that is currently acting as the low-side return path.

This was confirmed during resistor-based testing:
- Step 0 -> strongest on `IB`
- Step 2 -> strongest on `IC`
- Step 4 -> strongest on `IA`

## Repository contents

- `CUSTOM_FIRMEARE_ESC.ino`
  - main custom ESC firmware work
- `6PWM - TEST`
  - dedicated true 6PWM bring-up and bridge behavior testing
- `WhatWeHave.txt`
  - project reverse-engineering notes / architecture archive
- `ESC_LIME_GEN3_README.md`
  - focused hardware/firmware notes for this board

## Recommended next milestones
1. clean true 6PWM test into production-style bridge layer
2. finalize sector-aware current sense handling
3. stabilize open-loop startup and ramp
4. integrate BEMF zero-cross / sensorless transition
5. reorganize repo files and naming

## Warning
This is hardware-near ESC development.
Use:
- current-limited supply
- short test bursts
- temperature monitoring
- cautious bring-up sequence

## License
TBD⚡ STM32F103C8T6 ESC (FD2103S) – 6PWM BLDC Controller

BLDC motor controller firmware for STM32F103C8T6 using FD2103S half-bridge gate drivers and true 6PWM (TIM1 complementary outputs).

Architecture aligned with LimeGen3-class ESC designs.

🚀 Features

True 6PWM control (TIM1 CHx + CHxN)

Compatible with FD2103S drivers

Safe disarm / all-off state

Open-loop startup:

rotor alignment

controlled acceleration ramp

UART interface for live tuning

Sensorless control (planned)

🧠 Architecture

The control architecture follows a structure similar to LimeGen3 ESC controllers, including:

3-phase half-bridge topology

High-side PWM with complementary low-side control

Timer-driven commutation

Hardware dead-time handling via advanced timer (TIM1)

Separation of:

commutation logic

power stage control

startup sequencing

This ensures compatibility with typical scooter-class BLDC systems.

🧠 Driver Interface

FD2103S half-bridge driver:

HIN → High-side control (non-inverting)

LIN → Low-side control (internally inverted)

Built-in shoot-through protection

Control model:

OCREF = 1 → High-side ON, Low-side OFF  
OCREF = 0 → High-side OFF, Low-side ON  
🔌 Hardware
MCU

STM32F103C8T6 (Cortex-M3, 72 MHz)

Gate Drivers

FD2103S ×3

📍 Pin Mapping
Phase	High-side (HIN)	Low-side (LIN)
A	PA8 (TIM1_CH1)	PB13 (TIM1_CH1N)
B	PA9 (TIM1_CH2)	PB14 (TIM1_CH2N)
C	PA10 (TIM1_CH3)	PB15 (TIM1_CH3N)
⚙️ PWM Configuration

Timer: TIM1

Mode: PWM with complementary outputs

Frequency: 20 kHz

Dead-time: ~1 µs

Idle State
Signal	State
HIN	LOW
LIN	HIGH
🔁 6-Step Commutation
Step	High	Low	Floating
1	A	B	C
2	A	C	B
3	B	C	A
4	B	A	C
5	C	A	B
6	C	B	A
▶️ Open Loop Control

Startup sequence:

Rotor alignment

Release phase

Step-based commutation

Acceleration ramp

🎮 UART Controls
Key	Function
space	Start / Stop
[	Slower
]	Faster
p	Increase run duty
o	Decrease run duty
k	Increase align duty
j	Decrease align duty
u	Increase acceleration
i	Decrease acceleration
a	All OFF
s	Status
h	Help
⚠️ Safety

Use current-limited power supply during testing

Monitor temperature of MOSFETs and drivers

Ensure correct dead-time configuration

Verify wiring before power-up

📌 Status
Module	Status
6PWM Control	Implemented
Open Loop	Implemented
Sensorless	In progress
📜 License

MIT (or custom)
