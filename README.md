# KAMAIKASHI SENSORLESS ESC

Reverse-engineering and custom firmware development for a scooter-class BLDC ESC based on **STM32F103C8T6** and **FD2103S** gate drivers.

This repository documents the full bring-up path from raw board mapping to stable sensorless BLDC control experiments.

---

## Project goals

This project focuses on:

* reverse-engineering the original ESC hardware
* full **TIM1 true 6PWM complementary drive**
* reliable **safe OFF-state handling**
* low-side shunt **current sense interpretation**
* VBAT and analog input calibration
* open-loop startup strategies
* future **sensorless BEMF zero-cross control**
* repository-quality documentation of hardware findings

---

## Project maturity

This is currently a **bench-development repository**.

It is intended for:

* hardware bring-up
* reverse engineering
* bridge validation
* ADC / sensing validation
* safe motor spin tests
* startup and commutation experiments

It is **NOT road-ready vehicle firmware**.

---

## Confirmed working

### Hardware bring-up

* STM32F103C8T6 initialization
* TIM1 complementary 6PWM routing
* FD2103S gate-driver integration
* UART debug interface
* GPIO override for inactive phases

### Bridge behavior

* reliable per-phase OFF state
* inactive phase float mode
* high-side PWM control
* low-side static ON control
* low-side sticking issue understood and mitigated

### Analog and sensing

* VBAT divider scaling and calibration
* BEMF ADC input mapping
* low-side shunt current sensing behavior
* sector/state dependent current interpretation
* NTC thermal input mapped

### Bench motor tests

* open-loop spin success
* sinusoidal drive experiments
* 6-step commutation validation
* current-limited supply testing

---

## Still in progress

* final 6-step commutation cleanup
* sector-aware current reconstruction
* startup ramp tuning
* robust wheel restart logic
* BEMF zero-cross detection
* open-loop → closed-loop transition
* undervoltage and timeout fault layer
* production-safe thermal protection

---

## Start here

If you want the **current main firmware direction**, start with:

```text
firmware/main/CUSTOM_FIRMWARE_ESC.ino
```

All other firmware files should be treated as:

* experimental
* archived
* single-purpose test sketches
* bring-up tools

---

## Repository layout

```text
KAMAIKASHI-SENSORLESS-ESC/
│
├─ README.md
├─ LICENSE
├─ .gitignore
│
├─ docs/
│  ├─ hardware_notes.md
│  ├─ pinmap.md
│  ├─ current_sense.md
│  ├─ bridge_behavior.md
│  └─ bench_safety.md
│
├─ firmware/
│  ├─ main/
│  │  └─ CUSTOM_FIRMWARE_ESC.ino
│  │
│  ├─ experimental/
│  │  ├─ 6pwm_bridge_test.ino
│  │  ├─ open_loop_sinusoidal_test.ino
│  │  └─ alt_test_firmware.ino
│  │
│  └─ archive/
│     └─ old_versions/
│
└─ media/
   ├─ board_photo_top.jpg
   ├─ board_photo_annotated.jpg
   └─ block_diagram.png
```

---

## Hardware platform

### MCU

* **STM32F103C8T6**

### Gate drivers

* **FD2103S x3**

### PWM power stage

| MCU Pin | Function |
| ------- | -------- |
| PA8     | HIN_A    |
| PA9     | HIN_B    |
| PA10    | HIN_C    |
| PB13    | LIN_A    |
| PB14    | LIN_B    |
| PB15    | LIN_C    |

### ADC / sensing

| MCU Pin | Signal        |
| ------- | ------------- |
| PA0     | BEMF_A        |
| PA1     | BEMF_B        |
| PA2     | BEMF_C        |
| PA3     | Throttle / SP |
| PA4     | IA            |
| PA5     | IB            |
| PA6     | IC            |
| PA7     | VBAT          |
| PB0     | NTC           |

### UART

| MCU Pin | Signal |
| ------- | ------ |
| PB6     | TX     |
| PB7     | RX     |

### Other discovered GPIO

| MCU Pin | Function       |
| ------- | -------------- |
| PB1     | Brake          |
| PB2     | Status LED     |
| PB4     | DisplayControl |
| PB9     | Back Light     |
| PB11    | Front Light    |

---

## Important engineering findings

### Safe OFF state

A major practical finding of this project:

> disabling TIM1 PWM alone is **not always enough** for a true OFF bridge state.

Reliable inactive phase OFF required GPIO forcing:

* **HIN = LOW**
* **LIN = HIGH**

This fully solved the observed **low-side MOSFET sticking behavior** during bench tests.

---

### Current sense behavior

The board uses **low-side shunt current sensing**.

This means current interpretation is:

* commutation-sector dependent
* valid only when the return low-side path is conducting
* not equal to permanent true phase-current measurement

This is critical for future:

* overcurrent protection
* torque estimation
* startup fault detection
* current limiting

---

### VBAT scaling

VBAT measurement is implemented through a resistor divider.

The final scaling factor should always be **bench-calibrated against real supply voltage**, because divider assumptions may differ between board revisions.

---

## Quick start

### Requirements

* STM32F103 target board
* matching ESC PCB revision
* Arduino IDE or PlatformIO
* STM32 core toolchain
* current-limited bench power supply
* UART terminal
* oscilloscope / logic analyzer recommended

### First power-up procedure

1. Flash `CUSTOM_FIRMWARE_ESC.ino`
2. verify pin mapping against your board
3. power from current-limited supply
4. verify idle bridge OFF state
5. check VBAT ADC scaling
6. verify UART debug output
7. only then connect motor for short spin bursts

---

## Bench safety

This is **power electronics development**.

Always use:

* current-limited lab supply
* short spin bursts
* thermal monitoring of MOSFETs and drivers
* wheel off ground
* no body contact with spinning rotor
* emergency power disconnect nearby

Do **not** directly test first revisions on a full battery pack.

---

## Recommended debug tools

Recommended instrumentation used during development:

* Hantek 6022BL oscilloscope
* logic analyzer
* UART terminal
* thermal camera or IR thermometer
* bench PSU with current limit

Useful signals to observe:

* TIM1 complementary outputs
* LIN inactive logic level
* phase BEMF waveform
* low-side shunt amplifier output
* VBAT divider ADC waveform

---

## Next milestones

1. extract bridge control into reusable layer
2. finalize sector-aware current sense logic
3. improve startup ramp and relock behavior
4. implement robust BEMF zero-cross detection
5. add sensorless closed-loop handoff
6. add undervoltage and timeout fault handling
7. add thermal protection logic
8. clean experimental firmware into reusable modules

---

## Long-term roadmap

### Firmware architecture

* `hal_pwm`
* `bridge_control`
* `adc_sampling`
* `bemf_detection`
* `current_limit`
* `fault_manager`
* `uart_cli`

### Control roadmap

* stable 6-step startup
* adaptive ramp
* freewheel relock
* BEMF timing advance
* zero-cross debounce
* current-based stall detection
* thermal derating

---

## Disclaimer

This repository is intended for:

* research
* learning
* reverse engineering
* custom firmware development
* BLDC control experimentation

Use at your own risk.

The author does **not recommend direct vehicle deployment** until:

* startup reliability
* fault handling
* thermal safety
* current limiting
* sensorless closed-loop stability

are fully validated.

---

## Author notes

The strongest current value of this repository is not only firmware code, but also:

* documented board discoveries
* real-world bridge behavior findings
* safe OFF-state solution
* practical low-side current-sense interpretation
* reverse-engineered pin mapping

This makes the repository useful both as:

* working firmware base
* hardware reverse-engineering reference
* BLDC ESC development notes

---

## License

Recommended: **MIT License**
