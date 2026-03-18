⚡ STM32F103C8T6 ESC (FD2103S) – 6PWM BLDC Controller

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
