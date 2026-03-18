#include <Arduino.h>
#include "stm32f1xx_hal.h"

// ============================================================
// STM32F103 + FD2103S
// TRUE 6PWM / TIM1 CHx + CHxN
// OPEN LOOP TEST
//
// PA8  -> TIM1_CH1  -> HIN_A
// PA9  -> TIM1_CH2  -> HIN_B
// PA10 -> TIM1_CH3  -> HIN_C
//
// PB13 -> TIM1_CH1N -> LIN_A
// PB14 -> TIM1_CH2N -> LIN_B
// PB15 -> TIM1_CH3N -> LIN_C
//
// FD2103S trick used here:
// - CHx active high
// - CHxN polarity LOW
// - Idle states:
//     CHx  idle LOW
//     CHxN idle HIGH
//
// Result:
// OCREF=1 => HIN=1, LIN=1 => HS ON, LS OFF
// OCREF=0 => HIN=0, LIN=0 => HS OFF, LS ON
//
// So:
// - PWM channel = synchronous half-bridge PWM
// - Compare=0   = low-side fully ON
// - Disabled    = floating phase
// ============================================================

static const uint32_t UART_BAUD = 9600;
HardwareSerial U(PB7, PB6);

// ---------------- PINS ----------------
const uint8_t PIN_LED = PB2;

// ---------------- TIM1 / PWM ----------------
TIM_HandleTypeDef htim1;

static const uint32_t PWM_FREQ_HZ = 20000;
static const uint16_t PWM_MAX_8BIT = 255;
static uint16_t tim1_arr = 3599;

// duty
static uint8_t runDuty8   = 45;
static uint8_t alignDuty8 = 35;

// open-loop ramp
static bool runOpenLoop = false;
static uint8_t stepIndex = 0;

static uint32_t openLoopStartUs = 18000;
static uint32_t openLoopTargetUs = 5000;
static uint32_t openLoopStepUs = 18000;
static uint32_t openLoopAccelUs = 120;
static uint32_t nextStepUs = 0;

static const uint16_t ALIGN_TIME_MS = 120;
static const uint16_t ALIGN_RELEASE_MS = 30;

// logical phases: 0=A, 1=B, 2=C
static const int8_t stepHigh[6] = { 0, 0, 1, 1, 2, 2 };
static const int8_t stepLow[6]  = { 1, 2, 2, 0, 0, 1 };
static const int8_t stepFlt[6]  = { 2, 1, 0, 2, 1, 0 };

// ============================================================
// HELPERS
// ============================================================

uint16_t duty8ToTicks(uint8_t d8) {
  uint32_t t = ((uint32_t)d8 * (uint32_t)tim1_arr) / PWM_MAX_8BIT;
  if (t > tim1_arr) t = tim1_arr;
  return (uint16_t)t;
}

static inline void setCompareTicks(uint8_t ch, uint16_t ticks) {
  if (ticks > tim1_arr) ticks = tim1_arr;
  switch (ch) {
    case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ticks); break;
    case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ticks); break;
    case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ticks); break;
  }
}

static inline void enablePair(uint8_t ch, bool en) {
  switch (ch) {
    case 0:
      if (en) htim1.Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
      else    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
      break;
    case 1:
      if (en) htim1.Instance->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
      else    htim1.Instance->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
      break;
    case 2:
      if (en) htim1.Instance->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
      else    htim1.Instance->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
      break;
  }
}

void all_off_safe() {
  // compare zero
  setCompareTicks(0, 0);
  setCompareTicks(1, 0);
  setCompareTicks(2, 0);

  // disable channel pairs
  enablePair(0, false);
  enablePair(1, false);
  enablePair(2, false);

  // force idle state onto pins
  __HAL_TIM_MOE_DISABLE(&htim1);
  delayMicroseconds(5);
  __HAL_TIM_MOE_ENABLE(&htim1);
}

// ============================================================
// TIM1 INIT
// ============================================================

void tim1Init6PWM() {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PA8/9/10 = CH1/2/3
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PB13/14/15 = CH1N/2N/3N
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  uint32_t timclk = HAL_RCC_GetPCLK2Freq();
  uint32_t period = (timclk / PWM_FREQ_HZ) - 1;
  if (period > 65535) period = 65535;
  tim1_arr = (uint16_t)period;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = tim1_arr;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    while (1) {}
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;

  // CHx polarity normal
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;

  // CHxN polarity inverted
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;

  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // OFF state:
  // CHx  idle LOW
  // CHxN idle HIGH
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;

  // bez przesady na start, ~1 us dead-time
  sBreakDeadTimeConfig.DeadTime = 72;

  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;

  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  __HAL_TIM_MOE_ENABLE(&htim1);

  all_off_safe();
}

// ============================================================
// COMMUTATION
// ============================================================

// phase mode using one channel pair:
// - disabled            => floating
// - enabled, compare=0  => low-side fully ON
// - enabled, compare=d  => PWM half-bridge on high phase
void setPhaseFloat(uint8_t ph) {
  setCompareTicks(ph, 0);
  enablePair(ph, false);
}

void setPhaseLowOn(uint8_t ph) {
  setCompareTicks(ph, 0);
  enablePair(ph, true);
}

void setPhaseHighPWM(uint8_t ph, uint16_t dutyTicks) {
  setCompareTicks(ph, dutyTicks);
  enablePair(ph, true);
}

void applyStep6PWM(uint8_t step, uint8_t duty8) {
  step %= 6;
  stepIndex = step;

  uint16_t dutyTicks = duty8ToTicks(duty8);

  uint8_t h = (uint8_t)stepHigh[step];
  uint8_t l = (uint8_t)stepLow[step];
  uint8_t f = (uint8_t)stepFlt[step];

  // start from safe all-off
  all_off_safe();
  delayMicroseconds(5);

  setPhaseFloat(f);
  setPhaseLowOn(l);
  setPhaseHighPWM(h, dutyTicks);
}

void advanceStep() {
  stepIndex = (stepIndex + 1) % 6;
  applyStep6PWM(stepIndex, runDuty8);
}

// ============================================================
// OPEN LOOP
// ============================================================

void alignRotor() {
  // hold one fixed vector briefly
  stepIndex = 0;
  applyStep6PWM(stepIndex, alignDuty8);
  delay(ALIGN_TIME_MS);
  all_off_safe();
  delay(ALIGN_RELEASE_MS);
}

void startOpenLoop() {
  if (runOpenLoop) return;

  alignRotor();

  stepIndex = 0;
  openLoopStepUs = openLoopStartUs;
  applyStep6PWM(stepIndex, runDuty8);
  nextStepUs = micros() + openLoopStepUs;
  runOpenLoop = true;

  U.println("OPEN_LOOP_START");
}

void stopOpenLoop() {
  runOpenLoop = false;
  all_off_safe();
  U.println("OPEN_LOOP_STOP");
}

void openLoopTask() {
  if (!runOpenLoop) return;

  uint32_t now = micros();
  if ((int32_t)(now - nextStepUs) < 0) return;

  advanceStep();

  if (openLoopStepUs > openLoopTargetUs + openLoopAccelUs) {
    openLoopStepUs -= openLoopAccelUs;
  } else {
    openLoopStepUs = openLoopTargetUs;
  }

  nextStepUs = now + openLoopStepUs;
}

// ============================================================
// UART
// ============================================================

void printStatus() {
  U.print("RUN=");
  U.print(runOpenLoop ? 1 : 0);
  U.print(" STEP=");
  U.print(stepIndex);
  U.print(" DUTY=");
  U.print(runDuty8);
  U.print(" ALIGN=");
  U.print(alignDuty8);
  U.print(" STEP_US=");
  U.print(openLoopStepUs);
  U.print(" START_US=");
  U.print(openLoopStartUs);
  U.print(" TARGET_US=");
  U.print(openLoopTargetUs);
  U.print(" ACCEL_US=");
  U.println(openLoopAccelUs);
}

void printHelp() {
  U.println();
  U.println("=== FD2103S TRUE 6PWM OPEN LOOP ===");
  U.println("space -> start/stop");
  U.println("[     -> slower target");
  U.println("]     -> faster target");
  U.println("p     -> run duty +5");
  U.println("o     -> run duty -5");
  U.println("k     -> align duty +5");
  U.println("j     -> align duty -5");
  U.println("u     -> accel faster (+)");
  U.println("i     -> accel slower (-)");
  U.println("a     -> all off");
  U.println("s     -> status");
  U.println("h     -> help");
  U.println("===============================");
  U.println();
}

void handleCommand(char c) {
  switch (c) {
    case ' ':
      if (runOpenLoop) stopOpenLoop();
      else startOpenLoop();
      break;

    case '[':
      openLoopTargetUs = min<uint32_t>(openLoopTargetUs + 500, 30000);
      U.print("TARGET_US=");
      U.println(openLoopTargetUs);
      break;

    case ']':
      if (openLoopTargetUs > 1000) openLoopTargetUs -= 500;
      if (openLoopTargetUs < 1000) openLoopTargetUs = 1000;
      U.print("TARGET_US=");
      U.println(openLoopTargetUs);
      break;

    case 'p':
      if (runDuty8 < 250) runDuty8 += 5;
      U.print("RUN_DUTY=");
      U.println(runDuty8);
      if (runOpenLoop) applyStep6PWM(stepIndex, runDuty8);
      break;

    case 'o':
      if (runDuty8 > 5) runDuty8 -= 5;
      U.print("RUN_DUTY=");
      U.println(runDuty8);
      if (runOpenLoop) applyStep6PWM(stepIndex, runDuty8);
      break;

    case 'k':
      if (alignDuty8 < 250) alignDuty8 += 5;
      U.print("ALIGN_DUTY=");
      U.println(alignDuty8);
      break;

    case 'j':
      if (alignDuty8 > 5) alignDuty8 -= 5;
      U.print("ALIGN_DUTY=");
      U.println(alignDuty8);
      break;

    case 'u':
      openLoopAccelUs = min<uint32_t>(openLoopAccelUs + 20, 2000);
      U.print("ACCEL_US=");
      U.println(openLoopAccelUs);
      break;

    case 'i':
      if (openLoopAccelUs > 20) openLoopAccelUs -= 20;
      U.print("ACCEL_US=");
      U.println(openLoopAccelUs);
      break;

    case 'a':
      stopOpenLoop();
      break;

    case 's':
      printStatus();
      break;

    case 'h':
      printHelp();
      break;

    default:
      break;
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  U.begin(UART_BAUD);

  tim1Init6PWM();

  U.println("FD2103S TRUE 6PWM open-loop ready");
  printHelp();
}

void loop() {
  while (U.available()) {
    char c = (char)U.read();
    if (c != '\r' && c != '\n') {
      handleCommand(c);
    }
  }

  openLoopTask();

  static uint32_t tLed = 0;
  static bool led = false;
  uint32_t now = millis();

  if (runOpenLoop) {
    if (now - tLed > 120) {
      tLed = now;
      led = !led;
      digitalWrite(PIN_LED, led);
    }
  } else {
    digitalWrite(PIN_LED, LOW);
  }
}