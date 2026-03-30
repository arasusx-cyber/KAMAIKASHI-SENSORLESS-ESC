#include <Arduino.h>
#include "stm32f1xx.h"
#include <math.h>

// ============================================================
// STM32F103 + FD2103S
// TRUE 6PWM / TIM1 complementary outputs
// OPEN-LOOP SINUSOIDAL DRIVE - improved
//
// PA8  -> TIM1_CH1  -> A_HIN
// PA9  -> TIM1_CH2  -> B_HIN
// PA10 -> TIM1_CH3  -> C_HIN
//
// PB13 -> TIM1_CH1N -> A_LIN
// PB14 -> TIM1_CH2N -> B_LIN
// PB15 -> TIM1_CH3N -> C_LIN
//
// UART: PB6 TX, PB7 RX
//
// Keys:
// SPACE -> emergency OFF
// e     -> enable/start
// f     -> freewheel / coast
// 0     -> neutral + zero targets
// r     -> reverse toggle
// [ ]   -> modulation target down/up
// - =   -> frequency target down/up
// , .   -> phase offset down/up
// t     -> telemetry on/off
// p     -> print status once
// h     -> help
// ============================================================

HardwareSerial U(PB7, PB6);

// ---------------- USER CONFIG ----------------
static const uint32_t UART_BAUD      = 38400;
static const uint32_t PWM_FREQ_HZ    = 20000;
static const uint32_t CONTROL_HZ     = 2000;

// F1 dead-time coding is not fully linear.
// 36 ticks at 72 MHz is about 500 ns and still in the simple linear range.
static const uint8_t DEADTIME_RAW    = 36;

// start conservative
static const float MOD_MIN           = 0.00f;
static const float MOD_MAX           = 0.92f;
static const float FREQ_MIN          = 0.00f;
static const float FREQ_MAX          = 250.0f;

static const float MOD_STEP_FINE     = 0.01f;
static const float FREQ_STEP_FINE    = 0.5f;
static const float PHASE_STEP_DEG    = 2.0f;

// ramp rates per second
static const float MOD_SLEW_PER_S    = 0.35f;
static const float FREQ_SLEW_PER_S   = 20.0f;

// keep away from 0/100%
static const float DUTY_MIN          = 0.02f;
static const float DUTY_MAX          = 0.98f;

// ---------------- STATE ----------------
enum DriveMode : uint8_t {
  MODE_FREEWHEEL = 0,
  MODE_RUN       = 1
};

static volatile DriveMode driveMode = MODE_FREEWHEEL;
static volatile bool emergencyStop = false;
static volatile bool reverseDir = false;

static volatile float elecFreqHz = 0.0f;       // actual
static volatile float modulation = 0.0f;       // actual
static volatile float targetFreqHz = 5.0f;     // target
static volatile float targetMod = 0.08f;       // target
static volatile float phaseOffsetDeg = 0.0f;

static uint16_t tim1_arr = 0;
static float theta = 0.0f;
static const float TWO_PI_F = 6.2831853071795864769f;
static uint32_t lastControlUs = 0;

// telemetry
static bool telemetryEnabled = false;
static uint32_t telemetryPeriodMs = 250;
static uint32_t lastTelemetryMs = 0;

// ---------------- Helpers ----------------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float moveToward(float cur, float target, float maxStep) {
  if (target > cur) {
    cur += maxStep;
    if (cur > target) cur = target;
  } else if (target < cur) {
    cur -= maxStep;
    if (cur < target) cur = target;
  }
  return cur;
}

static inline uint16_t clampDutyToCCR(float duty01) {
  duty01 = clampf(duty01, DUTY_MIN, DUTY_MAX);
  return (uint16_t)(duty01 * (float)tim1_arr);
}

// ---------------- Low-level GPIO/TIM1 ----------------
static void setupGPIO_TIM1_6PWM() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
               |  RCC_APB2ENR_IOPBEN
               |  RCC_APB2ENR_AFIOEN
               |  RCC_APB2ENR_TIM1EN;

  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

  // PA8, PA9, PA10 = AF PP 50 MHz
  GPIOA->CRH &= ~(
      GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
      GPIO_CRH_MODE9 | GPIO_CRH_CNF9 |
      GPIO_CRH_MODE10 | GPIO_CRH_CNF10
  );
  GPIOA->CRH |= (
      (GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1) | GPIO_CRH_CNF8_1 |
      (GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1) | GPIO_CRH_CNF9_1 |
      (GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1) | GPIO_CRH_CNF10_1
  );

  // PB13, PB14, PB15 = AF PP 50 MHz
  GPIOB->CRH &= ~(
      GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
      GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
      GPIO_CRH_MODE15 | GPIO_CRH_CNF15
  );
  GPIOB->CRH |= (
      (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1) | GPIO_CRH_CNF13_1 |
      (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1) | GPIO_CRH_CNF14_1 |
      (GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1) | GPIO_CRH_CNF15_1
  );
}

static void setupTIM1() {
  const uint32_t tim_clk = 72000000UL;

  // Center-aligned:
  // Fpwm = tim_clk / (2 * (PSC+1) * (ARR+1))
  uint32_t arr = tim_clk / (2UL * PWM_FREQ_HZ);
  if (arr == 0) arr = 1;
  arr -= 1;
  if (arr > 65535) arr = 65535;
  tim1_arr = (uint16_t)arr;

  TIM1->CR1   = 0;
  TIM1->CR2   = 0;
  TIM1->SMCR  = 0;
  TIM1->DIER  = 0;
  TIM1->CCER  = 0;
  TIM1->CCMR1 = 0;
  TIM1->CCMR2 = 0;
  TIM1->BDTR  = 0;

  TIM1->PSC = 0;
  TIM1->ARR = tim1_arr;

  // center-aligned mode 1 + ARR preload
  TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_ARPE;

  // PWM mode 1 + preload CH1/2/3
  TIM1->CCMR1 |=
      (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
      (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

  TIM1->CCMR2 |=
      (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  TIM1->CCR1 = tim1_arr / 2;
  TIM1->CCR2 = tim1_arr / 2;
  TIM1->CCR3 = tim1_arr / 2;

  // Enable CHx + CHxN
  TIM1->CCER |=
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE;

  // Complementary outputs inverted only
  TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;

  TIM1->BDTR =
      ((uint32_t)DEADTIME_RAW) |
      TIM_BDTR_OSSR |
      TIM_BDTR_OSSI |
      TIM_BDTR_MOE;

  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void neutralPWM() {
  TIM1->CCR1 = tim1_arr / 2;
  TIM1->CCR2 = tim1_arr / 2;
  TIM1->CCR3 = tim1_arr / 2;
}

static void outputsEnable(bool en) {
  if (en) TIM1->BDTR |= TIM_BDTR_MOE;
  else    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// true freewheel: disable channel outputs, not only MOE
static void outputsFreewheel() {
  TIM1->CCER &= ~(
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE
  );
}

static void outputsReEnableChannels() {
  TIM1->CCER |=
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE;

  TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
}

static void setFreewheelMode() {
  driveMode = MODE_FREEWHEEL;
  neutralPWM();
  outputsFreewheel();
  outputsEnable(false);
}

static void armRunMode() {
  outputsReEnableChannels();
  neutralPWM();
  outputsEnable(true);
  driveMode = MODE_RUN;
  emergencyStop = false;
}

static void emergencyOffNow() {
  emergencyStop = true;
  targetFreqHz = 0.0f;
  targetMod = 0.0f;
  elecFreqHz = 0.0f;
  modulation = 0.0f;
  setFreewheelMode();
  U.println("\n!!! EMERGENCY OFF !!!");
}

// ---------------- Sine generator ----------------
static void updateSineOpenLoop() {
  const uint32_t nowUs = micros();
  const uint32_t periodUs = 1000000UL / CONTROL_HZ;
  if ((uint32_t)(nowUs - lastControlUs) < periodUs) return;

  const float dt = (float)(nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;

  if (driveMode != MODE_RUN || emergencyStop) {
    return;
  }

  // smooth ramps
  modulation = moveToward(modulation, clampf(targetMod, MOD_MIN, MOD_MAX), MOD_SLEW_PER_S * dt);
  elecFreqHz = moveToward(elecFreqHz, clampf(targetFreqHz, FREQ_MIN, FREQ_MAX), FREQ_SLEW_PER_S * dt);

  const float dir = reverseDir ? -1.0f : 1.0f;
  theta += dir * TWO_PI_F * elecFreqHz * dt;

  while (theta >= TWO_PI_F) theta -= TWO_PI_F;
  while (theta < 0.0f) theta += TWO_PI_F;

  const float phi = phaseOffsetDeg * (TWO_PI_F / 360.0f);

  // 3-phase sine centered at 50%
  const float a = 0.5f + 0.5f * modulation * sinf(theta + phi);
  const float b = 0.5f + 0.5f * modulation * sinf(theta + phi - 2.09439510239f);
  const float c = 0.5f + 0.5f * modulation * sinf(theta + phi + 2.09439510239f);

  TIM1->CCR1 = clampDutyToCCR(a);
  TIM1->CCR2 = clampDutyToCCR(b);
  TIM1->CCR3 = clampDutyToCCR(c);
}

// ---------------- UI ----------------
static void printStatus() {
  U.print("MODE=");
  U.print(driveMode == MODE_RUN ? "RUN" : "FREE");
  U.print(" ESTOP=");
  U.print(emergencyStop ? 1 : 0);
  U.print(" REV=");
  U.print(reverseDir ? 1 : 0);
  U.print(" F=");
  U.print(elecFreqHz, 2);
  U.print("Hz Ftar=");
  U.print(targetFreqHz, 2);
  U.print(" M=");
  U.print(modulation, 3);
  U.print(" Mtar=");
  U.print(targetMod, 3);
  U.print(" PHI=");
  U.print(phaseOffsetDeg, 1);
  U.print(" CCR=");
  U.print(TIM1->CCR1);
  U.print(",");
  U.print(TIM1->CCR2);
  U.print(",");
  U.println(TIM1->CCR3);
}

static void printHelp() {
  U.println();
  U.println("=== KEY CONTROL ===");
  U.println("SPACE : emergency OFF");
  U.println("e     : enable/start");
  U.println("f     : freewheel/coast");
  U.println("0     : neutral + zero targets");
  U.println("r     : reverse toggle");
  U.println("[     : modulation target -");
  U.println("]     : modulation target +");
  U.println("-     : frequency target -");
  U.println("=     : frequency target +");
  U.println(",     : phase offset -");
  U.println(".     : phase offset +");
  U.println("t     : telemetry on/off");
  U.println("p     : print status");
  U.println("h     : help");
  U.println("===================");
  U.println();
}

static void handleKey(char ch) {
  switch (ch) {
    case ' ':
      emergencyOffNow();
      break;

    case 'e':
    case 'E':
      armRunMode();
      U.println("drive enabled");
      break;

    case 'f':
    case 'F':
      targetFreqHz = 0.0f;
      targetMod = 0.0f;
      elecFreqHz = 0.0f;
      modulation = 0.0f;
      setFreewheelMode();
      U.println("freewheel");
      break;

    case '0':
      targetFreqHz = 0.0f;
      targetMod = 0.0f;
      elecFreqHz = 0.0f;
      modulation = 0.0f;
      theta = 0.0f;
      armRunMode();
      neutralPWM();
      U.println("neutral reset");
      break;

    case 'r':
    case 'R':
      reverseDir = !reverseDir;
      U.print("REV=");
      U.println(reverseDir ? 1 : 0);
      break;

    case '[':
      targetMod = clampf(targetMod - MOD_STEP_FINE, MOD_MIN, MOD_MAX);
      U.print("Mtar=");
      U.println(targetMod, 3);
      break;

    case ']':
      targetMod = clampf(targetMod + MOD_STEP_FINE, MOD_MIN, MOD_MAX);
      U.print("Mtar=");
      U.println(targetMod, 3);
      break;

    case '-':
      targetFreqHz = clampf(targetFreqHz - FREQ_STEP_FINE, FREQ_MIN, FREQ_MAX);
      U.print("Ftar=");
      U.println(targetFreqHz, 2);
      break;

    case '=':
    case '+':
      targetFreqHz = clampf(targetFreqHz + FREQ_STEP_FINE, FREQ_MIN, FREQ_MAX);
      U.print("Ftar=");
      U.println(targetFreqHz, 2);
      break;

    case ',':
      phaseOffsetDeg -= PHASE_STEP_DEG;
      U.print("PHI=");
      U.println(phaseOffsetDeg, 1);
      break;

    case '.':
      phaseOffsetDeg += PHASE_STEP_DEG;
      U.print("PHI=");
      U.println(phaseOffsetDeg, 1);
      break;

    case 't':
    case 'T':
      telemetryEnabled = !telemetryEnabled;
      U.print("telemetry ");
      U.println(telemetryEnabled ? "ON" : "OFF");
      break;

    case 'p':
    case 'P':
      printStatus();
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    default:
      break;
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  U.begin(UART_BAUD);
  delay(200);

  U.println("\nSTM32F103 TRUE 6PWM SIN OPEN LOOP");
  U.println("Center-aligned PWM, complementary outputs, dead-time.");
  U.println("Open loop only. Low voltage first.");

  setupGPIO_TIM1_6PWM();
  setupTIM1();

  setFreewheelMode();
  lastControlUs = micros();

  printHelp();
  printStatus();
}

void loop() {
  updateSineOpenLoop();

  while (U.available()) {
    char ch = (char)U.read();
    handleKey(ch);
  }

  if (telemetryEnabled) {
    uint32_t now = millis();
    if (now - lastTelemetryMs >= telemetryPeriodMs) {
      lastTelemetryMs = now;
      printStatus();
    }
  }
}
