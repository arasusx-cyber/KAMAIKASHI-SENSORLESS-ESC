#include <Arduino.h>
#include "stm32f1xx.h"
#include <math.h>

// ============================================================
// STM32F103 + FD2103S
// TRUE 6PWM / TIM1 complementary outputs
// BENCH FIRMWARE for SENSORLESS TUNING + ZERO-CROSS MONITOR
//
// PWM / driver:
// PA8  -> TIM1_CH1  -> A_HIN
// PA9  -> TIM1_CH2  -> B_HIN
// PA10 -> TIM1_CH3  -> C_HIN
//
// PB13 -> TIM1_CH1N -> A_LIN
// PB14 -> TIM1_CH2N -> B_LIN
// PB15 -> TIM1_CH3N -> C_LIN
//
// ADC:
// PA0 -> BEMF_A
// PA1 -> BEMF_B
// PA2 -> BEMF_C
// PA4 -> IA
// PA5 -> IB
// PA6 -> IC
// PA7 -> VBAT
//
// UART:
// PB6 -> TX
// PB7 -> RX
//
// IMPORTANT:
// - open-loop only
// - ZC is MONITOR ONLY for now
// - telemetry is intentionally lightweight during RUN
// ============================================================

// ============================================================
// SECTION: UART
// ============================================================
HardwareSerial U(PB7, PB6);   // RX, TX

// ============================================================
// SECTION: PINS
// ============================================================
#define PIN_BEMF_A PA0
#define PIN_BEMF_B PA1
#define PIN_BEMF_C PA2
#define PIN_IA     PA4
#define PIN_IB     PA5
#define PIN_IC     PA6
#define PIN_VBAT   PA7

// ============================================================
// SECTION: CONFIG
// ============================================================
static const uint32_t UART_BAUD        = 38400;
static const uint32_t PWM_FREQ_HZ      = 20000;
static const uint32_t CONTROL_HZ       = 2000;

static const float ADC_VREF            = 3.3f;
static const float VBAT_DIV            = 30.1f;   // skalibruj do swojej płytki

static const float MOD_MIN             = 0.00f;
static const float MOD_MAX             = 0.92f;
static const float FREQ_MIN            = 0.00f;
static const float FREQ_MAX            = 1650.0f;

static const float MOD_STEP_FINE       = 0.01f;
static const float FREQ_STEP_FINE      = 0.5f;
static const float PHASE_STEP_DEG      = 2.0f;

static const float MOD_SLEW_PER_S      = 0.35f;
static const float FREQ_SLEW_PER_S     = 20.0f;

static const float DUTY_MIN            = 0.02f;
static const float DUTY_MAX            = 0.98f;

// startowy deadtime
static uint8_t deadtimeRaw             = 36;      // ~500ns @72MHz

// zero-cross config
static int16_t zcThresholdAdc          = 12;      // histereza w ADC counts
static bool zcTelemetryEnabled         = true;

// ============================================================
// SECTION: STATE
// ============================================================
enum DriveMode : uint8_t {
  MODE_FREEWHEEL = 0,
  MODE_RUN       = 1
};

static volatile DriveMode driveMode    = MODE_FREEWHEEL;
static volatile bool emergencyStop     = false;
static volatile bool reverseDir        = false;

// actual
static volatile float elecFreqHz       = 0.0f;
static volatile float modulation       = 0.0f;

// target
static volatile float targetFreqHz     = 5.0f;
static volatile float targetMod        = 0.08f;
static volatile float phaseOffsetDeg   = 0.0f;

// timer
static uint16_t tim1_arr               = 0;

// phase accumulator
static float theta                     = 0.0f;
static const float TWO_PI_F            = 6.2831853071795864769f;

// timing
static uint32_t lastControlUs          = 0;

// telemetry
static bool telemetryEnabled           = true;
static bool verboseTelemetry           = true;
static uint32_t telemetryPeriodMs      = 200;
static uint32_t lastTelemetryMs        = 0;

// round-robin pages
enum TelemetryPage : uint8_t {
  TELE_PAGE_SHORT = 0,
  TELE_PAGE_ADC   = 1,
  TELE_PAGE_ZC    = 2
};
static TelemetryPage telePage = TELE_PAGE_SHORT;

// ADC offsets / calibration
static uint16_t iaOffset               = 2048;
static uint16_t ibOffset               = 2048;
static uint16_t icOffset               = 2048;

static uint16_t bemfAOffset            = 2048;
static uint16_t bemfBOffset            = 2048;
static uint16_t bemfCOffset            = 2048;

// last ADC snapshot and processed values
static int16_t lastBnA                 = 0;
static int16_t lastBnB                 = 0;
static int16_t lastBnC                 = 0;

static int16_t filtBnA                 = 0;
static int16_t filtBnB                 = 0;
static int16_t filtBnC                 = 0;

// ============================================================
// SECTION: HELPER MATH
// ============================================================
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

static inline float adcToVolt(uint16_t adc) {
  return (ADC_VREF * (float)adc) / 4095.0f;
}

// ============================================================
// SECTION: ADC
// ============================================================
static inline uint16_t adcReadFast(uint8_t pin) {
  return analogRead(pin);
}

struct AdcSnapshot {
  uint16_t bemfA;
  uint16_t bemfB;
  uint16_t bemfC;
  uint16_t ia;
  uint16_t ib;
  uint16_t ic;
  uint16_t vbat;
};

enum ZcSign : uint8_t {
  ZC_SIGN_UNKNOWN = 0,
  ZC_SIGN_NEG     = 1,
  ZC_SIGN_POS     = 2
};

struct ZcPhaseState {
  ZcSign sign;
  uint32_t lastCrossUs;
  uint32_t intervalUs;
  bool lastEdgeRising;
  uint32_t crossCount;
};

// explicit prototypes to avoid Arduino auto-prototype nonsense
static AdcSnapshot readAllAdc();
static void updateOneZcPhase(ZcPhaseState &st, int16_t v, uint32_t nowUs);

static AdcSnapshot lastAdc = {0,0,0,0,0,0,0};

static AdcSnapshot readAllAdc() {
  AdcSnapshot s;
  s.bemfA = adcReadFast(PIN_BEMF_A);
  s.bemfB = adcReadFast(PIN_BEMF_B);
  s.bemfC = adcReadFast(PIN_BEMF_C);
  s.ia    = adcReadFast(PIN_IA);
  s.ib    = adcReadFast(PIN_IB);
  s.ic    = adcReadFast(PIN_IC);
  s.vbat  = adcReadFast(PIN_VBAT);
  return s;
}

static float readVBATVolts(uint16_t raw) {
  return adcToVolt(raw) * VBAT_DIV;
}

// ============================================================
// SECTION: GPIO MODE SWITCHING
// ============================================================
static void bridgePinsToAFPP() {
  GPIOA->CRH &= ~(
      GPIO_CRH_MODE8  | GPIO_CRH_CNF8  |
      GPIO_CRH_MODE9  | GPIO_CRH_CNF9  |
      GPIO_CRH_MODE10 | GPIO_CRH_CNF10
  );
  GPIOA->CRH |= (
      (GPIO_CRH_MODE8_0  | GPIO_CRH_MODE8_1)  | GPIO_CRH_CNF8_1  |
      (GPIO_CRH_MODE9_0  | GPIO_CRH_MODE9_1)  | GPIO_CRH_CNF9_1  |
      (GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1) | GPIO_CRH_CNF10_1
  );

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

static void bridgePinsToGPIOOutputs() {
  GPIOA->CRH &= ~(
      GPIO_CRH_MODE8  | GPIO_CRH_CNF8  |
      GPIO_CRH_MODE9  | GPIO_CRH_CNF9  |
      GPIO_CRH_MODE10 | GPIO_CRH_CNF10
  );
  GPIOA->CRH |= (
      (GPIO_CRH_MODE8_0  | GPIO_CRH_MODE8_1)  |
      (GPIO_CRH_MODE9_0  | GPIO_CRH_MODE9_1)  |
      (GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1)
  );

  GPIOB->CRH &= ~(
      GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
      GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
      GPIO_CRH_MODE15 | GPIO_CRH_CNF15
  );
  GPIOB->CRH |= (
      (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1) |
      (GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1) |
      (GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1)
  );
}

// ============================================================
// SECTION: DIRECT DRIVER GPIO STATES
// ============================================================
// FD2103S logic assumed here:
// HIN OFF = LOW
// LIN OFF = HIGH
// LIN ON  = LOW

static void forceDriverFreewheelState() {
  // HIN low -> HS OFF
  GPIOA->BRR = GPIO_BRR_BR8 | GPIO_BRR_BR9 | GPIO_BRR_BR10;

  // LIN high -> LS OFF
  GPIOB->BSRR = GPIO_BSRR_BS13 | GPIO_BSRR_BS14 | GPIO_BSRR_BS15;
}

static void forceDriverBrakeState() {
  // HIN low -> HS OFF
  GPIOA->BRR = GPIO_BRR_BR8 | GPIO_BRR_BR9 | GPIO_BRR_BR10;

  // LIN low -> LS ON
  GPIOB->BRR = GPIO_BRR_BR13 | GPIO_BRR_BR14 | GPIO_BRR_BR15;
}

// ============================================================
// SECTION: TIM1 INIT
// ============================================================
static void setupGPIO_TIM1_6PWM() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
               |  RCC_APB2ENR_IOPBEN
               |  RCC_APB2ENR_AFIOEN
               |  RCC_APB2ENR_TIM1EN;

  AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
  bridgePinsToAFPP();
}

static void applyDeadtimeRaw(uint8_t dt) {
  deadtimeRaw = dt;
  TIM1->BDTR &= ~(0xFFu);
  TIM1->BDTR |= deadtimeRaw;
}

static void setupTIM1() {
  const uint32_t tim_clk = 72000000UL;

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

  TIM1->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_ARPE;

  TIM1->CCMR1 |=
      (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
      (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

  TIM1->CCMR2 |=
      (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;

  TIM1->CCR1 = tim1_arr / 2;
  TIM1->CCR2 = tim1_arr / 2;
  TIM1->CCR3 = tim1_arr / 2;

  TIM1->CCER |=
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE;

  TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;

  TIM1->BDTR =
      ((uint32_t)deadtimeRaw) |
      TIM_BDTR_OSSR |
      TIM_BDTR_OSSI |
      TIM_BDTR_MOE;

  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

// ============================================================
// SECTION: TIM1 OUTPUT HELPERS
// ============================================================
static void neutralPWM() {
  TIM1->CCR1 = tim1_arr / 2;
  TIM1->CCR2 = tim1_arr / 2;
  TIM1->CCR3 = tim1_arr / 2;
}

static void outputsEnable(bool en) {
  if (en) TIM1->BDTR |= TIM_BDTR_MOE;
  else    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

static void outputsDisableChannels() {
  TIM1->CCER &= ~(
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE
  );
}

static void outputsEnableChannels() {
  TIM1->CCER |=
      TIM_CCER_CC1E  | TIM_CCER_CC1NE |
      TIM_CCER_CC2E  | TIM_CCER_CC2NE |
      TIM_CCER_CC3E  | TIM_CCER_CC3NE;

  TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
}

// ============================================================
// SECTION: MODES
// ============================================================
static void setFreewheelMode() {
  driveMode = MODE_FREEWHEEL;

  neutralPWM();
  outputsEnable(false);
  outputsDisableChannels();

  bridgePinsToGPIOOutputs();
  forceDriverFreewheelState();
}

static void armRunMode() {
  bridgePinsToAFPP();
  outputsEnableChannels();
  neutralPWM();
  TIM1->EGR = TIM_EGR_UG;
  outputsEnable(true);

  driveMode = MODE_RUN;
  emergencyStop = false;
}

static void emergencyOffNow() {
 /* emergencyStop = true;
  targetFreqHz = 0.0f;
  targetMod = 0.0f;
  elecFreqHz = 0.0f;
  modulation = 0.0f;
  */
  setFreewheelMode();
  U.println("\n!!! EMERGENCY OFF !!!");
}

// ============================================================
// SECTION: ADC CALIBRATION
// ============================================================
static void calibrateAdcOffsets() {
  const int N = 128;
  uint32_t sumIA = 0, sumIB = 0, sumIC = 0;
  uint32_t sumBA = 0, sumBB = 0, sumBC = 0;

  bool prevTele = telemetryEnabled;
  telemetryEnabled = false;

  setFreewheelMode();
  delay(100);

  for (int i = 0; i < N; i++) {
    sumBA += adcReadFast(PIN_BEMF_A);
    sumBB += adcReadFast(PIN_BEMF_B);
    sumBC += adcReadFast(PIN_BEMF_C);
    sumIA += adcReadFast(PIN_IA);
    sumIB += adcReadFast(PIN_IB);
    sumIC += adcReadFast(PIN_IC);
    delay(2);
  }

  bemfAOffset = sumBA / N;
  bemfBOffset = sumBB / N;
  bemfCOffset = sumBC / N;

  iaOffset = sumIA / N;
  ibOffset = sumIB / N;
  icOffset = sumIC / N;

  telemetryEnabled = prevTele;

  U.println();
  U.println("ADC calibration done:");
  U.print("BEMF offsets: ");
  U.print(bemfAOffset); U.print(", ");
  U.print(bemfBOffset); U.print(", ");
  U.println(bemfCOffset);

  U.print("I offsets   : ");
  U.print(iaOffset); U.print(", ");
  U.print(ibOffset); U.print(", ");
  U.println(icOffset);
  U.println();
}

// ============================================================
// SECTION: SINE GENERATOR
// ============================================================
static void updateSineOpenLoop() {
  const uint32_t nowUs = micros();
  const uint32_t periodUs = 1000000UL / CONTROL_HZ;
  if ((uint32_t)(nowUs - lastControlUs) < periodUs) return;

  const float dt = (float)(nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;

  if (driveMode != MODE_RUN || emergencyStop) {
    return;
  }

  modulation = moveToward(modulation, clampf(targetMod, MOD_MIN, MOD_MAX), MOD_SLEW_PER_S * dt);
  elecFreqHz = moveToward(elecFreqHz, clampf(targetFreqHz, FREQ_MIN, FREQ_MAX), FREQ_SLEW_PER_S * dt);

  const float dir = reverseDir ? -1.0f : 1.0f;
  theta += dir * TWO_PI_F * elecFreqHz * dt;

  while (theta >= TWO_PI_F) theta -= TWO_PI_F;
  while (theta < 0.0f)      theta += TWO_PI_F;

  const float phi = phaseOffsetDeg * (TWO_PI_F / 360.0f);

  const float a = 0.5f + 0.5f * modulation * sinf(theta + phi);
  const float b = 0.5f + 0.5f * modulation * sinf(theta + phi - 2.09439510239f);
  const float c = 0.5f + 0.5f * modulation * sinf(theta + phi + 2.09439510239f);

  TIM1->CCR1 = clampDutyToCCR(a);
  TIM1->CCR2 = clampDutyToCCR(b);
  TIM1->CCR3 = clampDutyToCCR(c);
}

// ============================================================
// SECTION: ZERO-CROSS DETECTION
// ============================================================
static ZcPhaseState zcA = {ZC_SIGN_UNKNOWN, 0, 0, false, 0};
static ZcPhaseState zcB = {ZC_SIGN_UNKNOWN, 0, 0, false, 0};
static ZcPhaseState zcC = {ZC_SIGN_UNKNOWN, 0, 0, false, 0};

static inline void iirFilterBn(int16_t rawA, int16_t rawB, int16_t rawC) {
  filtBnA += (rawA - filtBnA) >> 2;
  filtBnB += (rawB - filtBnB) >> 2;
  filtBnC += (rawC - filtBnC) >> 2;
}

static void updateOneZcPhase(ZcPhaseState &st, int16_t v, uint32_t nowUs) {
  ZcSign newSign = st.sign;

  if (v > zcThresholdAdc) {
    newSign = ZC_SIGN_POS;
  } else if (v < -zcThresholdAdc) {
    newSign = ZC_SIGN_NEG;
  } else {
    return;
  }

  if (st.sign == ZC_SIGN_UNKNOWN) {
    st.sign = newSign;
    return;
  }

  if (newSign != st.sign) {
    st.intervalUs = nowUs - st.lastCrossUs;
    st.lastCrossUs = nowUs;
    st.lastEdgeRising = (st.sign == ZC_SIGN_NEG && newSign == ZC_SIGN_POS);
    st.crossCount++;
    st.sign = newSign;
  }
}

static void updateBemfAndZc() {
  lastAdc = readAllAdc();
  AdcSnapshot &s = lastAdc;

  int16_t neutral = ((int32_t)s.bemfA + (int32_t)s.bemfB + (int32_t)s.bemfC) / 3;

  lastBnA = (int16_t)s.bemfA - neutral;
  lastBnB = (int16_t)s.bemfB - neutral;
  lastBnC = (int16_t)s.bemfC - neutral;

  iirFilterBn(lastBnA, lastBnB, lastBnC);

  uint32_t nowUs = micros();
  updateOneZcPhase(zcA, filtBnA, nowUs);
  updateOneZcPhase(zcB, filtBnB, nowUs);
  updateOneZcPhase(zcC, filtBnC, nowUs);
}

// ============================================================
// SECTION: TELEMETRY
// ============================================================
static void printStatusShort() {
  U.print("F=");
  U.print(elecFreqHz, 1);
  U.print(" M=");
  U.print(modulation, 2);
  U.print(" DT=");
  U.print(deadtimeRaw);
  U.print(" ZA=");
  U.print(zcA.intervalUs);
  U.print(" ZB=");
  U.print(zcB.intervalUs);
  U.print(" ZC=");
  U.println(zcC.intervalUs);
}

static void printStatusFull() {
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
  U.print(" DT=");
  U.print(deadtimeRaw);
  U.print(" Zth=");
  U.print(zcThresholdAdc);
  U.print(" CCR=");
  U.print(TIM1->CCR1);
  U.print(",");
  U.print(TIM1->CCR2);
  U.print(",");
  U.println(TIM1->CCR3);
}

static void printAdcTelemetryFromSnapshot() {
  const AdcSnapshot &s = lastAdc;

  int16_t ia = (int16_t)s.ia - (int16_t)iaOffset;
  int16_t ib = (int16_t)s.ib - (int16_t)ibOffset;
  int16_t ic = (int16_t)s.ic - (int16_t)icOffset;

  int16_t ba = (int16_t)s.bemfA - (int16_t)bemfAOffset;
  int16_t bb = (int16_t)s.bemfB - (int16_t)bemfBOffset;
  int16_t bc = (int16_t)s.bemfC - (int16_t)bemfCOffset;

  int16_t neutral = ((int32_t)s.bemfA + (int32_t)s.bemfB + (int32_t)s.bemfC) / 3;
  int16_t baN = (int16_t)s.bemfA - neutral;
  int16_t bbN = (int16_t)s.bemfB - neutral;
  int16_t bcN = (int16_t)s.bemfC - neutral;

  U.print("VBAT=");
  U.print(readVBATVolts(s.vbat), 2);
  U.print("V");

  if (verboseTelemetry) {
    U.print(" Bn=");
    U.print(baN); U.print(",");
    U.print(bbN); U.print(",");
    U.print(bcN);

    U.print(" Bf=");
    U.print(filtBnA); U.print(",");
    U.print(filtBnB); U.print(",");
    U.print(filtBnC);

    U.print(" I=");
    U.print(ia); U.print(",");
    U.print(ib); U.print(",");
    U.print(ic);

    U.print(" Boff=");
    U.print(ba); U.print(",");
    U.print(bb); U.print(",");
    U.print(bc);
  }

  U.println();
}

static void printZcTelemetry() {
  if (!zcTelemetryEnabled) return;

  U.print("ZC A:");
  U.print(zcA.crossCount);
  U.print("/");
  U.print(zcA.lastEdgeRising ? "R" : "F");
  U.print("/");
  U.print(zcA.intervalUs);

  U.print(" B:");
  U.print(zcB.crossCount);
  U.print("/");
  U.print(zcB.lastEdgeRising ? "R" : "F");
  U.print("/");
  U.print(zcB.intervalUs);

  U.print(" C:");
  U.print(zcC.crossCount);
  U.print("/");
  U.print(zcC.lastEdgeRising ? "R" : "F");
  U.print("/");
  U.println(zcC.intervalUs);
}

static void printHelp() {
  U.println();
  U.println("=== ESC SENSORLESS BENCH + ZC ===");
  U.println("SPACE : EMERGENCY OFF");
  U.println("e     : enable RUN");
  U.println("f     : FREEWHEEL");
  U.println("0     : neutral reset");
  U.println("r     : reverse toggle");
  U.println("[ ]   : target modulation -/+");
  U.println("- +   : target electrical freq -/+");
  U.println(", .   : phase offset -/+");
  U.println("{ }   : deadtime raw -/+");
  U.println("z x   : ZC threshold -/+");
  U.println("t     : telemetry ON/OFF");
  U.println("1     : telemetry 100 ms");
  U.println("2     : telemetry 200 ms");
  U.println("5     : telemetry 500 ms");
  U.println("v     : verbose ADC telemetry ON/OFF");
  U.println("k     : ZC telemetry ON/OFF");
  U.println("c     : ADC recalibration");
  U.println("p     : print full status once");
  U.println("h     : help");
  U.println("=================================");
  U.println();
}

// ============================================================
// SECTION: INPUT HANDLING
// ============================================================
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

    case '+':
    case '=':
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

    case '{':
      if (deadtimeRaw > 1) deadtimeRaw--;
      applyDeadtimeRaw(deadtimeRaw);
      U.print("DT=");
      U.println(deadtimeRaw);
      break;

    case '}':
      if (deadtimeRaw < 127) deadtimeRaw++;
      applyDeadtimeRaw(deadtimeRaw);
      U.print("DT=");
      U.println(deadtimeRaw);
      break;

    case 'z':
    case 'Z':
      if (zcThresholdAdc > 2) zcThresholdAdc -= 1;
      U.print("Zth=");
      U.println(zcThresholdAdc);
      break;

    case 'x':
    case 'X':
      if (zcThresholdAdc < 200) zcThresholdAdc += 1;
      U.print("Zth=");
      U.println(zcThresholdAdc);
      break;

    case 't':
    case 'T':
      telemetryEnabled = !telemetryEnabled;
      U.print("telemetry ");
      U.println(telemetryEnabled ? "ON" : "OFF");
      break;

    case '1':
      telemetryPeriodMs = 100;
      U.println("telemetry 100 ms");
      break;

    case '2':
      telemetryPeriodMs = 200;
      U.println("telemetry 200 ms");
      break;

    case '5':
      telemetryPeriodMs = 500;
      U.println("telemetry 500 ms");
      break;

    case 'v':
    case 'V':
      verboseTelemetry = !verboseTelemetry;
      U.print("verbose ADC ");
      U.println(verboseTelemetry ? "ON" : "OFF");
      break;

    case 'k':
    case 'K':
      zcTelemetryEnabled = !zcTelemetryEnabled;
      U.print("ZC telemetry ");
      U.println(zcTelemetryEnabled ? "ON" : "OFF");
      break;

    case 'c':
    case 'C':
      calibrateAdcOffsets();
      break;

    case 'p':
    case 'P':
      printStatusFull();
      printAdcTelemetryFromSnapshot();
      printZcTelemetry();
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    default:
      break;
  }
}

// ============================================================
// SECTION: TELEMETRY SCHEDULER
// ============================================================
static void serviceTelemetry() {
  if (!telemetryEnabled) return;

  uint32_t now = millis();
  if (now - lastTelemetryMs < telemetryPeriodMs) return;
  lastTelemetryMs = now;

  if (driveMode == MODE_RUN) {
    switch (telePage) {
      case TELE_PAGE_SHORT:
        printStatusShort();
        telePage = TELE_PAGE_ADC;
        break;

      case TELE_PAGE_ADC:
        if (verboseTelemetry) {
          printAdcTelemetryFromSnapshot();
        } else {
          printStatusShort();
        }
        telePage = TELE_PAGE_ZC;
        break;

      case TELE_PAGE_ZC:
        if (zcTelemetryEnabled) {
          printZcTelemetry();
        } else {
          printStatusShort();
        }
        telePage = TELE_PAGE_SHORT;
        break;
    }
  } else {
    switch (telePage) {
      case TELE_PAGE_SHORT:
        printStatusFull();
        telePage = TELE_PAGE_ADC;
        break;

      case TELE_PAGE_ADC:
        printAdcTelemetryFromSnapshot();
        telePage = TELE_PAGE_ZC;
        break;

      case TELE_PAGE_ZC:
        printZcTelemetry();
        telePage = TELE_PAGE_SHORT;
        break;
    }
  }
}

// ============================================================
// SECTION: SETUP / LOOP
// ============================================================
void setup() {
  pinMode(PIN_BEMF_A, INPUT_ANALOG);
  pinMode(PIN_BEMF_B, INPUT_ANALOG);
  pinMode(PIN_BEMF_C, INPUT_ANALOG);
  pinMode(PIN_IA, INPUT_ANALOG);
  pinMode(PIN_IB, INPUT_ANALOG);
  pinMode(PIN_IC, INPUT_ANALOG);
  pinMode(PIN_VBAT, INPUT_ANALOG);

  analogReadResolution(12);

  U.begin(UART_BAUD);
  delay(200);

  U.println("\nSTM32F103 ESC sensorless bench firmware");
  U.println("Open-loop sine + BEMF/current/VBAT telemetry + ZC monitor");
  U.println("Telemetry optimized for 38400 baud");
  U.println("Freewheel/ESTOP: HIN=0, LIN=1");
  U.println("Low voltage first.");

  setupGPIO_TIM1_6PWM();
  setupTIM1();

  setFreewheelMode();
  lastControlUs = micros();

  calibrateAdcOffsets();
  lastAdc = readAllAdc();

  printHelp();
  printStatusFull();
}

void loop() {
  // fast path first
  updateSineOpenLoop();
  updateBemfAndZc();

  // commands
  while (U.available()) {
    char ch = (char)U.read();
    handleKey(ch);
  }

  // slow path
  serviceTelemetry();
}
