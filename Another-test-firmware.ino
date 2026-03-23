#include <Arduino.h>
#include <HardwareSerial.h>
#include "stm32f1xx_hal.h"

// ============================================================
// ESC STM32F103 TEST FW
// - TRUE 6PWM active phases
// - GPIO hard OFF for OFF phases
// - BEMF telemetry
// - low-side current sense telemetry
// - VBAT with UV protection
// - open-loop run + startup ramp
// - UART key control
//
// BOARD MODEL:
// PA8  -> TIM1_CH1  -> HIN_A
// PA9  -> TIM1_CH2  -> HIN_B
// PA10 -> TIM1_CH3  -> HIN_C
//
// PB13 -> TIM1_CH1N -> LIN_A
// PB14 -> TIM1_CH2N -> LIN_B
// PB15 -> TIM1_CH3N -> LIN_C
//
// SAFE OFF PER PHASE:
// HIN = LOW
// LIN = HIGH
//
// IMPORTANT:
// Current shunts are only on low-side emitters.
// Current readings are only meaningful when the corresponding
// low-side path is actually conducting.
// ============================================================

// ================== UART ==================
HardwareSerial U(PB7, PB6);
static const uint32_t UART_BAUD = 38400;

// ================== LED ==================
#define LED_PIN PB2

// ================== CORE PINS ==================
// ADC
#define BEMF_A_PIN PA0
#define BEMF_B_PIN PA1
#define BEMF_C_PIN PA2
#define THROTTLE_PIN PA3
#define IA_PIN PA4
#define IB_PIN PA5
#define IC_PIN PA6
#define VBAT_PIN PA7
#define NTC_PIN PB0
#define BRAKE_PIN PB1

// TIM1 bridge pins
#define A_HIN_PIN GPIO_PIN_8
#define B_HIN_PIN GPIO_PIN_9
#define C_HIN_PIN GPIO_PIN_10
#define HIN_PORT GPIOA

#define A_LIN_PIN GPIO_PIN_13
#define B_LIN_PIN GPIO_PIN_14
#define C_LIN_PIN GPIO_PIN_15
#define LIN_PORT GPIOB

// ================== TIM1 CONFIG ==================
TIM_HandleTypeDef htim1;

static const uint32_t TIM_CLK_HZ = 72000000UL;
static const uint32_t PWM_FREQ_HZ = 20000UL;
static const uint16_t TIM_PSC = 0;

// center aligned:
// Fpwm = TIMclk / (2 * (PSC+1) * (ARR+1))
static const uint16_t TIM_ARR =
    (uint16_t)((TIM_CLK_HZ / (2UL * (TIM_PSC + 1UL) * PWM_FREQ_HZ)) - 1UL);

static const uint8_t DEADTIME_TICKS = 72; // ~1us @72MHz

// ================== ADC / CAL ==================
static const float ADC_VREF = 3.3f;
static const float ADC_FS   = 4095.0f;
static const float VBAT_SCALE = 31.1f;

// low-side shunt offsets
static const int IA_OFF = 2008;
static const int IB_OFF = 1968;
static const int IC_OFF = 2001;

// relative gains from resistor tests
static const float IA_GAIN = 1.02f;
static const float IB_GAIN = 1.11f;
static const float IC_GAIN = 0.89f;

// ================== VBAT PROTECTION ==================
static const float VBAT_UV_TRIP    = 31.5f;
static const float VBAT_UV_RELEASE = 33.0f;

// ignore absurd readings and hold last sane VBAT
static const float VBAT_VALID_MIN = 20.0f;
static const float VBAT_VALID_MAX = 45.0f;

// ================== CONTROL STATE ==================
enum PhaseMode : uint8_t {
  PHASE_OFF = 0,
  PHASE_PWM = 1,
  PHASE_LOW = 2
};

static bool bridgeEnabled = false;
static bool autoRun = false;
static bool plotterMode = false;
static bool uvFault = false;
static bool telemetryEnabled = true;

static uint8_t dutyPct = 8;       // active duty
static uint8_t stepIndex = 0;     // 0..5
static uint32_t autoStepPeriodUs = 2500;

// startup ramp config
static uint8_t startupDutyStartPct = 6;
static uint8_t startupDutyEndPct   = 16;
static uint32_t startupPeriodStartUs = 9000;
static uint32_t startupPeriodEndUs   = 2200;
static uint16_t startupSteps = 260;
static bool startupLeavesAutorun = true;

// telemetry timing
uint32_t tTelemetry = 0;
uint32_t tBlink = 0;
uint32_t tAutoUs = 0;

// vbat tracking
static float lastGoodVbat = 0.0f;
static bool haveGoodVbat = false;

// ================== TELEMETRY STRUCT ==================
struct Telemetry {
  int rawBemfA;
  int rawBemfB;
  int rawBemfC;

  int rawIa;
  int rawIb;
  int rawIc;

  int rawVbat;
  int rawThrottle;
  int rawNtc;
  int rawBrake;

  float bemfA_V;
  float bemfB_V;
  float bemfC_V;

  float ia_V;
  float ib_V;
  float ic_V;

  float ia_mV_corr;
  float ib_mV_corr;
  float ic_mV_corr;

  float vbat_V;
  float throttle_V;
  float ntc_V;
};

// ================== HELPERS ==================

static inline float rawToAdcV(int raw) {
  return ((float)raw * ADC_VREF) / ADC_FS;
}

static inline float rawDeltaToMilliV(float rawDelta) {
  return rawDelta * (ADC_VREF * 1000.0f / ADC_FS);
}

static inline float rawToVbatV(int raw) {
  return rawToAdcV(raw) * VBAT_SCALE;
}

static inline uint16_t dutyToCcr(uint8_t pct) {
  if (pct == 0) return 0;
  if (pct >= 100) return TIM_ARR;
  return (uint16_t)(((uint32_t)(TIM_ARR + 1U) * pct) / 100U);
}

static inline uint16_t lerpU16(uint16_t a, uint16_t b, uint16_t i, uint16_t n) {
  if (n == 0) return a;
  return (uint16_t)(a + (((int32_t)(b - a) * i) / n));
}

static inline uint32_t lerpU32(uint32_t a, uint32_t b, uint16_t i, uint16_t n) {
  if (n == 0) return a;
  return (uint32_t)(a + (((int32_t)(b - a) * i) / n));
}

// ================== TIMER LOW LEVEL ==================

static inline void setCCR1(uint16_t v) { TIM1->CCR1 = v; }
static inline void setCCR2(uint16_t v) { TIM1->CCR2 = v; }
static inline void setCCR3(uint16_t v) { TIM1->CCR3 = v; }

static inline void ch1Enable(bool en) {
  if (en) TIM1->CCER |=  (TIM_CCER_CC1E | TIM_CCER_CC1NE);
  else    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
}
static inline void ch2Enable(bool en) {
  if (en) TIM1->CCER |=  (TIM_CCER_CC2E | TIM_CCER_CC2NE);
  else    TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
}
static inline void ch3Enable(bool en) {
  if (en) TIM1->CCER |=  (TIM_CCER_CC3E | TIM_CCER_CC3NE);
  else    TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

static inline void outputsMasterEnable(bool en) {
  if (en) TIM1->BDTR |= TIM_BDTR_MOE;
  else    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

// ================== GPIO MODE SWITCH ==================

static void pinAsGpioPP(GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef s = {0};
  s.Pin = pin;
  s.Mode = GPIO_MODE_OUTPUT_PP;
  s.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &s);
}

static void pinAsAfPP(GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef s = {0};
  s.Pin = pin;
  s.Mode = GPIO_MODE_AF_PP;
  s.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &s);
}

// ================== HARD OFF PER PHASE ==================

static void forcePhaseAOffGPIO() {
  ch1Enable(false);
  pinAsGpioPP(HIN_PORT, A_HIN_PIN);
  pinAsGpioPP(LIN_PORT, A_LIN_PIN);
  HAL_GPIO_WritePin(HIN_PORT, A_HIN_PIN, GPIO_PIN_RESET); // HIN LOW
  HAL_GPIO_WritePin(LIN_PORT, A_LIN_PIN, GPIO_PIN_SET);   // LIN HIGH = OFF
}

static void forcePhaseBOffGPIO() {
  ch2Enable(false);
  pinAsGpioPP(HIN_PORT, B_HIN_PIN);
  pinAsGpioPP(LIN_PORT, B_LIN_PIN);
  HAL_GPIO_WritePin(HIN_PORT, B_HIN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_PORT, B_LIN_PIN, GPIO_PIN_SET);
}

static void forcePhaseCOffGPIO() {
  ch3Enable(false);
  pinAsGpioPP(HIN_PORT, C_HIN_PIN);
  pinAsGpioPP(LIN_PORT, C_LIN_PIN);
  HAL_GPIO_WritePin(HIN_PORT, C_HIN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_PORT, C_LIN_PIN, GPIO_PIN_SET);
}

static void allPhasesHardOffGPIO() {
  outputsMasterEnable(false);

  setCCR1(0);
  setCCR2(0);
  setCCR3(0);

  forcePhaseAOffGPIO();
  forcePhaseBOffGPIO();
  forcePhaseCOffGPIO();

  TIM1->EGR = TIM_EGR_UG;
}

static void phaseAtoTimer() {
  pinAsAfPP(HIN_PORT, A_HIN_PIN);
  pinAsAfPP(LIN_PORT, A_LIN_PIN);
}

static void phaseBtoTimer() {
  pinAsAfPP(HIN_PORT, B_HIN_PIN);
  pinAsAfPP(LIN_PORT, B_LIN_PIN);
}

static void phaseCtoTimer() {
  pinAsAfPP(HIN_PORT, C_HIN_PIN);
  pinAsAfPP(LIN_PORT, C_LIN_PIN);
}

// ================== PHASE APPLY ==================
// PHASE_OFF: GPIO hard off
// PHASE_PWM: true complementary PWM
// PHASE_LOW: hardware-specific minus phase test state

static void applyPhaseA(PhaseMode mode, uint16_t ccr) {
  switch (mode) {
    case PHASE_OFF:
      forcePhaseAOffGPIO();
      setCCR1(0);
      break;
    case PHASE_PWM:
      phaseAtoTimer();
      setCCR1(ccr);
      ch1Enable(true);
      break;
    case PHASE_LOW:
      phaseAtoTimer();
      setCCR1(0);
      ch1Enable(true);
      break;
  }
}

static void applyPhaseB(PhaseMode mode, uint16_t ccr) {
  switch (mode) {
    case PHASE_OFF:
      forcePhaseBOffGPIO();
      setCCR2(0);
      break;
    case PHASE_PWM:
      phaseBtoTimer();
      setCCR2(ccr);
      ch2Enable(true);
      break;
    case PHASE_LOW:
      phaseBtoTimer();
      setCCR2(0);
      ch2Enable(true);
      break;
  }
}

static void applyPhaseC(PhaseMode mode, uint16_t ccr) {
  switch (mode) {
    case PHASE_OFF:
      forcePhaseCOffGPIO();
      setCCR3(0);
      break;
    case PHASE_PWM:
      phaseCtoTimer();
      setCCR3(ccr);
      ch3Enable(true);
      break;
    case PHASE_LOW:
      phaseCtoTimer();
      setCCR3(0);
      ch3Enable(true);
      break;
  }
}

// ================== STEP TABLE ==================

static void getStepModes(uint8_t s, PhaseMode &a, PhaseMode &b, PhaseMode &c) {
  switch (s % 6) {
    case 0: a = PHASE_PWM; b = PHASE_LOW; c = PHASE_OFF; break; // A+ B- C float
    case 1: a = PHASE_PWM; b = PHASE_OFF; c = PHASE_LOW; break; // A+ C- B float
    case 2: a = PHASE_OFF; b = PHASE_PWM; c = PHASE_LOW; break; // B+ C- A float
    case 3: a = PHASE_LOW; b = PHASE_PWM; c = PHASE_OFF; break; // B+ A- C float
    case 4: a = PHASE_LOW; b = PHASE_OFF; c = PHASE_PWM; break; // C+ A- B float
    case 5: a = PHASE_OFF; b = PHASE_LOW; c = PHASE_PWM; break; // C+ B- A float
  }
}

static void applyCurrentStep() {
  uint16_t ccr = dutyToCcr(dutyPct);

  allPhasesHardOffGPIO();
  delayMicroseconds(3);

  if (!bridgeEnabled || uvFault) {
    return;
  }

  PhaseMode a, b, c;
  getStepModes(stepIndex, a, b, c);

  applyPhaseA(a, ccr);
  applyPhaseB(b, ccr);
  applyPhaseC(c, ccr);

  TIM1->EGR = TIM_EGR_UG;
  outputsMasterEnable(true);
}

static void bridgeEnable() {
  bridgeEnabled = true;
  applyCurrentStep();
}

static void bridgeDisable() {
  bridgeEnabled = false;
  autoRun = false;
  allPhasesHardOffGPIO();
}

// ================== VBAT SANITY / UV ==================

static bool isVbatSane(float v) {
  return (v >= VBAT_VALID_MIN && v <= VBAT_VALID_MAX);
}

static float trustVbat(float v) {
  if (isVbatSane(v)) {
    lastGoodVbat = v;
    haveGoodVbat = true;
    return v;
  }
  if (haveGoodVbat) return lastGoodVbat;
  return v;
}

static void updateUvProtection(float vbat) {
  bool prev = uvFault;

  if (!uvFault) {
    if (vbat < VBAT_UV_TRIP) uvFault = true;
  } else {
    if (vbat > VBAT_UV_RELEASE) uvFault = false;
  }

  if (!prev && uvFault) {
    bridgeDisable();
    U.print("UV FAULT! VBAT=");
    U.println(vbat, 2);
  }

  if (prev && !uvFault) {
    U.print("UV CLEARED VBAT=");
    U.println(vbat, 2);
  }
}

// ================== ADC ==================

static int readAdcAvg(uint8_t pin, uint8_t samples = 8) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / samples);
}

static Telemetry readTelemetry() {
  Telemetry t;

  t.rawBemfA = readAdcAvg(BEMF_A_PIN, 4);
  t.rawBemfB = readAdcAvg(BEMF_B_PIN, 4);
  t.rawBemfC = readAdcAvg(BEMF_C_PIN, 4);

  t.rawIa = readAdcAvg(IA_PIN, 8);
  t.rawIb = readAdcAvg(IB_PIN, 8);
  t.rawIc = readAdcAvg(IC_PIN, 8);

  t.rawVbat = readAdcAvg(VBAT_PIN, 8);
  t.rawThrottle = readAdcAvg(THROTTLE_PIN, 4);
  t.rawNtc = readAdcAvg(NTC_PIN, 4);
  t.rawBrake = digitalRead(BRAKE_PIN);

  t.bemfA_V = rawToAdcV(t.rawBemfA);
  t.bemfB_V = rawToAdcV(t.rawBemfB);
  t.bemfC_V = rawToAdcV(t.rawBemfC);

  t.ia_V = rawToAdcV(t.rawIa);
  t.ib_V = rawToAdcV(t.rawIb);
  t.ic_V = rawToAdcV(t.rawIc);

  t.ia_mV_corr = rawDeltaToMilliV((t.rawIa - IA_OFF) * IA_GAIN);
  t.ib_mV_corr = rawDeltaToMilliV((t.rawIb - IB_OFF) * IB_GAIN);
  t.ic_mV_corr = rawDeltaToMilliV((t.rawIc - IC_OFF) * IC_GAIN);

  t.vbat_V = trustVbat(rawToVbatV(t.rawVbat));
  t.throttle_V = rawToAdcV(t.rawThrottle);
  t.ntc_V = rawToAdcV(t.rawNtc);

  return t;
}

// ================== PRINT ==================

static void printHelp() {
  U.println();
  U.println("=== ESC TEST FW HELP ===");
  U.println("h  help");
  U.println("i  status");
  U.println("e  enable bridge");
  U.println("o  hard off / stop");
  U.println("c  clear UV fault if VBAT recovered");
  U.println();
  U.println("0..5 direct step");
  U.println("n  next step");
  U.println("p  prev step");
  U.println();
  U.println("+  duty +1%");
  U.println("-  duty -1%");
  U.println("]  faster auto-run");
  U.println("[  slower auto-run");
  U.println("r  toggle auto-run");
  U.println("s  run startup ramp");
  U.println();
  U.println("g  toggle plotter mode");
  U.println("x  pause telemetry");
  U.println("v  resume telemetry");
  U.println("t  one-shot telemetry print");
  U.println();
  U.println("NOTE:");
  U.println("- OFF phase uses GPIO hard-off");
  U.println("- current sense is low-side / sector dependent");
  U.println();
}

static void printStepText(uint8_t s) {
  U.print("STEP ");
  U.print(s);
  U.print(" = ");
  switch (s % 6) {
    case 0: U.println("A+ B- C float"); break;
    case 1: U.println("A+ C- B float"); break;
    case 2: U.println("B+ C- A float"); break;
    case 3: U.println("B+ A- C float"); break;
    case 4: U.println("C+ A- B float"); break;
    case 5: U.println("C+ B- A float"); break;
  }
}

static void printStatus() {
  Telemetry t = readTelemetry();
  updateUvProtection(t.vbat_V);

  U.println();
  U.println("=== ESC TEST STATUS ===");
  U.print("Bridge      = "); U.println(bridgeEnabled ? "ENABLED" : "DISABLED");
  U.print("AutoRun     = "); U.println(autoRun ? "ON" : "OFF");
  U.print("Plotter     = "); U.println(plotterMode ? "ON" : "OFF");
  U.print("Telemetry   = "); U.println(telemetryEnabled ? "ON" : "PAUSED");
  U.print("UV Fault    = "); U.println(uvFault ? "YES" : "NO");
  U.print("Duty        = "); U.print(dutyPct); U.println("%");
  U.print("AutoPeriod  = "); U.print(autoStepPeriodUs); U.println(" us");
  printStepText(stepIndex);

  U.print("VBAT        = "); U.print(t.vbat_V, 3); U.print(" V (raw "); U.print(t.rawVbat); U.println(")");
  U.print("Throttle    = "); U.print(t.throttle_V, 3); U.print(" V (raw "); U.print(t.rawThrottle); U.println(")");

  U.print("BEMF A/B/C  = ");
  U.print(t.bemfA_V, 3); U.print(" / ");
  U.print(t.bemfB_V, 3); U.print(" / ");
  U.print(t.bemfC_V, 3); U.println(" V");

  U.print("IA/IB/IC mV = ");
  U.print(t.ia_mV_corr, 2); U.print(" / ");
  U.print(t.ib_mV_corr, 2); U.print(" / ");
  U.print(t.ic_mV_corr, 2); U.println(" mV");

  U.println();
}

static void printTelemetryText() {
  Telemetry t = readTelemetry();
  updateUvProtection(t.vbat_V);

  U.print("STEP=");
  U.print(stepIndex);
  U.print(" DUTY=");
  U.print(dutyPct);
  U.print(" EN=");
  U.print(bridgeEnabled ? 1 : 0);
  U.print(" RUN=");
  U.print(autoRun ? 1 : 0);
  U.print(" UV=");
  U.print(uvFault ? 1 : 0);

  U.print(" | VBAT=");
  U.print(t.vbat_V, 3);

  U.print(" | BEMF=");
  U.print(t.bemfA_V, 3);
  U.print(",");
  U.print(t.bemfB_V, 3);
  U.print(",");
  U.print(t.bemfC_V, 3);

  U.print(" | Icorr_mV=");
  U.print(t.ia_mV_corr, 2);
  U.print(",");
  U.print(t.ib_mV_corr, 2);
  U.print(",");
  U.print(t.ic_mV_corr, 2);

  U.print(" | THR=");
  U.print(t.throttle_V, 3);

  U.println();
}

static void printTelemetryPlotter() {
  Telemetry t = readTelemetry();
  updateUvProtection(t.vbat_V);

  U.print("bemfA:");
  U.print(t.bemfA_V, 3);
  U.print(",bemfB:");
  U.print(t.bemfB_V, 3);
  U.print(",bemfC:");
  U.print(t.bemfC_V, 3);

  U.print(",ia_mV:");
  U.print(t.ia_mV_corr, 2);
  U.print(",ib_mV:");
  U.print(t.ib_mV_corr, 2);
  U.print(",ic_mV:");
  U.print(t.ic_mV_corr, 2);

  U.print(",vbat:");
  U.print(t.vbat_V, 3);

  U.print(",step:");
  U.print((float)stepIndex, 1);

  U.print(",duty:");
  U.print((float)dutyPct, 1);

  U.println();
}

// ================== STARTUP RAMP ==================

static void runStartupRamp() {
  if (uvFault) {
    U.println("STARTUP BLOCKED: UV fault active");
    return;
  }

  U.println("STARTUP RAMP BEGIN");
  bridgeEnabled = true;
  autoRun = false;

  for (uint16_t i = 0; i < startupSteps; i++) {
    Telemetry t = readTelemetry();
    updateUvProtection(t.vbat_V);

    if (uvFault) {
      U.println("STARTUP ABORTED: UV fault");
      bridgeDisable();
      return;
    }

    dutyPct = (uint8_t)lerpU16(startupDutyStartPct, startupDutyEndPct, i, startupSteps - 1);
    uint32_t per = lerpU32(startupPeriodStartUs, startupPeriodEndUs, i, startupSteps - 1);

    applyCurrentStep();
    delayMicroseconds(per);

    stepIndex = (stepIndex + 1) % 6;

    if ((i % 32) == 0) {
      U.print("START i=");
      U.print(i);
      U.print(" duty=");
      U.print(dutyPct);
      U.print(" per=");
      U.print(per);
      U.print(" vbat=");
      U.println(t.vbat_V, 2);
    }
  }

  dutyPct = startupDutyEndPct;
  autoStepPeriodUs = startupPeriodEndUs;

  if (startupLeavesAutorun) {
    autoRun = true;
    bridgeEnabled = true;
    applyCurrentStep();
    U.println("STARTUP DONE -> AUTORUN ON");
  } else {
    bridgeEnabled = true;
    autoRun = false;
    applyCurrentStep();
    U.println("STARTUP DONE -> HOLD");
  }
}

// ================== INIT ==================

static void initBaseGPIO() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(BRAKE_PIN, INPUT);
  analogReadResolution(12);

  // start safe
  pinAsGpioPP(HIN_PORT, A_HIN_PIN);
  pinAsGpioPP(HIN_PORT, B_HIN_PIN);
  pinAsGpioPP(HIN_PORT, C_HIN_PIN);

  pinAsGpioPP(LIN_PORT, A_LIN_PIN);
  pinAsGpioPP(LIN_PORT, B_LIN_PIN);
  pinAsGpioPP(LIN_PORT, C_LIN_PIN);

  HAL_GPIO_WritePin(HIN_PORT, A_HIN_PIN | B_HIN_PIN | C_HIN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_PORT, A_LIN_PIN | B_LIN_PIN | C_LIN_PIN, GPIO_PIN_SET);
}

static void initTim1True6PWM() {
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM_PSC;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = TIM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    U.println("HAL_TIM_PWM_Init FAILED");
    while (1) {}
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; // HIN idle LOW
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET; // LIN idle HIGH

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) while (1) {}
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) while (1) {}
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) while (1) {}

  TIM_BreakDeadTimeConfigTypeDef sBDTR = {0};
  sBDTR.OffStateRunMode = TIM_OSSR_DISABLE;
  sBDTR.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBDTR.LockLevel = TIM_LOCKLEVEL_OFF;
  sBDTR.DeadTime = DEADTIME_TICKS;
  sBDTR.BreakState = TIM_BREAK_DISABLE;
  sBDTR.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBDTR.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBDTR) != HAL_OK) while (1) {}

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  allPhasesHardOffGPIO();
}

// ================== KEY HANDLER ==================

static void handleKey(char c) {
  switch (c) {
    case 'h':
      printHelp();
      break;

    case 'i':
      printStatus();
      break;

    case 'e':
      if (!uvFault) {
        bridgeEnable();
        U.println("BRIDGE ENABLED");
      } else {
        U.println("ENABLE BLOCKED: UV fault");
      }
      break;

    case 'o':
      bridgeDisable();
      U.println("HARD OFF");
      break;

    case 'c': {
      Telemetry t = readTelemetry();
      if (t.vbat_V > VBAT_UV_RELEASE) {
        uvFault = false;
        U.print("UV CLEARED MANUALLY, VBAT=");
        U.println(t.vbat_V, 2);
      } else {
        U.print("CANNOT CLEAR UV, VBAT=");
        U.println(t.vbat_V, 2);
      }
      break;
    }

    case 'n':
      stepIndex = (stepIndex + 1) % 6;
      applyCurrentStep();
      printStepText(stepIndex);
      break;

    case 'p':
      stepIndex = (stepIndex == 0) ? 5 : (stepIndex - 1);
      applyCurrentStep();
      printStepText(stepIndex);
      break;

    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      stepIndex = (uint8_t)(c - '0');
      applyCurrentStep();
      printStepText(stepIndex);
      break;

    case '+':
      if (dutyPct < 95) dutyPct++;
      applyCurrentStep();
      U.print("DUTY=");
      U.print(dutyPct);
      U.println("%");
      break;

    case '-':
      if (dutyPct > 0) dutyPct--;
      applyCurrentStep();
      U.print("DUTY=");
      U.print(dutyPct);
      U.println("%");
      break;

    case ']':
      if (autoStepPeriodUs > 500) autoStepPeriodUs -= 100;
      U.print("AUTO PERIOD=");
      U.print(autoStepPeriodUs);
      U.println(" us");
      break;

    case '[':
      if (autoStepPeriodUs < 50000) autoStepPeriodUs += 100;
      U.print("AUTO PERIOD=");
      U.print(autoStepPeriodUs);
      U.println(" us");
      break;

    case 'r':
      autoRun = !autoRun;
      U.print("AUTORUN=");
      U.println(autoRun ? "ON" : "OFF");
      break;

    case 's':
      runStartupRamp();
      break;

    case 'g':
      plotterMode = !plotterMode;
      U.print("PLOTTER MODE=");
      U.println(plotterMode ? "ON" : "OFF");
      break;

    case 'x':
      telemetryEnabled = false;
      U.println("TELEMETRY PAUSED");
      break;

    case 'v':
      telemetryEnabled = true;
      U.println("TELEMETRY RESUMED");
      break;

    case 't':
      if (plotterMode) printTelemetryPlotter();
      else printTelemetryText();
      break;

    default:
      break;
  }
}

// ================== SETUP ==================

void setup() {
  U.begin(UART_BAUD);
  delay(250);

  U.println();
  U.println("ESC STM32F103 TEST FW");
  U.println("TRUE 6PWM + BEMF + SENSING + UV + STARTUP");
  U.println("Init...");

  initBaseGPIO();
  initTim1True6PWM();

  printHelp();
  printStatus();
}

// ================== LOOP ==================

void loop() {
  while (U.available()) {
    char c = (char)U.read();
    handleKey(c);
  }

  // periodic UV check
  Telemetry t = readTelemetry();
  updateUvProtection(t.vbat_V);

  // autorun
  if (bridgeEnabled && autoRun && !uvFault) {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - tAutoUs) >= autoStepPeriodUs) {
      tAutoUs = nowUs;
      stepIndex = (stepIndex + 1) % 6;
      applyCurrentStep();
    }
  }

  // LED state
  if (uvFault) {
    if (millis() - tBlink >= 100) {
      tBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  } else if (!bridgeEnabled) {
    digitalWrite(LED_PIN, LOW);
  } else if (!autoRun) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (millis() - tBlink >= 200) {
      tBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  // telemetry
  uint32_t telemetryPeriod = plotterMode ? 35 : 300;
  if (telemetryEnabled && (millis() - tTelemetry >= telemetryPeriod)) {
    tTelemetry = millis();
    if (plotterMode) printTelemetryPlotter();
    else printTelemetryText();
  }
}
