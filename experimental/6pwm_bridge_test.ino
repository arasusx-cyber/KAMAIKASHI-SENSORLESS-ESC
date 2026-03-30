#include <Arduino.h>
#include <HardwareSerial.h>
#include "stm32f1xx_hal.h"

// ============================================================
// ESC STM32F103 Debug vol4
// TRUE 6PWM TEST + HARD OFF PER PHASE
//
// PA8  -> TIM1_CH1  -> HIN_A
// PA9  -> TIM1_CH2  -> HIN_B
// PA10 -> TIM1_CH3  -> HIN_C
//
// PB13 -> TIM1_CH1N -> LIN_A
// PB14 -> TIM1_CH2N -> LIN_B
// PB15 -> TIM1_CH3N -> LIN_C
//
// DRIVER ASSUMPTION:
// - HIN active HIGH
// - LIN active LOW
//
// SAFE OFF FOR ONE PHASE:
// - HIN = LOW
// - LIN = HIGH
//
// IMPORTANT:
// Active phases use TIM1 true complementary PWM.
// OFF phases are forced by GPIO, not trusted to TIM1 disable alone.
// ============================================================

// ================== UART ==================
HardwareSerial U(PB7, PB6);
static const uint32_t UART_BAUD = 9600;

// ================== LED ==================
#define LED_PIN PB2

// ================== TIM1 ==================
TIM_HandleTypeDef htim1;

static const uint32_t TIM_CLK_HZ   = 72000000UL;
static const uint32_t PWM_FREQ_HZ  = 20000UL;  // 20kHz
static const uint16_t TIM_PSC      = 0;

// center-aligned:
// Fpwm = TIMclk / (2 * (PSC+1) * (ARR+1))
static const uint16_t TIM_ARR =
    (uint16_t)((TIM_CLK_HZ / (2UL * (TIM_PSC + 1UL) * PWM_FREQ_HZ)) - 1UL);

// około 1us deadtime przy 72MHz
static const uint8_t DEADTIME_TICKS = 72;

// ================== PIN DEFINES ==================
#define A_HIN_PIN GPIO_PIN_8
#define B_HIN_PIN GPIO_PIN_9
#define C_HIN_PIN GPIO_PIN_10
#define HIN_PORT  GPIOA

#define A_LIN_PIN GPIO_PIN_13
#define B_LIN_PIN GPIO_PIN_14
#define C_LIN_PIN GPIO_PIN_15
#define LIN_PORT  GPIOB

// ================== STATE ==================
enum PhaseMode : uint8_t {
  PHASE_OFF = 0,
  PHASE_PWM = 1,
  PHASE_LOW = 2
};

enum RunMode : uint8_t {
  MODE_STEP = 0,
  MODE_MANUAL = 1
};

static bool bridgeEnabled = false;
static RunMode runMode = MODE_STEP;
static bool autoRun = false;

static uint8_t dutyPct = 10;          // start bezpiecznie
static uint8_t stepIndex = 0;         // 0..5
static uint32_t autoStepPeriodUs = 3000;

static PhaseMode manA = PHASE_OFF;
static PhaseMode manB = PHASE_OFF;
static PhaseMode manC = PHASE_OFF;

uint32_t tBlink = 0;
uint32_t tDot = 0;
uint32_t tAutoUs = 0;

// ================== HELPERS ==================

static const char* phaseModeName(PhaseMode m) {
  switch (m) {
    case PHASE_OFF: return "OFF";
    case PHASE_PWM: return "PWM";
    case PHASE_LOW: return "LOW";
    default: return "?";
  }
}

static const char* runModeName(RunMode m) {
  switch (m) {
    case MODE_STEP: return "STEP";
    case MODE_MANUAL: return "MANUAL";
    default: return "?";
  }
}

static inline uint16_t dutyToCcr(uint8_t pct) {
  if (pct == 0) return 0;
  if (pct >= 100) return TIM_ARR;
  return (uint16_t)(((uint32_t)(TIM_ARR + 1U) * pct) / 100U);
}

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void pinAsAfPP(GPIO_TypeDef* port, uint16_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
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

// ================== RETURN PHASE TO TIM1 ==================

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

// ================== APPLY SINGLE PHASE ==================
// PHASE_PWM:
//   true complementary PWM on phase
//
// PHASE_LOW:
//   hardware-specific trick for this board:
//   CCR=0 with enabled complementary channel pair.
//   This assumes your inverted CHxN setup where low-side becomes the active path.
//
// PHASE_OFF:
//   hard OFF by GPIO (HIN LOW, LIN HIGH)

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

static void applyCurrentConfiguration() {
  uint16_t ccr = dutyToCcr(dutyPct);

  // najpierw totalnie bezpiecznie wszystko wyłącz
  allPhasesHardOffGPIO();
  delayMicroseconds(3);

  if (!bridgeEnabled) {
    return;
  }

  PhaseMode a = PHASE_OFF;
  PhaseMode b = PHASE_OFF;
  PhaseMode c = PHASE_OFF;

  if (runMode == MODE_STEP) {
    getStepModes(stepIndex, a, b, c);
  } else {
    a = manA;
    b = manB;
    c = manC;
  }

  // ustaw fazy
  applyPhaseA(a, ccr);
  applyPhaseB(b, ccr);
  applyPhaseC(c, ccr);

  TIM1->EGR = TIM_EGR_UG;
  outputsMasterEnable(true);
}

static void bridgeEnable() {
  bridgeEnabled = true;
  applyCurrentConfiguration();
}

static void bridgeDisable() {
  bridgeEnabled = false;
  autoRun = false;
  allPhasesHardOffGPIO();
}

static PhaseMode nextPhaseMode(PhaseMode m) {
  switch (m) {
    case PHASE_OFF: return PHASE_PWM;
    case PHASE_PWM: return PHASE_LOW;
    case PHASE_LOW: return PHASE_OFF;
    default: return PHASE_OFF;
  }
}

// ================== PRINT ==================

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

static void printHelp() {
  U.println();
  U.println("=== TRUE 6PWM + HARD OFF HELP ===");
  U.println("General:");
  U.println("  h  - help");
  U.println("  i  - status");
  U.println("  e  - enable bridge");
  U.println("  o  - disable bridge / all hard OFF");
  U.println("  +  - duty +1%");
  U.println("  -  - duty -1%");
  U.println();
  U.println("Modes:");
  U.println("  s  - STEP mode");
  U.println("  m  - MANUAL mode");
  U.println("  r  - AUTO RUN toggle (STEP mode)");
  U.println();
  U.println("STEP mode:");
  U.println("  n  - next step");
  U.println("  p  - previous step");
  U.println("  0..5 - direct step");
  U.println();
  U.println("AUTO RUN:");
  U.println("  [  - slower");
  U.println("  ]  - faster");
  U.println();
  U.println("MANUAL mode:");
  U.println("  a  - phase A OFF->PWM->LOW");
  U.println("  b  - phase B OFF->PWM->LOW");
  U.println("  c  - phase C OFF->PWM->LOW");
  U.println();
}

static void printStatus() {
  U.println();
  U.println("=== TRUE 6PWM STATUS ===");
  U.print("Bridge:   "); U.println(bridgeEnabled ? "ENABLED" : "DISABLED");
  U.print("Mode:     "); U.println(runModeName(runMode));
  U.print("AutoRun:  "); U.println(autoRun ? "ON" : "OFF");
  U.print("Duty:     "); U.print(dutyPct); U.println("%");
  U.print("PWM freq: "); U.print(PWM_FREQ_HZ); U.println(" Hz");
  U.print("ARR:      "); U.println(TIM_ARR);
  U.print("Auto us:  "); U.println(autoStepPeriodUs);

  if (runMode == MODE_STEP) {
    printStepText(stepIndex);
  } else {
    U.print("Phase A:  "); U.println(phaseModeName(manA));
    U.print("Phase B:  "); U.println(phaseModeName(manB));
    U.print("Phase C:  "); U.println(phaseModeName(manC));
  }

  U.print("CCR1: "); U.print(TIM1->CCR1);
  U.print("  CCR2: "); U.print(TIM1->CCR2);
  U.print("  CCR3: "); U.println(TIM1->CCR3);

  U.print("CCER=0x"); U.println(TIM1->CCER, HEX);
  U.print("BDTR=0x"); U.println(TIM1->BDTR, HEX);
  U.println();
}

// ================== INIT ==================

static void initBaseGPIO() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    U.println("Config CH1 FAILED");
    while (1) {}
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    U.println("Config CH2 FAILED");
    while (1) {}
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    U.println("Config CH3 FAILED");
    while (1) {}
  }

  TIM_BreakDeadTimeConfigTypeDef sBDTR = {0};
  sBDTR.OffStateRunMode = TIM_OSSR_DISABLE;
  sBDTR.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBDTR.LockLevel = TIM_LOCKLEVEL_OFF;
  sBDTR.DeadTime = DEADTIME_TICKS;
  sBDTR.BreakState = TIM_BREAK_DISABLE;
  sBDTR.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBDTR.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBDTR) != HAL_OK) {
    U.println("BDTR config FAILED");
    while (1) {}
  }

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  allPhasesHardOffGPIO();
}

// ================== UART HANDLER ==================

static void handleChar(char c) {
  switch (c) {
    case 'h':
      printHelp();
      break;

    case 'i':
      printStatus();
      break;

    case 'e':
      bridgeEnable();
      U.println("Bridge ENABLED");
      break;

    case 'o':
      bridgeDisable();
      U.println("Bridge DISABLED / HARD OFF");
      break;

    case '+':
      if (dutyPct < 95) dutyPct++;
      applyCurrentConfiguration();
      U.print("Duty = "); U.print(dutyPct); U.println("%");
      break;

    case '-':
      if (dutyPct > 0) dutyPct--;
      applyCurrentConfiguration();
      U.print("Duty = "); U.print(dutyPct); U.println("%");
      break;

    case 's':
      runMode = MODE_STEP;
      autoRun = false;
      applyCurrentConfiguration();
      U.println("Mode = STEP");
      printStepText(stepIndex);
      break;

    case 'm':
      runMode = MODE_MANUAL;
      autoRun = false;
      applyCurrentConfiguration();
      U.println("Mode = MANUAL");
      break;

    case 'r':
      if (runMode == MODE_STEP) {
        autoRun = !autoRun;
        U.print("AutoRun = ");
        U.println(autoRun ? "ON" : "OFF");
      } else {
        U.println("AutoRun only in STEP mode");
      }
      break;

    case 'n':
      if (runMode == MODE_STEP) {
        stepIndex = (stepIndex + 1) % 6;
        applyCurrentConfiguration();
        printStepText(stepIndex);
      }
      break;

    case 'p':
      if (runMode == MODE_STEP) {
        stepIndex = (stepIndex == 0) ? 5 : (stepIndex - 1);
        applyCurrentConfiguration();
        printStepText(stepIndex);
      }
      break;

    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      if (runMode == MODE_STEP) {
        stepIndex = (uint8_t)(c - '0');
        applyCurrentConfiguration();
        printStepText(stepIndex);
      }
      break;

    case '[':
      if (autoStepPeriodUs < 50000) autoStepPeriodUs += 250;
      U.print("Auto step period = ");
      U.print(autoStepPeriodUs);
      U.println(" us");
      break;

    case ']':
      if (autoStepPeriodUs > 500) autoStepPeriodUs -= 250;
      U.print("Auto step period = ");
      U.print(autoStepPeriodUs);
      U.println(" us");
      break;

    case 'a':
      if (runMode == MODE_MANUAL) {
        manA = nextPhaseMode(manA);
        applyCurrentConfiguration();
        U.print("Phase A = ");
        U.println(phaseModeName(manA));
      }
      break;

    case 'b':
      if (runMode == MODE_MANUAL) {
        manB = nextPhaseMode(manB);
        applyCurrentConfiguration();
        U.print("Phase B = ");
        U.println(phaseModeName(manB));
      }
      break;

    case 'c':
      if (runMode == MODE_MANUAL) {
        manC = nextPhaseMode(manC);
        applyCurrentConfiguration();
        U.print("Phase C = ");
        U.println(phaseModeName(manC));
      }
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
  U.println("ESC STM32F103 Debug vol4");
  U.println("TRUE 6PWM TEST + HARD OFF PER PHASE");
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
    handleChar(c);
  }

  if (bridgeEnabled && autoRun && runMode == MODE_STEP) {
    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - tAutoUs) >= autoStepPeriodUs) {
      tAutoUs = nowUs;
      stepIndex = (stepIndex + 1) % 6;
      applyCurrentConfiguration();
    }
  }

  if (!bridgeEnabled) {
    digitalWrite(LED_PIN, LOW);
  } else if (!autoRun) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (millis() - tBlink >= 100) {
      tBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }

  if (millis() - tDot >= 2000) {
    tDot = millis();
    U.print(".");
  }
}
