#include <Arduino.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS (ESP32 WROOM 32E)
//
//  Avoided: GPIO 0,2,12,15 (strapping), GPIO 6–11 (flash),
//           GPIO 34–39 (input-only, used for ADC only)
// ─────────────────────────────────────────────

// TB6612FNG Motor Driver
#define PIN_AIN1  26
#define PIN_AIN2  25
#define PIN_PWMA  27   // LEDC output
#define PIN_STBY  14

// Quadrature Encoder — any GPIO is interrupt-capable on ESP32
#define PIN_ENCA  18   // Channel A — hardware interrupt
#define PIN_ENCB  19   // Channel B — polled inside ISR

// ACS712 Current Sensor
// ⚠️  Voltage divider required (10kΩ/10kΩ) between ACS712 OUT and this pin
//     ACS712 runs on 5V (output 0–5V). Divider halves it → 0–2.5V (safe for 3.3V ADC)
#define PIN_CURRENT  34  // ADC1 channel — input-only pin, ideal for analog

// ─────────────────────────────────────────────
//  LEDC (ESP32 PWM) CONFIGURATION
//  Using 8-bit resolution so the 0–255 user interface stays identical
// ─────────────────────────────────────────────
#define LEDC_CHANNEL    0
#define LEDC_FREQ_HZ    5000   // 5 kHz — good for most DC motors
#define LEDC_RESOLUTION 8      // 8-bit → duty range 0–255

// ─────────────────────────────────────────────
//  ACS712 CONSTANTS
//
//  Sensor powered at 5V:  mV_sensor = -180·A + 2500
//  After 10k/10k divider: mV_adc    = mV_sensor / 2
//  ESP32 ADC reference:   3300 mV, 12-bit (0–4095)
//
//  Solving for current:
//    mV_sensor = mV_adc × 2
//    A = (2500 - mV_sensor) / 180
//      = (2500 - mV_adc × 2) / 180
// ─────────────────────────────────────────────
static constexpr float ACS712_ZERO_ADC_MV   = 1260.0f; // mV at ADC pin corresponding to zero current
static constexpr float ACS712_SENS_MV_A     =  97.04f; // Sensitivity magnitude at sensor output (mV/A)
static constexpr float ADC_REF_MV           = 3300.0f; // ESP32 ADC reference (mV)
static constexpr float ADC_RESOLUTION       = 4095.0f; // 12-bit
static constexpr int   CURRENT_SAMPLES     =   5;

// ─────────────────────────────────────────────
//  ENCODER & GEARBOX CONSTANTS
//  11 pulses/motor-rev, gear ratio 1:472.7272
//  → 5200 pulses per output revolution
// ─────────────────────────────────────────────
static constexpr float    PULSES_PER_MOTOR_REV  =   11.0f;
static constexpr float    GEAR_RATIO            =  472.7272f;
static constexpr float    PULSES_PER_OUTPUT_REV =  PULSES_PER_MOTOR_REV * GEAR_RATIO; // 5200.0

static constexpr uint32_t CONTROL_PERIOD_MS     = 20;

// ─────────────────────────────────────────────
//  TELEMETRY TIMING
// ─────────────────────────────────────────────
static constexpr uint32_t TELEMETRY_MS = 500;
static uint32_t lastTelemetry          = 0;

static constexpr float RAD_PER_SEC_TO_RPM = 60.0f / (2.0f * PI);

// PID gains from Simulink
static constexpr float PID_KC = 134.345399f;
static constexpr float PID_TI = 0.110091f;
static constexpr float PID_TD = 0.036606f;

// Back-calculation anti-windup gain.
static const float PID_KB = 1.0f / sqrtf(PID_TI * PID_TD);

static constexpr float PWM_MIN = 0.0f;
static constexpr float PWM_MAX = 255.0f;

struct PIDState {
  float setpointRadPerSec = 0.0f;
  float integrator        = 0.0f;
  float prevMeasurement   = 0.0f;
  bool  enabled           = false;
  uint32_t lastUpdateMs   = 0;
} pid;

// ─────────────────────────────────────────────
//  MOTOR STATE
// ─────────────────────────────────────────────
struct MotorState {
  uint8_t pwm     = 0;
  bool    forward = true;
  bool    running = false;
} motor;

// ─────────────────────────────────────────────
//  ENCODER STATE
//
//  volatile: shared between ISR and main loop.
//  On ESP32 (32-bit Xtensa), a 32-bit aligned read is atomic at the hardware
//  level, but FreeRTOS can preempt tasks, so we still use critical sections
//  to be safe and to satisfy the compiler's memory-ordering requirements.
// ─────────────────────────────────────────────
volatile long encoderPulses = 0;

long     speedPulseSnapshot = 0;
uint32_t speedLastCalcMs    = 0;
float    outputRadPerSec    = 0.0f;

// FreeRTOS critical section handle (used instead of noInterrupts on ESP32)
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────
//  PROTOTYPES
// ─────────────────────────────────────────────
void  applyMotor();
void  stopMotor();
float readCurrentAmps();
void  processCommand(const String& raw);
void  updateSpeedRadPerSec();
void  updatePidControl();
float clampf(float value, float lowerBound, float upperBound);
void  resetPidState();
void  printTelemetry(float amps);
void  printHelp();
void  IRAM_ATTR encoderISR();

// ═════════════════════════════════════════════
//  ENCODER ISR
//
//  IRAM_ATTR: forces the function into IRAM (internal RAM) so it can execute
//  even if the flash cache is busy. Required for ISRs on ESP32.
//
//  portENTER/EXIT_CRITICAL_ISR: FreeRTOS-safe spinlock for ISR context.
//  Use this instead of noInterrupts() inside an ISR on ESP32.
//
//  Direction: ENCA ↑ while ENCB=HIGH → forward (+1)
//             ENCA ↑ while ENCB=LOW  → reverse (-1)
// ═════════════════════════════════════════════
void IRAM_ATTR encoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  if (digitalRead(PIN_ENCB) == HIGH) {
    encoderPulses++;
  } else {
    encoderPulses--;
  }
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Motor driver pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  // PIN_PWMA configured by LEDC — do NOT call pinMode on it separately

  // LEDC setup — replaces analogWrite() on ESP32
  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcAttachPin(PIN_PWMA, LEDC_CHANNEL);

  // Encoder pins
  // INPUT_PULLUP in case encoder has open-collector outputs
  pinMode(PIN_ENCA, INPUT_PULLUP);
  pinMode(PIN_ENCB, INPUT_PULLUP);

  // Any GPIO can trigger interrupts on ESP32 — no pin constraint like the Uno
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), encoderISR, RISING);

  stopMotor();
  digitalWrite(PIN_STBY, HIGH);  // Release driver from standby

  speedLastCalcMs = millis();
  resetPidState();
  printHelp();
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  updateSpeedRadPerSec();
  updatePidControl();

  if (now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    printTelemetry(readCurrentAmps());
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) processCommand(cmd);
  }
}

// ═════════════════════════════════════════════
//  SPEED CALCULATION
// ═════════════════════════════════════════════
void updateSpeedRadPerSec() {
  uint32_t now     = millis();
  uint32_t elapsed = now - speedLastCalcMs;
  if (elapsed < CONTROL_PERIOD_MS) return;

  // FreeRTOS-safe read of volatile long from main-loop context
  portENTER_CRITICAL(&encoderMux);
  long currentPulses = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  long  deltaPulses = currentPulses - speedPulseSnapshot;
  float elapsedSec  = (float)elapsed / 1000.0f;

  float outputRevPerSec = ((float)deltaPulses / PULSES_PER_OUTPUT_REV) / elapsedSec;
  outputRadPerSec       = outputRevPerSec * (2.0f * PI);
  speedPulseSnapshot = currentPulses;
  speedLastCalcMs    = now;
}

void updatePidControl() {
  if (!pid.enabled) return;

  uint32_t now = millis();
  uint32_t elapsed = now - pid.lastUpdateMs;
  if (elapsed < CONTROL_PERIOD_MS) return;

  float dt = (float)elapsed / 1000.0f;

  float error = pid.setpointRadPerSec - outputRadPerSec;

  float proportional = PID_KC * error;
  float derivative    = -PID_KC * PID_TD * ((outputRadPerSec - pid.prevMeasurement) / dt);
  float unsatOutput   = proportional + pid.integrator + derivative;
  float satOutput     = clampf(unsatOutput, -PWM_MAX, PWM_MAX);

  pid.integrator += dt * ((PID_KC / PID_TI) * error + PID_KB * (satOutput - unsatOutput));
  pid.prevMeasurement = outputRadPerSec;
  pid.lastUpdateMs = now;

  motor.forward = (satOutput >= 0.0f);
  motor.pwm = (uint8_t)clampf(fabsf(satOutput), PWM_MIN, PWM_MAX);
  motor.running = true;
  applyMotor();
}

float clampf(float value, float lowerBound, float upperBound) {
  if (value < lowerBound) return lowerBound;
  if (value > upperBound) return upperBound;
  return value;
}

void resetPidState() {
  pid.integrator = 0.0f;
  pid.prevMeasurement = outputRadPerSec;
  pid.lastUpdateMs = millis();
}

// ═════════════════════════════════════════════
//  COMMAND PARSER
// ═════════════════════════════════════════════
void processCommand(const String& raw) {
  String cmd = raw;
  cmd.toUpperCase();

  // ── S <0-255> / SPEED <0-255> ───────────────
  if (cmd.startsWith("S ") || cmd.startsWith("SPEED ")) {
    int sp    = cmd.substring(cmd.indexOf(' ') + 1).toInt();
    motor.pwm = (uint8_t)constrain(sp, 0, 255);
    if (motor.running) applyMotor();
    Serial.print(F("[OK] PWM set to "));
    Serial.print(motor.pwm);
    Serial.println(F("/255 (manual mode)"));

  // ── SP <rad/s> / SET <rad/s> ────────────────
  } else if (cmd.startsWith(F("SP ")) || cmd.startsWith(F("SET "))) {
    float target = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    pid.setpointRadPerSec = target;
    Serial.print(F("[OK] Setpoint set to "));
    Serial.print(pid.setpointRadPerSec * RAD_PER_SEC_TO_RPM, 3);
    Serial.println(F(" RPM"));

  // ── PID ON / OFF ────────────────────────────
  } else if (cmd == F("PID ON")) {
    pid.enabled = true;
    motor.running = true;
    resetPidState();
    applyMotor();
    Serial.println(F("[OK] PID enabled"));
  } else if (cmd == F("PID OFF")) {
    pid.enabled = false;
    stopMotor();
    Serial.println(F("[OK] PID disabled"));

  // ── F / FWD ─────────────────────────────────
  } else if (cmd == F("F") || cmd == F("FWD") || cmd == F("FORWARD")) {
    motor.forward = true;
    if (motor.running) applyMotor();
    Serial.println(F("[OK] Direction → FORWARD"));

  // ── R / REV ─────────────────────────────────
  } else if (cmd == F("R") || cmd == F("REV") || cmd == F("REVERSE")) {
    motor.forward = false;
    if (motor.running) applyMotor();
    Serial.println(F("[OK] Direction → REVERSE"));

  // ── GO / START ───────────────────────────────
  } else if (cmd == F("GO") || cmd == F("START")) {
    motor.running = true;
    if (pid.enabled) resetPidState();
    applyMotor();
    Serial.println(F("[OK] Motor started"));

  // ── STOP / X ────────────────────────────────
  } else if (cmd == F("STOP") || cmd == F("X")) {
    motor.running = false;
    pid.enabled = false;
    stopMotor();
    resetPidState();
    Serial.println(F("[OK] Motor stopped (coast)"));

  // ── ENC — on-demand encoder snapshot ────────
  } else if (cmd == F("ENC") || cmd == F("ENCODER")) {
    portENTER_CRITICAL(&encoderMux);
    long p = encoderPulses;
    portEXIT_CRITICAL(&encoderMux);
    float outputRevs = (float)p / PULSES_PER_OUTPUT_REV;
    Serial.print(F("[ENC] Pulses="));
    Serial.print(p);
    Serial.print(F(" | Output revs="));
    Serial.print(outputRevs, 5);
    Serial.print(F(" | RPM="));
    Serial.println(outputRadPerSec * RAD_PER_SEC_TO_RPM, 3);

  // ── RESET — zero the encoder counter ────────
  } else if (cmd == F("RESET")) {
    portENTER_CRITICAL(&encoderMux);
    encoderPulses = 0;
    portEXIT_CRITICAL(&encoderMux);
    speedPulseSnapshot = 0;
    outputRadPerSec  = 0.0f;
    Serial.println(F("[OK] Encoder counter reset to 0"));

  // ── C / CURRENT — on-demand reading ─────────
  } else if (cmd == F("C") || cmd == F("CURRENT")) {
    Serial.print(F("[CURRENT] "));
    Serial.print(readCurrentAmps(), 4);
    Serial.println(F(" A"));

  // ── STBY ON/OFF ─────────────────────────────
  } else if (cmd == F("STBY ON")) {
    digitalWrite(PIN_STBY, HIGH);
    Serial.println(F("[OK] STBY HIGH — driver enabled"));
  } else if (cmd == F("STBY OFF")) {
    motor.running = false;
    stopMotor();
    digitalWrite(PIN_STBY, LOW);
    Serial.println(F("[OK] STBY LOW — driver in standby"));

  // ── H / HELP ────────────────────────────────
  } else if (cmd == F("H") || cmd == F("HELP")) {
    printHelp();

  } else {
    Serial.print(F("[ERR] Unknown command: "));
    Serial.println(raw);
  }
}

// ═════════════════════════════════════════════
//  MOTOR CONTROL
// ═════════════════════════════════════════════
void applyMotor() {
  if (!motor.running || motor.pwm == 0) { stopMotor(); return; }
  if (motor.forward) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }
  ledcWrite(LEDC_CHANNEL, motor.pwm);
}

void stopMotor() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(LEDC_CHANNEL, 0);
}

float readADC() {
  long sum = 0;
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    sum += analogRead(PIN_CURRENT);
    delayMicroseconds(50);
  }
  float avgADC    = (float)sum / CURRENT_SAMPLES;
  return ((avgADC / ADC_RESOLUTION) * ADC_REF_MV);
}

// ═════════════════════════════════════════════
//  CURRENT SENSING (ACS712 with voltage divider)
//
//  ACS712 at 5V supply outputs:
//    - Zero current: 2500 mV
//    - Sensitivity: ±180 mV/A
//    - Formula: I_sensed = (V_out - 2500) / 180
//
//  Voltage divider (10k+10k) halves the output:
//    - V_adc = V_out / 2
//    - At zero current: V_adc = 1250 mV
// ═════════════════════════════════════════════
float readCurrentAmps() {
  float adcMV   = readADC();
  return (ACS712_ZERO_ADC_MV - adcMV) / ACS712_SENS_MV_A;
}

// ═════════════════════════════════════════════
//  SERIAL OUTPUT
// ═════════════════════════════════════════════
void printTelemetry(float amps) {
  portENTER_CRITICAL(&encoderMux);
  long p = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);
  float outputRevs = (float)p / PULSES_PER_OUTPUT_REV;

  Serial.print(F("[TEL] ADC (mV) ="));
  Serial.print(readADC());
  Serial.print(F(" | I="));
  Serial.print(amps, 3);
  Serial.print(F(" A | PWM="));
  Serial.print(motor.pwm);
  Serial.print(F("/255 | Dir="));
  Serial.print(motor.forward ? F("FWD") : F("REV"));
  Serial.print(F(" | Motor="));
  Serial.print(motor.running ? F("ON ") : F("OFF"));
  Serial.print(F(" | PID="));
  Serial.print(pid.enabled ? F("ON") : F("OFF"));
  Serial.print(F(" | SP RPM="));
  Serial.print(pid.setpointRadPerSec * RAD_PER_SEC_TO_RPM, 2);
  Serial.print(F(" | Pulses="));
  Serial.print(p);
  Serial.print(F(" | Revs="));
  Serial.print(outputRevs, 4);
  Serial.print(F(" | RPM="));
  Serial.println(outputRadPerSec * RAD_PER_SEC_TO_RPM, 2);
}

void printHelp() {
  Serial.println(F("╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Infusion Pump — ESP32 Open Loop + Encoder     ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  S <0-255>    Set PWM speed                    ║"));
  Serial.println(F("║  SP <rad/s>   Set PID speed setpoint           ║"));
  Serial.println(F("║  PID ON/OFF   Enable / disable PID control      ║"));
  Serial.println(F("║  F / FWD      Direction → Forward              ║"));
  Serial.println(F("║  R / REV      Direction → Reverse              ║"));
  Serial.println(F("║  GO / START   Start motor                      ║"));
  Serial.println(F("║  STOP / X     Stop motor (coast)               ║"));
  Serial.println(F("║  C            Read current (A)                 ║"));
  Serial.println(F("║  ENC          Read encoder position & RPM      ║"));
  Serial.println(F("║  RESET        Zero encoder counter             ║"));
  Serial.println(F("║  STBY ON/OFF  Enable / disable driver          ║"));
  Serial.println(F("║  H / HELP     Show this menu                   ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  Telemetry every 500 ms | control: 20 ms       ║"));
  Serial.println(F("║  Gear ratio 1:472.73 → 5200 pulses/output rev  ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝"));
}