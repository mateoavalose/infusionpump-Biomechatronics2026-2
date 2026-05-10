#include <Arduino.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS (ESP32 WROOM 32E)
//
//  Avoided: GPIO 0,2,12,15 (strapping), GPIO 6–11 (flash),
//           GPIO 34–39 (input-only, used for ADC only)
// ─────────────────────────────────────────────

// TB6612FNG Motor Driver
#define PIN_AIN1  25
#define PIN_AIN2  26
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
static constexpr float ACS712_ZERO_MV    = 2500.0f; // Sensor zero-current output (mV)
static constexpr float ACS712_SENS_MV_A  =  180.0f; // Sensitivity magnitude (mV/A)
static constexpr float DIVIDER_RATIO     =    2.0f;  // Voltage divider ratio (10k+10k)
static constexpr float ADC_REF_MV        = 3300.0f;  // ESP32 ADC reference (mV)
static constexpr float ADC_RESOLUTION    = 4095.0f;  // 12-bit
static constexpr int   CURRENT_SAMPLES   =   10;

// ─────────────────────────────────────────────
//  ENCODER & GEARBOX CONSTANTS
//  11 pulses/motor-rev, gear ratio 1:472.7272
//  → 5200 pulses per output revolution
// ─────────────────────────────────────────────
static constexpr float    PULSES_PER_MOTOR_REV  =   11.0f;
static constexpr float    GEAR_RATIO            =  472.7272f;
static constexpr float    PULSES_PER_OUTPUT_REV =  PULSES_PER_MOTOR_REV * GEAR_RATIO; // 5200.0

static constexpr uint32_t RPM_WINDOW_MS         = 50;  // Window for RPM averaging (increased to smooth quantization)

// ─────────────────────────────────────────────
//  TELEMETRY TIMING
// ─────────────────────────────────────────────
static constexpr uint32_t TELEMETRY_MS = 10;  // ⚠️  IMPORTANT: Change this if needed!
                                               // For fast motor transients (<100ms), use 5-10ms
                                               // For slower motors, 20ms is OK
                                               // MUST match TELEMETRY_MS in MATLAB read-esp32.m
static uint32_t lastTelemetry          = 0;

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

long     rpmPulseSnapshot = 0;
uint32_t rpmLastCalcMs    = 0;
float    outputRPM        = 0.0f;
float    omega            = 0.0f; 

// FreeRTOS critical section handle (used instead of noInterrupts on ESP32)
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────
//  PROTOTYPES
// ─────────────────────────────────────────────
void  applyMotor();
void  stopMotor();
float readCurrentAmps();
void  processCommand(const String& raw);
void  updateRPM();
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

  rpmLastCalcMs = millis();
  printHelp();
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  updateRPM();

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
//  RPM CALCULATION
// ═════════════════════════════════════════════
void updateRPM() {
  uint32_t now     = millis();
  uint32_t elapsed = now - rpmLastCalcMs;
  if (elapsed < RPM_WINDOW_MS) return;

  // FreeRTOS-safe read of volatile long from main-loop context
  portENTER_CRITICAL(&encoderMux);
  long currentPulses = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  long  deltaPulses = currentPulses - rpmPulseSnapshot;
  float elapsedMin  = (float)elapsed / 60000.0f;

  outputRPM        = ((float)deltaPulses / PULSES_PER_OUTPUT_REV) / elapsedMin;
  omega            = (outputRPM / 60.0f) * 2.0f * PI;
  rpmPulseSnapshot = currentPulses;
  rpmLastCalcMs    = now;
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
    Serial.print(F("/255  (~"));
    Serial.print((motor.pwm / 255.0f) * 6.0f, 2);
    Serial.println(F(" RPM estimated)"));

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
    applyMotor();
    Serial.println(F("[OK] Motor started"));

  // ── STOP / X ────────────────────────────────
  } else if (cmd == F("STOP") || cmd == F("X")) {
    motor.running = false;
    stopMotor();
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
    Serial.println(outputRPM, 3);

  // ── RESET — zero the encoder counter ────────
  } else if (cmd == F("RESET")) {
    portENTER_CRITICAL(&encoderMux);
    encoderPulses = 0;
    portEXIT_CRITICAL(&encoderMux);
    rpmPulseSnapshot = 0;
    outputRPM        = 0.0f;
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
//  Uses ledcWrite() instead of analogWrite()
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
  Serial.println(F("[STEP_APPLIED]"));  // Exact timing marker for MATLAB sync
}

void stopMotor() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(LEDC_CHANNEL, 0);
}

// ═════════════════════════════════════════════
//  CURRENT SENSING
//
//  ADC reads voltage at the divider midpoint (mV_adc = mV_sensor / 2).
//  Reconstruct sensor voltage then apply ACS712 formula:
//    A = (2500 - mV_sensor) / 180
//
//  Note: ESP32 ADC has known non-linearity near 0V and 3.3V rails.
//  For higher accuracy in a future revision, use analogReadMilliVolts()
//  (available in ESP-IDF / Arduino-ESP32 ≥ v2.0) with ADC calibration.
// ═════════════════════════════════════════════
float readCurrentAmps() {
  long sum = 0;
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    sum += analogRead(PIN_CURRENT);
    delayMicroseconds(250);
  }
  float avgADC    = (float)sum / CURRENT_SAMPLES;
  float adcMV     = (avgADC / ADC_RESOLUTION) * ADC_REF_MV;   // mV at ADC pin
  float sensorMV  = adcMV * DIVIDER_RATIO;                     // reconstruct full sensor output
  return (ACS712_ZERO_MV - sensorMV) / ACS712_SENS_MV_A;
}

// ═════════════════════════════════════════════
//  SERIAL OUTPUT
// ═════════════════════════════════════════════
void printTelemetry(float amps) {

  portENTER_CRITICAL(&encoderMux);
  long p = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);
  uint32_t t = millis();

  Serial.print(t);
  Serial.print(",");
  Serial.print(motor.pwm);
  Serial.print(",");
  Serial.print(motor.forward ? 1 : -1);
  Serial.print(",");
  Serial.print(p);
  Serial.print(",");
  Serial.print(outputRPM, 4);
  Serial.print(",");
  Serial.print(omega, 4);
  Serial.print(",");
  Serial.println(amps, 5);
}

void printHelp() {
  Serial.println(F("╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Infusion Pump — ESP32 Open Loop + Encoder     ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  S <0-255>    Set PWM speed                    ║"));
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
  Serial.println(F("║  Telemetry every 10 ms | RPM window: 50 ms     ║"));
  Serial.println(F("║  Gear ratio 1:472.73 → 5200 pulses/output rev  ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝"));
}