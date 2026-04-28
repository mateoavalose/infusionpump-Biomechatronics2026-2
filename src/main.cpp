#include <Arduino.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────

// TB6612FNG
#define PIN_AIN1  7   // Direction bit 1
#define PIN_AIN2  8   // Direction bit 2
#define PIN_PWMA  5   // PWM speed
#define PIN_STBY  6   // Standby release

// Encoder
// ENCA must be on a hardware-interrupt pin (Arduino Uno: 2 or 3)
// ENCB only needs a digital read
#define PIN_ENCA  2   // Channel A — INT0 (rising edge interrupt)
#define PIN_ENCB  4   // Channel B — direction sense (polled in ISR)

// ACS712 Current Sensor
#define PIN_CURRENT  A0

// ─────────────────────────────────────────────
//  ENCODER & GEARBOX CONSTANTS
//
//  Motor datasheet / measured:
//    11 pulses per motor-shaft revolution (single channel, rising edges)
//    Gear ratio: 1 : 472.7272  (472.7272 motor turns → 1 output turn)
//
//  Verification:
//    At 100% PWM the encoder fires at ~520 Hz
//    520 Hz / 11 pulses = 47.27 motor rev/s = 2836 RPM motor
//    2836 RPM / 472.7272 = 6 RPM output 
//
//  Pulses per OUTPUT revolution = 11 × 472.7272 ≈ 5200
// ─────────────────────────────────────────────
static constexpr float    PULSES_PER_MOTOR_REV  = 11.0f;
static constexpr float    GEAR_RATIO            = 472.727272f;
static constexpr float    PULSES_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO; // 5200.0

// RPM is recalculated every this many milliseconds
static constexpr uint32_t RPM_WINDOW_MS         = 1000;

// ─────────────────────────────────────────────
//  ACS712 CONSTANTS
//  Formula: mV = -180·A + 2500  ->  A = (2500 - mV) / 180
// ─────────────────────────────────────────────
static constexpr float ACS712_ZERO_MV   = 2500.0f;
static constexpr float ACS712_SENS_MV_A =  180.0f;
static constexpr float VCC_MV           = 5000.0f;
static constexpr float ADC_RESOLUTION   = 1024.0f;
static constexpr int   CURRENT_SAMPLES  = 30;

// ─────────────────────────────────────────────
//  TELEMETRY TIMING
// ─────────────────────────────────────────────
static constexpr uint32_t TELEMETRY_MS = 500;
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
//  ENCODER STATE  (volatile: shared with ISR)
//
//  encoderPulses: signed running total of rising edges on ENCA.
//    Incremented when ENCB is HIGH at the moment ENCA rises: forward.
//    Decremented when ENCB is LOW  at the moment ENCA rises: reverse.
//    This is direction-aware, so reversing the motor winds the count back.
// ─────────────────────────────────────────────
volatile long encoderPulses = 0;

// RPM tracking (non-volatile; only written in main loop with interrupts off)
long     rpmPulseSnapshot = 0;
uint32_t rpmLastCalcMs    = 0;
float    outputRPM        = 0.0f;

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
void  encoderISR();   // Interrupt Service Routine

// ═════════════════════════════════════════════
//  ENCODER ISR
//  Called on every RISING edge of ENCA.
//  Reads ENCB immediately to determine direction.
//
//  Quadrature truth table (standard):
//    ENCA LOW while ENCB HIGH: forward  (+1)
//    ENCA HIGH while ENCB LOW:  reverse  (-1)
//
//  Note: digitalRead inside an ISR is safe on AVR but adds ~3–4 µs.
//  At 520 Hz max pulse rate the ISR fires every ~1.9 ms.
// ═════════════════════════════════════════════
void encoderISR() {
  if (digitalRead(PIN_ENCB) == HIGH) {
    encoderPulses++;
  } else {
    encoderPulses--;
  }
}

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Motor driver pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  // Encoder pins — INPUT_PULLUP in case the encoder has open-collector outputs.
  // If encoder has its own pull-ups, INPUT also works fine.
  pinMode(PIN_ENCA, INPUT_PULLUP);
  pinMode(PIN_ENCB, INPUT_PULLUP);

  // Attach interrupt — RISING edge only (counts one edge per pulse)
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), encoderISR, RISING);

  stopMotor();
  digitalWrite(PIN_STBY, HIGH);   // Release driver from standby

  rpmLastCalcMs = millis();
  printHelp();
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  updateRPM();  // Non-blocking, runs on its own timer

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
//  RPM CALCULATION  (non-blocking, called every loop)
//
//  Strategy: once per RPM_WINDOW_MS, snapshot encoderPulses
//  with interrupts disabled (prevents a torn read of the long),
//  compute the delta since the last snapshot, then convert:
//
//    RPM_output = (delta_pulses / pulses_per_output_rev)
//                 / (elapsed_ms / 60000)
//
//  This handles any PWM: slower PWM = fewer pulses = lower RPM.
//  The signed delta also gives negative RPM when running in reverse,
//  which is useful for debugging direction issues.
// ═════════════════════════════════════════════
void updateRPM() {
  uint32_t now = millis();
  uint32_t elapsed = now - rpmLastCalcMs;

  if (elapsed < RPM_WINDOW_MS) return;

  // Atomically read the volatile long (AVR: 4 bytes, not atomic by default)
  noInterrupts();
  long currentPulses = encoderPulses;
  interrupts();

  long  deltaPulses = currentPulses - rpmPulseSnapshot;
  float elapsedMin  = (float)elapsed / 60000.0f;

  outputRPM        = ((float)deltaPulses / PULSES_PER_OUTPUT_REV) / elapsedMin;
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
    int sp = cmd.substring(cmd.indexOf(' ') + 1).toInt();
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
    noInterrupts();
    long p = encoderPulses;
    interrupts();
    float outputRevs = (float)p / PULSES_PER_OUTPUT_REV;
    Serial.print(F("[ENC] Pulses="));
    Serial.print(p);
    Serial.print(F(" | Output revs="));
    Serial.print(outputRevs, 5);
    Serial.print(F(" | RPM="));
    Serial.println(outputRPM, 3);

  // ── RESET — zero the encoder counter ────────
  } else if (cmd == F("RESET")) {
    noInterrupts();
    encoderPulses = 0;
    interrupts();
    rpmPulseSnapshot = 0;
    outputRPM        = 0.0f;
    Serial.println(F("[OK] Encoder counter reset to 0"));

  // ── C / CURRENT — on-demand reading ─────────
  } else if (cmd == F("C") || cmd == F("CURRENT")) {
    Serial.print(F("[CURRENT] "));
    Serial.print(readCurrentAmps(), 4);
    Serial.println(F(" A"));

  // ── STBY ON/OFF ──────────────────────────────
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
//  TB6612FNG truth table:
//   AIN1  AIN2  → AOUT
//    H     L    → Forward (CW)
//    L     H    → Reverse (CCW)
//    L     L    → Coast
//    H     H    → Brake
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
  analogWrite(PIN_PWMA, motor.pwm);
}

void stopMotor() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  analogWrite(PIN_PWMA, 0);
}

// ═════════════════════════════════════════════
//  CURRENT SENSING
// ═════════════════════════════════════════════
float readCurrentAmps() {
  long sum = 0;
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    sum += analogRead(PIN_CURRENT);
    delayMicroseconds(250);
  }
  float avgADC = (float)sum / CURRENT_SAMPLES;
  float voutMV = (avgADC / ADC_RESOLUTION) * VCC_MV;
  return (ACS712_ZERO_MV - voutMV) / ACS712_SENS_MV_A;
}

// ═════════════════════════════════════════════
//  SERIAL OUTPUT
// ═════════════════════════════════════════════
void printTelemetry(float amps) {
  noInterrupts();
  long p = encoderPulses;
  interrupts();
  float outputRevs = (float)p / PULSES_PER_OUTPUT_REV;

  Serial.print(F("[TEL] I="));
  Serial.print(amps, 3);
  Serial.print(F(" A | PWM="));
  Serial.print(motor.pwm);
  Serial.print(F("/255 | Dir="));
  Serial.print(motor.forward ? F("FWD") : F("REV"));
  Serial.print(F(" | Motor="));
  Serial.print(motor.running ? F("ON ") : F("OFF"));
  Serial.print(F(" | Pulses="));
  Serial.print(p);
  Serial.print(F(" | Revs="));
  Serial.print(outputRevs, 4);
  Serial.print(F(" | RPM="));
  Serial.println(outputRPM, 2);
}

void printHelp() {
  Serial.println(F("╔════════════════════════════════════════════════╗"));
  Serial.println(F("║   Infusion Pump — Open Loop + Encoder          ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  S <0-255>    Set PWM speed                    ║"));
  Serial.println(F("║  F / FWD      Direction → Forward              ║"));
  Serial.println(F("║  R / REV      Direction → Reverse              ║"));
  Serial.println(F("║  GO / START   Start motor                      ║"));
  Serial.println(F("║  STOP / X     Stop motor (coast)               ║"));
  Serial.println(F("║  C            Read current (A)                 ║"));
  Serial.println(F("║  ENC          Read encoder position & RPM      ║"));
  Serial.println(F("║  RESET        Zero the encoder counter         ║"));
  Serial.println(F("║  STBY ON/OFF  Enable / disable driver          ║"));
  Serial.println(F("║  H / HELP     Show this menu                   ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  Telemetry every 500 ms | RPM window: 1000 ms  ║"));
  Serial.println(F("║  Gear ratio 1:472.73 → 5200 pulses/output rev  ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝"));
}