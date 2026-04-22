#include <Arduino.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────
// TB6612FNG Motor Driver
#define PIN_AIN1  3   // Direction bit 1  (your "AI3")
#define PIN_AIN2  4   // Direction bit 2  (your "AI2")
#define PIN_PWMA  5   // PWM speed signal (your "PWM(A)")
#define PIN_STBY  2   // Standby — MUST be HIGH to enable driver
                      // Wire TB6612FNG STBY → Arduino pin 2

// ACS712 Current Sensor
#define PIN_CURRENT  A0

// ─────────────────────────────────────────────
//  ACS712 CONSTANTS
//  Datasheet / user formula: mV = -180·A + 2500
//  → A = (2500 - Vout_mV) / 180
// ─────────────────────────────────────────────
static constexpr float ACS712_ZERO_MV   = 2500.0f;  // Output at 0 A (mV)
static constexpr float ACS712_SENS_MV_A =  180.0f;  // |Sensitivity| mV/A
static constexpr float VCC_MV           = 5000.0f;  // Arduino 5 V rail
static constexpr float ADC_RESOLUTION   = 1024.0f;  // 10-bit ADC

static constexpr int   CURRENT_SAMPLES  = 30;       // Averaging window

// ─────────────────────────────────────────────
//  MOTOR STATE
// ─────────────────────────────────────────────
struct MotorState {
  uint8_t  pwm      = 0;
  bool     forward  = true;
  bool     running  = false;
} motor;

// ─────────────────────────────────────────────
//  TELEMETRY TIMING
// ─────────────────────────────────────────────
static constexpr uint32_t TELEMETRY_MS = 500;
static uint32_t lastTelemetry = 0;

// ─────────────────────────────────────────────
//  PROTOTYPES
// ─────────────────────────────────────────────
void applyMotor();
void stopMotor();
float readCurrentAmps();
void processCommand(const String& raw);
void printHelp();
void printTelemetry(float amps);

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB-Serial on native USB boards */ }

  // Driver pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  stopMotor();                   // Safe default
  digitalWrite(PIN_STBY, HIGH); // Take driver out of standby

  printHelp();
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  // ── Periodic telemetry ──────────────────────
  uint32_t now = millis();
  if (now - lastTelemetry >= TELEMETRY_MS) {
    lastTelemetry = now;
    printTelemetry(readCurrentAmps());
  }

  // ── Serial command handling ──────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) processCommand(cmd);
  }
}

// ═════════════════════════════════════════════
//  COMMAND PARSER
// ═════════════════════════════════════════════
void processCommand(const String& raw) {
  String cmd = raw;
  cmd.toUpperCase();

  // ── S <0-255>  or  SPEED <0-255> ────────────
  if (cmd.startsWith("S ") || cmd.startsWith("SPEED ")) {
    int sp = cmd.substring(cmd.indexOf(' ') + 1).toInt();
    motor.pwm = (uint8_t)constrain(sp, 0, 255);
    if (motor.running) applyMotor();
    Serial.print(F("[OK] PWM speed set to "));
    Serial.println(motor.pwm);

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
    Serial.println(F("[OK] Motor stopped"));

  // ── C / CURRENT — single on-demand reading ──
  } else if (cmd == F("C") || cmd == F("CURRENT")) {
    Serial.print(F("[CURRENT] "));
    Serial.print(readCurrentAmps(), 4);
    Serial.println(F(" A"));

  // ── STBY ON / OFF — useful for debugging ────
  } else if (cmd == F("STBY ON")) {
    digitalWrite(PIN_STBY, HIGH);
    Serial.println(F("[OK] STBY HIGH — driver enabled"));
  } else if (cmd == F("STBY OFF")) {
    stopMotor();
    motor.running = false;
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

// TB6612FNG truth table:
//  AIN1  AIN2  PWM   →  Output
//   H     L    PWM   →  Forward (CW)
//   L     H    PWM   →  Reverse (CCW)
//   L     L    any   →  Coast / stop
//   H     H    any   →  Brake
void applyMotor() {
  if (!motor.running || motor.pwm == 0) {
    stopMotor();
    return;
  }
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
  // Coast stop — both LOW
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
    delayMicroseconds(250); // small gap between samples
  }
  float avgADC   = (float)sum / CURRENT_SAMPLES;
  float voutMV   = (avgADC / ADC_RESOLUTION) * VCC_MV;
  // mV = -180·A + 2500  →  A = (2500 - mV) / 180
  return (ACS712_ZERO_MV - voutMV) / ACS712_SENS_MV_A;
}

// ═════════════════════════════════════════════
//  SERIAL OUTPUT HELPERS
// ═════════════════════════════════════════════
void printTelemetry(float amps) {
  Serial.print(F("[TEL] I="));
  Serial.print(amps, 3);
  Serial.print(F(" A | PWM="));
  Serial.print(motor.pwm);
  Serial.print(F("/255 | Dir="));
  Serial.print(motor.forward ? F("FWD") : F("REV"));
  Serial.print(F(" | Motor="));
  Serial.println(motor.running ? F("ON") : F("OFF"));
}

void printHelp() {
  Serial.println(F("╔══════════════════════════════════════════╗"));
  Serial.println(F("║   Infusion Pump — Open Loop Motor Test   ║"));
  Serial.println(F("╠══════════════════════════════════════════╣"));
  Serial.println(F("║  S <0-255>    Set PWM speed              ║"));
  Serial.println(F("║  F / FWD      Direction → Forward        ║"));
  Serial.println(F("║  R / REV      Direction → Reverse        ║"));
  Serial.println(F("║  GO           Start motor                ║"));
  Serial.println(F("║  STOP / X     Stop motor (coast)         ║"));
  Serial.println(F("║  C            Read current now           ║"));
  Serial.println(F("║  STBY ON/OFF  Enable/disable driver      ║"));
  Serial.println(F("║  H / HELP     Show this menu             ║"));
  Serial.println(F("╠══════════════════════════════════════════╣"));
  Serial.println(F("║  Telemetry prints every 500 ms           ║"));
  Serial.println(F("╚══════════════════════════════════════════╝"));
}