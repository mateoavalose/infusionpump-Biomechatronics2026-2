#include <Arduino.h>

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────

// TB6612FNG Motor Driver
#define PIN_AIN1  26
#define PIN_AIN2  25
#define PIN_PWMA  27
#define PIN_STBY  14

// Hall Effect Encoder
#define PIN_ENCA  18
#define PIN_ENCB  19

// ACS712 Current Sensor ADC
#define PIN_CURRENT  34

// ─────────────────────────────────────────────
//  LEDC (PWM) CONFIGURATION
// ─────────────────────────────────────────────
#define LEDC_CHANNEL    0
#define LEDC_FREQ_HZ    5000
#define LEDC_RESOLUTION 8

// ─────────────────────────────────────────────
//  ACS712 CONSTANTS
// ─────────────────────────────────────────────
static constexpr float ACS712_ZERO_ADC_MV = 1260.0f;
static constexpr float ACS712_SENS_MV_A   =   97.04f;
static constexpr float ADC_REF_MV         = 3300.0f;
static constexpr float ADC_RESOLUTION     = 4095.0f;
static constexpr int   CURRENT_SAMPLES    = 30;

// ─────────────────────────────────────────────
//  ENCODER & GEARBOX CONSTANTS
// ─────────────────────────────────────────────
static constexpr float PULSES_PER_MOTOR_REV = 11.0f;
static constexpr float GEAR_RATIO           = 472.7272f;
static constexpr int   ENC_EDGES_PER_PULSE  = 2;
static constexpr float PULSES_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO * ENC_EDGES_PER_PULSE;

// ─────────────────────────────────────────────
//  SPEED CALCULATION & FILTERING
// ─────────────────────────────────────────────
static constexpr uint32_t SPEED_UPDATE_MS   = 20;
static constexpr float    EMA_ALPHA         = 0.05f;

// ─────────────────────────────────────────────
//  TELEMETRY TIMING
// ─────────────────────────────────────────────
static constexpr uint32_t TELEMETRY_MS = 20;
static uint32_t lastTelemetry = 0;

static constexpr float RAD_PER_SEC_TO_RPM = 60.0f / (2.0f * PI);

// ─────────────────────────────────────────────
//  PID - SS GAINS FROM SIMULINK AND TIMING
// ─────────────────────────────────────────────
static constexpr float PID_KC = 134.345399f;
static constexpr float PID_TI = 0.110091f;
static constexpr float PID_TD = 0.036606f;
static const float PID_KB = 1.0f / sqrtf(PID_TI * PID_TD);

static constexpr float SS_K_OMEGA    = 0.268254f;
static constexpr float SS_K_CURRENT  = 57.986618f;
static constexpr float SS_K_INTEGRAL = -1220.314070f;
static constexpr float SS_K_FF       = 0.039477f;

static constexpr float PWM_MIN = 0.0f;
static constexpr float PWM_MAX = 255.0f;

static constexpr uint32_t CONTROL_PERIOD_MS = 50;
static constexpr uint32_t CURRENT_CHECK_MS = 20;
static constexpr uint32_t CURRENT_FAULT_ARM_MS = 300;
static constexpr uint32_t CURRENT_FAULT_TRIP_MS = 250;
static constexpr float    CURRENT_FAULT_PERCENT = 0.30f;
static constexpr float    CURRENT_FAULT_BASELINE_ALPHA = 0.02f;
static constexpr float    CURRENT_FAULT_MIN_BASELINE_A = 0.05f;

// ─────────────────────────────────────────────
//  INFUSION PUMP GEOMETRY
// ─────────────────────────────────────────────
static constexpr float SYRINGE_DIAMETER_MM = 29.0f;           // Syringe inner diameter [mm]
static constexpr float SCREW_PITCH_TURNS_PER_INCH = 13.0f;    // Screw pitch: 13 turns per inch
static constexpr float MM_PER_INCH = 25.4f;
static constexpr float MM_PER_SCREW_TURN = MM_PER_INCH / SCREW_PITCH_TURNS_PER_INCH;
static constexpr float SYRINGE_AREA_MM2 = (SYRINGE_DIAMETER_MM / 2.0f) * (SYRINGE_DIAMETER_MM / 2.0f) * PI;
static constexpr float VOLUME_PER_OUTPUT_REV_ML = (SYRINGE_AREA_MM2 * MM_PER_SCREW_TURN) / 1000.0f;

enum class ControlMode {
  Manual,
  PID,
  SS
};

// ─────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────
struct MotorState {
  uint8_t pwm     = 0;
  bool    forward = true;
  bool    running = false;
} motor;

struct PIDState {
  float setpointRadPerSec = 0.0f;
  float integrator        = 0.0f;
  float prevMeasurement   = 0.0f;
  bool  enabled           = false;
  uint32_t lastUpdateMs   = 0;
} pid;

struct SSState {
  float setpointRadPerSec = 0.0f;
  float errorIntegral     = 0.0f;
  uint32_t lastUpdateMs   = 0;
} ss;

struct CurrentFaultState {
  float baselineAmps = 0.0f;
  uint32_t motorStartMs = 0;
  uint32_t overThresholdSinceMs = 0;
  bool baselineValid = false;
  bool armed = false;
  bool latched = false;
  float thresholdPercent = CURRENT_FAULT_PERCENT;
  uint32_t armDelayMs = CURRENT_FAULT_ARM_MS;
  uint32_t tripDelayMs = CURRENT_FAULT_TRIP_MS;
} currentFault;

struct InfusionState {
  float targetVolumeMl = 0.0f;          // Target volume to inject [mL]
  float flowRateMlPerMin = 0.0f;        // Flow rate [mL/min]
  float volumeInjectedMl = 0.0f;        // Volume injected so far [mL]
  long pulsesAtStartOfInfusion = 0;     // Encoder pulses when infusion started
  bool infusing = false;                // Whether currently infusing
  uint32_t infusionStartMs = 0;         // When infusion started
} infusion;

ControlMode controlMode = ControlMode::Manual;

volatile long encoderPulses = 0;

long     speedPulseSnapshot = 0;
uint32_t speedLastCalcMs    = 0;
float    outputRadPerSec    = 0.0f;

bool matlabReadableOutput = true;
bool stepMarkerPending    = false;
uint32_t lastCurrentCheckMs = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// ─────────────────────────────────────────────
//  PROTOTYPES
// ─────────────────────────────────────────────
void  applyMotor();
void  stopMotor();
float readADC();
float readCurrentAmps();
void  processCommand(const String& raw);
void  updateSpeedRadPerSec();
void  updatePidControl();
void  updateSsControl();
void  updateCurrentFaultMonitor(float amps);
float clampf(float value, float lowerBound, float upperBound);
void  resetPidState();
void  resetSsState();
void  resetCurrentFaultState();
void  armCurrentFaultMonitor();
void  tripCurrentFault(const __FlashStringHelper* reason);
void  setControlMode(ControlMode mode);
const __FlashStringHelper* controlModeName();
void  updateInfusionControl();
void  startInfusion();
void  stopInfusion();
void  resetInfusionState();
void  printTelemetry(float amps);
void  printHelp();
void  IRAM_ATTR encoderISR();

// ═════════════════════════════════════════════
//  ENCODER ISR
// ═════════════════════════════════════════════
void IRAM_ATTR encoderISR() {
  portENTER_CRITICAL_ISR(&encoderMux);
  int a = digitalRead(PIN_ENCA);
  int b = digitalRead(PIN_ENCB);
  if (a == b) {
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

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  ledcAttachPin(PIN_PWMA, LEDC_CHANNEL);

  pinMode(PIN_ENCA, INPUT_PULLUP);
  pinMode(PIN_ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), encoderISR, CHANGE);

  stopMotor();
  digitalWrite(PIN_STBY, HIGH);

  speedLastCalcMs = millis();
  resetPidState();
  resetCurrentFaultState();
  printHelp();
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  updateSpeedRadPerSec();
  if ((now - lastCurrentCheckMs) >= CURRENT_CHECK_MS) {
    lastCurrentCheckMs = now;
    float amps = readCurrentAmps();
    updateCurrentFaultMonitor(amps);
  }
  if (controlMode == ControlMode::PID) {
    updatePidControl();
  } else if (controlMode == ControlMode::SS) {
    updateSsControl();
  }
  
  updateInfusionControl();

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
  uint32_t now = millis();
  uint32_t elapsed = now - speedLastCalcMs;
  if (elapsed < SPEED_UPDATE_MS) return;

  portENTER_CRITICAL(&encoderMux);
  long currentPulses = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  long  deltaPulses = currentPulses - speedPulseSnapshot;
  float elapsedSec  = (float)elapsed / 1000.0f;
  float revPerSec   = ((float)deltaPulses / PULSES_PER_OUTPUT_REV) / elapsedSec;

  float rawRadPerSec = revPerSec * (2.0f * PI);
  
  // Apply Exponential Moving Average (EMA) filter
  outputRadPerSec = (EMA_ALPHA * rawRadPerSec) + ((1.0f - EMA_ALPHA) * outputRadPerSec);
  
  speedPulseSnapshot = currentPulses;
  speedLastCalcMs    = now;
}

// ═════════════════════════════════════════════
//  PID CONTROL (BACK-CALCULATION ANTI-WINDUP)
// ═════════════════════════════════════════════
void updatePidControl() {
  if (!pid.enabled) return;

  uint32_t now = millis();
  uint32_t elapsed = now - pid.lastUpdateMs;
  if (elapsed < CONTROL_PERIOD_MS) return;

  float dt = (float)elapsed / 1000.0f;

  float error = pid.setpointRadPerSec - outputRadPerSec;
  float proportional = PID_KC * error;
  float derivative = -PID_KC * PID_TD * ((outputRadPerSec - pid.prevMeasurement) / dt);
  float unsatOutput = proportional + pid.integrator + derivative;
  float satOutput = clampf(unsatOutput, -PWM_MAX, PWM_MAX);

  pid.integrator += dt * ((PID_KC / PID_TI) * error + PID_KB * (satOutput - unsatOutput));
  pid.prevMeasurement = outputRadPerSec;
  pid.lastUpdateMs = now;

  motor.forward = (satOutput >= 0.0f);
  motor.pwm = (uint8_t)clampf(fabsf(satOutput), PWM_MIN, PWM_MAX);
  motor.running = true;
  applyMotor();
}

void updateSsControl() {
  if (controlMode != ControlMode::SS || !motor.running) return;

  uint32_t now = millis();
  uint32_t elapsed = now - ss.lastUpdateMs;
  if (elapsed < CONTROL_PERIOD_MS) return;

  float dt = (float)elapsed / 1000.0f;
  float speedError = ss.setpointRadPerSec - outputRadPerSec;
  ss.errorIntegral += speedError * dt;

  float currentAmps = readCurrentAmps();
  float controlVoltage = SS_K_FF * ss.setpointRadPerSec
                       - SS_K_OMEGA * outputRadPerSec
                       - SS_K_CURRENT * currentAmps
                       - SS_K_INTEGRAL * ss.errorIntegral;

  float satOutput = clampf(controlVoltage, -PWM_MAX, PWM_MAX);

  motor.forward = (satOutput >= 0.0f);
  motor.pwm = (uint8_t)clampf(fabsf(satOutput), PWM_MIN, PWM_MAX);
  motor.running = true;
  ss.lastUpdateMs = now;
  applyMotor();
}

void updateCurrentFaultMonitor(float amps) {
  uint32_t now = millis();

  if (!motor.running || currentFault.latched) {
    currentFault.overThresholdSinceMs = 0;
    return;
  }

  if (!currentFault.armed) {
    if ((now - currentFault.motorStartMs) >= currentFault.armDelayMs) {
      currentFault.armed = true;
      currentFault.baselineAmps = fabsf(amps);
      currentFault.baselineValid = true;
    } else {
      return;
    }
  }

  float absAmps = fabsf(amps);
  if (!currentFault.baselineValid) {
    currentFault.baselineAmps = absAmps;
    currentFault.baselineValid = true;
  } else {
    currentFault.baselineAmps = (CURRENT_FAULT_BASELINE_ALPHA * absAmps)
                              + ((1.0f - CURRENT_FAULT_BASELINE_ALPHA) * currentFault.baselineAmps);
  }

  float referenceBaseline = clampf(currentFault.baselineAmps, CURRENT_FAULT_MIN_BASELINE_A, 1.0e9f);
  float tripThreshold = referenceBaseline * (1.0f + currentFault.thresholdPercent);

  if (absAmps > tripThreshold) {
    if (currentFault.overThresholdSinceMs == 0) {
      currentFault.overThresholdSinceMs = now;
    } else if ((now - currentFault.overThresholdSinceMs) >= currentFault.tripDelayMs) {
      tripCurrentFault(F("overcurrent"));
    }
  } else {
    currentFault.overThresholdSinceMs = 0;
  }
}

void updateInfusionControl() {
  if (!infusion.infusing) return;

  uint32_t now = millis();
  
  portENTER_CRITICAL(&encoderMux);
  long currentPulses = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  long deltaPulses = currentPulses - infusion.pulsesAtStartOfInfusion;
  infusion.volumeInjectedMl = (float)deltaPulses * VOLUME_PER_OUTPUT_REV_ML / PULSES_PER_OUTPUT_REV;

  if (infusion.volumeInjectedMl >= infusion.targetVolumeMl) {
    stopInfusion();
    Serial.print(F("[INFO] Infusion complete. Volume injected: "));
    Serial.print(infusion.volumeInjectedMl, 2);
    Serial.println(F(" mL"));
  }
}

void startInfusion() {
  if (infusion.targetVolumeMl <= 0.0f) {
    Serial.println(F("[ERR] Target volume must be > 0 mL"));
    return;
  }
  if (infusion.flowRateMlPerMin <= 0.0f) {
    Serial.println(F("[ERR] Flow rate must be > 0 mL/min"));
    return;
  }

  float rpmRequired = infusion.flowRateMlPerMin / VOLUME_PER_OUTPUT_REV_ML;
  float radPerSecRequired = rpmRequired * 2.0f * PI / 60.0f;

  portENTER_CRITICAL(&encoderMux);
  infusion.pulsesAtStartOfInfusion = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  infusion.volumeInjectedMl = 0.0f;
  infusion.infusing = true;
  infusion.infusionStartMs = millis();

  pid.setpointRadPerSec = radPerSecRequired;
  ss.setpointRadPerSec = radPerSecRequired;
  motor.running = true;
  controlMode = ControlMode::PID;
  resetPidState();
  armCurrentFaultMonitor();
  applyMotor();

  Serial.print(F("[OK] Infusion started: "));
  Serial.print(infusion.targetVolumeMl, 2);
  Serial.print(F(" mL at "));
  Serial.print(infusion.flowRateMlPerMin, 2);
  Serial.print(F(" mL/min (RPM: "));
  Serial.print(rpmRequired, 2);
  Serial.println(F(")"));
}

void stopInfusion() {
  infusion.infusing = false;
  motor.running = false;
  pid.enabled = false;
  controlMode = ControlMode::Manual;
  stopMotor();
  resetPidState();
}

void resetInfusionState() {
  infusion.targetVolumeMl = 0.0f;
  infusion.flowRateMlPerMin = 0.0f;
  infusion.volumeInjectedMl = 0.0f;
  infusion.pulsesAtStartOfInfusion = 0;
  infusion.infusing = false;
  infusion.infusionStartMs = 0;
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

void resetSsState() {
  ss.errorIntegral = 0.0f;
  ss.setpointRadPerSec = pid.setpointRadPerSec;
  ss.lastUpdateMs = millis();
}

void resetCurrentFaultState() {
  currentFault.baselineAmps = 0.0f;
  currentFault.motorStartMs = millis();
  currentFault.overThresholdSinceMs = 0;
  currentFault.baselineValid = false;
  currentFault.armed = false;
  currentFault.latched = false;
  digitalWrite(PIN_STBY, HIGH);
}

void armCurrentFaultMonitor() {
  currentFault.motorStartMs = millis();
  currentFault.overThresholdSinceMs = 0;
  currentFault.baselineValid = false;
  currentFault.armed = false;
}

void tripCurrentFault(const __FlashStringHelper* reason) {
  currentFault.latched = true;
  motor.running = false;
  pid.enabled = false;
  controlMode = ControlMode::Manual;
  stopMotor();
  resetPidState();
  resetSsState();
  digitalWrite(PIN_STBY, LOW);
  Serial.print(F("[FAULT] Current trip: "));
  Serial.println(reason);
}

void setControlMode(ControlMode mode) {
  controlMode = mode;
  pid.enabled = (mode == ControlMode::PID);

  if (mode == ControlMode::PID) {
    resetPidState();
  } else if (mode == ControlMode::SS) {
    resetSsState();
  }
}

const __FlashStringHelper* controlModeName() {
  switch (controlMode) {
    case ControlMode::Manual: return F("MANUAL");
    case ControlMode::PID:    return F("PID");
    case ControlMode::SS:     return F("SS");
  }
  return F("UNKNOWN");
}

// ═════════════════════════════════════════════
//  COMMAND PARSER
// ═════════════════════════════════════════════
void processCommand(const String& raw) {
  String cmd = raw;
  cmd.toUpperCase();

  if (cmd == F("MATLAB ON") || cmd == F("FMT MATLAB")) {
    matlabReadableOutput = true;
    Serial.println(F("[OK] MATLAB readable telemetry enabled"));

  } else if (cmd == F("MATLAB OFF") || cmd == F("FMT HUMAN")) {
    matlabReadableOutput = false;
    Serial.println(F("[OK] Human readable telemetry enabled"));

  } else if (cmd.startsWith(F("OC ")) || cmd.startsWith(F("OVERCURRENT ")) || cmd.startsWith(F("CURFAULT "))) {
    int firstSpace = cmd.indexOf(' ');
    String rest = cmd.substring(firstSpace + 1);
    int secondSpace = rest.indexOf(' ');
    String percentStr = (secondSpace >= 0) ? rest.substring(0, secondSpace) : rest;
    String delayStr = (secondSpace >= 0) ? rest.substring(secondSpace + 1) : String(F("250"));

    float percentValue = percentStr.toFloat();
    if (percentValue > 1.0f) {
      percentValue /= 100.0f;
    }
    currentFault.thresholdPercent = clampf(percentValue, 0.0f, 10.0f);
    currentFault.tripDelayMs = (uint32_t)delayStr.toInt();
    if (currentFault.tripDelayMs < 1) {
      currentFault.tripDelayMs = 1;
    }

    Serial.print(F("[OK] Overcurrent threshold set to "));
    Serial.print(currentFault.thresholdPercent * 100.0f, 1);
    Serial.print(F("% above baseline, delay="));
    Serial.print(currentFault.tripDelayMs);
    Serial.println(F(" ms"));

  } else if (cmd.startsWith(F("VOL ")) || cmd.startsWith(F("VOLUME "))) {
    float vol = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    infusion.targetVolumeMl = vol;
    Serial.print(F("[OK] Target volume set to "));
    Serial.print(infusion.targetVolumeMl, 2);
    Serial.println(F(" mL"));

  } else if (cmd.startsWith(F("FLOW "))) {
    float flow = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    infusion.flowRateMlPerMin = flow;
    Serial.print(F("[OK] Flow rate set to "));
    Serial.print(infusion.flowRateMlPerMin, 2);
    Serial.println(F(" mL/min"));

  } else if (cmd == F("INFUSE") || cmd == F("INJECT")) {
    startInfusion();

  } else if (cmd == F("MODE MANUAL") || cmd == F("MANUAL")) {
    setControlMode(ControlMode::Manual);
    pid.enabled = false;
    stopMotor();
    Serial.println(F("[OK] Control mode -> MANUAL"));

  } else if (cmd == F("MODE PID") || cmd == F("PID MODE")) {
    setControlMode(ControlMode::PID);
    motor.running = true;
    armCurrentFaultMonitor();
    applyMotor();
    Serial.println(F("[OK] Control mode -> PID"));

  } else if (cmd == F("MODE SS") || cmd == F("SS ON") || cmd == F("SS MODE")) {
    setControlMode(ControlMode::SS);
    motor.running = true;
    armCurrentFaultMonitor();
    applyMotor();
    Serial.println(F("[OK] Control mode -> SS SERVO"));

  } else if (cmd.startsWith("S ") || cmd.startsWith("SPEED ")) {
    setControlMode(ControlMode::Manual);
    int sp = cmd.substring(cmd.indexOf(' ') + 1).toInt();
    motor.pwm = (uint8_t)constrain(sp, 0, 255);
    if (motor.running) {
      stepMarkerPending = true;
      applyMotor();
    }
    Serial.print(F("[OK] PWM set to "));
    Serial.print(motor.pwm);
    Serial.println(F("/255 (manual mode)"));

  } else if (cmd.startsWith(F("SP ")) || cmd.startsWith(F("SET "))) {
    float target = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    pid.setpointRadPerSec = target;
    ss.setpointRadPerSec = target;
    Serial.print(F("[OK] Setpoint set to "));
    Serial.print(pid.setpointRadPerSec * RAD_PER_SEC_TO_RPM, 3);
    Serial.println(F(" RPM"));

  } else if (cmd == F("PID ON")) {
    setControlMode(ControlMode::PID);
    motor.running = true;
    armCurrentFaultMonitor();
    stepMarkerPending = true;
    applyMotor();
    Serial.println(F("[OK] PID enabled"));

  } else if (cmd == F("PID OFF")) {
    setControlMode(ControlMode::Manual);
    stopMotor();
    Serial.println(F("[OK] PID disabled"));

  } else if (cmd == F("F") || cmd == F("FWD") || cmd == F("FORWARD")) {
    motor.forward = true;
    stepMarkerPending = true;
    if (motor.running) applyMotor();
    Serial.println(F("[OK] Direction -> FORWARD"));

  } else if (cmd == F("R") || cmd == F("REV") || cmd == F("REVERSE")) {
    motor.forward = false;
    stepMarkerPending = true;
    if (motor.running) applyMotor();
    Serial.println(F("[OK] Direction -> REVERSE"));

  } else if (cmd == F("GO") || cmd == F("START")) {
    motor.running = true;
    stepMarkerPending = true;
    armCurrentFaultMonitor();
    if (controlMode == ControlMode::PID) resetPidState();
    if (controlMode == ControlMode::SS) resetSsState();
    applyMotor();
    Serial.println(F("[OK] Motor started"));

  } else if (cmd == F("STOP") || cmd == F("X")) {
    motor.running = false;
    controlMode = ControlMode::Manual;
    pid.enabled = false;
    resetSsState();
    stopMotor();
    resetPidState();
    Serial.println(F("[OK] Motor stopped (coast)"));

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

  } else if (cmd == F("RESET")) {
    // Stop motor and clear all control signals
    stopMotor();
    motor.pwm = 0;
    motor.forward = true;
    motor.running = false;
    
    // Reset encoder and speed tracking
    portENTER_CRITICAL(&encoderMux);
    encoderPulses = 0;
    portEXIT_CRITICAL(&encoderMux);
    speedPulseSnapshot = 0;
    outputRadPerSec = 0.0f;
    
    // Reset all control modes and setpoints
    pid.setpointRadPerSec = 0.0f;
    pid.enabled = false;
    ss.setpointRadPerSec = 0.0f;
    controlMode = ControlMode::Manual;
    
    // Reset state estimators and accumulators
    resetPidState();
    resetSsState();
    resetInfusionState();
    resetCurrentFaultState();
    
    Serial.println(F("[OK] Full system reset: motor, encoder, setpoints, PID, SS, infusion, faults"));

  } else if (cmd == F("C") || cmd == F("CURRENT")) {
    Serial.print(F("[CURRENT] "));
    Serial.print(readCurrentAmps(), 4);
    Serial.println(F(" A"));

  } else if (cmd == F("STBY ON")) {
    digitalWrite(PIN_STBY, HIGH);
    Serial.println(F("[OK] STBY HIGH - driver enabled"));

  } else if (cmd == F("STBY OFF")) {
    motor.running = false;
    controlMode = ControlMode::Manual;
    pid.enabled = false;
    resetSsState();
    stopMotor();
    digitalWrite(PIN_STBY, LOW);
    Serial.println(F("[OK] STBY LOW - driver in standby"));

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
  if (!motor.running || motor.pwm == 0) {
    stopMotor();
    return;
  }

  digitalWrite(PIN_STBY, HIGH);

  if (motor.forward) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }

  ledcWrite(LEDC_CHANNEL, motor.pwm);

  if (stepMarkerPending) {
    Serial.println(F("[STEP_APPLIED]"));
    stepMarkerPending = false;
  }
}

void stopMotor() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(LEDC_CHANNEL, 0);
}

// ═════════════════════════════════════════════
//  CURRENT SENSING
// ═════════════════════════════════════════════
float readADC() {
  long sum = 0;
  for (int i = 0; i < CURRENT_SAMPLES; i++) {
    sum += analogRead(PIN_CURRENT);
    delayMicroseconds(50);
  }

  float avgADC = (float)sum / CURRENT_SAMPLES;
  return ((avgADC / ADC_RESOLUTION) * ADC_REF_MV);
}

float readCurrentAmps() {
  float adcMV = readADC();
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
  float rpm = outputRadPerSec * RAD_PER_SEC_TO_RPM;

  if (matlabReadableOutput) {
    uint32_t t = millis();
    Serial.print(t);
    Serial.print(',');
    Serial.print(motor.pwm);
    Serial.print(',');
    Serial.print(motor.forward ? 1 : -1);
    Serial.print(',');
    Serial.print(p);
    Serial.print(',');
    Serial.print(rpm, 4);
    Serial.print(',');
    Serial.print(outputRadPerSec, 4);
    Serial.print(',');
    Serial.println(amps, 5);
    return;
  }

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
  Serial.print(F(" | Mode="));
  Serial.print(controlModeName());
  Serial.print(F(" | OC="));
  Serial.print(currentFault.latched ? F("TRIP") : (currentFault.armed ? F("ARM") : F("WAIT")));
  if (infusion.infusing) {
    Serial.print(F(" | Infusing=YES | Vol="));
    Serial.print(infusion.volumeInjectedMl, 2);
    Serial.print(F("/"));
    Serial.print(infusion.targetVolumeMl, 2);
    Serial.print(F(" mL | Flow="));
    Serial.print(infusion.flowRateMlPerMin, 2);
    Serial.print(F(" mL/min"));
  }
  Serial.print(F(" | SP RPM="));
  Serial.print(pid.setpointRadPerSec * RAD_PER_SEC_TO_RPM, 2);
  Serial.print(F(" | Pulses="));
  Serial.print(p);
  Serial.print(F(" | Revs="));
  Serial.print(outputRevs, 4);
  Serial.print(F(" | RPM="));
  Serial.println(rpm, 2);
}

void printHelp() {
  Serial.println(F("╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  Infusion Pump - ESP32 Control & Infusion     ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  INFUSION COMMANDS:                            ║"));
  Serial.println(F("║  VOL <mL>     Set target infusion volume       ║"));
  Serial.println(F("║  FLOW <mL/m>  Set infusion flow rate           ║"));
  Serial.println(F("║  INFUSE       Start infusion (auto-stop)       ║"));
  Serial.println(F("║                                                ║"));
  Serial.println(F("║  CONTROL MODES:                                ║"));
  Serial.println(F("║  S <0-255>    Set PWM speed                    ║"));
  Serial.println(F("║  SP <rad/s>   Set PID speed setpoint           ║"));
  Serial.println(F("║  PID ON/OFF   Enable / disable PID control      ║"));
  Serial.println(F("║  MODE MANUAL  Manual PWM mode                  ║"));
  Serial.println(F("║  MODE PID     Closed-loop PID mode             ║"));
  Serial.println(F("║  MODE SS      State-space servo mode           ║"));
  Serial.println(F("║  OC <pct> <ms> Overcurrent trip vs baseline    ║"));
  Serial.println(F("║  MATLAB ON/OFF Enable CSV telemetry for MATLAB ║"));
  Serial.println(F("║  F / FWD      Direction -> Forward             ║"));
  Serial.println(F("║  R / REV      Direction -> Reverse             ║"));
  Serial.println(F("║  GO / START   Start motor                      ║"));
  Serial.println(F("║  STOP / X     Stop motor (coast)               ║"));
  Serial.println(F("║  C            Read current (A)                 ║"));
  Serial.println(F("║  ENC          Read encoder position & speed    ║"));
  Serial.println(F("║  RESET        Zero encoder counter             ║"));
  Serial.println(F("║  STBY ON/OFF  Enable / disable driver          ║"));
  Serial.println(F("║  H / HELP     Show this menu                   ║"));
  Serial.println(F("╠════════════════════════════════════════════════╣"));
  Serial.println(F("║  Telemetry every 20 ms | control: 50 ms        ║"));
  Serial.println(F("║  Syringe: 29mm Ø, 50mL | Screw: 13 TPI         ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝"));
}
