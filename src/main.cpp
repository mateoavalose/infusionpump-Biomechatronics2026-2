#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────
//  WIFI & WEB SERVER CONFIGURATION
// ─────────────────────────────────────────────
const char* ssid = "SSID";
const char* password = "Password";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ─────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────
#define PIN_AIN1  26
#define PIN_AIN2  25
#define PIN_PWMA  27
#define PIN_STBY  14
#define PIN_ENCA  18
#define PIN_ENCB  19
#define PIN_CURRENT  34

// ─────────────────────────────────────────────
//  LEDC (PWM) CONFIGURATION
// ─────────────────────────────────────────────
#define LEDC_CHANNEL    0
#define LEDC_FREQ_HZ    5000
#define LEDC_RESOLUTION 8

// ─────────────────────────────────────────────
//  CURRENT SENSOR
// ─────────────────────────────────────────────
static constexpr float ACS712_ZERO_ADC_MV = 1260.0f;
static constexpr float ACS712_SENS_MV_A   =   97.04f;
static constexpr float ADC_REF_MV         = 3300.0f;
static constexpr float ADC_RESOLUTION     = 4095.0f;
static constexpr int   CURRENT_SAMPLES    = 30;

// ─────────────────────────────────────────────
//  MOTOR & CONTROL CONFIGURATION
// ─────────────────────────────────────────────
static constexpr float PULSES_PER_MOTOR_REV = 11.0f;
static constexpr float GEAR_RATIO           = 472.7272f;
static constexpr int   ENC_EDGES_PER_PULSE  = 2;
static constexpr float PULSES_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO * ENC_EDGES_PER_PULSE;

// ─────────────────────────────────────────────
//  CONTROL LOOP & TELEMETRY CONFIGURATION
// ─────────────────────────────────────────────
static constexpr uint32_t SPEED_UPDATE_MS   = 20;
static constexpr float    EMA_ALPHA         = 0.05f;


static constexpr uint32_t WEB_TELEMETRY_MS = 100; // 10Hz updates for the Web UI
static uint32_t lastWebTelemetry = 0;

// ─────────────────────────────────────────────
//  Rad/s to RPM conversion constant
// ─────────────────────────────────────────────
static constexpr float RAD_PER_SEC_TO_RPM = 60.0f / (2.0f * PI);

// ─────────────────────────────────────────────
//  PID & State-Space Controller Constants
// ─────────────────────────────────────────────
static constexpr float PID_KC = 134.345399f;
static constexpr float PID_TI = 0.110091f;
static constexpr float PID_TD = 0.036606f;
static const float PID_KB = 1.0f / sqrtf(PID_TI * PID_TD);

static constexpr float SS_K_OMEGA    = 0.268254f;
static constexpr float SS_K_CURRENT  = 57.986618f;
static constexpr float SS_K_INTEGRAL = -1220.314070f;
static constexpr float SS_K_FF       = 0.039477f;

// ─────────────────────────────────────────────
//  PWM LIMITS
// ─────────────────────────────────────────────
static constexpr float PWM_MIN = 0.0f;
static constexpr float PWM_MAX = 255.0f;

// ─────────────────────────────────────────────
//  CURRENT FAULT DETECTION CONFIGURATION
// ─────────────────────────────────────────────
static constexpr uint32_t CONTROL_PERIOD_MS = 50;
static constexpr uint32_t CURRENT_CHECK_MS = 20;
static constexpr uint32_t CURRENT_FAULT_ARM_MS = 300;
static constexpr uint32_t CURRENT_FAULT_TRIP_MS = 250;
static constexpr float    CURRENT_FAULT_PERCENT = 0.30f;
static constexpr float    CURRENT_FAULT_BASELINE_ALPHA = 0.0005f;
static constexpr float    CURRENT_FAULT_MIN_BASELINE_A = 0.05f;

// ─────────────────────────────────────────────
//  SYRINGE & INFUSION CALCULATIONS
// ─────────────────────────────────────────────
static constexpr float SYRINGE_DIAMETER_MM = 29.0f;           
static constexpr float SCREW_PITCH_TURNS_PER_INCH = 13.0f;    
static constexpr float MM_PER_INCH = 25.4f;
static constexpr float MM_PER_SCREW_TURN = MM_PER_INCH / SCREW_PITCH_TURNS_PER_INCH;
static constexpr float SYRINGE_AREA_MM2 = (SYRINGE_DIAMETER_MM / 2.0f) * (SYRINGE_DIAMETER_MM / 2.0f) * PI;
static constexpr float VOLUME_PER_OUTPUT_REV_ML = (SYRINGE_AREA_MM2 * MM_PER_SCREW_TURN) / 1000.0f;

// ─────────────────────────────────────────────
//  CONTROL MODES
// ─────────────────────────────────────────────
enum class ControlMode { Manual, PID, SS };

enum class InfusionControlMode {
  PID,
  SS
};

// ─────────────────────────────────────────────
//  STATE STRUCTURES
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
  bool warningActive = false;
  float thresholdPercent = CURRENT_FAULT_PERCENT;
  uint32_t armDelayMs = CURRENT_FAULT_ARM_MS;
  uint32_t tripDelayMs = CURRENT_FAULT_TRIP_MS;
} currentFault;

struct InfusionState {
  float targetVolumeMl = 0.0f;          // Target volume to inject [mL]
  float flowRateMlPerMin = 0.0f;        // Flow rate [mL/min]
  float targetRpm = 0.0f;               // Calculated infusion speed [RPM]
  float targetRadPerSec = 0.0f;         // Calculated infusion speed [rad/s]
  float volumeInjectedMl = 0.0f;        // Volume injected so far [mL]
  float baseVolumeInjectedMl = 0.0f;    // Volume injected before a pause [mL]
  long pulsesAtStartOfInfusion = 0;     // Encoder pulses when infusion started
  bool infusing = false;                // Whether currently infusing
  bool paused = false;                  // Whether infusion is paused
  uint32_t infusionStartMs = 0;         // When infusion started
} infusion;

InfusionControlMode infusionControlMode = InfusionControlMode::PID;

ControlMode controlMode = ControlMode::Manual;

volatile long encoderPulses = 0;
long     speedPulseSnapshot = 0;
uint32_t speedLastCalcMs    = 0;
float    outputRadPerSec    = 0.0f;
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
void  logMessage(const String& msg);
void  updateSpeedRadPerSec();
void  updatePidControl();
void  updateSsControl();
void  updateCurrentFaultMonitor(float amps);
float clampf(float value, float lowerBound, float upperBound);
void  resetPidState();
void  resetSsState();
void  resetCurrentFaultState();
void  armCurrentFaultMonitor();
void  tripCurrentFault(const String& reason);
void  setControlMode(ControlMode mode);
String controlModeName();
void  updateInfusionControl();
void  updateInfusionSetpoint();
void  setInfusionControlMode(InfusionControlMode mode);
const __FlashStringHelper* infusionControlModeName();
void  startInfusion();
void  stopInfusion();
void  resetInfusionState();
void  broadcastTelemetry(float amps);
void  IRAM_ATTR encoderISR();

// ═════════════════════════════════════════════
//  WEB UI HTML/JS/CSS (PROGMEM)
// ═════════════════════════════════════════════
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>BioFlow - Infusion Pump</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js" defer></script>
  <style>
    :root { --bg: #f4f4f9; --nav: #2c3e50; --primary: #3498db; --text: #333; --panel: #fff; --danger: #e74c3c; --success: #2ecc71;}
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: var(--bg); margin: 0; color: var(--text); }
    .header { background: var(--nav); color: white; padding: 15px 20px; display: flex; justify-content: space-between; align-items: center;}
    .status-badge { background: #7f8c8d; padding: 5px 10px; border-radius: 15px; font-size: 0.9em; font-weight: bold;}
    .tabs { display: flex; background: #ddd; }
    .tabs button { flex: 1; padding: 12px; cursor: pointer; border: none; background: #ddd; font-size: 16px; font-weight: bold; transition: 0.3s;}
    .tabs button:hover { background: #ccc; }
    .tabs button.active { background: var(--panel); border-bottom: 3px solid var(--primary); color: var(--primary);}
    .tab-content { display: none; padding: 20px; max-width: 900px; margin: auto; }
    .card { background: var(--panel); padding: 20px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); margin-bottom: 20px;}
    .form-group { margin-bottom: 15px; }
    label { display: block; font-weight: bold; margin-bottom: 5px; }
    input[type="number"], input[type="text"] { width: 100%; padding: 10px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box;}
    .btn { padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; color: white; background: var(--primary); font-size: 14px;}
    .btn-danger { background: var(--danger); }
    .btn-success { background: var(--success); }
    .btn:hover { opacity: 0.9; }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
    #logView { width: 100%; height: 300px; font-family: monospace; background: #1e1e1e; color: #00ff00; padding: 10px; border-radius: 4px; overflow-y: auto; resize: none; box-sizing: border-box;}
    .progress-bar-container { background: #e0e0e0; border-radius: 8px; height: 25px; margin-top: 15px; overflow: hidden; position: relative;}
    .progress-bar { background: var(--success); height: 100%; width: 0%; transition: width 0.2s; }
    .progress-text { position: absolute; width: 100%; text-align: center; top: 3px; font-size: 13px; font-weight: bold; color: #000;}
    canvas { background: white; border-radius: 8px; padding: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 15px;}
  </style>
</head>
<body>

  <div class="header">
    <h2>💉 BioFlow Controller</h2>
    <div style="display:flex; gap:10px;">
      <span id="occlusionStatus" class="status-badge" style="display:none;"></span>
      <span id="wsStatus" class="status-badge" style="background: var(--danger);">Offline</span>
      <span id="modeStatus" class="status-badge">MANUAL</span>
    </div>
  </div>

  <div class="tabs">
    <button class="tablink active" onclick="openTab(event, 'infusion')">Infusion</button>
    <button class="tablink" onclick="openTab(event, 'plotting')">Plotting</button>
    <button class="tablink" onclick="openTab(event, 'config')">Manual Config & Safety Limits</button>
    <button class="tablink" onclick="openTab(event, 'manual')">Manual Control & Reset</button>
    <button class="tablink" onclick="openTab(event, 'logs')">System Logs</button>
  </div>

  <div id="infusion" class="tab-content" style="display:block;">
    <div class="card grid-2">
      <div>
        <div class="form-group">
          <label>Target Volume (mL)</label>
          <input type="number" id="volIn" step="0.1" value="10">
        </div>
        <button class="btn" onclick="sendCmd('VOL ' + document.getElementById('volIn').value)">Set Volume</button>
      </div>
      <div>
        <div class="form-group">
          <label>Flow Rate (mL/min)</label>
          <input type="number" id="flowIn" step="0.1" value="5">
        </div>
        <button class="btn" onclick="sendCmd('FLOW ' + document.getElementById('flowIn').value)">Set Flow Rate</button>
      </div>
      <div>
        <div class="form-group">
          <label>Infusion Profile Mode</label>
          <select id="infModeSelect" style="width: 100%; padding: 10px;" onchange="sendCmd('INFUSE_MODE ' + this.value)">
            <option value="PID">PID closed-loop</option>
            <option value="SS">State-Space</option>
          </select>
        </div>
      </div>
    </div>
    <div class="card" style="text-align: center;">
      <button id="btnStart" class="btn btn-success" style="font-size: 20px; padding: 15px 40px;" onclick="sendCmd('INFUSE')">▶ START INFUSION</button>
      <button id="btnPause" class="btn btn-danger" style="display: none; font-size: 20px; padding: 15px 40px;" onclick="sendCmd('PAUSE')">⏸ PAUSE</button>
      <button id="btnResume" class="btn btn-success" style="display: none; font-size: 20px; padding: 15px 40px;" onclick="sendCmd('RESUME')">▶ RESUME</button>
      <button id="btnReset" class="btn btn-danger" style="display: none; font-size: 20px; padding: 15px 40px;" onclick="sendCmd('STOP_INFUSION')">⏹ RESET</button>
      
      <div class="progress-bar-container">
        <div class="progress-bar" id="progressBar"></div>
        <div class="progress-text" id="progressText">0.00 / 0.00 mL</div>
      </div>
    </div>
  </div>

  <div id="plotting" class="tab-content">
    <div style="margin-bottom: 15px; display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
      <div>
        <label>Graph 1 (Main):</label>
        <select id="g1A" onchange="clearCharts()" style="width:100%; padding:5px;">
          <option value="rpm">RPM Actual</option>
          <option value="sp">RPM Setpoint</option>
          <option value="I">Current (A)</option>
          <option value="I_lim_H">Current Limit (High)</option>
          <option value="I_lim_L">Current Limit (Low)</option>
          <option value="volI">Volume Injected (mL)</option>
          <option value="pwm">Motor PWM</option>
        </select>
      </div>
      <div>
        <label>Graph 1 (Secondary):</label>
        <select id="g1B" onchange="clearCharts()" style="width:100%; padding:5px;">
          <option value="none" selected>None</option>
          <option value="rpm">RPM Actual</option>
          <option value="sp">RPM Setpoint</option>
          <option value="I">Current (A)</option>
          <option value="I_lim_H">Current Limit (High)</option>
          <option value="I_lim_L">Current Limit (Low)</option>
          <option value="volI">Volume Injected (mL)</option>
          <option value="pwm">Motor PWM</option>
        </select>
      </div>
    </div>
    <div id="chartStatus" style="display:none; margin-bottom:10px; color: var(--danger); font-weight: bold;">
      Chart.js could not be loaded from CDN. Controls and telemetry remain available, but plotting is disabled.
    </div>
    <canvas id="chart1" height="100"></canvas>

    <div style="margin-bottom: 15px; display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
      <div>
        <label>Graph 2 (Main):</label>
        <select id="g2A" onchange="clearCharts()" style="width:100%; padding:5px;">
          <option value="I" selected>Current (A)</option>
          <option value="rpm">RPM Actual</option>
          <option value="sp">RPM Setpoint</option>
          <option value="I_lim_H">Current Limit (High)</option>
          <option value="I_lim_L">Current Limit (Low)</option>
          <option value="volI">Volume Injected (mL)</option>
          <option value="pwm">Motor PWM</option>
        </select>
      </div>
      <div>
        <label>Graph 2 (Secondary):</label>
        <select id="g2B" onchange="clearCharts()" style="width:100%; padding:5px;">
          <option value="none" selected>None</option>
          <option value="I">Current (A)</option>
          <option value="rpm">RPM Actual</option>
          <option value="sp">RPM Setpoint</option>
          <option value="I_lim_H">Current Limit (High)</option>
          <option value="I_lim_L">Current Limit (Low)</option>
          <option value="volI">Volume Injected (mL)</option>
          <option value="pwm">Motor PWM</option>
        </select>
      </div>
    </div>
    <canvas id="chart2" height="100"></canvas>
  </div>

  <div id="config" class="tab-content">
    <div class="card grid-2">
      <div>
        <h3>Control Mode</h3>
        <button class="btn" onclick="sendCmd('MODE MANUAL')">MANUAL</button>
        <button class="btn" onclick="sendCmd('MODE PID')">PID closed-loop</button>
        <button class="btn" onclick="sendCmd('MODE SS')">State-Space</button>
      </div>
      <div>
        <h3>Manual Overrides</h3>
        <div class="form-group">
          <label>Set PWM (0-255)</label>
          <input type="number" id="pwmIn" value="50">
        </div>
        <button class="btn" onclick="sendCmd('S ' + document.getElementById('pwmIn').value)">Set PWM</button>
        <hr>
        <div class="form-group">
          <label>Set Setpoint (rad/s)</label>
          <input type="number" id="spIn" step="0.1" value="10">
        </div>
        <button class="btn" onclick="sendCmd('SP ' + document.getElementById('spIn').value)">Set PID/SS Rad/s</button>
      </div>
    </div>
    <div class="card">
      <h3>Safety Limits</h3>
      <div class="form-group">
        <label>Overcurrent Trip (% above baseline)</label>
        <input type="number" id="ocPct" value="30">
      </div>
      <div class="form-group">
        <label>Trip Delay (ms)</label>
        <input type="number" id="ocDelay" value="250">
      </div>
      <button class="btn" onclick="sendCmd('OC ' + document.getElementById('ocPct').value + ' ' + document.getElementById('ocDelay').value)">Update Current Limits</button>
    </div>
  </div>

  <div id="manual" class="tab-content">
    <div class="card text-center">
      <h3>Jog & Alignment</h3>
      <button class="btn" onclick="sendCmd('FWD')">Direction: FORWARD</button>
      <button class="btn" onclick="sendCmd('REV')">Direction: REVERSE</button>
      <hr>
      <button class="btn btn-success" onclick="sendCmd('GO')">MOTOR GO</button>
      <button class="btn btn-danger" onclick="sendCmd('STOP')">MOTOR STOP</button>
      <hr>
      <button class="btn" onclick="sendCmd('STBY ON')">Enable Driver (STBY HIGH)</button>
      <button class="btn" onclick="sendCmd('STBY OFF')">Disable Driver (STBY LOW)</button>
      <hr>
      <button class="btn btn-danger" onclick="sendCmd('RESET')">SYSTEM RESET & ZERO ENCODER</button>
      <hr>
      <button class="btn btn-danger" onclick="sendCmd('REBOOT')">REBOOT ESP32</button>
    </div>
  </div>

  <div id="logs" class="tab-content">
    <div class="card">
      <h3>System Console</h3>
      <textarea id="logView" readonly></textarea>
      <br><br>
      <input type="text" id="cmdInput" placeholder="Send manual command" style="width: calc(100% - 100px); display:inline-block;">
      <button class="btn" onclick="sendCmd(document.getElementById('cmdInput').value)">Send</button>
    </div>
  </div>

  <script>
    // Tab switching logic
    function openTab(evt, tabName) {
      document.querySelectorAll('.tab-content').forEach(el => el.style.display = 'none');
      document.querySelectorAll('.tablink').forEach(el => el.classList.remove('active'));
      document.getElementById(tabName).style.display = 'block';
      evt.currentTarget.classList.add('active');
    }

    // Chart.js Setup
    const maxPoints = 50;
    let chart1 = null;
    let chart2 = null;

    function initCharts() {
      if (typeof Chart === 'undefined') {
        document.getElementById('chartStatus').style.display = 'block';
        return;
      }

      const ctx1 = document.getElementById('chart1').getContext('2d');
      const ctx2 = document.getElementById('chart2').getContext('2d');

      chart1 = new Chart(ctx1, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'Trace A', borderColor: '#3498db', data: [], tension: 0.1, pointRadius: 0, yAxisID: 'y' },
            { label: 'Trace B', borderColor: '#2ecc71', data: [], tension: 0.1, pointRadius: 0, yAxisID: 'y1' }
          ]
        },
        options: {
          animation: false,
          responsive: true,
          scales: {
            y: {
              type: 'linear',
              position: 'left',
              suggestedMin: -0.5,
              suggestedMax: 2.5
            },
            y1: {
              type: 'linear',
              position: 'right',
              suggestedMin: -0.5,
              suggestedMax: 2.5,
              grid: { drawOnChartArea: false }
            }
          }
        }
      });

      chart2 = new Chart(ctx2, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'Trace A', borderColor: '#e74c3c', data: [], tension: 0.1, pointRadius: 0, yAxisID: 'y' },
            { label: 'Trace B', borderColor: '#f1c40f', data: [], tension: 0.1, pointRadius: 0, yAxisID: 'y1' }
          ]
        },
        options: {
          animation: false,
          responsive: true,
          scales: {
            y: {
              type: 'linear',
              position: 'left',
              suggestedMin: -0.5,
              suggestedMax: 2.5
            },
            y1: {
              type: 'linear',
              position: 'right',
              suggestedMin: -0.5,
              suggestedMax: 2.5,
              grid: { drawOnChartArea: false }
            }
          }
        }
      });
    }

    function clearCharts() {
      if (!chart1 || !chart2) return;
      chart1.data.labels = []; 
      chart1.data.datasets[0].data = []; 
      chart1.data.datasets[1].data = []; 
      chart1.update();
      chart2.data.labels = []; 
      chart2.data.datasets[0].data = []; 
      chart2.data.datasets[1].data = []; 
      chart2.update();
    }

    function updateCharts(data) {
      if (!chart1 || !chart2) return;
      const timeStr = (data.t / 1000).toFixed(1) + "s";
      const g1A = document.getElementById('g1A').value;
      const g1B = document.getElementById('g1B').value;
      const g2A = document.getElementById('g2A').value;
      const g2B = document.getElementById('g2B').value;

      const isSameUnit = (a, b) => {
        if ((a === 'rpm' || a === 'sp') && (b === 'rpm' || b === 'sp')) return true;
        if ((a === 'I' || a === 'I_lim_H' || a === 'I_lim_L') && (b === 'I' || b === 'I_lim_H' || b === 'I_lim_L')) return true;
        return a === b;
      };

      const colorMap = {
        'rpm': '#3498db',     // Blue (Controlled)
        'sp': '#e74c3c',      // Red (Setpoint)
        'I': '#3498db',       // Blue (Controlled)
        'I_lim_H': '#e74c3c', // Red (Reference)
        'I_lim_L': '#e74c3c', // Red (Reference)
        'volI': '#3498db',    // Blue (Controlled)
        'pwm': '#2ecc71'      // Green (Manipulated)
      };

      const isCurrentAxis = (v) => v === 'I' || v === 'I_lim_H' || v === 'I_lim_L';
      chart1.options.scales.y.suggestedMin  = isCurrentAxis(g1A) ? -0.1 : -0.5;
      chart1.options.scales.y.suggestedMax  = isCurrentAxis(g1A) ? 1.0 : 2.5;
      chart1.options.scales.y1.suggestedMin = isCurrentAxis(g1B) ? -0.1 : -0.5;
      chart1.options.scales.y1.suggestedMax = isCurrentAxis(g1B) ? 1.0 : 2.5;
      chart2.options.scales.y.suggestedMin  = isCurrentAxis(g2A) ? -0.1 : -0.5;
      chart2.options.scales.y.suggestedMax  = isCurrentAxis(g2A) ? 1.0 : 2.5;
      chart2.options.scales.y1.suggestedMin = isCurrentAxis(g2B) ? -0.1 : -0.5;
      chart2.options.scales.y1.suggestedMax = isCurrentAxis(g2B) ? 1.0 : 2.5;

      chart1.data.datasets[1].yAxisID = isSameUnit(g1A, g1B) ? 'y' : 'y1';
      chart2.data.datasets[1].yAxisID = isSameUnit(g2A, g2B) ? 'y' : 'y1';
      chart1.options.scales.y1.display = !isSameUnit(g1A, g1B) && g1B !== "none";
      chart2.options.scales.y1.display = !isSameUnit(g2A, g2B) && g2B !== "none";

      if (chart1.data.labels.length > maxPoints) {
        chart1.data.labels.shift(); 
        chart1.data.datasets[0].data.shift();
        chart1.data.datasets[1].data.shift();
        chart2.data.labels.shift(); 
        chart2.data.datasets[0].data.shift();
        chart2.data.datasets[1].data.shift();
      }
      
      chart1.data.labels.push(timeStr);
      chart1.data.datasets[0].borderColor = colorMap[g1A] || '#333';
      chart1.data.datasets[0].label = document.getElementById('g1A').options[document.getElementById('g1A').selectedIndex].text;
      chart1.data.datasets[0].data.push(data[g1A]);
      
      if (g1B !== "none") {
        chart1.data.datasets[1].hidden = false;
        chart1.data.datasets[1].borderColor = colorMap[g1B] || '#333';
        chart1.data.datasets[1].label = document.getElementById('g1B').options[document.getElementById('g1B').selectedIndex].text;
        chart1.data.datasets[1].data.push(data[g1B]);
      } else {
         chart1.data.datasets[1].hidden = true;
         chart1.data.datasets[1].data.push(null);
      }
      chart1.update();

      chart2.data.labels.push(timeStr);
      chart2.data.datasets[0].borderColor = colorMap[g2A] || '#333';
      chart2.data.datasets[0].label = document.getElementById('g2A').options[document.getElementById('g2A').selectedIndex].text;
      chart2.data.datasets[0].data.push(data[g2A]);

      if (g2B !== "none") {
        chart2.data.datasets[1].hidden = false;
        chart2.data.datasets[1].borderColor = colorMap[g2B] || '#333';
        chart2.data.datasets[1].label = document.getElementById('g2B').options[document.getElementById('g2B').selectedIndex].text;
        chart2.data.datasets[1].data.push(data[g2B]);
      } else {
         chart2.data.datasets[1].hidden = true;
         chart2.data.datasets[1].data.push(null);
      }
      chart2.update();
    }

    // WebSocket Logic
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;

    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onopen = () => {
        document.getElementById('wsStatus').innerText = "Connected";
        document.getElementById('wsStatus').style.background = "var(--success)";
      };
      websocket.onclose = () => {
        document.getElementById('wsStatus').innerText = "Disconnected";
        document.getElementById('wsStatus').style.background = "var(--danger)";
        setTimeout(initWebSocket, 2000);
      };
      websocket.onmessage = onMessage;
    }

    function onMessage(event) {
      const data = JSON.parse(event.data);
      if (data.type === 'log') {
        const logView = document.getElementById('logView');
        logView.value += data.msg + "\n";
        logView.scrollTop = logView.scrollHeight;
      } 
      else if (data.type === 'tel') {
        // Update Status Badge
        document.getElementById('modeStatus').innerText = data.mode;
        if (data.flt) {
          document.getElementById('occlusionStatus').innerText = "Stopped for Occlusion";
          document.getElementById('occlusionStatus').style.background = "var(--danger)";
          document.getElementById('occlusionStatus').style.display = "inline-block";
        } else if (data.I_warn) {
          document.getElementById('occlusionStatus').innerText = "High Current Warning";
          document.getElementById('occlusionStatus').style.background = "#f39c12"; // Orange/Yellow
          document.getElementById('occlusionStatus').style.display = "inline-block";
        } else {
          document.getElementById('occlusionStatus').style.display = "none";
        }
        
        // Sync Infusion Buttons
        if (data.inf) {
          document.getElementById('btnStart').style.display = 'none';
          document.getElementById('btnPause').style.display = 'inline-block';
          document.getElementById('btnResume').style.display = 'none';
          document.getElementById('btnReset').style.display = 'inline-block';
        } else if (data.paused) {
          document.getElementById('btnStart').style.display = 'none';
          document.getElementById('btnPause').style.display = 'none';
          document.getElementById('btnResume').style.display = 'inline-block';
          document.getElementById('btnReset').style.display = 'inline-block';
        } else {
          document.getElementById('btnStart').style.display = 'inline-block';
          document.getElementById('btnPause').style.display = 'none';
          document.getElementById('btnResume').style.display = 'none';
          document.getElementById('btnReset').style.display = 'none';
        }

        // Update Infusion Progress
        if (data.inf || data.paused) {
          const pct = Math.min(100, (data.volI / data.volT) * 100);
          document.getElementById('progressBar').style.width = pct + "%";
          document.getElementById('progressText').innerText = data.volI.toFixed(2) + " / " + data.volT.toFixed(2) + " mL";
        } else {
          document.getElementById('progressBar').style.width = "0%";
          document.getElementById('progressText').innerText = "0.00 / 0.00 mL";
        }

        // Plotting
        updateCharts(data);
      }
    }

    function sendCmd(cmd) {
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(cmd);
      } else {
        alert("WebSocket Disconnected!");
      }
    }

    window.onload = () => {
      initCharts();
      initWebSocket();
    };
  </script>
</body>
</html>
)rawliteral";

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
//  WEBSOCKET EVENT HANDLER
// ═════════════════════════════════════════════
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String cmd = (char*)data;
      processCommand(cmd);
    }
  }
}

// ═════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // WiFi AP Setup
  WiFi.mode(WIFI_STA); // Explicitly set Station Mode
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[OK] Connected!");
  Serial.print("Go to this IP address in your browser: ");
  Serial.println(WiFi.localIP());

  // Web Server Setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", index_html);
  });
  
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP Server Started");

  // Hardware Setup
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
}

// ═════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════
void loop() {
  ws.cleanupClients();
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

  if (now - lastWebTelemetry >= WEB_TELEMETRY_MS) {
    lastWebTelemetry = now;
    broadcastTelemetry(readCurrentAmps());
  }

  // Still allow Serial commands for debugging
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) processCommand(cmd);
  }
}

// ═════════════════════════════════════════════
//  LOGGING OVER WEBSOCKETS
// ═════════════════════════════════════════════
void logMessage(const String& msg) {
  Serial.println(msg); // Local console
  if(ws.count() > 0) {
    JsonDocument doc;
    doc["type"] = "log";
    doc["msg"]  = msg;
    String out;
    serializeJson(doc, out);
    ws.textAll(out);
  }
}

// ═════════════════════════════════════════════
//  TELEMETRY BROADCAST
// ═════════════════════════════════════════════
void broadcastTelemetry(float amps) {
  if(ws.count() == 0) return; // Save cycles if no one is looking

  float referenceBaseline = clampf(currentFault.baselineAmps, CURRENT_FAULT_MIN_BASELINE_A, 1.0e9f);
  float tripH = currentFault.baselineValid ? (referenceBaseline * (1.0f + currentFault.thresholdPercent)) : 0.0f;
  float tripL = currentFault.baselineValid ? (referenceBaseline * (1.0f - currentFault.thresholdPercent)) : 0.0f;
  bool iWarn = false;
  if (motor.running && currentFault.baselineValid && !currentFault.latched) {
      if (fabsf(amps) >= tripH * 0.8f) iWarn = true;
  }

  JsonDocument doc;
  doc["type"] = "tel";
  doc["t"]    = millis();
  doc["rpm"]  = outputRadPerSec * RAD_PER_SEC_TO_RPM;
  doc["sp"]   = pid.setpointRadPerSec * RAD_PER_SEC_TO_RPM;
  doc["I"]    = amps;
  doc["mode"] = controlModeName();
  doc["inf"]  = infusion.infusing;
  doc["paused"] = infusion.paused;
  doc["volI"] = infusion.volumeInjectedMl;
  doc["volT"] = infusion.targetVolumeMl;
  doc["flt"]  = currentFault.latched;
  doc["pwm"]  = motor.running ? motor.pwm : 0;
  doc["I_lim_H"] = tripH;
  doc["I_lim_L"] = tripL;
  doc["I_warn"] = currentFault.warningActive;
  
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ═════════════════════════════════════════════
//  CONTROL LOOPS
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
  outputRadPerSec = (EMA_ALPHA * rawRadPerSec) + ((1.0f - EMA_ALPHA) * outputRadPerSec);
  
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

  //float currentAmps = readCurrentAmps();
  float controlVoltage = SS_K_FF * ss.setpointRadPerSec
                       - SS_K_OMEGA * outputRadPerSec
                       //- SS_K_CURRENT * currentAmps
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
    currentFault.warningActive = false;
    return;
  }

  if (!currentFault.armed) {
    if ((now - currentFault.motorStartMs) >= currentFault.armDelayMs) {
      currentFault.armed = true;
      currentFault.baselineAmps = fabsf(amps);
      currentFault.baselineValid = true;
    } else return;
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
  float warnThresholdHigh = referenceBaseline * (1.0f + currentFault.thresholdPercent * 0.8f);
  float warnThresholdLow = warnThresholdHigh * 0.95f;
  float tripThreshold = referenceBaseline * (1.0f + currentFault.thresholdPercent);

  if (!currentFault.warningActive) {
    currentFault.warningActive = (absAmps >= warnThresholdHigh);
  } else if (absAmps <= warnThresholdLow) {
    currentFault.warningActive = false;
  }

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

  portENTER_CRITICAL(&encoderMux);
  long currentPulses = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  long deltaPulses = currentPulses - infusion.pulsesAtStartOfInfusion;
  infusion.volumeInjectedMl = infusion.baseVolumeInjectedMl + ((float)deltaPulses * VOLUME_PER_OUTPUT_REV_ML / PULSES_PER_OUTPUT_REV);

  if (infusion.volumeInjectedMl >= infusion.targetVolumeMl) {
    stopInfusion();
    infusion.paused = false;
    logMessage("[INFO] Infusion complete. Vol: " + String(infusion.volumeInjectedMl, 2) + " mL");
  }
}

void updateInfusionSetpoint() {
  if (infusion.flowRateMlPerMin <= 0.0f) {
    infusion.targetRpm = 0.0f;
    infusion.targetRadPerSec = 0.0f;
    pid.setpointRadPerSec = 0.0f;
    ss.setpointRadPerSec = 0.0f;
    return;
  }

  infusion.targetRpm = infusion.flowRateMlPerMin / VOLUME_PER_OUTPUT_REV_ML;
  infusion.targetRadPerSec = infusion.targetRpm * 2.0f * PI / 60.0f;
  pid.setpointRadPerSec = infusion.targetRadPerSec;
  ss.setpointRadPerSec = infusion.targetRadPerSec;
}

void setInfusionControlMode(InfusionControlMode mode) {
  infusionControlMode = mode;
}

const __FlashStringHelper* infusionControlModeName() {
  switch (infusionControlMode) {
    case InfusionControlMode::PID: return F("PID");
    case InfusionControlMode::SS:  return F("SS");
  }
  return F("UNKNOWN");
}

void startInfusion() {
  if (infusion.targetVolumeMl <= 0.0f) { logMessage("[ERR] Target vol must be > 0"); return; }
  if (infusion.flowRateMlPerMin <= 0.0f) { logMessage("[ERR] Flow rate must be > 0"); return; }

  updateInfusionSetpoint();

  portENTER_CRITICAL(&encoderMux);
  infusion.pulsesAtStartOfInfusion = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  infusion.baseVolumeInjectedMl = 0.0f;
  infusion.volumeInjectedMl = 0.0f;
  infusion.infusing = true;
  infusion.paused = false;
  infusion.infusionStartMs = millis();

  motor.running = true;
  if (infusionControlMode == InfusionControlMode::PID) {
    setControlMode(ControlMode::PID);
  } else {
    setControlMode(ControlMode::SS);
  }
  armCurrentFaultMonitor();
  applyMotor();

  logMessage("[OK] Infusing: " + String(infusion.targetVolumeMl, 2) + "mL @ " + String(infusion.flowRateMlPerMin, 2) + " mL/min (RPM: " + String(infusion.targetRpm, 2) + ")");
}

void pauseInfusion() {
  if (!infusion.infusing) return;
  infusion.infusing = false;
  infusion.paused = true;
  infusion.baseVolumeInjectedMl = infusion.volumeInjectedMl;
  motor.running = false;
  pid.enabled = false;
  controlMode = ControlMode::Manual;
  stopMotor();
  resetPidState();
  logMessage("[OK] Infusion Paused.");
}

void resumeInfusion() {
  if (!infusion.paused) return;
  
  portENTER_CRITICAL(&encoderMux);
  infusion.pulsesAtStartOfInfusion = encoderPulses;
  portEXIT_CRITICAL(&encoderMux);

  infusion.infusing = true;
  infusion.paused = false;

  motor.running = true;
  if (infusionControlMode == InfusionControlMode::PID) {
    setControlMode(ControlMode::PID);
  } else {
    setControlMode(ControlMode::SS);
  }
  armCurrentFaultMonitor();
  applyMotor();
  logMessage("[OK] Infusion Resumed.");
}

void stopInfusion() {
  infusion.infusing = false;
  infusion.paused = false;
  motor.running = false;
  pid.enabled = false;
  controlMode = ControlMode::Manual;
  stopMotor();
  resetPidState();
  logMessage("[OK] Infusion Stopped.");
}

void resetInfusionState() {
  infusion.targetVolumeMl = 0.0f;
  infusion.flowRateMlPerMin = 0.0f;
  infusion.targetRpm = 0.0f;
  infusion.targetRadPerSec = 0.0f;
  infusion.volumeInjectedMl = 0.0f;
  infusion.baseVolumeInjectedMl = 0.0f;
  infusion.pulsesAtStartOfInfusion = 0;
  infusion.infusing = false;
  infusion.paused = false;
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
  currentFault.warningActive = false;
  digitalWrite(PIN_STBY, HIGH);
}

void armCurrentFaultMonitor() {
  currentFault.motorStartMs = millis();
  currentFault.overThresholdSinceMs = 0;
  currentFault.baselineValid = false;
  currentFault.armed = false;
  currentFault.warningActive = false;
}

void tripCurrentFault(const String& reason) {
  currentFault.latched = true;
  currentFault.warningActive = false;
  motor.running = false;
  pid.enabled = false;
  controlMode = ControlMode::Manual;
  stopMotor();
  resetPidState();
  resetSsState();
  digitalWrite(PIN_STBY, LOW);
  logMessage("[FAULT] Current trip: " + reason);
}

void setControlMode(ControlMode mode) {
  controlMode = mode;
  pid.enabled = (mode == ControlMode::PID);
  if (mode == ControlMode::PID) resetPidState();
  else if (mode == ControlMode::SS) resetSsState();
}

String controlModeName() {
  switch (controlMode) {
    case ControlMode::Manual: return "MANUAL";
    case ControlMode::PID:    return "PID";
    case ControlMode::SS:     return "SS";
  }
  return "UNKNOWN";
}

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
  float avgADC = (float)sum / CURRENT_SAMPLES;
  return ((avgADC / ADC_RESOLUTION) * ADC_REF_MV);
}

float readCurrentAmps() {
  float adcMV = readADC();
  return (ACS712_ZERO_ADC_MV - adcMV) / ACS712_SENS_MV_A - 1.05f; // Subtract 1A to roughly zero out the bad reading on the ADC
}

// ═════════════════════════════════════════════
//  COMMAND PARSER - WebLogger and Serial
// ═════════════════════════════════════════════
void processCommand(const String& raw) {
  String cmd = raw;
  cmd.toUpperCase();

  if (currentFault.latched && cmd != "RESET") {
    logMessage("[LOCKOUT] Press SYSTEM RESET to clear occlusion lockout.");
    return;
  }

  if (cmd.startsWith("OC ")) {
    int firstSpace = cmd.indexOf(' ');
    String rest = cmd.substring(firstSpace + 1);
    int secondSpace = rest.indexOf(' ');
    float percentValue = rest.substring(0, secondSpace).toFloat();
    if (percentValue > 1.0f) percentValue /= 100.0f;
    currentFault.thresholdPercent = clampf(percentValue, 0.0f, 10.0f);
    currentFault.tripDelayMs = (uint32_t)rest.substring(secondSpace + 1).toInt();
    logMessage("[OK] OC Limit updated");

  } else if (cmd.startsWith("VOL ")) {
    infusion.targetVolumeMl = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    logMessage("[OK] Target Vol: " + String(infusion.targetVolumeMl, 2) + " mL");

  } else if (cmd.startsWith("FLOW ")) {
    infusion.flowRateMlPerMin = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    logMessage("[OK] Flow Rate: " + String(infusion.flowRateMlPerMin, 2) + " mL/min");

  } else if (cmd == "INFUSE") { startInfusion();
  } else if (cmd == "PAUSE") { pauseInfusion();
  } else if (cmd == "RESUME") { resumeInfusion();
  } else if (cmd == "STOP_INFUSION") { stopInfusion();
  } else if (cmd == "INFUSE_MODE PID") { setInfusionControlMode(InfusionControlMode::PID); logMessage("[OK] Infusion Mode -> PID");
  } else if (cmd == "INFUSE_MODE SS") { setInfusionControlMode(InfusionControlMode::SS); logMessage("[OK] Infusion Mode -> SS");
  } else if (cmd == "MODE MANUAL") { setControlMode(ControlMode::Manual); stopMotor(); motor.running = false; infusion.infusing = false; infusion.paused = false; logMessage("[OK] Mode -> MANUAL");
  } else if (cmd == "MODE PID") { setControlMode(ControlMode::PID); motor.running = true; armCurrentFaultMonitor(); applyMotor(); logMessage("[OK] Mode -> PID");
  } else if (cmd == "MODE SS") { setControlMode(ControlMode::SS); motor.running = true; armCurrentFaultMonitor(); applyMotor(); logMessage("[OK] Mode -> SS");
  } else if (cmd.startsWith("S ")) {
    setControlMode(ControlMode::Manual);
    motor.pwm = (uint8_t)constrain(cmd.substring(cmd.indexOf(' ') + 1).toInt(), 0, 255);
    if (motor.running) applyMotor();
    logMessage("[OK] PWM: " + String(motor.pwm));
  } else if (cmd.startsWith("SP ")) {
    float target = cmd.substring(cmd.indexOf(' ') + 1).toFloat();
    pid.setpointRadPerSec = target; ss.setpointRadPerSec = target;
    logMessage("[OK] SP: " + String(target) + " rad/s");
  } else if (cmd == "FWD") { motor.forward = true; if(motor.running) applyMotor(); logMessage("[OK] Dir -> FWD");
  } else if (cmd == "REV") { motor.forward = false; if(motor.running) applyMotor(); logMessage("[OK] Dir -> REV");
  } else if (cmd == "GO") { motor.running = true; armCurrentFaultMonitor(); applyMotor(); logMessage("[OK] Motor Start");
  } else if (cmd == "STOP") { motor.running = false; stopMotor(); infusion.infusing = false; infusion.paused = false; pid.enabled = false; logMessage("[OK] Motor Stop");
  } else if (cmd == "RESET") {
    stopMotor(); motor.pwm = 0; motor.forward = true; motor.running = false;
    portENTER_CRITICAL(&encoderMux); encoderPulses = 0; portEXIT_CRITICAL(&encoderMux);
    resetPidState(); resetSsState(); resetInfusionState(); resetCurrentFaultState();
    logMessage("[OK] System Reset & Encoder Zeroed");
  } else if (cmd == "REBOOT") {
    logMessage("[INFO] Rebooting ESP32...");
    delay(500);
    ESP.restart();
  } else if (cmd == "STBY ON") { digitalWrite(PIN_STBY, HIGH); logMessage("[OK] Driver ON");
  } else if (cmd == "STBY OFF") { motor.running = false; stopMotor(); infusion.infusing = false; infusion.paused = false; pid.enabled = false; digitalWrite(PIN_STBY, LOW); logMessage("[OK] Driver OFF (Motor Stopped)");
  } else {
    logMessage("[?] Unknown Command: " + raw);
  }
}
