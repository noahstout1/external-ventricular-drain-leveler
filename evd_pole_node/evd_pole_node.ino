// =============================================================================
// EVD Leveler — POLE NODE (EVD-zero sensor + misalignment detection + realignment)
// -----------------------------------------------------------------------------
// ESP32 mounted on the IV pole with:
//   * a BMP581 at the EVD zero point
//   * NEMA 17 + TB6600 driver + TR8x2 lead screw raising/lowering the IV pole
// This ESP32 hosts a private WiFi network. The TRAGUS node joins it and streams
// raw tragus pressure/temperature over UDP.
//
// Measurement: air pressure falls ~0.118 Pa per cm of height, so
//   (P_tragus - P_evd) / 0.118  = height of EVD zero above the tragus, in cm.
// Each tragus sample is paired with the latest local EVD sample, averaged over
// a moving window, tared (sensor-to-sensor offset removed) and sanity-checked.
//
// Control logic (closed loop on the measured error):
//   1. Error must exceed TRIGGER_CM continuously for PERSIST_MS (ignores
//      coughs, head turns, someone bumping the bed).
//   2. Move the pole by -GAIN * error.
//   3. Throw away the averaging window, wait MECH_SETTLE_MS for the pole to
//      stop swaying, then refill the window with fresh samples.
//   4. If still outside TARGET_CM, repeat (max MAX_ITERATIONS), else done.
//   Motion stops immediately on: lost tragus link, sensor fault, limit switch,
//   e-stop, or a correction that would exceed travel limits.
//
// Libraries (Arduino Library Manager):
//   * "SparkFun BMP581 Arduino Library"
//   * "AccelStepper" by Mike McCauley
// Board: any ESP32 (esp32 Arduino core 3.x).
//
// Serial commands (115200 baud, end with newline):
//   t        tare / zero: EVD has just been manually leveled to the tragus
//   T        force tare even if temperatures are not yet stable
//   a        toggle automatic correction on/off
//   u <mm>   jog pole up   (auto must be off), e.g. "u 5"
//   d <mm>   jog pole down (auto must be off)
//   s        stop motion now
//   c        clear latched faults
//   z        set current position as 0 mm
//   r        toggle raw CSV logging
//   p        print detailed status
//   h        help
// BOOT button (GPIO0): short press = toggle auto, hold 3 s = tare.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <AccelStepper.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "evd_protocol.h"

// ============================ CONFIGURATION ==================================
// --- Sensor wiring ---
static const int I2C_SDA_PIN = 21;          // classic ESP32 DevKit defaults
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_CLOCK_HZ = 100000;

// --- Stepper wiring (TB6600 "common cathode": PUL-, DIR-, ENA- to ESP32 GND) ---
static const int STEP_PIN = 25;      // -> TB6600 PUL+
static const int DIR_PIN  = 26;      // -> TB6600 DIR+
static const int ENA_PIN  = 27;      // -> TB6600 ENA+   (-1 if not wired)
// On TB6600, driving ENA+ (opto ON) DISABLES the motor. So "enabled" = LOW.
static const uint8_t DRIVER_ENABLED_LEVEL = LOW;
static const bool    HOLD_WHEN_IDLE = true;  // keep coils energized when stopped

// Optional switches (-1 = not installed). Wired between pin and GND.
static const int  LIMIT_TOP_PIN    = -1;
static const int  LIMIT_BOTTOM_PIN = -1;
static const bool LIMITS_NORMALLY_CLOSED = true;  // NC = broken wire reads as tripped
static const int  ESTOP_PIN        = -1;
static const bool ESTOP_NORMALLY_CLOSED  = true;
static const int  BUTTON_PIN       = 0;           // BOOT button; -1 to disable
static const uint32_t TARE_HOLD_MS = 3000;
static const int  LED_PIN          = 2;           // onboard LED on most DevKits; -1 off

// --- Mechanics ---
static const float MOTOR_FULL_STEPS = 200.0f;     // 1.8 deg motor
static const float MICROSTEPS       = 8.0f;       // MUST match TB6600 DIP switches
static const float LEAD_MM_PER_REV  = 2.0f;       // TR8x2 = 2 mm per revolution
static const float STEPS_PER_MM = MOTOR_FULL_STEPS * MICROSTEPS / LEAD_MM_PER_REV; // 800
static const bool  DIR_INVERT = false;  // flip if "u" moves the pole DOWN
static const float MAX_SPEED_MM_S = 3.0f;
static const float ACCEL_MM_S2    = 3.0f;

// --- Travel limits (relative to power-on position or last 'z') ---
static const float SOFT_LIMIT_UP_MM   = 150.0f;
static const float SOFT_LIMIT_DOWN_MM = 150.0f;

// --- Filtering ---
static const float AVG_WINDOW_S = 3.0f;      // moving-average length
// The controller acts on the window MEAN, so judge noise by the uncertainty of
// that mean (sample spread / sqrt(N)), not by the spread of single samples.
// Two BMP581s alone give ~1.2 cm per-sample spread (~0.3 cm on a 15-sample mean).
static const float MAX_MEAN_UNC_CM = 0.5f;
// A real misalignment bigger than this almost certainly means a sensor fell
// off, the patient sat up, or the pole was bumped — a person should look.
static const float MAX_PLAUSIBLE_CM = 30.0f;

// --- Temperature handling ---
// BMP581 offset drifts ~0.5 Pa/K (typ). 0.5 Pa is ~4 cm of "fake" height, so
// the skin-warmed tragus sensor is the biggest error source in this design.
// Measure each sensor's coefficient on the bench and enter it here (Pa per K).
static const float TCO_TRAGUS_PA_PER_K = 0.0f;
static const float TCO_EVD_PA_PER_K    = 0.0f;
// If either sensor drifts this far from its temperature at tare, the reading
// is marked invalid (re-tare needed). Tighten/loosen after bench testing.
static const float MAX_TEMP_DRIFT_FROM_TARE_C = 2.0f;
// Temperature counts as "stable" when it changes less than this per minute.
static const float TEMP_STABLE_C_PER_MIN = 0.10f;

// --- Physics ---
static const double R_DRY_AIR = 287.05;      // J/(kg*K)
static const double G0        = 9.80665;     // m/s^2

// --- Control (agree these numbers with your clinical client) ---
static const float    TRIGGER_CM        = 1.5f;   // start correcting above this
static const float    TARGET_CM         = 0.5f;   // stop correcting inside this
static const uint32_t PERSIST_MS        = 5000;   // misalignment must last this long
static const uint32_t PERSIST_DROPOUT_MS = 3000;  // brief unusable data tolerated while confirming
static const uint32_t MECH_SETTLE_MS    = 1000;   // ignore samples after a move (sway)
static const uint32_t SETTLE_TIMEOUT_MS = 30000;  // give up waiting for fresh data
static const float    GAIN              = 0.9f;   // <1 avoids overshoot from noise
static const float    MIN_MOVE_MM       = 0.5f;
static const int      MAX_ITERATIONS    = 4;      // per correction cycle
static const uint32_t AUTO_TARE_AFTER_MS = 60000; // one forced tare this long after boot
static const uint32_t LINK_TIMEOUT_MS   = 1500;   // tragus packets must arrive this often
static const bool     AUTO_ON_BOOT      = false;  // safer: a person turns it on

// --- Timing ---
static const uint32_t STATUS_TX_MS    = 250;
static const uint32_t STATUS_PRINT_MS = 1000;
static const int      MAX_READ_ERRORS = 5;        // consecutive, before fault
// =============================================================================

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
BMP581 sensEvd;
WiFiUDP udp;

// ---- EVD (local) sensor ----
bool     evdOk = false;
uint8_t  evdAddr = 0;
float    evdOdrHz = 5.0f;
uint32_t evdPeriodMs = 200;
float    pEvd = 0, tEvd = 0;
uint32_t lastEvdReadMs = 0;      // last GOOD read
uint32_t lastEvdAttemptMs = 0;
int      consecutiveReadErrors = 0;

// ---- Tragus (remote) sensor ----
EvdTragusPacket tragus;
bool     haveTragus = false;
uint32_t lastTragusMs = 0;
uint32_t lastTragusSeq = 0;
uint16_t tragusHzX10 = 0;
IPAddress tragusIp;

// ---- Moving-average ring buffer of pressure difference (tragus - evd), Pa ----
static const int MAX_WINDOW = 128;
double   dpBuf[MAX_WINDOW];
int      windowLen = 15;
int      bufCount = 0, bufHead = 0;
uint32_t holdSamplesUntilMs = 0;   // after motion, ignore samples until then

// ---- Temperature history for stability check: one point every 5 s, 60 s span ----
static const int TEMP_HIST = 13;
float tempHistT[TEMP_HIST], tempHistE[TEMP_HIST];
int   tempHistCount = 0, tempHistHead = 0;
uint32_t lastTempHistMs = 0;
float slopeTragus = 0, slopeEvd = 0;   // degC per minute

// ---- Calibration (tare) ----
bool   calibrated = false;
bool   autoTareDone = false;
uint32_t lastAutoTareTryMs = 0;
double tareOffsetPa = 0;
float  tareTempTragus = 0, tareTempEvd = 0;

// ---- Measurement outputs ----
float    heightErrCm = 0, noiseCm = 0, meanUncCm = 0;  // noiseCm = per-sample 1-sigma
uint16_t measFlags = 0;

// ---- Control ----
EvdActuatorState state = AS_AUTO_OFF;
uint16_t latchedFaults = 0;      // need 'c' to clear
bool     autoEnabled = AUTO_ON_BOOT;
uint32_t persistStartMs = 0;
uint32_t persistLastUsableMs = 0;
int8_t   persistSign = 0;
uint32_t settleStartMs = 0;
int      iteration = 0;
float    cycleMovedMm = 0;
bool     wasMoving = false;

// ---- UI ----
bool     rawLogging = false;
uint32_t lastStatusTxMs = 0, lastPrintMs = 0, lastInitTryMs = 0, lastComputeMs = 0;
uint32_t txSeq = 0;
String   serialLine;
bool     buttonWasDown = false, buttonLongDone = false;
uint32_t buttonDownMs = 0;

// ------------------------------- Helpers -------------------------------------
float posMm()    { return stepper.currentPosition() / STEPS_PER_MM; }
float targetMm() { return stepper.targetPosition() / STEPS_PER_MM; }
bool  isMoving() { return stepper.distanceToGo() != 0; }

bool inputTripped(int pin, bool normallyClosed) {
  if (pin < 0) return false;
  bool low = digitalRead(pin) == LOW;          // INPUT_PULLUP: closed = LOW
  return normallyClosed ? !low : low;
}
bool topLimit()    { return inputTripped(LIMIT_TOP_PIN, LIMITS_NORMALLY_CLOSED); }
bool bottomLimit() { return inputTripped(LIMIT_BOTTOM_PIN, LIMITS_NORMALLY_CLOSED); }
bool estop()       { return inputTripped(ESTOP_PIN, ESTOP_NORMALLY_CLOSED); }

void driverEnable(bool on) {
  if (ENA_PIN < 0) return;
  digitalWrite(ENA_PIN, on ? DRIVER_ENABLED_LEVEL : !DRIVER_ENABLED_LEVEL);
}

// Immediate stop (low speed, so no deceleration ramp is needed).
void halt() {
  stepper.setCurrentPosition(stepper.currentPosition()); // target = here, speed 0
}

bool linkOk(uint32_t now) { return haveTragus && (now - lastTragusMs) < LINK_TIMEOUT_MS; }

// ------------------------------- EVD sensor ----------------------------------
float configureSensor(BMP581& s, const char* name) {
  // Highest oversampling = lowest noise. Then pick the fastest ODR the chip
  // reports as valid for that oversampling.
  bmp5_osr_odr_press_config osr = {};
  osr.osr_t = BMP5_OVERSAMPLING_8X;
  osr.osr_p = BMP5_OVERSAMPLING_128X;
  if (s.setOSRMultipliers(&osr) != BMP5_OK) {
    Serial.printf("[%s] WARNING: could not set oversampling\n", name);
  }
  const uint8_t odrCodes[] = {BMP5_ODR_10_HZ, BMP5_ODR_05_HZ, BMP5_ODR_04_HZ,
                              BMP5_ODR_03_HZ, BMP5_ODR_02_HZ, BMP5_ODR_01_HZ};
  const float odrVals[] = {10, 5, 4, 3, 2, 1};
  for (size_t i = 0; i < sizeof(odrCodes); i++) {
    if (s.setODRFrequency(odrCodes[i]) != BMP5_OK) continue;
    delay(20);                                // let OSR_EFF update
    bmp5_osr_odr_eff eff = {};
    if (s.getOSREffective(&eff) == BMP5_OK && eff.odr_is_valid) {
      Serial.printf("[%s] OSR_P=128x OSR_T=8x ODR=%.0f Hz\n", name, odrVals[i]);
      return odrVals[i];
    }
  }
  Serial.printf("[%s] WARNING: no valid ODR found, using 1 Hz\n", name);
  s.setODRFrequency(BMP5_ODR_01_HZ);
  return 1.0f;
}

bool initEvdSensor() {
  // Adafruit/SparkFun boards default to 0x47; 0x46 if the address jumper is set.
  const uint8_t addrs[] = {BMP581_I2C_ADDRESS_DEFAULT, BMP581_I2C_ADDRESS_SECONDARY};
  for (uint8_t a : addrs) {
    if (sensEvd.beginI2C(a, Wire) == BMP5_OK) {
      evdAddr = a;
      evdOdrHz = configureSensor(sensEvd, "EVD");
      evdPeriodMs = (uint32_t)(1000.0f / evdOdrHz);
      Serial.printf("EVD BMP581 at 0x%02X, sampling every %lu ms\n", a,
                    (unsigned long)evdPeriodMs);
      return true;
    }
  }
  Serial.println("ERROR: EVD BMP581 not found at 0x47 or 0x46 (check wiring).");
  return false;
}

bool readEvdSensor() {
  bmp5_sensor_data d = {0, 0};
  int8_t e = sensEvd.getSensorData(&d);
  // A pressure of 0 or wildly out of range also counts as a failed read.
  if (e != BMP5_OK || d.pressure < 30000 || d.pressure > 125000) {
    consecutiveReadErrors++;
    return false;
  }
  consecutiveReadErrors = 0;
  pEvd = d.pressure;
  tEvd = d.temperature;
  return true;
}

bool evdHealthy() { return evdOk && consecutiveReadErrors < MAX_READ_ERRORS; }

// ------------------------------ Processing -----------------------------------
void resetWindow() { bufCount = bufHead = 0; }

void setWindowFromRate(uint16_t hzX10) {
  float hz = hzX10 / 10.0f;
  if (hz <= 0) hz = 1;
  windowLen = constrain((int)lroundf(AVG_WINDOW_S * hz), 3, MAX_WINDOW);
  resetWindow();
  Serial.printf("Tragus rate %.1f Hz -> averaging %d samples (%.1f s)\n", hz,
                windowLen, windowLen / hz);
}

void pushSample(double dp) {
  dpBuf[bufHead] = dp;
  bufHead = (bufHead + 1) % windowLen;
  if (bufCount < windowLen) bufCount++;
}

void windowStats(double& mean, double& sd) {
  double s = 0;
  for (int i = 0; i < bufCount; i++) s += dpBuf[i];
  mean = s / bufCount;
  double v = 0;
  for (int i = 0; i < bufCount; i++) v += (dpBuf[i] - mean) * (dpBuf[i] - mean);
  sd = bufCount > 1 ? sqrt(v / (bufCount - 1)) : 0;
}

// Pa of pressure difference per cm of height, from current air density.
double paPerCm() {
  double rho = pEvd / (R_DRY_AIR * (tEvd + 273.15)); // EVD sensor ~ room air
  return rho * G0 / 100.0;                            // ~0.118 Pa/cm
}

void updateTempHistory(uint32_t now) {
  if (tempHistCount > 0 && now - lastTempHistMs < 5000) return;
  lastTempHistMs = now;
  tempHistT[tempHistHead] = tragus.temperatureC;
  tempHistE[tempHistHead] = tEvd;
  tempHistHead = (tempHistHead + 1) % TEMP_HIST;
  if (tempHistCount < TEMP_HIST) tempHistCount++;
  if (tempHistCount == TEMP_HIST) {
    int oldest = tempHistHead;                         // ring is full
    int newest = (tempHistHead + TEMP_HIST - 1) % TEMP_HIST;
    float minutes = (TEMP_HIST - 1) * 5.0f / 60.0f;
    slopeTragus = (tempHistT[newest] - tempHistT[oldest]) / minutes;
    slopeEvd    = (tempHistE[newest] - tempHistE[oldest]) / minutes;
  }
}

bool tempsStable() {
  return tempHistCount == TEMP_HIST &&
         fabsf(slopeTragus) < TEMP_STABLE_C_PER_MIN &&
         fabsf(slopeEvd) < TEMP_STABLE_C_PER_MIN;
}

void computeOutput(uint32_t now) {
  measFlags = 0;
  if (calibrated)    measFlags |= MF_CALIBRATED;
  if (tempsStable()) measFlags |= MF_TEMP_STABLE;
  if (!evdHealthy()) measFlags |= MF_SENSOR_FAULT;
  if (!linkOk(now))  measFlags |= MF_TRAGUS_LINK;
  else if (!(tragus.flags & TF_SENSOR_OK)) measFlags |= MF_TRAGUS_FAULT;
  if (measFlags & (MF_SENSOR_FAULT | MF_TRAGUS_LINK | MF_TRAGUS_FAULT)) return;
  if (bufCount < windowLen) { measFlags |= MF_WARMING_UP; return; }

  double meanDp, sdDp;
  windowStats(meanDp, sdDp);
  double k = paPerCm();

  // Temperature compensation of each sensor's offset since tare.
  double tempCorr = TCO_TRAGUS_PA_PER_K * (tragus.temperatureC - tareTempTragus)
                  - TCO_EVD_PA_PER_K    * (tEvd - tareTempEvd);
  double dp = meanDp - (calibrated ? (tareOffsetPa + tempCorr) : 0.0);

  // dp = P_tragus - P_evd. Higher = lower pressure, so dp > 0 means the EVD
  // sensor is ABOVE the tragus  ->  heightErrorCm > 0 (matches protocol).
  heightErrCm = (float)(dp / k);
  noiseCm     = (float)(sdDp / k);
  meanUncCm   = noiseCm / sqrtf((float)bufCount);

  bool valid = calibrated;
  if (meanUncCm > MAX_MEAN_UNC_CM) { measFlags |= MF_NOISY; valid = false; }
  if (fabsf(heightErrCm) > MAX_PLAUSIBLE_CM) { measFlags |= MF_IMPLAUSIBLE; valid = false; }
  if (calibrated &&
      (fabsf(tragus.temperatureC - tareTempTragus) > MAX_TEMP_DRIFT_FROM_TARE_C ||
       fabsf(tEvd - tareTempEvd) > MAX_TEMP_DRIFT_FROM_TARE_C)) {
    measFlags |= MF_TEMP_DRIFT; valid = false;
  }
  if (valid) measFlags |= MF_VALID;
}

// Called for every new tragus packet: pair it with the latest EVD sample.
void onTragusSample(uint32_t now, uint32_t prevTragusMs) {
  if (tragus.sampleHzX10 != tragusHzX10) {
    tragusHzX10 = tragus.sampleHzX10;
    setWindowFromRate(tragusHzX10);
  }
  // A gap in the stream means the window holds stale data: start over.
  if (prevTragusMs != 0 && now - prevTragusMs > LINK_TIMEOUT_MS) resetWindow();

  bool evdFresh = evdHealthy() && now - lastEvdReadMs <= 2 * evdPeriodMs + 50;
  if (!(tragus.flags & TF_SENSOR_OK) || !evdFresh) return;
  if (isMoving() || (int32_t)(now - holdSamplesUntilMs) < 0) return;

  double dp = (double)tragus.pressurePa - (double)pEvd;
  pushSample(dp);
  updateTempHistory(now);
  computeOutput(now);

  if (rawLogging) {
    Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n", (unsigned long)now,
                  tragus.pressurePa, pEvd, tragus.temperatureC, tEvd, dp, heightErrCm,
                  measFlags);
  }
}

// ------------------------------ Calibration ----------------------------------
bool doTare(bool force) {
  if (isMoving()) { Serial.println("TARE REFUSED: pole is moving."); return false; }
  if (!evdHealthy() || !linkOk(millis()) || bufCount < windowLen) {
    Serial.println("TARE REFUSED: sensors/link not ready or window not full yet.");
    return false;
  }
  if (!force && !tempsStable()) {
    Serial.printf("TARE REFUSED: temperatures still changing "
                  "(tragus %.2f, evd %.2f C/min; need < %.2f, and 60 s of history). "
                  "Wait, or send 'T' to force.\n",
                  slopeTragus, slopeEvd, TEMP_STABLE_C_PER_MIN);
    return false;
  }
  double meanDp, sdDp;
  windowStats(meanDp, sdDp);
  if (!force && sdDp / paPerCm() / sqrt((double)bufCount) > MAX_MEAN_UNC_CM) {
    Serial.println("TARE REFUSED: readings too noisy (draft? bumped?). Retry or 'T'.");
    return false;
  }
  tareOffsetPa   = meanDp;
  tareTempTragus = tragus.temperatureC;
  tareTempEvd    = tEvd;
  calibrated     = true;
  computeOutput(millis());
  Serial.printf("TARE OK: offset %.2f Pa (sensor-to-sensor), Ttragus %.2f C, Tevd %.2f C\n",
                tareOffsetPa, tareTempTragus, tareTempEvd);
  return true;
}

// Forced tare once, AUTO_TARE_AFTER_MS after boot (skipped if already tared by hand).
void autoTare(uint32_t now) {
  if (autoTareDone || now < AUTO_TARE_AFTER_MS) return;
  if (calibrated) { autoTareDone = true; return; }
  if (lastAutoTareTryMs != 0 && now - lastAutoTareTryMs < 1000) return;
  lastAutoTareTryMs = now;
  Serial.println("Auto-tare (60 s after boot)...");
  if (doTare(true)) autoTareDone = true;    // refused (moving/no link/filling) -> retry in 1 s
}

// ------------------------------- Control -------------------------------------
void enterFault(uint16_t f, const char* why) {
  halt();
  latchedFaults |= f;
  state = AS_FAULT;
  if (f & AF_ESTOP) driverEnable(false);
  Serial.printf("FAULT: %s (faults 0x%04X). Fix the cause, then send 'c'.\n",
                why, latchedFaults);
}

// Data good enough to *move* on (MF_VALID already implies calibrated + no faults).
bool measActionable() { return (measFlags & MF_VALID) != 0; }
// Data so bad that motion in progress must stop.
bool measDangerous() {
  return !(measFlags & MF_CALIBRATED) ||
         (measFlags & (MF_SENSOR_FAULT | MF_TRAGUS_FAULT | MF_IMPLAUSIBLE));
}

// Request a move to an absolute position (mm). Returns false if refused.
bool requestMoveTo(float tgtMm, bool automatic) {
  if (latchedFaults & AF_ESTOP) return false;
  if (tgtMm > SOFT_LIMIT_UP_MM || tgtMm < -SOFT_LIMIT_DOWN_MM) {
    if (automatic) enterFault(AF_SOFT_LIMIT, "correction exceeds soft travel limit");
    else Serial.println("Refused: beyond soft limit.");
    return false;
  }
  if (tgtMm > posMm() && topLimit()) {
    if (automatic) enterFault(AF_LIMIT_TOP, "top limit switch active");
    else Serial.println("Refused: top limit active.");
    return false;
  }
  if (tgtMm < posMm() && bottomLimit()) {
    if (automatic) enterFault(AF_LIMIT_BOTTOM, "bottom limit switch active");
    else Serial.println("Refused: bottom limit active.");
    return false;
  }
  driverEnable(true);
  stepper.moveTo(lroundf(tgtMm * STEPS_PER_MM));
  return true;
}

// One correction step based on the current error.
void startCorrection(float errCm) {
  float moveMm = -GAIN * errCm * 10.0f;          // error>0 (EVD high) -> down
  if (fabsf(moveMm) < MIN_MOVE_MM) { state = AS_IDLE; return; }
  iteration++;
  Serial.printf("CORRECT #%d: error %+.2f cm -> move %+.1f mm\n", iteration, errCm, moveMm);
  if (requestMoveTo(posMm() + moveMm, true)) {
    cycleMovedMm += fabsf(moveMm);
    state = AS_MOVING;
  }
}

void setAuto(bool on) {
  autoEnabled = on;
  halt();
  if (state != AS_FAULT) state = on ? AS_IDLE : AS_AUTO_OFF;
  Serial.printf("Automatic correction %s\n", on ? "ON" : "OFF");
}

// Any motion (automatic or jog) invalidates the averaging window.
void trackMotion(uint32_t now) {
  bool moving = isMoving();
  if (wasMoving && !moving) {
    resetWindow();
    holdSamplesUntilMs = now + MECH_SETTLE_MS;
    computeOutput(now);     // drop the pre-move VALID flag right away
    if (!HOLD_WHEN_IDLE) driverEnable(false);
  }
  wasMoving = moving;
}

void safetyChecks(uint32_t now) {
  if (ESTOP_PIN >= 0 && estop() && !(latchedFaults & AF_ESTOP)) {
    enterFault(AF_ESTOP, "E-STOP pressed");
  }
  long dir = stepper.distanceToGo();
  if (dir > 0 && topLimit())    enterFault(AF_LIMIT_TOP, "top limit hit while moving");
  if (dir < 0 && bottomLimit()) enterFault(AF_LIMIT_BOTTOM, "bottom limit hit while moving");

  // Automatic motion must stop if we lose trustworthy data.
  if (state == AS_MOVING) {
    if (!linkOk(now)) {
      halt(); state = AS_IDLE;
      Serial.println("STOP: link to tragus node lost during move.");
    } else if (measDangerous()) {
      halt(); state = AS_IDLE;
      Serial.println("STOP: sensor reported fault/implausible/uncalibrated.");
    }
  }
}

void controlStep(uint32_t now) {
  if (state == AS_FAULT || state == AS_AUTO_OFF) return;

  float err = heightErrCm;
  bool usable = linkOk(now) && measActionable();

  switch (state) {
    case AS_IDLE:
      if (usable && fabsf(err) > TRIGGER_CM) {
        state = AS_WAIT_PERSIST;
        persistStartMs = now;
        persistLastUsableMs = now;
        persistSign = err > 0 ? 1 : -1;
        Serial.printf("Misalignment %+.2f cm detected, confirming...\n", err);
      }
      break;

    case AS_WAIT_PERSIST: {
      if (!usable) {
        // A single noisy/late sample must not restart confirmation; only give
        // up if trustworthy data stays away for a while.
        if (now - persistLastUsableMs > PERSIST_DROPOUT_MS) {
          state = AS_IDLE;
          Serial.printf("Confirm aborted: no usable data for %lu ms: noise %.2f cm (mean +-%.2f) "
                        "link %s (age %lu ms) window %d/%d flags:",
                        (unsigned long)(now - persistLastUsableMs), noiseCm, meanUncCm,
                        linkOk(now) ? "ok" : "LOST", (unsigned long)(now - lastTragusMs),
                        bufCount, windowLen);
          printMeasFlags(measFlags);
          Serial.println();
        }
        break;
      }
      persistLastUsableMs = now;
      int8_t sign = err > 0 ? 1 : -1;
      if (fabsf(err) <= TRIGGER_CM || sign != persistSign) {
        state = AS_IDLE;                        // error genuinely went away
        Serial.printf("Misalignment cleared on its own (now %+.2f cm).\n", err);
      } else if (now - persistStartMs >= PERSIST_MS) {
        iteration = 0;
        cycleMovedMm = 0;
        startCorrection(err);
      }
      break;
    }

    case AS_MOVING:
      if (!isMoving()) {
        state = AS_SETTLING;
        settleStartMs = now;
      }
      break;

    case AS_SETTLING:
      // trackMotion() emptied the window when the move ended, so a usable
      // reading here is built only from post-move samples.
      if (!usable) {
        if (now - settleStartMs > SETTLE_TIMEOUT_MS) {   // data never came back
          Serial.println("No usable data after move; returning to IDLE.");
          state = AS_IDLE;
        }
        break;
      }
      if (fabsf(err) <= TARGET_CM) {
        Serial.printf("ALIGNED: error %+.2f cm after %d move(s), pos %+.1f mm\n",
                      err, iteration, posMm());
        state = AS_IDLE;
      } else if (iteration >= MAX_ITERATIONS) {
        enterFault(AF_NO_CONVERGE, "not converging (sensor drift? slipping nut?)");
      } else {
        startCorrection(err);
      }
      break;

    default: break;
  }
}

// ------------------------------- Network -------------------------------------
void receivePackets(uint32_t now) {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    // Discard wrong-size packets: an unread packet blocks parsePacket() forever.
    if (n != (int)sizeof(EvdTragusPacket)) { udp.clear(); continue; }
    EvdTragusPacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != EVD_MAGIC || p.version != EVD_PROTO_VERSION) continue;
    if (haveTragus && p.seq == lastTragusSeq) continue;      // duplicate
    uint32_t prevMs = haveTragus ? lastTragusMs : 0;
    tragus = p;
    lastTragusSeq = p.seq;
    haveTragus = true;
    lastTragusMs = now;
    tragusIp = udp.remoteIP();
    onTragusSample(now, prevMs);
    stepper.run();
  }
}

void sendStatus(uint32_t now) {
  if (!haveTragus) return;
  EvdStatusPacket s = {};
  s.magic = EVD_MAGIC;
  s.version = EVD_PROTO_VERSION;
  s.state = state;
  s.faults = latchedFaults | (linkOk(now) ? 0 : AF_LINK_LOST);
  s.seq = ++txSeq;
  s.lastTragusSeq = lastTragusSeq;
  s.measFlags = measFlags;
  s.autoOn = autoEnabled;
  s.heightErrorCm = heightErrCm;
  s.noiseCm = noiseCm;
  s.positionMm = posMm();
  s.targetMm = targetMm();
  udp.beginPacket(tragusIp, EVD_UDP_PORT_TRAGUS);
  udp.write((const uint8_t*)&s, sizeof(s));
  udp.endPacket();
}

// --------------------------------- UI ----------------------------------------
void printHelp() {
  Serial.println("Commands: t=tare  T=force tare  a=auto on/off  u <mm>=jog up  "
                 "d <mm>=jog down  s=stop  c=clear faults  z=zero position  "
                 "r=raw CSV  p=status  h=help");
}

void printMeasFlags(uint16_t f) {
  if (f & MF_VALID)        Serial.print(" VALID");
  if (f & MF_CALIBRATED)   Serial.print(" CAL");
  if (f & MF_TEMP_STABLE)  Serial.print(" T_STABLE");
  if (f & MF_SENSOR_FAULT) Serial.print(" EVD_SENSOR_FAULT");
  if (f & MF_TRAGUS_FAULT) Serial.print(" TRAGUS_SENSOR_FAULT");
  if (f & MF_TRAGUS_LINK)  Serial.print(" NO_TRAGUS_LINK");
  if (f & MF_IMPLAUSIBLE)  Serial.print(" IMPLAUSIBLE");
  if (f & MF_NOISY)        Serial.print(" NOISY");
  if (f & MF_TEMP_DRIFT)   Serial.print(" TEMP_DRIFT(re-tare)");
  if (f & MF_WARMING_UP)   Serial.print(" FILLING");
  if (!(f & MF_CALIBRATED)) Serial.print(" NEEDS_TARE");
}

void printStatusLine(uint32_t now) {
  Serial.printf("%-8s auto:%s pos %+7.2f mm | err %+6.2f cm (mean +-%.2f, sample +-%.2f) |",
                evdStateName(state), autoEnabled ? "ON " : "OFF", posMm(),
                heightErrCm, meanUncCm, noiseCm);
  printMeasFlags(measFlags);
  if (latchedFaults) Serial.printf(" | FAULTS 0x%04X", latchedFaults);
  Serial.printf(" | clients %d\n", WiFi.softAPgetStationNum());
}

void printDetail() {
  Serial.println("---- POLE NODE STATUS ----");
  Serial.printf("P tragus %.2f Pa  P evd %.2f Pa  raw dP %.2f Pa\n", tragus.pressurePa,
                pEvd, tragus.pressurePa - pEvd);
  Serial.printf("T tragus %.2f C  T evd %.2f C  slopes %.3f / %.3f C/min (hist %d/%d)\n",
                tragus.temperatureC, tEvd, slopeTragus, slopeEvd, tempHistCount, TEMP_HIST);
  Serial.printf("Pa per cm %.4f  window %d/%d  EVD ODR %.0f Hz  tragus %.1f Hz\n",
                paPerCm(), bufCount, windowLen, evdOdrHz, tragusHzX10 / 10.0f);
  Serial.printf("Calibrated %s  offset %.2f Pa  tare temps %.2f / %.2f C\n",
                calibrated ? "yes" : "no", tareOffsetPa, tareTempTragus, tareTempEvd);
  Serial.printf("steps/mm %.1f  speed %.1f mm/s  soft limits +%.0f/-%.0f mm\n",
                STEPS_PER_MM, MAX_SPEED_MM_S, SOFT_LIMIT_UP_MM, SOFT_LIMIT_DOWN_MM);
  Serial.printf("limits: top %s bottom %s  estop %s\n",
                LIMIT_TOP_PIN < 0 ? "n/a" : (topLimit() ? "TRIPPED" : "ok"),
                LIMIT_BOTTOM_PIN < 0 ? "n/a" : (bottomLimit() ? "TRIPPED" : "ok"),
                ESTOP_PIN < 0 ? "n/a" : (estop() ? "PRESSED" : "ok"));
  Serial.printf("iteration %d  cycle travel %.1f mm  last tragus seq %lu\n",
                iteration, cycleMovedMm, (unsigned long)lastTragusSeq);
  Serial.printf("AP %s  IP %s  clients %d\n", EVD_WIFI_SSID,
                WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum());
  Serial.println("--------------------------");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  float arg = cmd.length() > 1 ? cmd.substring(1).toFloat() : 0;
  switch (c) {
    case 't': doTare(false); break;
    case 'T': doTare(true); break;
    case 'a': setAuto(!autoEnabled); break;
    case 'u': case 'd': {
      if (autoEnabled) { Serial.println("Turn auto off ('a') before jogging."); break; }
      if (state == AS_FAULT) { Serial.println("Clear faults ('c') first."); break; }
      if (arg <= 0 || arg > 50) { Serial.println("Usage: u 5   (0 < mm <= 50)"); break; }
      float tgt = posMm() + (c == 'u' ? arg : -arg);
      if (requestMoveTo(tgt, false)) Serial.printf("Jog to %+.1f mm\n", tgt);
      break;
    }
    case 's': halt(); if (state == AS_MOVING) state = AS_IDLE; Serial.println("Stopped."); break;
    case 'c':
      latchedFaults = 0;
      if (estop()) { enterFault(AF_ESTOP, "E-STOP still pressed"); break; }
      driverEnable(HOLD_WHEN_IDLE);
      state = autoEnabled ? AS_IDLE : AS_AUTO_OFF;
      Serial.println("Faults cleared.");
      break;
    case 'z':
      if (isMoving()) { Serial.println("Stop first."); break; }
      stepper.setCurrentPosition(0);
      Serial.println("Position zeroed (soft limits now relative to here).");
      break;
    case 'r':
      rawLogging = !rawLogging;
      if (rawLogging)
        Serial.println("ms,p_tragus_pa,p_evd_pa,t_tragus_c,t_evd_c,dp_raw_pa,err_cm,flags");
      break;
    case 'p': printDetail(); break;
    case 'h': case '?': printHelp(); break;
    default: Serial.println("Unknown command, 'h' for help."); break;
  }
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(serialLine); serialLine = ""; }
    else if (serialLine.length() < 32) serialLine += ch;
  }
}

void handleButton(uint32_t now) {
  if (BUTTON_PIN < 0) return;
  bool down = digitalRead(BUTTON_PIN) == LOW;
  if (down && !buttonWasDown) { buttonDownMs = now; buttonLongDone = false; }
  if (down && !buttonLongDone && now - buttonDownMs >= TARE_HOLD_MS) {
    buttonLongDone = true;
    Serial.println("Button tare requested.");
    doTare(false);
  }
  if (!down && buttonWasDown && !buttonLongDone &&
      now - buttonDownMs > 50 && now - buttonDownMs < 1500) {
    setAuto(!autoEnabled);
  }
  buttonWasDown = down;
}

void updateLed(uint32_t now) {
  if (LED_PIN < 0) return;
  bool on;
  if (state == AS_FAULT)                                   on = (now / 100) % 2;  // fast blink
  else if (state == AS_MOVING || state == AS_SETTLING ||
           state == AS_WAIT_PERSIST)                       on = (now / 500) % 2;  // slow blink
  else if (state == AS_IDLE && measActionable())           on = true;             // solid
  else                                                     on = false;
  digitalWrite(LED_PIN, on);
}

// --------------------------------- Main --------------------------------------
void setup() {
  // Large TX buffer so status prints never block the stepper loop.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EVD leveler: POLE NODE ===");

  if (ENA_PIN >= 0) pinMode(ENA_PIN, OUTPUT);
  driverEnable(HOLD_WHEN_IDLE);
  if (LIMIT_TOP_PIN >= 0)    pinMode(LIMIT_TOP_PIN, INPUT_PULLUP);
  if (LIMIT_BOTTOM_PIN >= 0) pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);
  if (ESTOP_PIN >= 0)        pinMode(ESTOP_PIN, INPUT_PULLUP);
  if (BUTTON_PIN >= 0)       pinMode(BUTTON_PIN, INPUT_PULLUP);
  if (LED_PIN >= 0)          pinMode(LED_PIN, OUTPUT);

  stepper.setPinsInverted(DIR_INVERT, false, false);
  stepper.setMinPulseWidth(10);                     // TB6600 needs >= ~2.5 us
  stepper.setMaxSpeed(MAX_SPEED_MM_S * STEPS_PER_MM);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setCurrentPosition(0);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  evdOk = initEvdSensor();
  if (!evdOk) Serial.println("Will keep retrying sensor init every 5 s.");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  // max 1 client: only the tragus node may join
  if (!WiFi.softAP(EVD_WIFI_SSID, EVD_WIFI_PASS, EVD_WIFI_CHANNEL, 0, 1)) {
    Serial.println("ERROR: could not start WiFi access point.");
  }
  udp.begin(EVD_UDP_PORT_POLE);
  Serial.printf("AP \"%s\" up at %s, listening on UDP %d\n", EVD_WIFI_SSID,
                WiFi.softAPIP().toString().c_str(), EVD_UDP_PORT_POLE);

  state = autoEnabled ? AS_IDLE : AS_AUTO_OFF;
  if (ESTOP_PIN >= 0 && estop()) enterFault(AF_ESTOP, "E-STOP active at boot");
  printHelp();
  Serial.println("Level the EVD to the tragus by hand; the node will auto-tare 60 s after boot "
                 "(or send 't' sooner).");
}

void loop() {
  uint32_t now = millis();

  stepper.run();            // call as often as possible

  // Local EVD sensor at its own output rate (one I2C read is ~1 ms).
  if (!evdHealthy() && now - lastInitTryMs > 5000) {
    lastInitTryMs = now;
    evdOk = initEvdSensor();
    if (evdOk) consecutiveReadErrors = 0;
  }
  if (evdOk && now - lastEvdAttemptMs >= evdPeriodMs) {
    lastEvdAttemptMs = now;
    if (readEvdSensor()) lastEvdReadMs = now;
    stepper.run();
  }

  receivePackets(now);
  stepper.run();
  // Keep link/fault flags current even when no tragus packets arrive.
  if (now - lastComputeMs >= 100) { lastComputeMs = now; computeOutput(now); }
  trackMotion(now);
  safetyChecks(now);
  controlStep(now);
  autoTare(now);
  stepper.run();

  handleSerial();
  handleButton(now);
  updateLed(now);

  if (now - lastStatusTxMs >= STATUS_TX_MS) { lastStatusTxMs = now; sendStatus(now); }
  if (!rawLogging && now - lastPrintMs >= STATUS_PRINT_MS) {
    lastPrintMs = now;
    printStatusLine(now);
  }
}
