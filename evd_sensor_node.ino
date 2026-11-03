// =============================================================================
// EVD Leveler — SENSOR NODE (misalignment detection)
// -----------------------------------------------------------------------------
// Two BMP581 barometric sensors on one I2C bus:
//   * TRAGUS sensor  (taped at the patient's tragus)
//   * EVD sensor     (mounted at the EVD zero point on the pole)
// Air pressure falls ~0.118 Pa per cm of height, so the pressure difference
// between the two sensors gives their height difference. The result is
// averaged, sanity-checked and sent over WiFi (UDP) to the actuator ESP32.
//
// Libraries (Arduino Library Manager):
//   * "SparkFun BMP581 Arduino Library"  (works with any BMP581 breakout,
//      including Adafruit's)
// Board: any ESP32 (tested to compile with the esp32 Arduino core).
//
// Serial commands (115200 baud, newline-terminated):
//   t  tare / zero: EVD has just been manually leveled to the tragus
//   T  force tare even if temperatures are not yet stable
//   r  toggle raw CSV logging (for characterization tests)
//   p  print one detailed status block
//   h  help
// Button: hold the BOOT button (GPIO0) for 2 s = tare.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "evd_protocol.h"

// ============================ CONFIGURATION ==================================
// --- Wiring ---
static const int I2C_SDA_PIN = 21;          // classic ESP32 DevKit defaults
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_CLOCK_HZ = 100000; // keep low for long tragus cable
static const uint8_t ADDR_TRAGUS = BMP581_I2C_ADDRESS_DEFAULT;   // 0x47
static const uint8_t ADDR_EVD    = BMP581_I2C_ADDRESS_SECONDARY; // 0x46 (address jumper set)
static const int TARE_BUTTON_PIN = 0;        // BOOT button; -1 to disable
static const uint32_t TARE_HOLD_MS = 2000;

// --- Filtering ---
static const float AVG_WINDOW_S = 3.0f;      // moving-average length
// Above this 1-sigma spread (in the window) the reading is flagged NOISY.
static const float MAX_NOISE_CM = 1.5f;

// --- Sanity limits ---
// A real misalignment bigger than this almost certainly means a sensor fell
// off, the patient sat up, or the pole was bumped — a person should look.
static const float MAX_PLAUSIBLE_CM = 30.0f;

// --- Temperature handling (READ THE NOTES IN README) ---
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

// --- Timing ---
static const uint32_t STATUS_PRINT_MS   = 1000;
static const uint32_t WIFI_RETRY_MS     = 5000;
static const int      MAX_READ_ERRORS   = 5;  // consecutive, before fault
// =============================================================================

BMP581 sensTragus;
BMP581 sensEvd;
WiFiUDP udp;
IPAddress actuatorIp(EVD_ACTUATOR_IP_OCTETS);

// Sample timing (set from the ODR the sensors actually accept)
float    odrHz = 5.0f;
uint32_t samplePeriodMs = 200;

// Moving-average ring buffer of raw pressure difference (tragus - evd), Pa
static const int MAX_WINDOW = 128;
double   dpBuf[MAX_WINDOW];
int      windowLen = 15;
int      bufCount = 0, bufHead = 0;

// Latest raw values
float pTragus = 0, pEvd = 0, tTragus = 0, tEvd = 0;
int   consecutiveReadErrors = 0;
bool  sensorsOk = false;

// Temperature history for stability check: one point every 5 s, 60 s span
static const int TEMP_HIST = 13;
float tempHistT[TEMP_HIST], tempHistE[TEMP_HIST];
int   tempHistCount = 0, tempHistHead = 0;
uint32_t lastTempHistMs = 0;
float slopeTragus = 0, slopeEvd = 0;   // degC per minute

// Calibration (tare)
bool   calibrated = false;
double tareOffsetPa = 0;
float  tareTempTragus = 0, tareTempEvd = 0;

// Outputs
float   heightErrCm = 0, noiseCm = 0;
uint8_t flags = 0;
uint32_t txSeq = 0;

// Actuator status (for display only)
EvdActuatorPacket actStatus;
uint32_t lastActStatusMs = 0;
bool haveActStatus = false;

bool rawLogging = false;
uint32_t lastSampleMs = 0, lastPrintMs = 0, lastWifiAttemptMs = 0;
uint32_t buttonDownSinceMs = 0;
bool buttonTareDone = false;

// ------------------------------- Sensors -------------------------------------
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
    bmp5_osr_odr_eff eff = {};
    if (s.getOSREffective(&eff) == BMP5_OK && eff.odr_is_valid) {
      Serial.printf("[%s] OSR_P=128x OSR_T=8x ODR=%.0f Hz\n", name, odrVals[i]);
      return odrVals[i];
    }
  }
  Serial.printf("[%s] WARNING: no valid ODR found, using 1 Hz\n", name);
  return 1.0f;
}

void i2cScan() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " (nothing found — check wiring/power)");
}

bool initSensors() {
  bool ok = true;
  if (sensTragus.beginI2C(ADDR_TRAGUS, Wire) != BMP5_OK) {
    Serial.printf("ERROR: tragus BMP581 not found at 0x%02X\n", ADDR_TRAGUS);
    ok = false;
  }
  if (sensEvd.beginI2C(ADDR_EVD, Wire) != BMP5_OK) {
    Serial.printf("ERROR: EVD BMP581 not found at 0x%02X\n", ADDR_EVD);
    ok = false;
  }
  if (!ok) return false;
  float a = configureSensor(sensTragus, "TRAGUS");
  float b = configureSensor(sensEvd, "EVD");
  odrHz = min(a, b);                       // both must run at the same rate
  samplePeriodMs = (uint32_t)(1000.0f / odrHz);
  windowLen = constrain((int)lroundf(AVG_WINDOW_S * odrHz), 3, MAX_WINDOW);
  bufCount = bufHead = 0;
  Serial.printf("Sampling every %lu ms, averaging %d samples (%.1f s)\n",
                (unsigned long)samplePeriodMs, windowLen, windowLen / odrHz);
  return true;
}

bool readSensors() {
  bmp5_sensor_data a = {0, 0}, b = {0, 0};
  int8_t ea = sensTragus.getSensorData(&a);
  int8_t eb = sensEvd.getSensorData(&b);
  // A pressure of 0 or wildly out of range also counts as a failed read.
  bool okA = (ea == BMP5_OK) && a.pressure > 30000 && a.pressure < 125000;
  bool okB = (eb == BMP5_OK) && b.pressure > 30000 && b.pressure < 125000;
  if (!okA || !okB) {
    consecutiveReadErrors++;
    return false;
  }
  consecutiveReadErrors = 0;
  pTragus = a.pressure; tTragus = a.temperature;
  pEvd    = b.pressure; tEvd    = b.temperature;
  return true;
}

// ------------------------------ Processing -----------------------------------
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
  tempHistT[tempHistHead] = tTragus;
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

void computeOutput() {
  flags = 0;
  if (calibrated)    flags |= SF_CALIBRATED;
  if (tempsStable()) flags |= SF_TEMP_STABLE;
  if (!sensorsOk || consecutiveReadErrors >= MAX_READ_ERRORS) {
    flags |= SF_SENSOR_FAULT;
    return;
  }
  if (bufCount < windowLen) { flags |= SF_WARMING_UP; return; }

  double meanDp, sdDp;
  windowStats(meanDp, sdDp);
  double k = paPerCm();

  // Temperature compensation of each sensor's offset since tare.
  double tempCorr = TCO_TRAGUS_PA_PER_K * (tTragus - tareTempTragus)
                  - TCO_EVD_PA_PER_K    * (tEvd    - tareTempEvd);
  double dp = meanDp - (calibrated ? (tareOffsetPa + tempCorr) : 0.0);

  // dp = P_tragus - P_evd. Higher = lower pressure, so dp > 0 means the EVD
  // sensor is ABOVE the tragus  ->  heightErrorCm > 0 (matches protocol).
  heightErrCm = (float)(dp / k);
  noiseCm     = (float)(sdDp / k);

  bool valid = calibrated;
  if (noiseCm > MAX_NOISE_CM) { flags |= SF_NOISY; valid = false; }
  if (fabsf(heightErrCm) > MAX_PLAUSIBLE_CM) { flags |= SF_IMPLAUSIBLE; valid = false; }
  if (calibrated &&
      (fabsf(tTragus - tareTempTragus) > MAX_TEMP_DRIFT_FROM_TARE_C ||
       fabsf(tEvd - tareTempEvd) > MAX_TEMP_DRIFT_FROM_TARE_C)) {
    flags |= SF_TEMP_DRIFT; valid = false;
  }
  if (valid) flags |= SF_VALID;
}

// ------------------------------ Calibration ----------------------------------
bool doTare(bool force) {
  if (!sensorsOk || bufCount < windowLen) {
    Serial.println("TARE REFUSED: sensors not ready / window not full yet.");
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
  if (!force && sdDp / paPerCm() > MAX_NOISE_CM) {
    Serial.println("TARE REFUSED: readings too noisy (draft? bumped?). Retry or 'T'.");
    return false;
  }
  tareOffsetPa   = meanDp;
  tareTempTragus = tTragus;
  tareTempEvd    = tEvd;
  calibrated     = true;
  Serial.printf("TARE OK: offset %.2f Pa (sensor-to-sensor), Ttragus %.2f C, Tevd %.2f C\n",
                tareOffsetPa, tareTempTragus, tareTempEvd);
  return true;
}

// -------------------------------- Network ------------------------------------
void maintainWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (now - lastWifiAttemptMs < WIFI_RETRY_MS && lastWifiAttemptMs != 0) return;
  lastWifiAttemptMs = now;
  Serial.println("WiFi: connecting to actuator AP...");
  WiFi.disconnect();
  WiFi.begin(EVD_WIFI_SSID, EVD_WIFI_PASS);
}

void sendPacket() {
  if (WiFi.status() != WL_CONNECTED) return;
  EvdSensorPacket pkt = {};
  pkt.magic = EVD_MAGIC;
  pkt.version = EVD_PROTO_VERSION;
  pkt.flags = flags;
  pkt.seq = ++txSeq;
  pkt.heightErrorCm = heightErrCm;
  pkt.noiseCm = noiseCm;
  pkt.tempTragusC = tTragus;
  pkt.tempEvdC = tEvd;
  udp.beginPacket(actuatorIp, EVD_UDP_PORT_ACTUATOR);
  udp.write((const uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}

void receiveStatus(uint32_t now) {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    if (n != (int)sizeof(EvdActuatorPacket)) continue;  // ignore junk
    EvdActuatorPacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != EVD_MAGIC || p.version != EVD_PROTO_VERSION) continue;
    actStatus = p;
    haveActStatus = true;
    lastActStatusMs = now;
  }
}

// ------------------------------- UI / Serial ---------------------------------
void printHelp() {
  Serial.println("Commands: t=tare  T=force tare  r=raw CSV on/off  p=status  h=help");
}

void printFlags(uint8_t f) {
  if (f & SF_VALID)        Serial.print(" VALID");
  if (f & SF_CALIBRATED)   Serial.print(" CAL");
  if (f & SF_TEMP_STABLE)  Serial.print(" T_STABLE");
  if (f & SF_SENSOR_FAULT) Serial.print(" SENSOR_FAULT");
  if (f & SF_IMPLAUSIBLE)  Serial.print(" IMPLAUSIBLE");
  if (f & SF_NOISY)        Serial.print(" NOISY");
  if (f & SF_TEMP_DRIFT)   Serial.print(" TEMP_DRIFT(re-tare)");
  if (f & SF_WARMING_UP)   Serial.print(" FILLING");
  if (!(f & SF_CALIBRATED) && !(f & SF_SENSOR_FAULT)) Serial.print(" NEEDS_TARE");
}

void printStatusLine(uint32_t now) {
  Serial.printf("err %+6.2f cm  (+-%.2f)  Tt %.2f Te %.2f C  |", heightErrCm,
                noiseCm, tTragus, tEvd);
  printFlags(flags);
  Serial.print("  | WiFi ");
  Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "--");
  if (haveActStatus && now - lastActStatusMs < 3000) {
    Serial.printf("  act %s pos %+.1f mm", evdStateName(actStatus.state),
                  actStatus.positionMm);
    if (actStatus.faults) Serial.printf(" faults 0x%04X", actStatus.faults);
  } else {
    Serial.print("  act (no status)");
  }
  Serial.println();
}

void printDetail() {
  Serial.println("---- SENSOR NODE STATUS ----");
  Serial.printf("P tragus %.2f Pa  P evd %.2f Pa  raw dP %.2f Pa\n", pTragus, pEvd,
                pTragus - pEvd);
  Serial.printf("Pa per cm %.4f  window %d/%d  ODR %.0f Hz\n", paPerCm(), bufCount,
                windowLen, odrHz);
  Serial.printf("Temp slope tragus %.3f C/min, evd %.3f C/min (history %d/%d)\n",
                slopeTragus, slopeEvd, tempHistCount, TEMP_HIST);
  Serial.printf("Calibrated %s  offset %.2f Pa  tare temps %.2f / %.2f C\n",
                calibrated ? "yes" : "no", tareOffsetPa, tareTempTragus, tareTempEvd);
  Serial.printf("WiFi %s  IP %s  tx seq %lu\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "down",
                WiFi.localIP().toString().c_str(), (unsigned long)txSeq);
  Serial.println("----------------------------");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 't': doTare(false); break;
      case 'T': doTare(true); break;
      case 'r':
        rawLogging = !rawLogging;
        if (rawLogging)
          Serial.println("ms,p_tragus_pa,p_evd_pa,t_tragus_c,t_evd_c,dp_raw_pa,err_cm,flags");
        break;
      case 'p': printDetail(); break;
      case 'h': case '?': printHelp(); break;
      default: break;   // ignore newlines etc.
    }
  }
}

void handleButton(uint32_t now) {
  if (TARE_BUTTON_PIN < 0) return;
  bool down = digitalRead(TARE_BUTTON_PIN) == LOW;
  if (!down) { buttonDownSinceMs = 0; buttonTareDone = false; return; }
  if (buttonDownSinceMs == 0) buttonDownSinceMs = now;
  if (!buttonTareDone && now - buttonDownSinceMs >= TARE_HOLD_MS) {
    buttonTareDone = true;
    Serial.println("Button tare requested.");
    doTare(false);
  }
}

// --------------------------------- Main --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EVD leveler: SENSOR NODE ===");
  if (TARE_BUTTON_PIN >= 0) pinMode(TARE_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  i2cScan();
  sensorsOk = initSensors();
  if (!sensorsOk) Serial.println("Will keep retrying sensor init every 5 s.");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency
  WiFi.setAutoReconnect(true);
  udp.begin(EVD_UDP_PORT_SENSOR);
  printHelp();
  Serial.println("Level the EVD to the tragus by hand, wait for T_STABLE, then tare.");
}

void loop() {
  uint32_t now = millis();
  handleSerial();
  handleButton(now);
  maintainWifi(now);
  receiveStatus(now);

  // Retry sensor init if a sensor was missing or kept failing.
  static uint32_t lastInitTry = 0;
  if ((!sensorsOk || consecutiveReadErrors >= MAX_READ_ERRORS) &&
      now - lastInitTry > 5000) {
    lastInitTry = now;
    sensorsOk = initSensors();
    if (sensorsOk) consecutiveReadErrors = 0;
  }

  if (now - lastSampleMs >= samplePeriodMs) {
    lastSampleMs = now;
    if (sensorsOk && readSensors()) {
      pushSample((double)pTragus - (double)pEvd);
      updateTempHistory(now);
    }
    computeOutput();
    sendPacket();   // sent every sample, valid or not; actuator checks flags

    if (rawLogging) {
      Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n", (unsigned long)now,
                    pTragus, pEvd, tTragus, tEvd, pTragus - pEvd, heightErrCm, flags);
    }
  }

  if (!rawLogging && now - lastPrintMs >= STATUS_PRINT_MS) {
    lastPrintMs = now;
    printStatusLine(now);
  }
}
