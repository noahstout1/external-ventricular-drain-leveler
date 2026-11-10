// =============================================================================
// EVD Leveler — TRAGUS NODE
// -----------------------------------------------------------------------------
// ESP32 + one BMP581 barometric sensor taped at the patient's tragus.
// This node does no alignment math. It reads pressure/temperature at the
// sensor's output rate and streams every sample over WiFi (UDP) to the POLE
// node, which owns the EVD-zero BMP581, computes the height difference and
// drives the stepper. The pole node sends status back for display.
//
// Libraries (Arduino Library Manager):
//   * "SparkFun BMP581 Arduino Library"  (works with any BMP581 breakout,
//      including Adafruit's)
// Board: any ESP32 (esp32 Arduino core 3.x).
//
// Serial commands (115200 baud):
//   r  toggle raw CSV logging (for characterization tests)
//   p  print one detailed status block
//   h  help
// LED: solid = aligned/valid, slow blink = correcting or not yet valid,
//      fast blink = no link to pole node or a fault.
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "SparkFun_BMP581_Arduino_Library.h"
#include "evd_protocol.h"

// ============================ CONFIGURATION ==================================
// --- Wiring ---
static const int I2C_SDA_PIN = 21;          // classic ESP32 DevKit defaults
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_CLOCK_HZ = 100000; // keep low if the sensor cable is long
static const int LED_PIN = 2;                // onboard LED on most DevKits; -1 off

// --- Timing ---
static const uint32_t STATUS_PRINT_MS   = 2000;
static const float    LEVEL_BAND_CM     = 0.5f;  // shown as "level"; match TARGET_CM on pole
static const uint32_t WIFI_RETRY_MS     = 10000; // a WiFi join can take several s
static const uint32_t POLE_TIMEOUT_MS   = 2000;
static const int      MAX_READ_ERRORS   = 5;     // consecutive, before re-init
// =============================================================================

BMP581 sensor;
WiFiUDP udp;
IPAddress poleIp(EVD_POLE_IP_OCTETS);

bool     sensorOk = false;
uint8_t  sensorAddr = 0;
float    odrHz = 5.0f;
uint32_t samplePeriodMs = 200;
float    pressurePa = 0, tempC = 0;
int      consecutiveReadErrors = 0;
uint32_t txSeq = 0;

EvdStatusPacket poleStatus;
bool     havePoleStatus = false;
uint32_t lastPoleStatusMs = 0;

bool rawLogging = false;
bool poleWasOk = false;          // to announce link changes once
uint32_t lastSampleMs = 0, lastPrintMs = 0, lastWifiAttemptMs = 0, lastInitTryMs = 0;

// ------------------------------- Sensor --------------------------------------
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
  float hz = 0;
  for (size_t i = 0; i < sizeof(odrCodes) && hz == 0; i++) {
    if (s.setODRFrequency(odrCodes[i]) != BMP5_OK) continue;
    delay(20);                                // let OSR_EFF update
    bmp5_osr_odr_eff eff = {};
    if (s.getOSREffective(&eff) == BMP5_OK && eff.odr_is_valid) hz = odrVals[i];
  }
  if (hz == 0) {
    Serial.printf("[%s] WARNING: no valid ODR found, using 1 Hz\n", name);
    s.setODRFrequency(BMP5_ODR_01_HZ);
    hz = 1.0f;
  }
  // On-chip IIR low-pass filter: removes most sample-to-sample noise and short
  // air-pressure gusts (doors, HVAC). Writing it leaves the chip in standby,
  // so switch back to continuous (normal) mode afterwards.
  bmp5_iir_config iir = {};
  iir.set_iir_t = BMP5_IIR_FILTER_COEFF_3;
  iir.set_iir_p = BMP5_IIR_FILTER_COEFF_3;
  iir.shdw_set_iir_t = BMP5_ENABLE;
  iir.shdw_set_iir_p = BMP5_ENABLE;
  if (s.setFilterConfig(&iir) != BMP5_OK) {
    Serial.printf("[%s] WARNING: could not enable IIR filter\n", name);
  }
  if (s.setMode(BMP5_POWERMODE_NORMAL) != BMP5_OK) {
    Serial.printf("[%s] WARNING: could not return to normal mode\n", name);
  }
  return hz;
}

bool initSensor() {
  // Adafruit/SparkFun boards default to 0x47; 0x46 if the address jumper is set.
  const uint8_t addrs[] = {BMP581_I2C_ADDRESS_DEFAULT, BMP581_I2C_ADDRESS_SECONDARY};
  for (uint8_t a : addrs) {
    if (sensor.beginI2C(a, Wire) == BMP5_OK) {
      sensorAddr = a;
      odrHz = configureSensor(sensor, "TRAGUS");
      samplePeriodMs = (uint32_t)(1000.0f / odrHz);
      Serial.printf("Tragus sensor OK (%.0f readings/s)\n", odrHz);
      return true;
    }
  }
  Serial.println("** Tragus sensor NOT FOUND - check its wiring (retrying every 5 s)");
  return false;
}

bool readSensor() {
  bmp5_sensor_data d = {0, 0};
  int8_t e = sensor.getSensorData(&d);
  // A pressure of 0 or wildly out of range also counts as a failed read.
  if (e != BMP5_OK || d.pressure < 30000 || d.pressure > 125000) {
    consecutiveReadErrors++;
    return false;
  }
  consecutiveReadErrors = 0;
  pressurePa = d.pressure;
  tempC = d.temperature;
  return true;
}

// -------------------------------- Network ------------------------------------
void maintainWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttemptMs != 0 && now - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = now;
  WiFi.disconnect();
  WiFi.begin(EVD_WIFI_SSID, EVD_WIFI_PASS);
}

void sendSample(bool ok) {
  if (WiFi.status() != WL_CONNECTED) return;
  EvdTragusPacket pkt = {};
  pkt.magic = EVD_MAGIC;
  pkt.version = EVD_PROTO_VERSION;
  pkt.flags = ok ? TF_SENSOR_OK : 0;
  pkt.sampleHzX10 = (uint16_t)lroundf(odrHz * 10.0f);
  pkt.seq = ++txSeq;
  pkt.pressurePa = pressurePa;
  pkt.temperatureC = tempC;
  udp.beginPacket(poleIp, EVD_UDP_PORT_POLE);
  udp.write((const uint8_t*)&pkt, sizeof(pkt));
  udp.endPacket();
}

void receiveStatus(uint32_t now) {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    // Discard junk: an unread packet blocks parsePacket() forever.
    if (n != (int)sizeof(EvdStatusPacket)) { udp.clear(); continue; }
    EvdStatusPacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != EVD_MAGIC || p.version != EVD_PROTO_VERSION) continue;
    poleStatus = p;
    havePoleStatus = true;
    lastPoleStatusMs = now;
  }
}

bool poleOk(uint32_t now) {
  return havePoleStatus && now - lastPoleStatusMs < POLE_TIMEOUT_MS;
}

// --------------------- Plain-language status text ----------------------------
// Kept in this sketch (not evd_protocol.h) so an older copy of the header in
// the sketch folder still compiles.
const char* stateText(uint8_t s) {
  switch (s) {
    case AS_AUTO_OFF:     return "OFF";
    case AS_IDLE:         return "WATCHING";
    case AS_WAIT_PERSIST: return "CONFIRMING";
    case AS_MOVING:       return "MOVING";
    case AS_SETTLING:     return "SETTLING";
    case AS_FAULT:        return "FAULT";
    default:              return "?";
  }
}

// The single most important problem, in words, or nullptr if there is none.
const char* issueText(uint16_t f, uint16_t faults) {
  if (faults & AF_ESTOP)                         return "E-STOP pressed";
  if (faults & (AF_LIMIT_TOP | AF_LIMIT_BOTTOM)) return "Limit switch hit";
  if (faults & AF_SOFT_LIMIT)                    return "Pole at end of allowed travel";
  if (faults & AF_NO_CONVERGE)                   return "Could not align after several tries";
  if (faults & AF_CYCLE_LIMIT)                   return "Correction needed too much travel";
  if (f & MF_TRAGUS_LINK)                        return "No signal from tragus node";
  if (f & MF_SENSOR_FAULT)                       return "EVD sensor not responding";
  if (f & MF_TRAGUS_FAULT)                       return "Tragus sensor not responding";
  if (!(f & MF_CALIBRATED))                      return "Needs tare (click button)";
  if (f & MF_WARMING_UP)                         return "Collecting readings";
  if (f & MF_TEMP_DRIFT)                         return "Temperature drifted: level by hand, then tare";
  if (f & MF_IMPLAUSIBLE)                        return "Error too large: check sensors";
  if (f & MF_NOISY)                              return "Readings unsteady (draft or movement)";
  return nullptr;
}

// Writes "EVD level (+0.3 cm)", "EVD 2.1 cm HIGH", "EVD 1.8 cm LOW" or "EVD --".
void errorText(char* buf, size_t n, float errCm, uint16_t f, float levelBandCm) {
  bool have = (f & MF_CALIBRATED) &&
              !(f & (MF_SENSOR_FAULT | MF_TRAGUS_LINK | MF_TRAGUS_FAULT | MF_WARMING_UP));
  if (!have)                            snprintf(buf, n, "EVD --");
  else if (fabsf(errCm) <= levelBandCm) snprintf(buf, n, "EVD level (%+.1f cm)", errCm);
  else snprintf(buf, n, "EVD %.1f cm %s", fabsf(errCm), errCm > 0 ? "HIGH" : "LOW");
}

// ------------------------------- UI / Serial ---------------------------------
void printHelp() {
  Serial.println("Tare and auto on/off are done with the button on the POLE node.");
  Serial.println("Debug:    p details | r raw data | h help");
}

// Example: Tragus sensor OK | Auto ON  | WATCHING   | EVD level (+0.3 cm)  | OK
void printStatusLine(uint32_t now) {
  bool sensorGood = sensorOk && consecutiveReadErrors == 0;
  Serial.printf("Tragus sensor %-4s | ", sensorGood ? "OK" : "FAIL");
  if (!poleOk(now)) {
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected to WiFi, waiting for pole node"
                                                 : "Searching for pole node WiFi...");
    return;
  }
  char err[32];
  errorText(err, sizeof(err), poleStatus.heightErrorCm, poleStatus.measFlags,
                   LEVEL_BAND_CM);
  const char* msg = issueText(poleStatus.measFlags, poleStatus.faults & ~AF_LINK_LOST);
  if (!msg) {
    switch (poleStatus.state) {
      case AS_FAULT:        msg = "FAULT - check pole node"; break;
      case AS_WAIT_PERSIST: msg = "Misaligned - confirming before moving"; break;
      case AS_MOVING:       msg = "Moving pole"; break;
      case AS_SETTLING:     msg = "Checking result of move"; break;
      default:              msg = poleStatus.autoOn ? "OK" : "OK (auto is OFF)"; break;
    }
  }
  Serial.printf("Auto %-3s | %-10s | %-20s | %s\n", poleStatus.autoOn ? "ON" : "OFF",
                stateText(poleStatus.state), err, msg);
}

void printDetail() {
  Serial.println("---- TRAGUS NODE STATUS ----");
  Serial.printf("Sensor %s at 0x%02X  ODR %.0f Hz  read errors %d\n",
                sensorOk ? "OK" : "MISSING", sensorAddr, odrHz, consecutiveReadErrors);
  Serial.printf("P %.2f Pa  T %.2f C\n", pressurePa, tempC);
  Serial.printf("WiFi %s  IP %s  RSSI %d dBm  tx seq %lu\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "down",
                WiFi.localIP().toString().c_str(), WiFi.RSSI(), (unsigned long)txSeq);
  Serial.println("----------------------------");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r':
        rawLogging = !rawLogging;
        if (rawLogging) Serial.println("ms,p_tragus_pa,t_tragus_c,ok");
        break;
      case 'p': printDetail(); break;
      case 'h': case '?': printHelp(); break;
      default: break;   // ignore newlines etc.
    }
  }
}

void updateLed(uint32_t now) {
  if (LED_PIN < 0) return;
  bool on;
  if (!poleOk(now) || !sensorOk || poleStatus.state == AS_FAULT)  on = (now / 100) % 2;
  else if (poleStatus.state == AS_IDLE && (poleStatus.measFlags & MF_VALID)) on = true;
  else                                                             on = (now / 500) % 2;
  digitalWrite(LED_PIN, on);
}

// --------------------------------- Main --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n===== EVD LEVELER - TRAGUS NODE =====");
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  sensorOk = initSensor();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // lower latency
  WiFi.setAutoReconnect(true);
  udp.begin(EVD_UDP_PORT_TRAGUS);
  printHelp();
}

void loop() {
  uint32_t now = millis();
  handleSerial();
  maintainWifi(now);
  receiveStatus(now);
  updateLed(now);
  if (poleOk(now) != poleWasOk) {
    poleWasOk = poleOk(now);
    Serial.println(poleWasOk ? "** Connected to pole node" : "** Lost connection to pole node");
  }

  // Retry sensor init if the sensor was missing or kept failing.
  if ((!sensorOk || consecutiveReadErrors >= MAX_READ_ERRORS) &&
      now - lastInitTryMs > 5000) {
    lastInitTryMs = now;
    sensorOk = initSensor();
    if (sensorOk) consecutiveReadErrors = 0;
  }

  if (now - lastSampleMs >= samplePeriodMs) {
    lastSampleMs = now;
    bool ok = sensorOk && readSensor();
    sendSample(ok);   // sent every sample, good or not; pole node checks flags
    if (rawLogging) {
      Serial.printf("%lu,%.3f,%.3f,%d\n", (unsigned long)now, pressurePa, tempC, ok);
    }
  }

  if (!rawLogging && now - lastPrintMs >= STATUS_PRINT_MS) {
    lastPrintMs = now;
    printStatusLine(now);
  }
}
