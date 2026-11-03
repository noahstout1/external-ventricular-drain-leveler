// =============================================================================
// EVD Leveler — ACTUATOR NODE (automatic realignment)
// -----------------------------------------------------------------------------
// NEMA 17 + TB6600 driver + TR8x2 lead screw raising/lowering the IV pole.
// This ESP32 hosts a private WiFi network; the sensor ESP32 joins it and
// streams the height error (EVD zero minus tragus) over UDP.
//
// Control logic (closed loop on the measured error):
//   1. Error must exceed TRIGGER_CM continuously for PERSIST_MS (ignores
//      coughs, head turns, someone bumping the bed).
//   2. Move the pole by -GAIN * error.
//   3. Wait SETTLE_MS for the sensor's moving average to catch up.
//   4. If still outside TARGET_CM, repeat (max MAX_ITERATIONS), else done.
//   Motion stops immediately on: lost link, sensor fault, limit switch,
//   e-stop, or a correction that would exceed travel limits.
//
// Library: "AccelStepper" by Mike McCauley (Arduino Library Manager).
//
// Serial commands (115200 baud, end with newline):
//   a        toggle automatic correction on/off
//   u <mm>   jog pole up   (auto must be off), e.g. "u 5"
//   d <mm>   jog pole down (auto must be off)
//   s        stop motion now
//   c        clear latched faults
//   z        set current position as 0 mm
//   p        print detailed status
//   h        help
// Button: short press BOOT (GPIO0) = toggle auto.
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AccelStepper.h>
#include "evd_protocol.h"

// ============================ CONFIGURATION ==================================
// --- Wiring (TB6600 "common cathode": PUL-, DIR-, ENA- to ESP32 GND) ---
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
static const int  AUTO_BUTTON_PIN  = 0;           // BOOT button; -1 to disable
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

// --- Control (agree these numbers with your clinical client) ---
static const float    TRIGGER_CM       = 1.5f;   // start correcting above this
static const float    TARGET_CM        = 0.5f;   // stop correcting inside this
static const uint32_t PERSIST_MS       = 5000;   // misalignment must last this long
static const uint32_t SETTLE_MS        = 4000;   // >= sensor averaging window + 1 s
static const float    GAIN             = 0.9f;   // <1 avoids overshoot from noise
static const float    MIN_MOVE_MM      = 0.5f;
static const int      MAX_ITERATIONS   = 4;      // per correction cycle
static const float    MAX_CYCLE_MOVE_MM = 80.0f; // total travel per cycle
static const uint32_t LINK_TIMEOUT_MS  = 1500;
static const bool     AUTO_ON_BOOT     = false;  // safer: a person turns it on

static const uint32_t STATUS_TX_MS    = 250;
static const uint32_t STATUS_PRINT_MS = 1000;
// =============================================================================

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
WiFiUDP udp;

EvdActuatorState state = AS_AUTO_OFF;
uint16_t latchedFaults = 0;      // need 'c' to clear
bool     autoEnabled = AUTO_ON_BOOT;

// Latest sensor data
EvdSensorPacket lastPkt;
bool     havePkt = false;
uint32_t lastPktMs = 0;
uint32_t lastSeq = 0;
IPAddress sensorIp;
bool     haveSensorIp = false;

// Control bookkeeping
uint32_t persistStartMs = 0;
int8_t   persistSign = 0;
uint32_t settleStartMs = 0;
int      iteration = 0;
float    cycleMovedMm = 0;

uint32_t lastStatusTxMs = 0, lastPrintMs = 0;
uint32_t txSeq = 0;
String   serialLine;
bool     buttonWasDown = false;
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

void enterFault(uint16_t f, const char* why) {
  halt();
  latchedFaults |= f;
  state = AS_FAULT;
  if (f & AF_ESTOP) driverEnable(false);
  Serial.printf("FAULT: %s (faults 0x%04X). Fix the cause, then send 'c'.\n",
                why, latchedFaults);
}

bool linkOk(uint32_t now) { return havePkt && (now - lastPktMs) < LINK_TIMEOUT_MS; }

// Data good enough to *move* on.
bool pktActionable() {
  return (lastPkt.flags & SF_VALID) && (lastPkt.flags & SF_CALIBRATED) &&
         !(lastPkt.flags & (SF_SENSOR_FAULT | SF_IMPLAUSIBLE));
}
// Data so bad that motion in progress must stop.
bool pktDangerous() {
  return !(lastPkt.flags & SF_CALIBRATED) ||
         (lastPkt.flags & (SF_SENSOR_FAULT | SF_IMPLAUSIBLE));
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
  if (cycleMovedMm + fabsf(moveMm) > MAX_CYCLE_MOVE_MM) {
    enterFault(AF_CYCLE_LIMIT, "correction cycle exceeded max travel");
    return;
  }
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

// ------------------------------- Network -------------------------------------
void receivePackets(uint32_t now) {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    if (n != (int)sizeof(EvdSensorPacket)) continue;
    EvdSensorPacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != EVD_MAGIC || p.version != EVD_PROTO_VERSION) continue;
    if (havePkt && p.seq == lastSeq) continue;      // duplicate
    lastPkt = p;
    lastSeq = p.seq;
    havePkt = true;
    lastPktMs = now;
    sensorIp = udp.remoteIP();
    haveSensorIp = true;
  }
}

void sendStatus() {
  if (!haveSensorIp) return;
  EvdActuatorPacket s = {};
  s.magic = EVD_MAGIC;
  s.version = EVD_PROTO_VERSION;
  s.state = state;
  s.faults = latchedFaults | (linkOk(millis()) ? 0 : AF_LINK_LOST);
  s.seq = ++txSeq;
  s.lastSensorSeq = lastSeq;
  s.positionMm = posMm();
  s.targetMm = targetMm();
  udp.beginPacket(sensorIp, EVD_UDP_PORT_SENSOR);
  udp.write((const uint8_t*)&s, sizeof(s));
  udp.endPacket();
}

// ---------------------------- Safety checks ----------------------------------
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
      Serial.println("STOP: link to sensor lost during move.");
    } else if (pktDangerous()) {
      halt(); state = AS_IDLE;
      Serial.println("STOP: sensor reported fault/implausible/uncalibrated.");
    }
  }
}

// ---------------------------- Control state machine --------------------------
void controlStep(uint32_t now) {
  if (state == AS_FAULT || state == AS_AUTO_OFF) return;

  bool link = linkOk(now);
  float err = havePkt ? lastPkt.heightErrorCm : 0;
  bool usable = link && pktActionable();

  switch (state) {
    case AS_IDLE:
      if (usable && fabsf(err) > TRIGGER_CM) {
        state = AS_WAIT_PERSIST;
        persistStartMs = now;
        persistSign = err > 0 ? 1 : -1;
        Serial.printf("Misalignment %+.2f cm detected, confirming...\n", err);
      }
      break;

    case AS_WAIT_PERSIST: {
      int8_t sign = err > 0 ? 1 : -1;
      if (!usable || fabsf(err) <= TRIGGER_CM || sign != persistSign) {
        state = AS_IDLE;                        // transient, ignore it
        Serial.println("Misalignment cleared on its own (transient).");
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
        if (!HOLD_WHEN_IDLE) driverEnable(false);
      }
      break;

    case AS_SETTLING:
      if (now - settleStartMs < SETTLE_MS) break;
      // Only trust a packet that arrived after settling finished.
      if (!usable || lastPktMs < settleStartMs + SETTLE_MS) {
        if (now - settleStartMs > SETTLE_MS + 30000) {   // data never came back
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

// --------------------------------- UI ----------------------------------------
void printHelp() {
  Serial.println("Commands: a=auto on/off  u <mm>=jog up  d <mm>=jog down  s=stop  "
                 "c=clear faults  z=zero position  p=status  h=help");
}

void printStatusLine(uint32_t now) {
  Serial.printf("%-8s auto:%s pos %+7.2f mm tgt %+7.2f mm | ", evdStateName(state),
                autoEnabled ? "ON " : "OFF", posMm(), targetMm());
  if (linkOk(now)) {
    Serial.printf("err %+6.2f cm (+-%.2f) flags 0x%02X%s", lastPkt.heightErrorCm,
                  lastPkt.noiseCm, lastPkt.flags,
                  pktActionable() ? "" : " [not actionable]");
  } else {
    Serial.print("NO SENSOR LINK");
  }
  if (latchedFaults) Serial.printf(" | FAULTS 0x%04X", latchedFaults);
  Serial.printf(" | clients %d\n", WiFi.softAPgetStationNum());
}

void printDetail() {
  Serial.println("---- ACTUATOR NODE STATUS ----");
  Serial.printf("steps/mm %.1f  speed %.1f mm/s  soft limits +%.0f/-%.0f mm\n",
                STEPS_PER_MM, MAX_SPEED_MM_S, SOFT_LIMIT_UP_MM, SOFT_LIMIT_DOWN_MM);
  Serial.printf("limits: top %s bottom %s  estop %s\n",
                LIMIT_TOP_PIN < 0 ? "n/a" : (topLimit() ? "TRIPPED" : "ok"),
                LIMIT_BOTTOM_PIN < 0 ? "n/a" : (bottomLimit() ? "TRIPPED" : "ok"),
                ESTOP_PIN < 0 ? "n/a" : (estop() ? "PRESSED" : "ok"));
  Serial.printf("iteration %d  cycle travel %.1f mm  last sensor seq %lu\n",
                iteration, cycleMovedMm, (unsigned long)lastSeq);
  Serial.printf("AP %s  IP %s\n", EVD_WIFI_SSID, WiFi.softAPIP().toString().c_str());
  Serial.println("------------------------------");
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);
  float arg = cmd.length() > 1 ? cmd.substring(1).toFloat() : 0;
  switch (c) {
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
  if (AUTO_BUTTON_PIN < 0) return;
  bool down = digitalRead(AUTO_BUTTON_PIN) == LOW;
  if (down && !buttonWasDown) buttonDownMs = now;
  if (!down && buttonWasDown && now - buttonDownMs > 50 && now - buttonDownMs < 1500) {
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
  else if (state == AS_IDLE && linkOk(now))                on = true;             // solid
  else                                                     on = false;
  digitalWrite(LED_PIN, on);
}

// --------------------------------- Main --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EVD leveler: ACTUATOR NODE ===");

  if (ENA_PIN >= 0) pinMode(ENA_PIN, OUTPUT);
  driverEnable(HOLD_WHEN_IDLE);
  if (LIMIT_TOP_PIN >= 0)    pinMode(LIMIT_TOP_PIN, INPUT_PULLUP);
  if (LIMIT_BOTTOM_PIN >= 0) pinMode(LIMIT_BOTTOM_PIN, INPUT_PULLUP);
  if (ESTOP_PIN >= 0)        pinMode(ESTOP_PIN, INPUT_PULLUP);
  if (AUTO_BUTTON_PIN >= 0)  pinMode(AUTO_BUTTON_PIN, INPUT_PULLUP);
  if (LED_PIN >= 0)          pinMode(LED_PIN, OUTPUT);

  stepper.setPinsInverted(DIR_INVERT, false, false);
  stepper.setMinPulseWidth(10);                     // TB6600 needs >= ~2.5 us
  stepper.setMaxSpeed(MAX_SPEED_MM_S * STEPS_PER_MM);
  stepper.setAcceleration(ACCEL_MM_S2 * STEPS_PER_MM);
  stepper.setCurrentPosition(0);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  // max 1 client: only the sensor node may join
  if (!WiFi.softAP(EVD_WIFI_SSID, EVD_WIFI_PASS, EVD_WIFI_CHANNEL, 0, 1)) {
    Serial.println("ERROR: could not start WiFi access point.");
  }
  udp.begin(EVD_UDP_PORT_ACTUATOR);
  Serial.printf("AP \"%s\" up at %s, listening on UDP %d\n", EVD_WIFI_SSID,
                WiFi.softAPIP().toString().c_str(), EVD_UDP_PORT_ACTUATOR);

  state = autoEnabled ? AS_IDLE : AS_AUTO_OFF;
  if (ESTOP_PIN >= 0 && estop()) enterFault(AF_ESTOP, "E-STOP active at boot");
  printHelp();
}

void loop() {
  uint32_t now = millis();

  stepper.run();            // call as often as possible

  receivePackets(now);
  stepper.run();
  safetyChecks(now);
  controlStep(now);
  stepper.run();

  handleSerial();
  handleButton(now);
  updateLed(now);

  if (now - lastStatusTxMs >= STATUS_TX_MS) { lastStatusTxMs = now; sendStatus(); }
  if (now - lastPrintMs >= STATUS_PRINT_MS) { lastPrintMs = now; printStatusLine(now); }
}
