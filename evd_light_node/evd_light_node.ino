// =============================================================================
// EVD Leveler — LIGHT NODE (status light)
// -----------------------------------------------------------------------------
// ESP32 + WS2812/NeoPixel strip. Joins the pole node's WiFi and listens for the
// status packet the pole node broadcasts 4 times a second. The pole node picks
// the colour (EvdStatusPacket.lightColor); this node only displays it.
//   solid green    aligned
//   solid red      misaligned
//   solid yellow   realigning (pole moving / checking the result of a move)
//   blue pulse     not measuring yet (no tare, tare waiting, starting up)
//   flashing red   check system (fault, lost sensor or link)
// If status packets stop arriving, the light shows flashing red, so a dead or
// out-of-range pole node can never leave a stale green on.
//
// Wiring (same as the bench test):
//   ESP32 GPIO18 -> 74AHCT125 (1A -> 1Y) -> 330 ohm -> strip DIN
//   5 V adapter feeds strip, ESP32 5V/VIN and 74AHCT125 VCC; all grounds common
//   1000 uF across the strip's +5V/GND at the strip end
//
// Libraries: "Adafruit NeoPixel". Board: any ESP32 (esp32 core 3.x).
// evd_protocol.h must be IDENTICAL to the copies in the pole and tragus folders.
//
// Serial (115200): p details | c cycle all colours (test) | n LED chase (test) | h help
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#include "evd_protocol.h"

// ============================ CONFIGURATION ==================================
static const int      STRIP_PIN  = 18;
static const uint16_t NUM_PIXELS = 30;
static const uint8_t  BRIGHTNESS = 60;
static const uint32_t FRAME_MS   = 20;
static const uint32_t STATUS_TIMEOUT_MS = 2000;  // no status this long -> flashing red
static const uint32_t WIFI_RETRY_MS     = 10000;
// =============================================================================

Adafruit_NeoPixel strip(NUM_PIXELS, STRIP_PIN, NEO_GRB + NEO_KHZ800);
WiFiUDP udp;

EvdStatusPacket status;
bool     haveStatus = false;
uint32_t lastStatusMs = 0, lastWifiAttemptMs = 0, lastFrameMs = 0;
uint8_t  shownColor = 0xFF;            // for printing changes

enum TestMode : uint8_t { TEST_NONE, TEST_CYCLE, TEST_CHASE };
TestMode testMode = TEST_NONE;
uint32_t testStartMs = 0;

// ------------- Display: identical to the bench test -------------------------
void fill(uint8_t r, uint8_t g, uint8_t b) {
  strip.fill(strip.Color(r, g, b));
  strip.show();
}

void showColor(uint8_t c, uint32_t now) {
  switch (c) {
    case LC_GREEN:  fill(0, 255, 0);   break;
    case LC_RED:    fill(255, 0, 0);   break;
    case LC_YELLOW: fill(255, 150, 0); break;
    case LC_ATTENTION:
      if ((now / 250) % 2) fill(255, 0, 0); else fill(0, 0, 0);
      break;
    case LC_NOT_READY:
    default: {
      float ph = (now % 3000) / 3000.0f;
      float k = 0.15f + 0.85f * (0.5f - 0.5f * cosf(2.0f * PI * ph));
      fill(0, 0, (uint8_t)(255 * k));
      break;
    }
  }
}

const char* colorName(uint8_t c) {
  switch (c) {
    case LC_GREEN:     return "GREEN (aligned)";
    case LC_RED:       return "RED (misaligned)";
    case LC_YELLOW:    return "YELLOW (realigning)";
    case LC_ATTENTION: return "FLASHING RED (check system)";
    default:           return "BLUE PULSE (not measuring yet)";
  }
}

// ------------------------------- Network -------------------------------------
void maintainWifi(uint32_t now) {
  if (WiFi.status() == WL_CONNECTED) return;
  if (lastWifiAttemptMs != 0 && now - lastWifiAttemptMs < WIFI_RETRY_MS) return;
  lastWifiAttemptMs = now;
  WiFi.disconnect();
  WiFi.begin(EVD_WIFI_SSID, EVD_WIFI_PASS);
}

void receiveStatus(uint32_t now) {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    // Discard junk: an unread packet blocks parsePacket() forever.
    if (n != (int)sizeof(EvdStatusPacket)) { udp.clear(); continue; }
    EvdStatusPacket p;
    udp.read((uint8_t*)&p, sizeof(p));
    if (p.magic != EVD_MAGIC || p.version != EVD_PROTO_VERSION) continue;
    status = p;
    haveStatus = true;
    lastStatusMs = now;
  }
}

// What the light should show right now.
uint8_t wantedColor(uint32_t now) {
  if (!haveStatus) return LC_NOT_READY;                      // starting up
  if (now - lastStatusMs > STATUS_TIMEOUT_MS) return LC_ATTENTION; // pole went silent
  return status.lightColor <= LC_ATTENTION ? status.lightColor : (uint8_t)LC_ATTENTION;
}

// --------------------------------- UI ----------------------------------------
void printHelp() {
  Serial.println("Keys: p details | c cycle all colours (test) | n LED chase (test) | h help");
}

void printDetail(uint32_t now) {
  Serial.println("---- LIGHT NODE STATUS ----");
  Serial.printf("WiFi %s  IP %s  RSSI %d dBm\n",
                WiFi.status() == WL_CONNECTED ? "connected" : "searching",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  if (haveStatus) {
    Serial.printf("Last status %lu ms ago: state %s, error %+.1f cm, auto %s\n",
                  (unsigned long)(now - lastStatusMs), evdStateName(status.state),
                  status.heightErrorCm, status.autoOn ? "ON" : "OFF");
  } else {
    Serial.println("No status received from the pole node yet");
  }
  Serial.printf("Showing: %s\n", colorName(wantedColor(now)));
  Serial.println("---------------------------");
}

void handleSerial(uint32_t now) {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'p': printDetail(now); break;
      case 'c': testMode = TEST_CYCLE; testStartMs = now;
                Serial.println("-> Test: cycling all colours, 3 s each"); break;
      case 'n': testMode = TEST_CHASE; testStartMs = now;
                Serial.printf("-> Test: LED 1..%u should light one at a time\n", NUM_PIXELS); break;
      case 'h': case '?': printHelp(); break;
      default: break;   // ignore newlines etc.
    }
  }
}

// Returns true while a test pattern is running (it owns the strip).
bool runTest(uint32_t now) {
  if (testMode == TEST_CYCLE) {
    static const uint8_t seq[] = {LC_GREEN, LC_RED, LC_YELLOW, LC_NOT_READY, LC_ATTENTION};
    uint32_t step = (now - testStartMs) / 3000;
    if (step >= sizeof(seq)) { testMode = TEST_NONE; shownColor = 0xFF;
                               Serial.println("   Test done - back to live status"); return false; }
    showColor(seq[step], now);
    return true;
  }
  if (testMode == TEST_CHASE) {
    uint32_t i = (now - testStartMs) / 150;
    if (i >= NUM_PIXELS) { testMode = TEST_NONE; shownColor = 0xFF;
                           Serial.println("   Chase done - back to live status"); return false; }
    strip.clear();
    strip.setPixelColor(i, strip.Color(255, 255, 255));
    strip.show();
    return true;
  }
  return false;
}

// --------------------------------- Main --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n===== EVD LEVELER - LIGHT NODE =====");
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // don't miss broadcast status packets
  WiFi.setAutoReconnect(true);
  udp.begin(EVD_UDP_PORT_STATUS);
  printHelp();
}

void loop() {
  uint32_t now = millis();
  handleSerial(now);
  maintainWifi(now);
  receiveStatus(now);

  if (now - lastFrameMs < FRAME_MS) return;
  lastFrameMs = now;
  if (runTest(now)) return;

  uint8_t c = wantedColor(now);
  if (c != shownColor) {
    shownColor = c;
    Serial.printf("** Light: %s\n", colorName(c));
  }
  showColor(c, now);
}
