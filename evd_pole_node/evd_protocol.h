// evd_protocol.h — shared message format for the EVD leveler.
// This file must be IDENTICAL in evd_tragus_node/ and evd_pole_node/.
// If you change anything here, copy it to both folders and bump EVD_PROTO_VERSION.
#pragma once
#include <stdint.h>

// ---------------- System layout ----------------
//   TRAGUS node : ESP32 + BMP581 taped at the patient's tragus. Streams raw
//                 pressure/temperature samples. No math, no motor.
//   POLE node   : ESP32 + BMP581 at the EVD zero point + TB6600/NEMA 17 lead
//                 screw. Hosts the WiFi network, computes the misalignment,
//                 and moves the IV pole.

// ---------------- Network ----------------
// The pole ESP32 creates its own private WiFi network (SoftAP); the tragus
// ESP32 joins it. No hospital/campus WiFi is needed.
#define EVD_WIFI_SSID      "EVD-LEVELER-01"   // make unique per device pair
#define EVD_WIFI_PASS      "change-me-1234"   // WPA2, must be >= 8 characters
#define EVD_WIFI_CHANNEL   6
#define EVD_POLE_IP_OCTETS 192, 168, 4, 1     // SoftAP default address
#define EVD_UDP_PORT_POLE   4210              // pole node listens here
#define EVD_UDP_PORT_TRAGUS 4211              // tragus node listens here (status)

#define EVD_MAGIC          0x21445645UL       // "EVD!" little-endian
#define EVD_PROTO_VERSION  2

// ---------------- Tragus -> Pole (one packet per sensor sample) ----------------
enum EvdTragusFlags : uint8_t {
  TF_SENSOR_OK = 1 << 0,   // pressure/temperature fields hold a fresh, good read
};

struct __attribute__((packed)) EvdTragusPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  flags;          // EvdTragusFlags
  uint16_t sampleHzX10;    // tragus sensor output rate * 10 (e.g. 50 = 5 Hz)
  uint32_t seq;            // increments every packet
  float    pressurePa;
  float    temperatureC;
};

// ---------------- Measurement flags (computed on the pole node) ----------------
// Sign convention (used everywhere):
//   heightErrorCm = height(EVD zero point) - height(tragus)
//   > 0  : EVD zero is ABOVE the tragus  -> pole must move DOWN
//   < 0  : EVD zero is BELOW the tragus  -> pole must move UP
enum EvdMeasFlags : uint16_t {
  MF_VALID        = 1 << 0,  // reading is trustworthy enough to act on
  MF_CALIBRATED   = 1 << 1,  // a tare (zero) has been done this power cycle
  MF_TEMP_STABLE  = 1 << 2,  // both sensor temperatures are steady
  MF_SENSOR_FAULT = 1 << 3,  // EVD (pole) BMP581 missing / read failures
  MF_IMPLAUSIBLE  = 1 << 4,  // error larger than physically plausible
  MF_NOISY        = 1 << 5,  // too much variation in the averaging window
  MF_TEMP_DRIFT   = 1 << 6,  // temperature moved too far from tare conditions
  MF_WARMING_UP   = 1 << 7,  // averaging window not yet full
  MF_TRAGUS_FAULT = 1 << 8,  // tragus node reports its BMP581 is failing
  MF_TRAGUS_LINK  = 1 << 9,  // no recent packets from the tragus node
};

// ---------------- Pole -> Tragus (status, for display) ----------------
enum EvdActuatorState : uint8_t {
  AS_AUTO_OFF = 0,   // automatic correction disabled (manual jog allowed)
  AS_IDLE,           // auto on, aligned (or waiting for valid data)
  AS_WAIT_PERSIST,   // misaligned, waiting to confirm it is not transient
  AS_MOVING,
  AS_SETTLING,       // move done, waiting for fresh averaged readings
  AS_FAULT,          // latched; needs a person to clear it
};

enum EvdActuatorFault : uint16_t {
  AF_NONE          = 0,
  AF_LINK_LOST     = 1 << 0,  // tragus link down; auto-clears when packets return
  AF_ESTOP         = 1 << 1,
  AF_LIMIT_TOP     = 1 << 2,
  AF_LIMIT_BOTTOM  = 1 << 3,
  AF_SOFT_LIMIT    = 1 << 4,  // correction would exceed allowed travel
  AF_NO_CONVERGE   = 1 << 5,  // still misaligned after max iterations
  AF_CYCLE_LIMIT   = 1 << 6,  // single correction cycle tried to move too far
};

struct __attribute__((packed)) EvdStatusPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  state;          // EvdActuatorState
  uint16_t faults;         // EvdActuatorFault bits
  uint32_t seq;
  uint32_t lastTragusSeq;  // last tragus packet the pole node accepted
  uint16_t measFlags;      // EvdMeasFlags
  uint8_t  autoOn;
  uint8_t  reserved;
  float    heightErrorCm;  // filtered, see sign convention above
  float    noiseCm;        // 1-sigma spread inside the averaging window
  float    positionMm;     // + = up, relative to power-on (or 'z' command)
  float    targetMm;
};

inline const char* evdStateName(uint8_t s) {
  switch (s) {
    case AS_AUTO_OFF:     return "AUTO_OFF";
    case AS_IDLE:         return "IDLE";
    case AS_WAIT_PERSIST: return "WAIT";
    case AS_MOVING:       return "MOVING";
    case AS_SETTLING:     return "SETTLING";
    case AS_FAULT:        return "FAULT";
    default:              return "?";
  }
}
