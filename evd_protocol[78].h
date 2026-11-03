// evd_protocol.h — shared message format for the EVD leveler.
// This file must be IDENTICAL in evd_sensor_node/ and evd_actuator_node/.
// If you change anything here, copy it to both folders and bump EVD_PROTO_VERSION.
#pragma once
#include <stdint.h>

// ---------------- Network ----------------
// The actuator ESP32 creates its own private WiFi network (SoftAP); the sensor
// ESP32 joins it. No hospital/campus WiFi is needed.
#define EVD_WIFI_SSID      "EVD-LEVELER-01"   // make unique per device pair
#define EVD_WIFI_PASS      "change-me-1234"   // WPA2, must be >= 8 characters
#define EVD_WIFI_CHANNEL   6
#define EVD_ACTUATOR_IP_OCTETS 192, 168, 4, 1 // SoftAP default address
#define EVD_UDP_PORT_ACTUATOR 4210            // actuator listens here
#define EVD_UDP_PORT_SENSOR   4211            // sensor listens here (status)

#define EVD_MAGIC          0x21445645UL       // "EVD!" little-endian
#define EVD_PROTO_VERSION  1

// ---------------- Sensor -> Actuator ----------------
// Sign convention (used everywhere):
//   heightErrorCm = height(EVD zero point) - height(tragus)
//   > 0  : EVD zero is ABOVE the tragus  -> pole must move DOWN
//   < 0  : EVD zero is BELOW the tragus  -> pole must move UP
enum EvdSensorFlags : uint8_t {
  SF_VALID        = 1 << 0,  // reading is trustworthy enough to act on
  SF_CALIBRATED   = 1 << 1,  // a tare (zero) has been done this power cycle
  SF_TEMP_STABLE  = 1 << 2,  // both sensor temperatures are steady
  SF_SENSOR_FAULT = 1 << 3,  // I2C read failures / sensor missing
  SF_IMPLAUSIBLE  = 1 << 4,  // error larger than physically plausible
  SF_NOISY        = 1 << 5,  // too much variation in the averaging window
  SF_TEMP_DRIFT   = 1 << 6,  // temperature moved too far from tare conditions
  SF_WARMING_UP   = 1 << 7,  // averaging window not yet full
};

struct __attribute__((packed)) EvdSensorPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  flags;          // EvdSensorFlags
  uint16_t reserved;
  uint32_t seq;            // increments every packet
  float    heightErrorCm;  // filtered, see sign convention above
  float    noiseCm;        // 1-sigma spread inside the averaging window
  float    tempTragusC;
  float    tempEvdC;
};

// ---------------- Actuator -> Sensor (status) ----------------
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
  AF_LINK_LOST     = 1 << 0,  // auto-clears when packets return
  AF_ESTOP         = 1 << 1,
  AF_LIMIT_TOP     = 1 << 2,
  AF_LIMIT_BOTTOM  = 1 << 3,
  AF_SOFT_LIMIT    = 1 << 4,  // correction would exceed allowed travel
  AF_NO_CONVERGE   = 1 << 5,  // still misaligned after max iterations
  AF_CYCLE_LIMIT   = 1 << 6,  // single correction cycle tried to move too far
};

struct __attribute__((packed)) EvdActuatorPacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  state;          // EvdActuatorState
  uint16_t faults;         // EvdActuatorFault bits
  uint32_t seq;
  uint32_t lastSensorSeq;  // last sensor packet the actuator accepted
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
