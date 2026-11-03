# External Ventricular Drain Leveler

Two ESP32s keep the EVD zero point level with the patient's tragus.

| Node | Folder | Hardware | Job |
|---|---|---|---|
| **Tragus** | `evd_tragus_node/` | ESP32 + BMP581 taped at the tragus | Streams raw pressure/temperature over WiFi |
| **Pole** | `evd_pole_node/` | ESP32 + BMP581 at the EVD zero point + TB6600 + NEMA 17 TR8x2 lead screw | Hosts the WiFi network, computes the misalignment, moves the IV pole |

Height error = (P_tragus − P_evd) / ~0.118 Pa per cm. Positive means the EVD zero is above the tragus, so the pole moves down.

## Libraries (Arduino Library Manager)
- SparkFun BMP581 Arduino Library (works with the Adafruit BMP581)
- AccelStepper (pole node only)

Board: **ESP32 Dev Module** (esp32 core 3.x). Open each folder as its own sketch. `evd_protocol.h` must be identical in both folders.

## Wiring
**Both nodes, BMP581:** SDA→GPIO21, SCL→GPIO22, VIN→3V3, GND→GND. Default address 0x47 (0x46 also detected).

**Pole node, TB6600 (common cathode):** PUL+→GPIO25, DIR+→GPIO26, ENA+→GPIO27; PUL−, DIR−, ENA− to ESP32 GND. Set the DIP switches to **1/8 microstep** (or change `MICROSTEPS`). Some TB6600 clones need 5 V signals; if the motor doesn't step reliably, add a 3.3→5 V level shifter.

Optional limit switches and an e-stop: set `LIMIT_TOP_PIN`, `LIMIT_BOTTOM_PIN` and `ESTOP_PIN` (normally-closed to GND).

## First-time setup
1. Change `EVD_WIFI_SSID` / `EVD_WIFI_PASS` in `evd_protocol.h`, then copy the file into both folders.
2. Flash both boards and open the pole node's Serial Monitor at 115200 baud with line ending set to Newline.
3. With auto off, send `u 5`. If the pole goes **down**, set `DIR_INVERT = true`.
4. Level the EVD to the tragus by hand. Wait until the status line shows `T_STABLE` (about 60 s of steady temperatures), then send `t` (or hold BOOT for 3 s).
5. Send `a` (or press BOOT briefly) to turn automatic correction on.

## Pole node commands
`t` tare · `T` force tare · `a` auto on/off · `u <mm>` / `d <mm>` jog · `s` stop · `c` clear faults · `z` zero position · `r` raw CSV · `p` details · `h` help

## Before any clinical use
- Measure each sensor's temperature coefficient on the bench and set `TCO_TRAGUS_PA_PER_K` / `TCO_EVD_PA_PER_K`. Skin warming of the tragus sensor is the largest error source (~4 cm per 0.5 Pa of drift).
- Agree `TRIGGER_CM`, `TARGET_CM`, `PERSIST_MS` and the soft travel limits with your clinical advisors, and set the soft limits to your lead screw's real travel.
- Fit physical limit switches and an e-stop.
