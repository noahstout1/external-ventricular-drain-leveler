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

**Pole node, push button:** one leg to **GPIO4**, the other leg to **GND**. No resistor is needed (the code turns on the ESP32's internal pull-up). The onboard BOOT button does the same thing.

| Pole node pin | Used for |
|---|---|
| GPIO21 / GPIO22 | EVD BMP581 (SDA / SCL) |
| GPIO25 / 26 / 27 | TB6600 PUL+ / DIR+ / ENA+ |
| GPIO4 | Push button |
| GPIO0 | Onboard BOOT button |
| GPIO2 | Onboard LED |

Optional limit switches and an e-stop: set `LIMIT_TOP_PIN`, `LIMIT_BOTTOM_PIN` and `ESTOP_PIN` (normally-closed to GND). GPIO32, GPIO33 and GPIO13 are free for these.

## First-time setup
1. Change `EVD_WIFI_SSID` / `EVD_WIFI_PASS` in `evd_protocol.h`, then copy the file into both folders.
2. Flash both boards and open the pole node's Serial Monitor at 115200 baud with line ending set to Newline.
3. With auto off, send `u 5`. If the pole goes **down**, flip `DIR_INVERT`.
4. Level the EVD to the tragus by hand. Click the button once (or send `t`). If the readings aren't steady yet, the tare waits and happens by itself once they are (see *When the tare happens* below). At power-up the pole node starts one of these waiting tares automatically (forced after 5 min if the readings never settle).
5. Double-click the button (or send `a`) to turn automatic correction on. Double-click again to turn it off.

## Button
| Action | Result | LED confirms |
|---|---|---|
| 1 click | Tare (zero) both pressure sensors. If the readings aren't steady, it waits and tares by itself once they are. | 1 flash = tared. Short blip every second = waiting. Rapid flicker = refused (e.g. pole moving or no tragus link). |
| 1 click while waiting | Tare right now, skipping the temperature wait | 1 flash |
| 2 clicks | Toggle automatic correction | 2 flashes = auto ON, 3 flashes = auto OFF |

Clicks must be under 0.4 s apart; holds longer than 1.5 s are ignored. No automatic corrections start while a tare is waiting.

## When the tare happens
A waiting tare (power-up, or a click when things aren't ready) checks once a second and tares as soon as all of these hold:
- **The zero has stopped drifting:** the trend of the pressure difference over the last 60 s is under 0.3 cm/min. This is the main test, because it measures the thing being zeroed directly.
- **Neither sensor is still warming or cooling fast:** both temperature trends are under 0.3 °C/min. This is a backstop, for example for a tragus sensor just taped onto skin.
- The 3 s average isn't noisy, the tragus link is up, both sensors are healthy, and the pole isn't moving.

Any pole move (automatic or jog) restarts the 60 s zero trend, because the zero jumps with the move. After 5 min a power-up tare is forced anyway. Click again (or `T`) to force a tare yourself. These thresholds are starting guesses: tune them with `set` once the hardware is running.

## Correction limits
- After each correction cycle, the next one can't start for 20 s (`CORRECTION_COOLDOWN_MS`). Turning auto on resets this.
- One cycle may move the pole at most 200 mm (`MAX_CYCLE_MOVE_MM`); more raises a `CYCLE_LIMIT` fault.
- `Temperature drifted` (`TEMP_DRIFT` in verbose mode) appears once either sensor is 0.5 °C away from its temperature at tare (`MAX_TEMP_DRIFT_FROM_TARE_C`). Auto stops correcting until you level by hand and tare again.

## Getting a good tare
- **Draft shields:** cover each BMP581 with a small cap that blocks air movement but not static pressure, e.g. a 3D-printed or film-canister cap with a ~1 mm vent hole or a plug of open-cell foam over the opening. Don't seal it airtight and don't press anything on the chip.
- **Tragus sensor:** put a thin foam pad (2–3 mm) between the sensor and the skin, and let it sit on the patient for 5–10 min before tare.
- **Keep heat away:** mount each sensor 10+ cm from the ESP32, TB6600 and motor, out of direct sunlight and vents.
- **Power up early:** turn both nodes on several minutes before leveling so the boards reach a steady temperature.
- Tare only once `STEADY` shows in verbose mode (the button's waiting tare does this for you).

## Pole node commands
`set` list tunable settings · `set <name> <value>` change one until reset (e.g. `set cooldown 10`) · `t` tare · `T` force tare · `a` auto on/off · `u <mm>` / `d <mm>` jog · `s` stop · `c` clear faults · `z` zero position · `r` raw CSV · `p` details · `v` verbose status · `h` help

## Tuning on the bench
Times are in seconds. Changes made with `set` are lost on reset, so copy the values you settle on into the sketch.

| Setting | Default | What it does |
|---|---|---|
| `stable_win` | 60 | How long the trends are measured over. This is the shortest possible wait before an automatic tare. |
| `stable_dp` | 0.3 | Drift of the zero (cm/min) that counts as steady |
| `stable_rate` | 0.3 | Temperature trend (°C/min, both sensors) allowed at tare (backstop) |
| `tare_max` | 300 | A waiting tare is forced after this long |
| `drift` | 0.5 | Temperature change (°C) since the tare that blocks auto |
| `trigger` / `target` | 1.5 / 0.5 | Error (cm) that starts a correction / counts as aligned |
| `persist` | 5 | How long a misalignment must last before the pole moves |
| `cooldown` | 20 | Wait after a correction cycle before the next one can start |
| `soft_up` / `soft_down` / `cycle_max` | 200 | Travel limits in mm. On a bench lead screw, set these to its real travel. |

Events print how long things took: `TARE DONE after waiting N s` (with the zero and temperature trends at that moment) and `ALIGNED ... N s after misalignment was first seen`.

## Reading the Serial Monitor
Every 2 s the pole node prints one status line:

```
Auto ON  | WATCHING   | EVD level (+0.3 cm)  | pole   +0.0 mm | OK
```

| Column | Meaning |
|---|---|
| `Auto ON/OFF` | Whether automatic correction is on |
| State | `OFF`, `WATCHING` (aligned, monitoring), `CONFIRMING` (misaligned, waiting 5 s), `MOVING`, `SETTLING` (checking the move), `FAULT` |
| `EVD ...` | `level` within 0.5 cm, otherwise how far the EVD is `HIGH` or `LOW`; `--` before tare or without data |
| `pole` | Pole position from power-up |
| Last column | The one thing to know or fix right now, in words |

Lines starting with `**` are events (tare done, correction started, aligned, faults). The tragus node prints the same summary without the pole position. For debugging, `v` switches to the detailed line with every flag, `p` prints raw pressures/temperatures and settings, and `r` streams raw CSV data.

## Before any clinical use
- Measure each sensor's temperature coefficient on the bench and set `TCO_TRAGUS_PA_PER_K` / `TCO_EVD_PA_PER_K`. Skin warming of the tragus sensor is the largest error source (~4 cm per 0.5 Pa of drift).
- Agree `TRIGGER_CM`, `TARGET_CM`, `PERSIST_MS` and the soft travel limits with your clinical advisors, and set the soft limits to your lead screw's real travel.
- Fit physical limit switches and an e-stop.
