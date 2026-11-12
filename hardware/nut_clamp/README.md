# Nut clamp (on the 19 mm section of the IV pole)

Revised from the team's `Clamp_A_ElectroCookie_6Hole_ScrewMount` / `Clamp_B_with_MotorDriver_Tray` (repo root) by `modify_nut_clamp.py`:

| | Original | Revised |
|---|---|---|
| Pole bore | 22.0 mm | **19.0 mm** (snug on the 19 mm section) |
| M5 holes, part A | 5.5 mm + nut slots | **4.2 mm pilots**: M5 screws self-thread into A |
| M5 holes, part B | 5.5 mm | 5.5 mm clearance (screw heads go on B's outside) |
| Protoboard holder | on part A | **upside down under part B's TB6600 tray** |
| Lead-screw hole | 10.6 mm, 36 mm from pole axis | unchanged (matches the motor clamp) |

Why only one half is threaded: if the screw threaded into both halves, it would lock into the first one and could never pull the halves together. Clearance on B + threads in A is what lets the screw clamp.

## Files
- `Nut_Clamp_A_Threaded.stl`: plain half, 4.2 mm pilot holes.
- `Nut_Clamp_B_DriverTray_BoardUnder.stl`: half with the TB6600 tray on top and the protoboard holder hanging underneath (its six screw bosses face down, so the protoboard screws on upside down). Four 8 mm holes in the holder line up with the tray's TB6600 mounting holes so a screwdriver can still reach them.
- Both are in print orientation (split face on the bed). The original STEP files in the repo root were **not** updated.

## Hardware
- 4x **M5 x 30 mm** screws, inserted from part B's outside and threaded into part A (about 11 mm of thread).
- Protoboard: **4x M2** screws in the corner bosses (1.6 mm pilot holes) and **2x M3** screws in the two middle bosses (2.5 mm pilot holes). Both self-thread into the plastic; the holes are about 4.5 mm deep, so with a 1.6 mm board use **M2 x 6** and **M3 x 6** (8 mm is too long).
- The TB6600 uses the same screws as before.

## Assembly
1. Screw the TB6600 into the tray first (through the access holes in the holder), then screw the protoboard onto the holder upside down (M2 in the corners, M3 in the middle). Drive them in slowly the first time; they cut their own thread.
2. Put both halves around the 19 mm section with the lead-screw half-holes lined up, and drive the four M5 screws in evenly. The first time, go slowly; the screw cuts its own thread.
3. Rotate the nut clamp on the pole so its lead-screw hole sits directly above the motor clamp's motor shaft (36 mm from the pole's center at both clamps), and fit the TR8 nut.
