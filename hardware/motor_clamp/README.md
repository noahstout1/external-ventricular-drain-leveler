# NEMA 17 motor clamp for the IV pole

Two-piece split collar that clamps onto the IV pole. Part A holds the NEMA 17 so the lead screw runs **parallel to the pole**; part B carries a sleeve that the **TB6600 driver** slides into, on the opposite side of the pole.

| File | Pole size |
|---|---|
| `clamp_pole_58mm_dia_part_A.stl` / `_part_B.stl` | 29 mm **radius** (58 mm diameter) |
| `clamp_pole_29mm_dia_part_A.stl` / `_part_B.stl` | 29 mm **diameter** (14.5 mm radius) |

Print one A and one B for your pole size. Other sizes: `python make_clamp.py <pole_radius_mm> <name>` (needs `pip install manifold3d numpy`).

## Hardware
- TB6600 sleeve is sized for a **96 x 71 x 37 mm** driver (+0.5 mm per side). Measure yours; if it differs, change `DRV_L`, `DRV_W`, `DRV_T` in `make_clamp.py` and rerun.
- 4x M5 bolts, 25 mm long, + 4x M5 nuts (nuts sit in the hex pockets on part B)
- 4x M3 x 10 mm screws (plate to motor)
- Optional: a strip of rubber (old bike inner tube) inside the collar for extra grip

## Printing
- PETG or ABS preferred (PLA creeps under constant clamp load). 4 walls, 40%+ infill.
- STLs are already in print orientation (both upside down, flat collar top on the bed). No supports needed; the small corner ledges at the top of part B's sleeve print as short overhangs.

## Assembly
1. Bolt the motor to the underside of the plate with the M3 screws, shaft pointing up through the center hole. (It can also sit on top with the shaft pointing down.) Turn the motor so its cable faces away from the pole.
2. Put the two halves around the pole, drop the nuts into part B's pockets, and tighten the four M5 bolts evenly until the clamp can't slide. The 2 mm gap between halves is the squeeze room. Don't close it completely.
3. Slide the TB6600 into part B's sleeve from the top until it rests on the corner ledges. Both ends stay open for the screw terminals; the side windows give access to the DIP switches and airflow to the heatsink. For extra security, loop a zip tie through the side windows around the driver.
4. The lead-screw axis ends up 58 mm (large pole) / 44 mm (small pole) from the pole's center. Mount the nut bracket at that offset.
