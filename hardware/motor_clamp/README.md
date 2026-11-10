# NEMA 17 motor clamp for the IV pole

Two-piece split collar that clamps onto the IV pole. Part A holds the NEMA 17 so the lead screw runs **parallel to the pole**; part B is the other half of the collar.

| File | Pole size |
|---|---|
| `clamp_pole_58mm_dia_part_A.stl` / `_part_B.stl` | 29 mm **radius** (58 mm diameter) |
| `clamp_pole_29mm_dia_part_A.stl` / `_part_B.stl` | 29 mm **diameter** (14.5 mm radius) |

Print one A and one B for your pole size. Other sizes: `python make_clamp.py <pole_radius_mm> <name>` (needs `pip install manifold3d numpy`).

## Hardware
- 4x M5 bolts, 25 mm long, + 4x M5 nuts (nuts sit in the hex pockets on part B)
- 4x M3 x 10 mm screws (plate to motor)
- Optional: a strip of rubber (old bike inner tube) inside the collar for extra grip

## Printing
- PETG or ABS preferred (PLA creeps under constant clamp load). 4 walls, 40%+ infill.
- STLs are already in print orientation (both upside down, flat collar top on the bed). No supports needed.

## Assembly
1. Bolt the motor to the underside of the plate with the M3 screws, shaft pointing up through the center hole. (It can also sit on top with the shaft pointing down.) Turn the motor so its cable faces away from the pole.
2. Put the two halves around the pole, drop the nuts into part B's pockets, and tighten the four M5 bolts evenly until the clamp can't slide. The 2 mm gap between halves is the squeeze room. Don't close it completely.
3. The lead-screw axis ends up 58 mm (large pole) / 44 mm (small pole) from the pole's center. Mount the nut bracket at that offset.
