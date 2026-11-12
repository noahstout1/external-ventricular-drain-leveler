# NEMA 17 motor clamp for the IV pole

Two-piece split collar that clamps onto the IV pole and holds the NEMA 17 so the lead screw runs **exactly parallel to the pole**.

It sits on the **29 mm** section of the pole. The nut clamp (`../nut_clamp`) sits on the **19 mm** section. The two sections share one center line, so the lead screw stays parallel only if it is the same distance from the center at both clamps:

| | Motor clamp | Nut clamp |
|---|---|---|
| Pole section | 29 mm | 19 mm |
| Bore | 29.4 mm (+0.2 mm per side, 3 mm squeeze gap) | 19.0 mm |
| Lead screw from pole center | **36.0 mm** | **36.0 mm** |

Rotate the clamps so the motor shaft sits directly under the nut clamp's lead-screw hole (`../system_preview.png`).

Print one each of `clamp_pole_29mm_dia_part_A.stl` and `clamp_pole_29mm_dia_part_B.stl`. For a different pole: `python make_clamp.py <pole_radius_mm> <name>` (needs `pip install manifold3d numpy`); the script refuses sizes where the collar would block the lead screw.

## Hardware
- 4x M5 bolts, 20–25 mm long, + 4x M5 hex nuts. Bolt heads go on **part A**; the nuts drop into the hex pockets on **part B**.
- 4x M3 x 10 mm screws (plate to motor), inserted from the collar side of the plate.
- 5 mm-to-8 mm shaft coupler (up to ~20 mm across) for the TR8 lead screw.
- Optional: a strip of rubber (old bike inner tube) inside the collar for extra grip.

## Printing
- PETG or ABS preferred (PLA creeps under constant clamp load). 4 walls, 40%+ infill.
- STLs are already in print orientation (both upside down, collar end and motor plate on the bed). No supports needed. The bolt holes are teardrop-shaped and the nut pockets point up so they print round instead of sagging.

## Assembly
1. Set the motor on the **outside** face of the plate (the face flush with the end of the collar), shaft pointing through the center hole toward the collar side. Screw it on with the 4 M3 screws from the collar side. Turn it so the cable faces away from the pole.
2. Put the two halves around the pole, drop the nuts into part B's pockets, push each bolt in from part A's side, and tighten the four M5 bolts evenly (criss-cross) until the clamp can't slide. The 3 mm gap between halves is the squeeze room; don't close it completely. If the gap closes before the clamp is tight, the pole is smaller than the bore: wrap it with a strip of rubber or regenerate with your measured radius.
3. Fit the coupler and lead screw. The screw runs alongside the collar, 36 mm from the pole's center. With the nut clamp above it, mount the clamp plate-down so the motor hangs below and the screw goes up (as in `../system_preview.png`).
4. Line the screw up with the nut clamp's lead-screw hole and check it with a small level or square against the pole.
