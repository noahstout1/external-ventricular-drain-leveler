"""NEMA 17 motor clamp for an IV pole -> two printable STL parts.

Part A: half collar + horizontal motor plate (with side ribs).
Part B: other half collar with M5 nut traps.
The halves are pulled together around the pole by 4x M5 bolts.

Usage:  python make_clamp.py [pole_radius_mm] [out_name]
Needs:  pip install manifold3d numpy
"""
import math
import sys

import numpy as np
from manifold3d import CrossSection, Manifold

# ----------------------------- parameters (mm) -----------------------------
POLE_R      = float(sys.argv[1]) if len(sys.argv) > 1 else 29.0
NAME        = sys.argv[2] if len(sys.argv) > 2 else "motor_clamp"
FIT_CLEAR   = 0.2     # added to the bore radius so it slides on before tightening
SPLIT_GAP   = 1.0     # each half stops this far from the split plane (2 mm squeeze room)
WALL        = 6.0     # collar wall thickness
H           = 40.0    # collar height (taller = more grip against slipping)
EAR_T       = 10.0    # thickness of each bolt ear, measured from the split plane
BOLT_D      = 5.5     # M5 clearance
BOLT_Z      = (9.0, 25.0)
NUT_AF      = 8.4     # M5 nut across-flats + clearance
NUT_DEPTH   = 4.5

MOTOR_W     = 42.3    # NEMA 17 body
MOTOR_GAP   = 2.0     # motor body to collar clearance
HOLE_SPACE  = 31.0    # NEMA 17 bolt pattern
M3_D        = 3.4
BOSS_D      = 23.0    # 22 mm pilot boss + clearance (shaft passes through)
PLATE_T     = 6.0
PLATE_HALF  = 28.0    # plate half-width (Y); ribs sit outside the motor body
RIB_T       = 5.0
SEG         = 160

bore = POLE_R + FIT_CLEAR
ro = bore + WALL
bolt_y = ro + 3.0
ear_out = ro + 9.0
motor_x = ro + MOTOR_GAP + MOTOR_W / 2      # lead-screw axis distance from pole axis
plate_end = motor_x + MOTOR_W / 2 + 4.0


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, circular_segments=seg).translate([x, y, z0])


def cyl_x(r, x0, x1, y, z, seg=48):
    # cylinder along +Z rotated onto +X:  (x, y, z) -> (z, y, -x)
    return (Manifold.cylinder(x1 - x0, r, circular_segments=seg)
            .rotate([0, 90, 0]).translate([x0, y, z]))


def rib(x0, x1, z_top, z_bot, y0, t):
    # triangle in the XZ plane, thickness t in +Y starting at y0
    tri = CrossSection([[(x0, z_top), (x0, z_bot), (x1, z_top)]])
    return tri.extrude(t).rotate([90, 0, 0]).translate([0, y0 + t, 0])


# ------------------------------- collar --------------------------------------
collar = cyl_z(ro, 0, H)
for s in (1, -1):
    y0, y1 = sorted((s * (bore + 1), s * ear_out))
    collar = collar + box(-EAR_T, EAR_T, y0, y1, 0, H)

holes = Manifold()
for s in (1, -1):
    for z in BOLT_Z:
        holes = holes + cyl_x(BOLT_D / 2, -EAR_T - 1, EAR_T + 1, s * bolt_y, z)

# --------------------------- motor plate (part A) ----------------------------
plate = box(0, plate_end, -PLATE_HALF, PLATE_HALF, H - PLATE_T, H)
x_rib0 = max(math.sqrt(max(ro**2 - PLATE_HALF**2, 0.0)) - 1.0, EAR_T + 3.0)
for y0 in (MOTOR_W / 2 + 1.85, -PLATE_HALF):
    plate = plate + rib(x_rib0, plate_end, H - PLATE_T + 0.01, 4.0, y0, RIB_T)

plate_cuts = cyl_z(BOSS_D / 2, H - PLATE_T - 1, H + 1, motor_x, 0)
for dx in (-HOLE_SPACE / 2, HOLE_SPACE / 2):
    for dy in (-HOLE_SPACE / 2, HOLE_SPACE / 2):
        plate_cuts = plate_cuts + cyl_z(M3_D / 2, H - PLATE_T - 1, H + 1,
                                        motor_x + dx, dy, seg=32)

bore_cut = cyl_z(bore, -1, H + 1)

big = 500
part_a = ((collar + plate) - bore_cut - holes - plate_cuts) ^ box(SPLIT_GAP, big, -big, big, -1, H + 1)
part_b = (collar - bore_cut - holes) ^ box(-big, -SPLIT_GAP, -big, big, -1, H + 1)

# hex nut traps on the outside face of part B's ears
nut_r = NUT_AF / 2 / math.cos(math.pi / 6)
for s in (1, -1):
    for z in BOLT_Z:
        part_b = part_b - cyl_x(nut_r, -EAR_T - 1, -EAR_T + NUT_DEPTH, s * bolt_y, z, seg=6)


# ------------------------------- export --------------------------------------
def write_stl(man, path):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype(np.float32)
    f = np.asarray(mesh.tri_verts)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as fh:
        fh.write(b"EVD motor clamp".ljust(80, b" "))
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


# Print orientations: both upside down, so the flat collar top (and part A's
# plate) sit on the bed and nothing needs support.
part_a_print = part_a.rotate([180, 0, 0])
part_b_print = part_b.rotate([180, 0, 0])
for label, man, out in (("A", part_a, part_a_print), ("B", part_b, part_b_print)):
    assert man.status().name == "NoError" and not man.is_empty()
    lo = np.array(out.bounding_box()[:3])
    out = out.translate([-lo[0], -lo[1], -lo[2]])          # sit on the bed at origin
    write_stl(out, f"{NAME}_part_{label}.stl")
    bb = out.bounding_box()
    print(f"part {label}: genus {man.genus()}, volume {man.volume() / 1000:.1f} cm^3, "
          f"size {bb[3]:.1f} x {bb[4]:.1f} x {bb[5]:.1f} mm, tris {man.num_tri()}")
print(f"pole bore {2 * bore:.1f} mm, lead-screw axis {motor_x:.1f} mm from pole axis")
