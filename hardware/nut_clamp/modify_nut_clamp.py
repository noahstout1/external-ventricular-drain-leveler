"""Build the revised nut clamp from the team's original STLs in the repo root.

Changes to Clamp_A_ElectroCookie_6Hole_ScrewMount / Clamp_B_with_MotorDriver_Tray:
  * Pole bore 22.0 -> 19.0 mm (the IV pole is 19 mm where this clamp sits).
  * M5 holes: part A gets 4.2 mm pilots so M5 screws self-thread into it;
    part B keeps 5.5 mm clearance (screw heads go on B's outside).
  * The breadboard (ElectroCookie) holder moves off part A and hangs, upside
    down, under part B's TB6600 tray. Screwdriver holes line up with the tray's
    four mounting holes.
Lead-screw hole (10.6 mm, 36 mm from the pole axis) and bolt pattern unchanged.

Usage:  python modify_nut_clamp.py      (needs: pip install manifold3d numpy)
"""
import os
import struct

import numpy as np
from manifold3d import Manifold, Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_A = os.path.join(ROOT, "Clamp_A_ElectroCookie_6Hole_ScrewMount.stl")
SRC_B = os.path.join(ROOT, "Clamp_B_with_MotorDriver_Tray.stl")

POLE_D       = 19.0
OLD_BORE_R   = 11.02
BLOCK_HALF_Z = 21.0        # block spans z = -21..21 (pole direction)
BLOCK_DEPTH  = 23.0        # each half is 23 mm deep from the split face
BOLTS        = [(-17.0, -12.5), (-17.0, 12.5), (53.0, -12.5), (53.0, 12.5)]  # (x, z)
OLD_HOLE_R   = 2.75
PILOT_D      = 4.2         # M5 self-tapping pilot in PETG/PLA
THREAD_DEPTH = 11.9        # plastic between split face and the old nut slot
TRAY_FLOOR_BOTTOM = -31.0
BOARD_TOP    = -18.9       # top of the breadboard holder's walls on the original part A
TRAY_LOWEST  = -43.0       # lowest point of part B (gussets near the block)
TRAY_HOLES   = [(-12.6, 35.8), (48.7, 35.8), (-12.6, 76.2), (48.7, 76.2)]
ACCESS_R     = 4.0


def load(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    rec = np.frombuffer(b[84:84 + n * 50], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tri = rec["v"].reshape(-1, 3)
    verts, idx = np.unique(tri.round(4), axis=0, return_inverse=True)
    return Manifold(Mesh(vert_properties=verts.astype(np.float32),
                         tri_verts=idx.reshape(-1, 3).astype(np.uint32)))


def write_stl(man, path):
    mesh = man.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3].astype(np.float32)
    f = np.asarray(mesh.tri_verts)
    tri = v[f]
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = nrm, tri
    with open(path, "wb") as fh:
        fh.write(b"EVD nut clamp".ljust(80, b" "))
        fh.write(np.uint32(len(f)).tobytes())
        fh.write(rec.tobytes())


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=128):
    return Manifold.cylinder(z1 - z0, r, circular_segments=seg).translate([x, y, z0])


def cyl_y(r, y0, y1, x, z, seg=64):
    # cylinder along +Z rotated onto +Y: rotate X by -90 maps (x, y, z) -> (x, z, -y)
    return Manifold.cylinder(y1 - y0, r, circular_segments=seg).rotate([-90, 0, 0]).translate([x, y0, z])


def rebore(part, side):
    """Fill the old 22 mm half-bore and cut the 19 mm one. side=+1 for B (y>=0), -1 for A."""
    y0, y1 = (0.0, OLD_BORE_R + 1) if side > 0 else (-OLD_BORE_R - 1, 0.0)
    fill = cyl_z(OLD_BORE_R + 0.05, -BLOCK_HALF_Z, BLOCK_HALF_Z) ^ box(-20, 20, y0, y1, -BLOCK_HALF_Z, BLOCK_HALF_Z)
    return (part + fill) - cyl_z(POLE_D / 2, -200, 200)


# Assembly frame (= part B's file frame): pole axis = Z through the origin, split
# face = plane y = 0, part B on +y, part A on -y, lead screw at (36, 0).
B0 = load(SRC_B)
A0 = load(SRC_A).rotate([90, 0, 0])          # A's file has the pole along Y

# ---------------- part A: plain threaded half ----------------
# holder only: floor + standoffs + walls (z -31..-19), not its gussets or the block face
board = A0 ^ box(-100, 100, -200, -BLOCK_DEPTH - 0.05, TRAY_FLOOR_BOTTOM, BOARD_TOP)
A = A0 ^ box(-100, 100, -BLOCK_DEPTH, 1, -BLOCK_HALF_Z, BLOCK_HALF_Z)
A = rebore(A, -1)
for x, z in BOLTS:
    A = A + cyl_y(OLD_HOLE_R + 0.05, -THREAD_DEPTH, 0.0, x, z)           # fill the 5.5 mm hole...
    A = A - cyl_y(PILOT_D / 2, -THREAD_DEPTH - 0.5, 0.5, x, z)           # ...and cut the pilot

# ---------------- part B: tray + board holder underneath ----------------
B = rebore(B0, +1)
# flip the holder upside down onto the +y side: (x, y, z) -> (x, -y, -z),
# then drop it so its (now top) floor face meets part B's lowest point
flipped = board.rotate([180, 0, 0])
fb = flipped.bounding_box()
holder = flipped.translate([0, 0, TRAY_LOWEST - fb[5]])
hb = holder.bounding_box()
# side walls tie the holder to the tray floor along its full length
for x0, x1 in ((hb[0], hb[0] + 4.0), (hb[3] - 4.0, hb[3])):
    holder = holder + box(x0, x1, hb[1], hb[4], TRAY_LOWEST - 0.3, TRAY_FLOOR_BOTTOM + 0.3)
# screwdriver access to the TB6600 tray holes through the holder
access = Manifold()
for x, y in TRAY_HOLES:
    access = access + cyl_z(ACCESS_R, hb[2] - 1, TRAY_FLOOR_BOTTOM - 0.01, x, y, seg=48)
B = B + (holder - access)

# ---------------- checks + export (print orientation: split face on the bed) ----------------
if __name__ == "__main__":
    for name, part in (("A", A), ("B", B)):
        assert part.status().name == "NoError" and len(part.decompose()) == 1, name
    out = {
        "Nut_Clamp_A_Threaded.stl": A.rotate([-90, 0, 0]),                   # -y -> +z
        "Nut_Clamp_B_DriverTray_BoardUnder.stl": B.rotate([90, 0, 0]),      # +y -> +z
    }
    for fname, part in out.items():
        lo = part.bounding_box()[:3]
        part = part.translate([-lo[0], -lo[1], -lo[2]])
        bb = part.bounding_box()
        write_stl(part, os.path.join(HERE, fname))
        print(f"{fname}: {bb[3]:.1f} x {bb[4]:.1f} x {bb[5]:.1f} mm, {part.volume() / 1000:.1f} cm^3")
