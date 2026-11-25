"""Speckle disc for turntable photogrammetry: black irregular blobs inlaid in a white disc.
Pattern is the bottom 0.4 mm -> print face-down (matte from the plate, only ~2 color changes), flip to use.
python make_platter.py -> platter_white.stl + platter_black.stl (load both as ONE object with 2 parts)"""
import subprocess
import numpy as np
import trimesh

D, T = 165.0, 1.6      # mm disc diameter (= platter top), total thickness
INLAY = 0.4            # mm pattern depth (2 layers @ 0.2)
GAP = 0.7              # mm min gap between blobs / to edge
OPENSCAD = r"C:\Program Files\OpenSCAD\openscad.exe"

rng = np.random.default_rng(11)


def blob(x, y, r):
    """Irregular closed outline: circle with random low-frequency wobble (more unique to feature matchers)."""
    a = np.linspace(0, 2 * np.pi, max(24, int(r * 12)), endpoint=False)
    k = np.arange(2, 7)
    amp = rng.uniform(0.02, 0.12, 5) * [1.0, 0.8, 0.6, 0.45, 0.35]  # soft wobble, no dominant lobe count
    wob = (amp[:, None] * np.cos(k[:, None] * a + rng.uniform(0, 2 * np.pi, 5)[:, None])).sum(0)
    rr = r * (1 + wob) / 1.05
    e, t = rng.uniform(0.55, 1.0), rng.uniform(0, np.pi)  # random stretch + rotation
    px, py = rr * np.cos(a), rr * np.sin(a) * e
    return np.c_[x + px * np.cos(t) - py * np.sin(t), y + px * np.sin(t) + py * np.cos(t)]


blobs = []  # (x, y, bounding r)
for d in np.concatenate([np.full(160, 8.0), rng.uniform(3.5, 5.5, 700), np.full(3000, 2.2)]):
    r = d / 2
    for _ in range(150):  # dart throwing, no overlaps
        x, y = rng.uniform(-D / 2, D / 2, 2)
        if np.hypot(x, y) + r + GAP > D / 2:
            continue
        if all(np.hypot(x - a, y - b) >= r + c + GAP for a, b, c in blobs):
            blobs.append((x, y, r))
            break
polys = [blob(*b) for b in blobs]
area = sum(0.5 * abs(np.dot(p[:, 0], np.roll(p[:, 1], 1)) - np.dot(p[:, 1], np.roll(p[:, 0], 1))) for p in polys)
print(f"{len(polys)} blobs, {area / (np.pi * D * D / 4):.0%} black")

shapes = "\n".join("polygon(" + str(np.round(p, 3).tolist()) + ");" for p in polys)
jobs = {
    "platter_white_inlay": f"linear_extrude(height={INLAY}) difference(){{ circle(d={D}, $fn=360); {shapes} }}",
    "platter_white_top": f"translate([0,0,{INLAY}]) cylinder(d={D}, h={T - INLAY}, $fn=360);",
    "platter_black": f"linear_extrude(height={INLAY}) union(){{ {shapes} }}",
}
for name, body in jobs.items():
    open(name + ".scad", "w").write(body + "\n")
    subprocess.run([OPENSCAD, "-o", name + ".stl", name + ".scad"], check=True, capture_output=True)
# white = patterned bottom layer + solid top, touching (not overlapping) -> one STL
# white stays two touching parts (inlay layer + solid top): each watertight, no coincident faces
print("platter_white_inlay.stl, platter_white_top.stl, platter_black.stl")
