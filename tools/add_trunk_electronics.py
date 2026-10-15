#!/usr/bin/env python3
"""
Adds the AI HAT+ (26 TOPS) and the Adafruit Feather M4 CAN to the trunk of an MJCF model, in place.

Masses are folded into the trunk's explicit <inertial> (point masses + parallel-axis shift, their own inertia is
negligible) and visual-only boxes are added so they show in the viewer. Idempotent: skips a model that already has
them.

  AI HAT+ 26 TOPS (Hailo-8): 65 x 56.5 x 5.5 mm board, 34-35 g (Waveshare / Probots listings) + ~6 g header and
      standoffs = 41 g, stacked on 16 mm standoffs above the Pi 5 + active cooler.
  Feather M4 CAN Express (ATSAME51): 50.8 x 22.8 x 7 mm, 5.5 g + ~2.5 g CAN/IMU wiring = 8 g, next to the IMU.

Usage: python3 tools/add_trunk_electronics.py open_duck_wheeled_v2.xml open_duck_wheeled_v2_gim6010.xml
"""
import re
import sys

import numpy as np

PARTS = [  # name, mass (kg), position in trunk frame (m), half-size (m), material
    ("ai_hat_plus_26tops", 0.041, (-0.006, 0.0, 0.042), (0.0325, 0.0283, 0.0028), "rpi_green"),
    ("feather_m4_can", 0.008, (-0.095, 0.0, 0.060), (0.0254, 0.0114, 0.0035), "gim4305_housing"),
]

INERTIAL = re.compile(r'(<body name="trunk_assembly"[^>]*>(?:(?!<body).)*?)'
                      r'<inertial pos="([^"]+)" mass="([^"]+)" diaginertia="([^"]+)"/>', re.S)


def add(path: str) -> None:
    s = open(path).read()
    if PARTS[0][0] in s:
        print(f"{path}: already has trunk electronics, skipped")
        return
    mt = INERTIAL.search(s)
    assert mt, f"{path}: trunk_assembly inertial not found"
    com = np.array([float(v) for v in mt.group(2).split()])
    m = float(mt.group(3))
    I = np.array([float(v) for v in mt.group(4).split()])

    masses = [m] + [p[1] for p in PARTS]
    points = [com] + [np.array(p[2]) for p in PARTS]
    M = sum(masses)
    c = sum(mi * pi for mi, pi in zip(masses, points)) / M

    def shift(mass, d):  # parallel axis, diagonal terms
        return mass * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])

    I_new = I + sum(shift(mi, pi - c) for mi, pi in zip(masses, points))
    new_inertial = (f'<inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{M:.3f}" '
                    f'diaginertia="{I_new[0]:.5f} {I_new[1]:.5f} {I_new[2]:.5f}"/>')
    geoms = "".join(
        f'\n        <!-- {name}: {mass * 1000:.0f} g (folded into the trunk inertial above) -->'
        f'\n        <geom name="{name}" class="visual" type="box" size="{h[0]} {h[1]} {h[2]}" '
        f'pos="{p[0]} {p[1]} {p[2]}" material="{mat}"/>'
        for name, mass, p, h, mat in PARTS)
    s = s[:mt.start()] + mt.group(1) + new_inertial + geoms + s[mt.end():]
    open(path, "w").write(s)
    print(f"{path}: trunk {m:.3f} -> {M:.3f} kg, com {com.round(4).tolist()} -> {c.round(4).tolist()}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        add(p)
