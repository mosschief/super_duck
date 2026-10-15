#!/usr/bin/env python3
"""
Adds the AI HAT+ (26 TOPS) and the PJRC Teensy 4.0 + Dual CAN Transceiver carrier (or Feather M4 CAN, both ~8 g)
to the trunk of an MJCF model, in place.

Masses are folded into the trunk's explicit <inertial> (point masses + parallel-axis shift, their own inertia is
negligible) and visual-only boxes are added so they show in the viewer. Idempotent: skips a model that already has
them.

  AI HAT+ 26 TOPS (Hailo-8): 65 x 56.5 x 5.5 mm board, 34-35 g (Waveshare / Probots listings) + ~6 g header and
      standoffs = 41 g, stacked on 16 mm standoffs above the Pi 5 + active cooler.
  PJRC Teensy 4.0 (600 MHz Cortex-M7, 35 x 18 mm) + Dual 3.3V CAN Transceiver Carrier + ICM-42688-P SPI IMU: ~8 g
      total (same mass envelope as the Feather M4 CAN Express single-bus alternative), mounted next to the IMU.

Usage: python3 tools/add_trunk_electronics.py open_duck_wheeled_v2.xml open_duck_wheeled_v2_gim6010.xml
"""
import re
import sys

import numpy as np

PARTS = [  # name, mass (kg), position in trunk frame (m), half-size (m), material
    ("ai_hat_plus_26tops", 0.041, (-0.006, 0.0, 0.042), (0.0325, 0.0283, 0.0028), "rpi_green"),
    ("teensy_40_dual_can", 0.008, (-0.095, 0.0, 0.060), (0.0210, 0.0110, 0.0035), "gim4305_housing"),
]

def inertial_re(body: str):
    """Matches a body's opening tag up to its explicit <inertial> (before any child body)."""
    return re.compile(rf'(<body name="{body}"[^>]*>(?:(?!<body).)*?)'
                      r'<inertial pos="([^"]+)" mass="([^"]+)" diaginertia="([^"]+)"/>', re.S)


def fold(s: str, body: str, parts, indent: str = "        ") -> str:
    """Folds point-mass `parts` (name, mass, pos, half-size, material) into `body`'s <inertial> and adds a visual box
    for each part whose half-size is not None. Returns the new MJCF text."""
    mt = inertial_re(body).search(s)
    assert mt, f"{body} inertial not found"
    com = np.array([float(v) for v in mt.group(2).split()])
    m = float(mt.group(3))
    I = np.array([float(v) for v in mt.group(4).split()])

    masses = [m] + [p[1] for p in parts]
    points = [com] + [np.array(p[2]) for p in parts]
    M = sum(masses)
    c = sum(mi * pi for mi, pi in zip(masses, points)) / M

    def shift(mass, d):  # parallel axis, diagonal terms
        return mass * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])

    I_new = I + sum(shift(mi, pi - c) for mi, pi in zip(masses, points))
    new_inertial = (f'<inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{M:.3f}" '
                    f'diaginertia="{I_new[0]:.5f} {I_new[1]:.5f} {I_new[2]:.5f}"/>')
    geoms = "".join(
        f'\n{indent}<!-- {name}: {mass * 1000:.0f} g (folded into the {body} inertial above) -->'
        + (f'\n{indent}<geom name="{name}" class="visual" type="box" size="{h[0]} {h[1]} {h[2]}" '
           f'pos="{p[0]} {p[1]} {p[2]}" material="{mat}"/>' if h is not None else "")
        for name, mass, p, h, mat in parts)
    print(f"  {body} {m:.3f} -> {M:.3f} kg, com {com.round(4).tolist()} -> {c.round(4).tolist()}")
    return s[:mt.start()] + mt.group(1) + new_inertial + geoms + s[mt.end():]


def add(path: str) -> None:
    s = open(path).read()
    if PARTS[0][0] in s:
        print(f"{path}: already has trunk electronics, skipped")
        return
    print(path)
    open(path, "w").write(fold(s, "trunk_assembly", PARTS))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        add(p)
