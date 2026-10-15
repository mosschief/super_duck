#!/usr/bin/env python3
"""
Sets the battery pack in the trunk of an MJCF model, in place: resizes the `battery_pack_6s` visual box and folds the
pack's extra mass (relative to the baseline the original 1.050 kg trunk was budgeted with) into the trunk inertial.

The original model doesn't say how much of the trunk mass is battery; this assumes it was a 6S1P 21700 pack
(6 x ~70 g cells + BMS/wiring ~ 0.48 kg), the size the power budget first pointed to.

  6s1p: 6 x 21700, ~0.48 kg, ~90-108 Wh   (baseline, no mass change)
  6s2p: 12 x 21700, ~0.90 kg, ~180-216 Wh (+0.42 kg), 73 x 89 x 68 mm (4 x 3 Molicel P45B cells lying along x,
        Ø21.55 x 70.15 max, +1.5 mm per side for strip and wrap: cad/purchased_parts.json), as low as fits under the Pi

Idempotent: records the applied pack in a comment and refuses to apply a second time.

Usage: python3 tools/battery_pack.py open_duck_wheeled_v2_gim6010.xml 6s2p
"""
import re
import sys

import numpy as np

PACKS = {  # extra mass vs. baseline (kg), box half-size (m), box centre in trunk frame (m)
    "6s2p": (0.42, (0.0365, 0.0445, 0.034), (-0.026, 0.0, -0.026)),
}
MARK = "battery pack set by tools/battery_pack.py"


def apply(path: str, pack: str) -> None:
    s = open(path).read()
    if MARK in s:
        print(f"{path}: battery already set ({re.search(MARK + r': (\S+)', s).group(1)}), skipped")
        return
    dm, half, pos = PACKS[pack]
    s, n = re.subn(r'<geom name="battery_pack_6s" class="visual" type="box" size="[^"]+" pos="[^"]+"',
                   f'<geom name="battery_pack_6s" class="visual" type="box" size="{half[0]} {half[1]} {half[2]}" '
                   f'pos="{pos[0]} {pos[1]} {pos[2]}"', s)
    assert n == 1, "battery_pack_6s geom not found"
    pat = re.compile(r'(<body name="trunk_assembly"[^>]*>(?:(?!<body).)*?)'
                     r'<inertial pos="([^"]+)" mass="([^"]+)" diaginertia="([^"]+)"/>', re.S)
    mt = pat.search(s)
    com = np.array([float(v) for v in mt.group(2).split()])
    m = float(mt.group(3))
    I = np.array([float(v) for v in mt.group(4).split()])
    p = np.array(pos)
    M = m + dm
    c = (m * com + dm * p) / M
    def shift(mass, d):
        return mass * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    box = dm / 3 * np.array([half[1] ** 2 + half[2] ** 2, half[0] ** 2 + half[2] ** 2, half[0] ** 2 + half[1] ** 2])
    I_new = I + shift(m, com - c) + box + shift(dm, p - c)
    new = (f'<inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{M:.3f}" '
           f'diaginertia="{I_new[0]:.5f} {I_new[1]:.5f} {I_new[2]:.5f}"/>'
           f'\n        <!-- {MARK}: {pack} (+{dm * 1000:.0f} g vs. the 6S1P baseline, folded into the inertial above) -->')
    s = s[:mt.start()] + mt.group(1) + new + s[mt.end():]
    open(path, "w").write(s)
    print(f"{path}: {pack}, trunk {m:.3f} -> {M:.3f} kg, com {com.round(4).tolist()} -> {c.round(4).tolist()}")


if __name__ == "__main__":
    apply(sys.argv[1], sys.argv[2])
