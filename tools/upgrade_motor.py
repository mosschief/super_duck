#!/usr/bin/env python3
"""
Swaps leg joints from the GIM4305-10 to the SteadyWin GIM6010-8 in an MJCF model, in place.

For each joint: the joint, its position actuator and its motor canister geom switch to the `gim6010` classes, and
the extra motor mass (388 g - 150 g = 238 g) is added at the joint centre (the body origin, where the canister sits)
with the GIM6010-8's own inertia as a cylinder about the joint axis (Ø80 x 44 mm incl. driver, manual rev1.2).
Needs the `gim6010` / `gim6010_canister` default classes (present in open_duck_wheeled_v2_gim6010.xml).
Idempotent: joints already on gim6010 are skipped.

Usage: python3 tools/upgrade_motor.py open_duck_wheeled_v2_gim6010.xml left_hip_roll right_hip_roll
"""
import re
import sys

import numpy as np

ADD_MASS = 0.388 - 0.150
R_MOTOR, H_MOTOR = 0.040, 0.044


def upgrade(s: str, joint: str) -> str:
    jm = re.search(rf'<joint name="{joint}" [^>]*axis="([^"]+)"[^>]*class="(gim4305|gim6010)"', s)
    assert jm, f"joint {joint} not found"
    if jm.group(2) == "gim6010":
        print(f"  {joint}: already gim6010, skipped")
        return s
    axis = np.abs(np.array([float(v) for v in jm.group(1).split()]))
    s = s[:jm.start()] + jm.group(0).replace('class="gim4305"', 'class="gim6010"') + s[jm.end():]
    s, n = re.subn(rf'(<position name="{joint}" joint="{joint}" class=")gim4305(")', r"\1gim6010\2", s)
    assert n == 1, f"actuator for {joint} not found"
    s, n = re.subn(rf'<geom name="{joint}_gim4305" class="gim4305_canister"',
                   f'<geom name="{joint}_gim6010" class="gim6010_canister"', s)
    assert n == 1, f"canister geom for {joint} not found"

    # The joint's body inertial (the <inertial> right after this joint)
    pat = re.compile(rf'(<joint name="{joint}" [^>]*/>\s*)<inertial pos="([^"]+)" mass="([^"]+)" diaginertia="([^"]+)"/>')
    mt = pat.search(s)
    assert mt, f"inertial after {joint} not found"
    com = np.array([float(v) for v in mt.group(2).split()])
    m = float(mt.group(3))
    I = np.array([float(v) for v in mt.group(4).split()])
    M = m + ADD_MASS
    c = m * com / M                                  # motor mass sits at the body origin (joint centre)
    def shift(mass, d):
        return mass * np.array([d[1] ** 2 + d[2] ** 2, d[0] ** 2 + d[2] ** 2, d[0] ** 2 + d[1] ** 2])
    trans = ADD_MASS * (3 * R_MOTOR ** 2 + H_MOTOR ** 2) / 12
    own = np.where(axis > 0.5, 0.5 * ADD_MASS * R_MOTOR ** 2, trans)  # cylinder about the joint axis
    I_new = I + shift(m, com - c) + own + shift(ADD_MASS, -c)
    new = (f'<inertial pos="{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}" mass="{M:.3f}" '
           f'diaginertia="{I_new[0]:.6f} {I_new[1]:.6f} {I_new[2]:.6f}"/>')
    s = s[:mt.start()] + mt.group(1) + new + s[mt.end():]
    print(f"  {joint}: body {m:.3f} -> {M:.3f} kg")
    return s


if __name__ == "__main__":
    path, joints = sys.argv[1], sys.argv[2:]
    s = open(path).read()
    assert 'class="gim6010"' in s, "model has no gim6010 class"
    for j in joints:
        s = upgrade(s, j)
    open(path, "w").write(s)
