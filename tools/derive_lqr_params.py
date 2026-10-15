#!/usr/bin/env python3
"""
Derives the LQR's lumped body parameters from an MJCF at the `home_balanced` keyframe: mass, CoM height above the
wheel axle and pitch inertia about the CoM, all for the body *excluding* the wheels (the wheeled-inverted-pendulum
model treats the wheels separately). Paste the result into the robot's `RobotProfile.params`.

Usage: python3 tools/derive_lqr_params.py open_duck_wheeled_v2_gim6010.xml
"""
import sys

import mujoco
import numpy as np

WHEELS = ("foot_assembly", "foot_assembly_2")


def derive(path: str) -> dict:
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    wheel_ids = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in WHEELS}
    bodies = [b for b in range(1, m.nbody) if b not in wheel_ids]
    M = sum(m.body_mass[b] for b in bodies)
    com = sum(m.body_mass[b] * d.xipos[b] for b in bodies) / M
    Iyy = 0.0
    for b in bodies:
        R = d.ximat[b].reshape(3, 3)
        r = d.xipos[b] - com
        Iyy += (R @ np.diag(m.body_inertia[b]) @ R.T)[1, 1] + m.body_mass[b] * (r[0] ** 2 + r[2] ** 2)
    axle_z = np.mean([d.xpos[w][2] for w in wheel_ids])
    return dict(body_mass=round(float(M), 3), com_height=round(float(com[2] - axle_z), 4),
                body_pitch_inertia=round(float(Iyy), 4))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(p, derive(p))
