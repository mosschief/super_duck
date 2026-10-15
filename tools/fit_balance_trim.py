#!/usr/bin/env python3
"""
Measures a robot profile's balance lean vs. leg fold and fits `initial_pitch_trim` + `posture_coeffs`.

Holds each crouch command (-0.24..+0.24) for 80 s with the posture feedforward off, so the trim learner converges to
the true balance lean, records the *measured* leg fold, and fits lean = c0 + c1*fold + c2*fold^2. Paste c0 into
`initial_pitch_trim` and (c1, c2) into `posture_coeffs`. Re-run whenever mass moves (motors, battery, electronics).
Runs the holds in parallel.

Usage: python3 tools/fit_balance_trim.py gim6010
"""
import dataclasses
import pathlib
import sys
from multiprocessing import Pool

import mujoco
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import simulate_wheeled_duck as swd  # noqa: E402

CROUCHES = (-0.24, -0.12, 0.0, 0.12, 0.24)


def hold(args):
    robot, crouch = args
    prof = dataclasses.replace(swd.PROFILES[robot], posture_coeffs=(0.0, 0.0))
    m = mujoco.MjModel.from_xml_path(swd.profile_xml(prof))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    c = swd.ScaledWheeledDuckController(prof)
    c.crouch_delta = crouch
    folds = []
    for _ in range(int(80 / m.opt.timestep)):
        swd.step_robot(m, d, c)
        if d.time > 70:
            folds.append(swd.read_sensors(m, d)[6] - (c.nominal_hip_pitch + c.nominal_knee_pitch))
    return crouch, float(np.mean(folds)), c.pitch_trim, float(d.qvel[0])


if __name__ == "__main__":
    robot = sys.argv[1] if len(sys.argv) > 1 else swd.DEFAULT_ROBOT
    with Pool(len(CROUCHES)) as pool:
        rows = pool.map(hold, [(robot, cr) for cr in CROUCHES])
    for cr, f, trim, v in rows:
        print(f"crouch {cr:+.2f}: measured fold {f:+.4f}, balance lean {trim:+.5f} rad (residual v {v:+.4f} m/s)")
    f = np.array([r[1] for r in rows])
    p = np.array([r[2] for r in rows])
    c2, c1, c0 = np.polyfit(f, p, 2)
    err = np.degrees(np.polyval([c2, c1, c0], f) - p)
    print(f"\ninitial_pitch_trim={c0:.4f}, posture_coeffs=({c1:.4f}, {c2:.4f})   fit error max {np.abs(err).max():.3f} deg")
