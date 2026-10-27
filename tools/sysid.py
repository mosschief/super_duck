#!/usr/bin/env python3
"""
System identification for sim-to-real: fits a joint's inertia, viscous damping, Coulomb friction and (for a leg
joint) gravity load from a bench log, so the MJCF's `armature`, `damping` and `frictionloss` can be set from the
real motors.

Bench test (robot on a stand, wheels off the ground, every other joint clamped): drive one joint with a torque chirp (GIM MIT mode, kp = kd = 0,
the torque command as feed-forward) and log time, the motor's reported torque (N*m), position (rad) and velocity
(rad/s) at >= 500 Hz. Keep the chirp small enough that friction is a visible share of the torque (wheel: ~0.08 N*m
peak, knee: ~0.4 N*m): at large amplitude the viscous term swamps it and the friction estimate goes off. Model, linear in the unknowns:
    tau = J*qdd + b*qd + fc*sign(qd) + g1*sin(q) + g2*cos(q)
J is the joint's total inertia about its axis: the reflected rotor (MJCF `armature`) plus the links it carries.
The fit integrates the model over 50 ms windows (momentum form), so no acceleration is differentiated from the
noisy velocity. Stiction is not modelled.

Usage:
  python3 tools/sysid.py fit log.csv [--gravity]   # columns: t, tau, q, qd (header line allowed)
  python3 tools/sysid.py selftest                  # synthetic logs from the MJCF: must recover its parameters
"""
import argparse
import math
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def fit(t, tau, q, qd, gravity=False, window=0.05):
    """Least-squares fit of the model integrated over short windows (momentum form: no differentiated velocity, so
    encoder / velocity noise doesn't bias it). Returns dict(J, b, fc[, g1, g2], rms_residual)."""
    def cum(x):
        return np.concatenate([[0.0], np.cumsum(0.5 * (x[1:] + x[:-1]) * np.diff(t))])
    dt = np.median(np.diff(t))
    k = max(3, int(round(0.005 / dt)) | 1)                   # ~5 ms smoothing of the window-end velocities
    v = np.convolve(qd, np.ones(k) / k, mode="same")
    n = max(2, int(round(window / dt)))
    i0 = np.arange(k, len(t) - n - k, n // 2)
    i1 = i0 + n
    cols = [v[i1] - v[i0], q[i1] - q[i0], (cum(np.sign(v)))[i1] - cum(np.sign(v))[i0]]
    if gravity:
        cols += [cum(np.sin(q))[i1] - cum(np.sin(q))[i0], cum(np.cos(q))[i1] - cum(np.cos(q))[i0]]
    A = np.stack(cols, axis=1)
    y = cum(tau)[i1] - cum(tau)[i0]
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    names = ["J", "b", "fc"] + (["g1", "g2"] if gravity else [])
    out = dict(zip(names, map(float, x)))
    out["rms_residual"] = float(np.sqrt(np.mean((A @ x - y) ** 2)) / (n * dt))   # as a mean torque, N*m
    return out


def synthetic(joint, actuator, seconds=8.0, amp=0.6, noise=True, seed=0):
    """Logs a torque chirp on one joint of the gim6010 model hanging in the air (base welded), with 14-bit encoder
    quantization and torque / velocity noise. Returns (t, tau, q, qd), the model and the joint id."""
    import mujoco
    import simulate_wheeled_duck as swd
    spec = mujoco.MjSpec.from_file(swd.profile_xml(swd.PROFILES["gim6010"]))
    spec.delete(spec.joint("floating_base"))
    spec.body("base").pos = [0, 0, 0.6]
    spec.delete(spec.key("home_balanced"))
    m = spec.compile()
    d = mujoco.MjData(m)
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
    qa, va = m.jnt_qposadr[j], m.jnt_dofadr[j]
    # torque-controlled test: make the actuator a pure motor (MIT mode, kp = kd = 0)
    m.actuator_biasprm[a] = 0
    m.actuator_gainprm[a, 0] = 1.0
    m.actuator_ctrlrange[a] = [-20, 20]
    m.actuator_ctrllimited[a] = 1
    for b in range(m.nu):   # hold every other joint at its zero
        if b != a and m.actuator_biastype[b]:
            d.ctrl[b] = 0.0
    m.jnt_limited[j] = 0
    for k in range(m.njnt):   # the bench clamps every other joint (else their wobble couples into this one)
        if k != j:
            m.dof_armature[m.jnt_dofadr[k]] = 50.0
    rng = np.random.default_rng(seed)
    tick = 2 * math.pi / 16384
    rows = []
    for i in range(int(seconds / m.opt.timestep)):
        tt = d.time
        f = 0.3 + 2.7 * tt / seconds                              # 0.3 -> 3 Hz chirp
        d.ctrl[a] = amp * math.sin(2 * math.pi * f * tt) + 0.15 * amp * np.sign(math.sin(2 * math.pi * 0.25 * tt))
        mujoco.mj_step(m, d)
        if i % 2 == 0:                                           # 1 kHz log
            tq = float(d.actuator_force[a])
            q, v = float(d.qpos[qa]), float(d.qvel[va])
            if noise:
                tq += rng.normal(0, 0.01)
                q = round(q / tick) * tick
                v += rng.normal(0, 0.02)
            rows.append((d.time, tq, q, v))
    t, tau, q, qd = map(np.array, zip(*rows))
    return (t, tau, q, qd), m, j


def selftest():
    import mujoco
    ok = True
    for joint, act, gravity in (("left_wheel", "left_wheel_motor", False), ("left_knee", "left_knee", True)):
        (t, tau, q, qd), m, j = synthetic(joint, act, amp=0.4 if gravity else 0.08)
        r = fit(t, tau, q, qd, gravity=gravity)
        dof = m.jnt_dofadr[j]
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        # true inertia about the axis: armature + the subtree's inertia (from the joint-space mass matrix)
        M = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, M)
        true = {"J": float(M[dof, dof]), "b": float(m.dof_damping[dof]), "fc": float(m.dof_frictionloss[dof])}
        line = []
        for k, v in true.items():
            err = abs(r[k] - v) / v
            ok &= err < 0.15
            line.append(f"{k} {r[k]:.5g} (model {v:.5g}, {100 * err:.0f}%)")
        print(f"{'PASS' if all(abs(r[k] - v) / v < 0.15 for k, v in true.items()) else 'FAIL'}  {joint}: "
              + ", ".join(line) + (f", gravity {math.hypot(r['g1'], r['g2']):.3f} N*m" if gravity else "")
              + f", residual {r['rms_residual']:.3f} N*m")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("log")
    f.add_argument("--gravity", action="store_true", help="leg joint: fit the gravity load too")
    sub.add_parser("selftest")
    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    data = np.genfromtxt(args.log, delimiter=",", names=None, skip_header=0, invalid_raise=False)
    data = data[~np.isnan(data).any(axis=1)]
    r = fit(*data[:, :4].T, gravity=args.gravity)
    print(", ".join(f"{k} {v:.5g}" for k, v in r.items()))
    print("MJCF: armature = J minus the carried links' inertia (from the model), damping = b, frictionloss = fc")


if __name__ == "__main__":
    main()
