#!/usr/bin/env python3
"""
Scripted scenario tests for `simulate_wheeled_duck.py`, scored against MuJoCo ground truth (not the controller's
own estimates). Runs through the same `step_robot()` tick as the interactive sim.

Usage:
  python3 test_scenarios.py                   # ideal sensors, default robot profile (gim6010)
  python3 test_scenarios.py --robot gim4305   # the original all-GIM4305 design
  python3 test_scenarios.py --realism 20      # each scenario on 20 randomized robots (noise, latency, mass, grip...)
  python3 test_scenarios.py --xml other.xml   # override the profile's model file
"""

import argparse
import contextlib
import io
import math
import pathlib

import mujoco
import numpy as np

import simulate_wheeled_duck as swd

PROFILE = swd.PROFILES[swd.DEFAULT_ROBOT]
XML = swd.profile_xml(PROFILE)
KEYS = {"up": swd.KEY_UP, "down": swd.KEY_DOWN, "left": swd.KEY_LEFT, "right": swd.KEY_RIGHT, "space": swd.KEY_SPACE}
FALLEN_BASE_Z = 0.22  # m; nominal base height is 0.356
# Motor continuous (thermal) rating by peak torque, i.e. by which motor an actuator's forcerange says it is:
# GIM4305-10 (3.47 N*m peak, 1.0 N*m rated) and GIM6010-8 (11 N*m peak, 5 N*m rated).
RATED_TORQUE = {3.47: 1.0, 11.0: 5.0}
# Sim-to-real settings used for --realism runs (control architecture, IMU, leg CAN rate, noise...).
REALISM_CFG = swd.RealismConfig()
# Builds the controller for a model; swap it to test a variant with retuned parameters.
make_controller = lambda model: swd.ScaledWheeledDuckController(PROFILE)


def run(events=(), duration=8.0, pushes=(), realism_seed=None, torques=None):
    """Simulates one scenario. events: [(t, key)], pushes: [(t, Fx newtons)] applied 0.1 s to the base.
    If `torques` is a list, each step's [time, actuator forces (nu), actuator velocities (nu)] row is appended.

    Returns (log, fell, realism) where log rows are
    [t, x, y, z, v_fwd, pitch_deg, yaw_deg, wheel_clearance, wheel_saturated].
    """
    model = mujoco.MjModel.from_xml_path(XML)
    realism = swd.SimToReal(model, REALISM_CFG, seed=realism_seed) if realism_seed is not None else None
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    ctl = make_controller(model)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    wheels = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("foot_assembly", "foot_assembly_2")]
    events = sorted(events)
    log = []
    with contextlib.redirect_stdout(io.StringIO()):  # silence keypress / jump prints
        for _ in range(int(duration / model.opt.timestep)):
            while events and data.time >= events[0][0]:
                key = events.pop(0)[1]
                ctl.handle_keypress(KEYS[key] if key in KEYS else ord(key.upper()))
            data.xfrc_applied[base] = 0
            for t_push, force in pushes:
                if t_push <= data.time < t_push + 0.1:
                    data.xfrc_applied[base, 0] = force
            info = swd.step_robot(model, data, ctl, realism)
            if torques is not None:
                torques.append(np.concatenate(([data.time], data.actuator_force, data.actuator_velocity)))

            _, pitch, yaw = swd.quat_to_euler_zyx(data.qpos[3:7])
            v_fwd = data.qvel[0] * math.cos(yaw) + data.qvel[1] * math.sin(yaw)
            clearance = min(data.xpos[w][2] for w in wheels) - ctl.params.wheel_radius
            saturated = max(abs(info["tau_left"]), abs(info["tau_right"])) > 0.98 * ctl.params.max_wheel_torque
            log.append((data.time, data.qpos[0], data.qpos[1], data.qpos[2], v_fwd, math.degrees(pitch),
                        math.degrees(yaw), clearance, saturated))
            if data.qpos[2] < FALLEN_BASE_Z:
                return np.array(log), True, realism
    return np.array(log), False, realism


def window(log, t0, t1):
    return log[(log[:, 0] >= t0) & (log[:, 0] < t1)]


# Each scenario returns (passed, fell, metrics-string). Pass criteria are deliberately explicit.
def stand_still(seed):
    log, fell, _ = run(duration=8.0, realism_seed=seed)
    w = window(log, 4, 8)
    drift = np.ptp(w[:, 1]) * 1000 if len(w) else float("nan")
    return (not fell and drift < 60), fell, f"x drift {drift:.0f} mm, pitch std {w[:, 5].std():.2f} deg"


def drive_turn_brake(seed):
    ev = [(1.0 + 0.01 * i, "up") for i in range(4)] + [(4.0 + 0.01 * i, "left") for i in range(6)] + [(8.0, "space")]
    log, fell, _ = run(ev, 12.0, realism_seed=seed)
    if fell:
        return False, True, f"fell at t={log[-1, 0]:.2f}"
    cruise = window(log, 3, 4)[:, 4].mean()
    yaw = window(log, 6.5, 8)[:, 6].mean()
    stop = np.abs(window(log, 10, 12)[:, 4]).max()
    ok = abs(cruise - 0.6) < 0.1 and abs(yaw - 90) < 5 and stop < 0.08
    return ok, False, f"cruise {cruise:.2f} m/s (cmd 0.60), yaw {yaw:.0f} deg, |v| after brake {stop:.3f} m/s"


def brake_settle(seed):
    log, fell, _ = run([(1.0 + 0.01 * i, "up") for i in range(4)] + [(5.0, "space")], 12.0, realism_seed=seed)
    if fell:
        return False, True, f"fell at t={log[-1, 0]:.2f}"
    after = window(log, 5.0, 12.0)
    i_stop = int(np.argmax(np.abs(after[:, 4]) < 0.02))  # first moment it comes to rest after Space
    stop_x = after[i_stop, 1]
    rollback = (stop_x - after[i_stop:, 1].min()) * 100
    hold = window(log, 10.0, 12.0)
    settle = abs(hold[-1, 1] - stop_x) * 100
    return (rollback < 6 and settle < 3), False, f"roll-back {rollback:.1f} cm, final offset from stop point {settle:.1f} cm"


def crouch_stand(seed):
    ev = [(1.0 + 0.01 * i, "z") for i in range(4)] + [(4.0 + 0.01 * i, "x") for i in range(7)]  # deepest -> tallest
    log, fell, _ = run(ev, 8.0, realism_seed=seed)
    wander = np.abs(log[:, 1] - log[0, 1]).max() * 100  # crouching should not drive the robot anywhere
    return (not fell and wander < 8), fell, f"max |pitch| {np.abs(log[:, 5]).max():.1f} deg, wander {wander:.1f} cm"


def push(force):
    def scenario(seed):
        log, fell, _ = run(duration=6.0, pushes=[(2.0, force)], realism_seed=seed)
        excursion = np.abs(log[:, 1]).max()
        return (not fell and excursion < 1.5), fell, f"max pitch {np.abs(window(log, 2, 6)[:, 5]).max():.0f} deg, excursion {excursion:.2f} m"
    scenario.__name__ = f"push {force:+d} N"
    return scenario


# GIM motor groups by actuator index: (left, right). Neck/head (8-11) are STS3215 serial servos, not checked here.
MOTOR_GROUPS = {"hip_yaw": (0, 4), "hip_roll": (1, 5), "hip_pitch": (2, 6), "knee": (3, 7), "wheel": (12, 13)}
DRIVE_EVENTS = ([(1.0 + 0.01 * i, "up") for i in range(4)]                       # 0.6 m/s ...
                + [(3.0 + 4.0 * k + 0.01 * i, "left" if k % 2 == 0 else "right")  # ... weaving +/-45 deg every 4 s
                   for k in range(5) for i in range(3)])


def thermal(seed):
    """Heat check: RMS torque of every GIM motor vs. its continuous rating, in sustained windows: standing, holding
    the deepest usable crouch, and cruising at 0.6 m/s while weaving. (A motor's thermal time constant is minutes,
    so RMS over a sustained activity is the right proxy.)"""
    windows = []
    torques = []
    # 3 presses = crouch 0.24. At the 0.28 maximum the sagging knee rests on its 90 deg joint limit, so the stop
    # (not the motor) carries part of the load and the RMS torque reads misleadingly low.
    log, fell, _ = run([(10.0 + 0.01 * i, "z") for i in range(3)], 20.0, realism_seed=seed, torques=torques)
    if fell:
        return False, True, f"fell (stand/crouch run) at t={log[-1, 0]:.2f}"
    tq = np.array(torques)
    windows += [("stand", tq, (4, 10)), ("crouch", tq, (12, 20))]
    torques = []
    log, fell, _ = run(DRIVE_EVENTS, 24.0, realism_seed=seed, torques=torques)
    if fell:
        return False, True, f"fell (drive run) at t={log[-1, 0]:.2f}"
    windows.append(("drive", np.array(torques), (4, 24)))

    model = mujoco.MjModel.from_xml_path(XML)
    parts, worst = [], 0.0
    for joint, cols in MOTOR_GROUPS.items():
        rated = RATED_TORQUE[round(float(model.actuator_forcerange[cols[0], 1]), 2)]
        best = (0.0, "", 0.0)
        for label, tq, (t0, t1) in windows:
            w = tq[(tq[:, 0] >= t0) & (tq[:, 0] < t1), 1:]
            rms = max(np.sqrt(np.mean(w[:, c] ** 2)) for c in cols)
            if rms / rated > best[0]:
                best = (rms / rated, label, rms)
        worst = max(worst, best[0])
        parts.append(f"{joint} {100 * best[0]:.0f}% ({best[2]:.2f} N·m, {best[1]})")
    return worst < 0.7, False, ", ".join(parts) + "  [worst window per motor; pass < 70% of rated]"


def jump(seed):
    log, fell, _ = run([(2.0, "c")], 6.0, realism_seed=seed)
    clear = log[:, 7].max() * 1000
    return (not fell and clear > 10), fell, f"wheel clearance {clear:.1f} mm, max |pitch| {np.abs(window(log, 2, 6)[:, 5]).max():.0f} deg"


def jump_driving(seed):
    log, fell, _ = run([(1.0, "up"), (1.01, "up"), (1.02, "up"), (4.0, "c")], 8.0, realism_seed=seed)
    clear = log[:, 7].max() * 1000
    return (not fell and clear > 10), fell, f"wheel clearance {clear:.1f} mm, max |pitch| {np.abs(window(log, 4, 8)[:, 5]).max():.0f} deg"


SCENARIOS = [stand_still, drive_turn_brake, brake_settle, crouch_stand,
             push(20), push(30), push(-30), push(40), push(-40), jump, jump_driving, thermal]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--realism", type=int, default=0, metavar="N", help="run each scenario on N randomized robots")
    ap.add_argument("-k", default="", help="only run scenarios whose name contains this text")
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT,
                    help="robot profile to test (default: %(default)s)")
    ap.add_argument("--xml", default=None, help="override the profile's MJCF model")
    ap.add_argument("--arch", choices=("mcu", "pi"), default="mcu", help="realism: balance loop on the Feather (mcu) or Pi")
    ap.add_argument("--imu", choices=("bno055", "fast"), default="fast", help="realism: IMU model")
    ap.add_argument("--leg-rate", type=float, default=100.0, help="realism: leg CAN update rate (Hz)")
    args = ap.parse_args()
    globals()["REALISM_CFG"] = swd.RealismConfig(arch=args.arch, imu=args.imu, leg_rate_hz=args.leg_rate)
    globals()["PROFILE"] = swd.PROFILES[args.robot]
    globals()["XML"] = args.xml or swd.profile_xml(PROFILE)
    print(f"Robot profile '{PROFILE.name}' ({pathlib.Path(XML).name})"
          + (f", realism: arch={args.arch} imu={args.imu} legs@{args.leg_rate:.0f}Hz" if args.realism else ""))
    scenarios = [s for s in SCENARIOS if args.k in s.__name__]

    if args.realism == 0:
        for sc in scenarios:
            ok, fell, msg = sc(None)
            print(f"{'PASS' if ok else 'FAIL'}  {sc.__name__:18s} {'(FELL) ' if fell else ''}{msg}")
        return

    for sc in scenarios:
        results = [sc(seed) for seed in range(args.realism)]
        passed = sum(r[0] for r in results)
        falls = sum(r[1] for r in results)
        print(f"{passed:3d}/{args.realism} pass, {falls:2d} falls  {sc.__name__}")
        for seed, (ok, fell, msg) in enumerate(results):
            if not ok:
                m = mujoco.MjModel.from_xml_path(XML)
                print(f"        seed {seed:2d}: {msg}  [{swd.SimToReal(m, REALISM_CFG, seed=seed).describe()}]")


if __name__ == "__main__":
    main()
