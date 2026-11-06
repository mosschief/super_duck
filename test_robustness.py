#!/usr/bin/env python3
"""
Robustness scenarios for the gim6010 build, on top of `test_scenarios.py` (same `run()` tick, same pass/fail
style). Groups (`-k` filters by name):

  terrain   slopes held and driven (fore/aft, cross), a ramp up / down, door thresholds and curbs (both wheels,
            one wheel), uneven floor, low-grip floors
  power     battery sag (motor speed-torque curves at the cell voltage), e-stop at speed (everything goes limp)
  faults    CAN frames lost (driver holds the last command), driver timeouts (motor limp), stale feedback,
            IMU mounting error, gyro bias drift, vibration noise
  impact    falls at speed, driving into a wall, drops, jumps landing on a slope / one wheel
  head      head commanded over its whole range while balancing (the neck envelope clip must keep it off the trunk)
  endurance 20 min mixed duty cycle: motor RMS torque in 5 min windows, joint load cycles for fatigue
            (writes cad/fatigue_cycles.md / .json)

Fall = a body other than the legs' wheels / shins touching the ground, or the trunk tipping past 60 deg. "REPORT"
lines are measurements without a pass criterion (impact forces for the CAD).

Usage:
  python3 test_robustness.py                 # everything, ideal sensors
  python3 test_robustness.py -k slope        # only scenarios whose name contains "slope"
  python3 test_robustness.py --realism 5     # each scenario on 5 randomized robots (slow)
  python3 test_robustness.py --list
"""
import argparse
import contextlib
import dataclasses
import io
import json
import math
import pathlib

import mujoco
import numpy as np

import simulate_wheeled_duck as swd
import test_scenarios as T

ROOT = pathlib.Path(__file__).resolve().parent
WHEEL_BODIES = ("foot_assembly", "foot_assembly_2")
LEG_BODIES = ("knee_and_ankle_assembly_2", "knee_and_ankle_assembly_4") + WHEEL_BODIES   # shins may scrape
HEAD_BODIES = ("head_pitch_to_yaw", "neck_yaw_assembly", "head_assembly", "left_antenna", "right_antenna")
TIRE_RADIAL_LIMIT = 500.0   # N, GIM4305-10 output bearing radial rating (cad/purchased_parts.json)


# ---------------------------------------------------------------- helpers
def body_ids(m, names):
    return {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in names}


class Contacts:
    """Per-step ground contact bookkeeping: peak normal force per robot geom against the world (floor, terrain)."""

    def __init__(self):
        self.peak = {}
        self.first = None
        self.wheel_bodies = None

    def __call__(self, m, d):
        if self.wheel_bodies is None:
            self.wheel_bodies = body_ids(m, WHEEL_BODIES)
        f = np.zeros(6)
        step = {}
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 == 0) == (b2 == 0):
                continue
            g = c.geom2 if b1 == 0 else c.geom1
            mujoco.mj_contactForce(m, d, i, f)
            fw = c.frame.reshape(3, 3).T @ f[:3]           # contact frame -> world
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            step[name] = step.get(name, np.zeros(3)) + fw   # all contact points of a geom add up
            if self.first is None and m.geom_bodyid[g] not in self.wheel_bodies:
                self.first = (name, float(d.time))
        for name, fw in step.items():
            self.peak[name] = max(self.peak.get(name, 0.0), float(np.linalg.norm(fw)))

    def wheels(self):
        return max(self.peak.get(n, 0.0) for n in ("left_foot_bottom_tpu", "right_foot_bottom_tpu"))

    def non_wheel(self):
        return {k: v for k, v in self.peak.items() if "foot_bottom_tpu" not in k}


def make_fallen(contacts=None, obstacles=()):
    """Fall test: a trunk / neck / head body on the ground, or the trunk tipped past 60 deg. `obstacles`: world geoms
    (by name) that are not ground, such as a wall: touching one is a bump, not a fall (a fall onto the floor next
    to it, or tipping over, still counts)."""
    state = {}

    def fallen(m, d):
        if "ok" not in state:
            state["ok"] = body_ids(m, LEG_BODIES) | body_ids(m, ("hip_roll_assembly", "hip_roll_assembly_2",
                                                                 "left_roll_to_pitch_assembly",
                                                                 "right_roll_to_pitch_assembly",
                                                                 "knee_and_ankle_assembly",
                                                                 "knee_and_ankle_assembly_3"))
            state["skip"] = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in obstacles}
        if contacts is not None:
            contacts(m, d)
        up = d.xmat[1].reshape(3, 3)[:, 2]
        if up[2] < math.cos(math.radians(60)):
            return True
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in state["skip"] or c.geom2 in state["skip"]:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 == 0) != (b2 == 0) and (b2 if b1 == 0 else b1) not in state["ok"]:
                return True
        return False
    return fallen


def chain(*fns):
    fns = [f for f in fns if f is not None]
    return (lambda *a: [f(*a) for f in fns]) if fns else None


def add_box(spec, name, half, pos, euler=(0, 0, 0)):
    q = np.zeros(4)
    mujoco.mju_euler2Quat(q, np.radians(euler), "xyz")
    spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(half), pos=list(pos), quat=list(q),
                            rgba=[0.45, 0.5, 0.58, 1])


STEP_EDGE = 0.003   # m, edge radius of a step (threshold / curb): see `add_step`


def add_step(spec, name, x0, x1, y0, y1, h):
    """A step of height h over x0..x1 (a threshold, or a curb when long) built so the tire only ever touches capsules:
    a row of touching STEP_EDGE-radius capsules across the path, crests at h, over a core box up to their axes.

    Why not a plain box: against a *cylinder* (the tire), MuJoCo's convex collider (native EPA in 3.17, and libccd's
    MPR on long boxes) reports a box or mesh *face* normal at a box edge instead of the true radial one. With the
    wheel on a 20 mm edge it reports the front face's horizontal normal at 2.4 mm depth where the geometry gives
    (0.64, -0.76) at 3.7 mm. A horizontal normal turns the edge into a vertical wall the tire can only climb by
    friction (mu 1.25, so it slips and spins at the 3.47 N*m stall), and the robot fell at every 20-30 mm step.
    It does the same at any box corner the tire rolls over, including a flat top's corners just behind a rounded
    nose (the tire sinks ~3 mm into the edge, far enough to reach them). Capsule-cylinder contacts come out radial,
    and the core box's edges sit on the capsule axes, out of reach. The front capsule is a 3 mm arris, which a sharp
    real threshold has anyway; it eases the climb by < 1 deg of contact angle (20 mm: 43.1 -> 42.1 deg from vertical).
    The ridged top (6 mm pitch) sags 0.06 mm between crests under a 75 mm wheel, and the tire's ~40 mm contact patch
    spans ~6 of them, so it rolls as on a flat top."""
    rho = min(STEP_EDGE, h / 2.0)
    n = max(1, int(round((x1 - x0) / (2 * rho))))
    pitch = (x1 - x0 - 2 * rho) / max(n - 1, 1)
    for i in range(n):
        xe = x0 + rho + i * pitch
        spec.worldbody.add_geom(name=f"{name}_ridge{i}", type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[rho, 0, 0],
                                fromto=[xe, y0 + rho, h - rho, xe, y1 - rho, h - rho], rgba=[0.45, 0.5, 0.58, 1])
    if h - rho > 1e-6:
        add_box(spec, f"{name}_core", ((x1 - x0) / 2 - rho, (y1 - y0) / 2, (h - rho) / 2),
                ((x0 + x1) / 2, (y0 + y1) / 2, (h - rho) / 2))


def tilt_world(axis, deg):
    """(spec_edit, init) tilting the floor about the world origin by `deg` about `axis`, robot spawned on it."""
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array(axis, float), math.radians(deg))

    def spec_edit(spec):
        floor = spec.geom("floor")
        floor.quat = list(q)

    def init(m, d):
        p = np.zeros(3)
        mujoco.mju_rotVecQuat(p, d.qpos[0:3].copy(), q)
        d.qpos[0:3] = p
        out = np.zeros(4)
        mujoco.mju_mulQuat(out, q, d.qpos[3:7].copy())
        d.qpos[3:7] = out
    return spec_edit, init


def drive(t0, v, presses=None):
    """Key events that bring the command to v (0.15 m/s per press) starting at t0."""
    n = presses if presses is not None else int(round(abs(v) / 0.15))
    return [(t0 + 0.01 * i, "up" if v > 0 else "down") for i in range(n)]


def result(ok, fell, msg):
    return bool(ok and not fell), fell, msg


REPORT = "report"   # marker: ok is None -> printed as REPORT


def run(seed, **kw):
    kw.setdefault("fallen", make_fallen())
    return T.run(realism_seed=seed, **kw)


def hold_drift(log, t0, t1):
    w = T.window(log, t0, t1)
    return float(np.hypot(np.ptp(w[:, 1]), np.ptp(w[:, 2]))) * 100 if len(w) else float("nan")


# ---------------------------------------------------------------- terrain
def slope(deg, direction):
    """Parks on a slope, drives 0.4 m/s along it for 3 s, brakes and holds. direction: up / down (facing) / cross."""
    axis, sign = {"up": ((0, 1, 0), -1), "down": ((0, 1, 0), 1), "cross": ((1, 0, 0), 1)}[direction]
    spec_edit, init = tilt_world(axis, sign * deg)

    def scenario(seed):
        ev = drive(8.0, 0.45) + [(11.0, "space")]
        log, fell, _ = run(seed, events=ev, duration=17.0, spec_edit=spec_edit, init=init)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        # It starts with no idea of the slope: the first seconds are the trim learner rolling it back to its hold point
        park, hold = hold_drift(log, 5, 8), hold_drift(log, 14, 17)
        return result(park < 5 and hold < 5, False, f"parked drift {park:.1f} cm (5-8 s), after brake {hold:.1f} cm (14-17 s)")
    scenario.__name__ = f"slope {direction} {deg} deg"
    return scenario


def ramp(deg, up=True, L=1.2):
    """Flat -> ramp -> platform (or platform -> ramp down -> flat) at 0.4 m/s, brake on the ramp, then carry on."""
    th = math.radians(deg)
    x0, h = 0.6, 0.05
    H = L * math.sin(th)

    def spec_edit(spec):
        if up:
            c = np.array([x0, 0, 0]) + L / 2 * np.array([math.cos(th), 0, math.sin(th)]) - h * np.array([-math.sin(th), 0, math.cos(th)])
            add_box(spec, "ramp", (L / 2, 0.6, h), c, (0, -deg, 0))
            add_box(spec, "platform", (1.0, 0.6, H / 2), (x0 + L * math.cos(th) + 1.0, 0, H / 2))
        else:
            add_box(spec, "platform", (0.5 + x0 / 2, 0.6, H / 2), (x0 / 2 - 0.5, 0, H / 2))
            c = np.array([x0, 0, H]) + L / 2 * np.array([math.cos(th), 0, -math.sin(th)]) - h * np.array([math.sin(th), 0, math.cos(th)])
            add_box(spec, "ramp", (L / 2, 0.6, h), c, (0, deg, 0))

    def init(m, d):
        if not up:
            d.qpos[2] += H

    def scenario(seed):
        t_ramp = (x0 + 0.5 * L) / 0.45 + 1.5
        ev = drive(1.0, 0.45) + [(t_ramp, "space")] + drive(t_ramp + 4.0, 0.45)
        log, fell, _ = run(seed, events=ev, duration=t_ramp + 8.0, spec_edit=spec_edit, init=init)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s (x {log[-1, 1]:.2f} m)"
        held = hold_drift(log, t_ramp + 2.0, t_ramp + 4.0)
        end_x = log[-1, 1]
        target = x0 + L * math.cos(th) + 0.3
        return result(held < 5 and end_x > target, False,
                      f"held on the ramp to {held:.1f} cm, reached x {end_x:.2f} m (needs > {target:.2f})")
    scenario.__name__ = f"ramp {'up' if up else 'down'} {deg} deg"
    return scenario


def threshold(h_mm, v=0.6, side=None, curb=False):
    """A door threshold (30 mm deep strip) or a curb (step up) across the path at x = 1.0, at speed v. side='left'
    puts it under the left wheel only. Built with `add_step` (a 3 mm edge radius), not a box: MuJoCo's collider turns
    a box edge into a vertical wall for the cylinder tire.

    All the heights here are within the wheel's torque: on a sharp edge of height h the wheel pivots about the edge
    at sin(a) = sqrt(r^2 - (r - h)^2) / r from vertical, needing tau = W r sin(a) per wheel (W = 34.5 N wheel load):
    20 mm 1.76 N*m, 30 mm 2.07 N*m, against 3.47 N*m stall. Friction needs mu > tan(a) - (forward push from the
    body); tan(a) = 0.93 at 20 mm and 1.33 at 30 mm against the tire's 1.25, so 30 mm needs a ~1 N push per wheel,
    which the robot's forward lean onto the edge supplies."""
    h = h_mm / 1000.0

    def spec_edit(spec):
        y0, y1 = (0.03, 0.4) if side == "left" else (-0.4, 0.4)
        if curb:
            add_step(spec, "curb", 1.0, 4.0, y0, y1, h)
        else:
            add_step(spec, "threshold", 0.985, 1.015, y0, y1, h)

    def scenario(seed):
        log, fell, _ = run(seed, events=drive(1.0, v) + [(1.0 + 2.6 / v, "space")], duration=3.0 + 2.6 / v + 3,
                           spec_edit=spec_edit)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s (x {log[-1, 1]:.2f} m)"
        crossed = log[-1, 1] > 1.3
        pitch = np.abs(T.window(log, 1.0, 1.0 + 2.6 / v)[:, 5]).max()
        return result(crossed, False, f"{'crossed' if crossed else 'stopped at'} x {log[-1, 1]:.2f} m, max |pitch| {pitch:.0f} deg")
    kind = "curb" if curb else "threshold"
    scenario.__name__ = f"{kind} {h_mm} mm{' left wheel' if side else ''} @{v:.1f}"
    return scenario


def uneven(amp_mm):
    """Drives a weave over a random height field (amp = peak-to-peak height, ~8 cm bump spacing)."""
    def scenario(seed):
        rng = np.random.default_rng(100 + (seed or 0))
        n = 161  # 4 m x 4 m at 2.5 cm
        z = rng.normal(size=(n, n))
        k = np.exp(-0.5 * (np.arange(-6, 7) / 2.0) ** 2)
        k /= k.sum()
        for ax in (0, 1):
            z = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), ax, z)
        z = (z - z.min()) / np.ptp(z)
        yy, xx = np.meshgrid(np.linspace(-2, 2, n), np.linspace(-2, 2, n), indexing="ij")
        r = np.hypot(xx + 1.5, yy)            # flat pad at the start (robot spawns at x = -1.5)
        z = z * np.clip((r - 0.15) / 0.2, 0, 1) + 0.5 * (1 - np.clip((r - 0.15) / 0.2, 0, 1))
        top = amp_mm / 1000.0

        def spec_edit(spec):
            hf = spec.add_hfield(name="terrain", size=[2.0, 2.0, top, 0.02], nrow=n, ncol=n, userdata=z.flatten().tolist())
            spec.worldbody.add_geom(name="terrain", type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname=hf.name,
                                    pos=[0, 0, -0.5 * top], rgba=[0.45, 0.5, 0.58, 1])

        def init(m, d):
            d.qpos[0] = -1.5

        ev = drive(1.0, 0.6) + [(3.0 + 3.0 * k_ + 0.01 * i, "left" if k_ % 2 == 0 else "right") for k_ in range(3) for i in range(2)]
        log, fell, _ = run(seed, events=ev + [(10.0, "space")], duration=12.0, spec_edit=spec_edit, init=init)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        return result(True, False, f"drove {np.hypot(log[-1, 1] + 1.5, log[-1, 2]):.1f} m, max |pitch| {np.abs(log[:, 5]).max():.0f} deg")
    scenario.__name__ = f"uneven floor {amp_mm} mm"
    return scenario


def low_grip(mu, base):
    """Runs one of test_scenarios' scenarios with floor and tire friction set to mu."""
    def init(m, d):
        for n in swd.TIRE_GEOMS:
            m.geom_friction[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n), 0] = mu

    def scenario(seed):
        old = T.run
        T.run = lambda *a, **kw: old(*a, init=init, fallen=make_fallen(), **kw)
        try:
            return base(seed)
        finally:
            T.run = old
    scenario.__name__ = f"grip mu {mu:.1f} {base.__name__}"
    return scenario


def turn_at_speed(v, deg):
    """A heading step of `deg` (15 deg per key press, all at once) while cruising at v."""
    def scenario(seed):
        n = int(round(deg / 15))
        log, fell, _ = run(seed, events=drive(1.0, v) + [(4.0 + 0.01 * i, "left") for i in range(n)] + [(8.0, "space")],
                           duration=10.0)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.2f} s"
        yaw = T.window(log, 7.0, 8.0)[:, 6].mean()
        return result(abs(yaw - deg) < 8, False, f"yaw {yaw:.0f} deg (cmd {deg})")
    scenario.__name__ = f"turn {deg} deg @{v:.1f} m/s"
    return scenario


# ---------------------------------------------------------------- power
holder_volts = {}   # pack voltage of the running battery() scenario, for top_speed's expected cap


def battery(volts, base):
    """Runs a scenario with both motors' torque-speed curves at the pack voltage (no-load speed ~ V / 24 V).
    Stall torque is current-limited by the drivers and stays put down to ~19 V."""
    def hook(m, d, ctl):
        ctl.bus_voltage = volts

    def scenario(seed):
        old_run = T.run
        T.run = lambda *a, **kw: old_run(*a, hook=hook, fallen=make_fallen(), **kw)
        holder_volts["v"] = volts
        try:
            return base(seed)
        finally:
            T.run = old_run
            holder_volts.pop("v", None)
    scenario.__name__ = f"battery {volts:.1f} V {base.__name__}"
    return scenario


def top_speed(seed):
    """Top speed (MAX_SPEED, 2.5 m/s) for 4 s, then brake. The controller's speed cap sets what it reaches at the pack
    voltage: all of it at 25.2 V, 2.43 m/s at 22.2 V, 2.30 m/s at the 21 V cutoff."""
    vmax = swd.ScaledWheeledDuckController.MAX_SPEED
    probe = T.make_controller(None)
    probe.filtered_voltage = holder_volts.get("v")
    want = probe.speed_cap()
    log, fell, _ = T.run(drive(1.0, vmax) + [(7.0, "space")], 11.0, realism_seed=seed)
    if fell:
        return False, True, f"fell at t={log[-1, 0]:.1f} s"
    v = T.window(log, 5.5, 7.0)[:, 4].mean()
    stop = np.abs(T.window(log, 9.5, 11)[:, 4]).max()
    return (v > want - 0.1 and stop < 0.08), False, (f"cruise {v:.2f} m/s (cmd {vmax:.2f}, capped to {want:.2f}), "
                                                     f"|v| after brake {stop:.3f} m/s")


def low_battery(v0):
    """The pack sags from 22 V to 20.5 V over 10 s while cruising at v0: the robot must stop and stay up, and ignore
    the driver and the jump key afterwards."""
    def hook(m, d, ctl):
        ctl.bus_voltage = 22.0 - 1.5 * min(d.time / 10.0, 1.0)

    def scenario(seed):
        log, fell, _ = run(seed, events=drive(1.0, v0) + drive(14.0, 0.6) + [(15.0, "c")], duration=18.0, hook=hook)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        v = np.abs(T.window(log, 14.0, 18.0)[:, 4]).max()
        lift = T.window(log, 14.0, 18.0)[:, 7].max() * 1000
        return result(v < 0.1 and lift < 5, False, f"parked: |v| {v:.3f} m/s after the keys, wheels off the floor {lift:.1f} mm")
    scenario.__name__ = f"low battery while driving @{v0:.1f}"
    return scenario


def cruise_push(v, force):
    """A shove (0.1 s) while cruising. Forward is the hard way: the wheels must speed up to get back under the trunk."""
    def scenario(seed):
        log, fell, _ = run(seed, events=drive(1.0, v), duration=9.0, pushes=[(5.0, force)])
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        return True, False, f"max |pitch| {np.abs(T.window(log, 5, 9)[:, 5]).max():.0f} deg"
    scenario.__name__ = f"push {force:+d} N @{v:.1f} m/s"
    return scenario


_LIMP = set()                      # actuators without power right now (see Limp)
_apply_kd = swd.apply_leg_servo_kd


def _apply_kd_limp(model, kd):
    """step_robot re-applies the leg servo damping every tick; keep unpowered actuators at zero force."""
    _apply_kd(model, kd)
    for a in _LIMP:
        model.actuator_gainprm[a, 0] = 0.0
        model.actuator_biasprm[a, 1:3] = 0.0


swd.apply_leg_servo_kd = _apply_kd_limp
_t_run = T.run


EXTRA_HOOK = None   # tools/export_fea_loads.py: a hook(model, data, ctl) added to every run


def _run_fresh(*a, **kw):
    _LIMP.clear()   # a new model starts powered
    if EXTRA_HOOK is not None:
        kw["hook"] = chain(kw.get("hook"), EXTRA_HOOK)
    return _t_run(*a, **kw)


T.run = _run_fresh


class Limp:
    """Fault layer: during [t0, t1) the listed actuators lose power (zero torque; position servos go slack, no
    damping: an unpowered BLDC driver does not short its phases)."""

    def __init__(self, inner, model, data, windows):
        self.inner, self.m, self.d, self.windows = inner, model, data, windows
        self.gain, self.bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()

    def sense(self, *s):
        return self.inner.sense(*s) if self.inner else s

    def actuate(self, ctrl):
        out = self.inner.actuate(ctrl) if self.inner else np.array(ctrl, float)
        limp = {a for t0, t1, acts in self.windows if t0 <= self.d.time < t1 for a in acts}
        for a in _LIMP - limp:   # power back: restore the servo gains
            self.m.actuator_gainprm[a, 0] = self.gain[a, 0]
            self.m.actuator_biasprm[a, 1:3] = self.bias[a, 1:3]
        _LIMP.clear()
        _LIMP.update(limp)
        for a in limp:
            out[a] = 0.0
            self.m.actuator_gainprm[a, 0] = 0.0
            self.m.actuator_biasprm[a, 1:3] = 0.0
        return out


ALL_ACTUATORS = tuple(range(14))


def estop(v):
    """E-stop pressed while driving at v: every motor and servo goes limp (the anti-spark switch cuts the pack)."""
    def scenario(seed):
        c = Contacts()
        t_cut = 1.0 + (3.0 if v else 2.0)
        holder = {}

        def init(m, d):
            holder["md"] = (m, d)

        def faults(realism):
            m, d = holder["md"]
            layer = Limp(realism, m, d, [(t_cut, 99.0, ALL_ACTUATORS)])
            return layer
        log, fell, _ = T.run(drive(1.0, v) if v else [], t_cut + 2.5, realism_seed=seed, init=init, faults=faults,
                             fallen=lambda m, d: (c(m, d), False)[1])
        hits = sorted(c.non_wheel().items(), key=lambda kv: -kv[1])[:4]
        first = f"first {c.first[0]} at +{c.first[1] - t_cut:.2f} s; " if c.first else ""
        return None, False, first + "peak ground force " + ", ".join(f"{k} {f:.0f} N" for k, f in hits)
    scenario.__name__ = f"e-stop @{v:.1f} m/s"
    return scenario


# ---------------------------------------------------------------- faults
class FaultLayer:
    """Wraps the (optional) realism layer with scheduled CAN / sensor faults.
    hold:  [(t0, t1, actuators)] the drivers keep applying the last command they received (lost frames)
    stale: [(t0, t1)] the controller's sensor frames stop updating (it keeps the last ones)
    imu_offset_deg: constant IMU pitch mounting error; gyro_drift_dps_per_s: pitch-gyro bias ramp;
    vib: (gyro dps RMS per m/s of speed, orientation deg RMS per m/s) wheel-vibration noise"""

    def __init__(self, inner, data, hold=(), stale=(), imu_offset_deg=0.0, gyro_drift=0.0, vib=(0.0, 0.0), seed=0):
        self.inner, self.d = inner, data
        self.hold, self.stale = hold, stale
        self.last, self.last_s = None, None
        q = np.zeros(4)
        mujoco.mju_axisAngle2Quat(q, np.array([0.0, 1.0, 0.0]), math.radians(imu_offset_deg))
        self.q_off, self.drift, self.vib = q, math.radians(gyro_drift), vib
        self.rng = np.random.default_rng(1000 + (seed or 0))

    def sense(self, quat, gyro, *rest):
        t = self.d.time
        quat, gyro = np.array(quat, float), np.array(gyro, float)
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, quat, self.q_off)
        gyro[1] += self.drift * t
        speed = abs(self.d.qvel[0])
        if self.vib[0]:
            gyro += np.radians(self.rng.normal(0, self.vib[0] * speed, 3))
            e = np.radians(self.rng.normal(0, self.vib[1] * speed, 3))
            dq = np.array([1.0, *(0.5 * e)])
            mujoco.mju_mulQuat(q, q.copy(), dq / np.linalg.norm(dq))
        s = (q, gyro, *rest)
        if self.inner:
            s = self.inner.sense(*s)
        if any(t0 <= t < t1 for t0, t1 in self.stale) and self.last_s is not None:
            return self.last_s
        self.last_s = s
        return s

    def actuate(self, ctrl):
        out = self.inner.actuate(ctrl) if self.inner else np.array(ctrl, float)
        t = self.d.time
        if self.last is not None:
            for t0, t1, acts in self.hold:
                if t0 <= t < t1:
                    out[list(acts)] = self.last[list(acts)]
        self.last = out.copy()
        return out


LEFT_BUS = (0, 1, 2, 10)   # CAN1: left hip roll, hip pitch, knee, wheel


def can_fault(name, events, duration, hold=(), limp=(), stale=()):
    def scenario(seed):
        holder = {}

        def init(m, d):
            holder["md"] = (m, d)

        def faults(realism):
            m, d = holder["md"]
            inner = Limp(realism, m, d, limp) if limp else realism
            return FaultLayer(inner, d, hold=hold, stale=stale, seed=seed)
        log, fell, _ = run(seed, events=events, duration=duration, init=init, faults=faults)
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.2f} s"
        return True, False, f"max |pitch| {np.abs(log[:, 5]).max():.0f} deg, x at end {log[-1, 1]:.2f} m"
    scenario.__name__ = name
    return scenario


def imu_fault(name, imu_offset_deg=0.0, gyro_drift=0.0, vib=(0.0, 0.0), speed=0.0):
    def scenario(seed):
        holder = {}

        def init(m, d):
            holder["d"] = d
        ev = drive(1.0, speed) + [(14.0, "space")] if speed else []
        log, fell, _ = run(seed, events=ev, duration=24.0 if speed else 20.0, init=init,
                           faults=lambda r: FaultLayer(r, holder["d"], imu_offset_deg=imu_offset_deg,
                                                       gyro_drift=gyro_drift, vib=vib, seed=seed))
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        drift = hold_drift(log, log[-1, 0] - 4, log[-1, 0])
        return result(drift < 6, False, f"drift over the last 4 s {drift:.1f} cm, max |pitch| {np.abs(log[:, 5]).max():.0f} deg")
    scenario.__name__ = name
    return scenario


# ---------------------------------------------------------------- impact
def fall_at_speed(v, force=90.0):
    def scenario(seed):
        c = Contacts()
        t_push = 4.0 if v else 2.0
        log, fell, _ = T.run(drive(1.0, v) if v else [], t_push + 3.0, pushes=[(t_push, force)], realism_seed=seed,
                             fallen=lambda m, d: (c(m, d), False)[1])
        hits = sorted(c.non_wheel().items(), key=lambda kv: -kv[1])[:4]
        if not hits:
            return None, False, f"stayed up (wheel peak {c.wheels():.0f} N)"
        return None, False, f"first {c.first[0]}; peak ground force " + ", ".join(f"{k} {f:.0f} N" for k, f in hits)
    scenario.__name__ = f"fall +{force:.0f} N @{v:.1f} m/s"
    return scenario


def wall(v):
    """Drives at v into a wall 1.45 m ahead. The head sticks out ahead of the wheels, so it is always what hits the
    wall first, and nothing the controller senses (IMU, encoders, motor current) can see the wall coming: the hit
    itself can't be avoided, so the wall counts as an obstacle, not as ground, for the fall test. To pass, the robot
    must stay up (not touch the floor with anything but its legs, not tip past 60 deg), keep its pitch under 30 deg,
    and be standing off the wall (no contact) for the last 1.5 s instead of leaning on it."""
    def spec_edit(spec):
        add_box(spec, "wall", (0.05, 1.0, 0.5), (1.5, 0, 0.5))

    def scenario(seed):
        c = Contacts()
        touching = []

        def contacts(m, d):
            on = False
            for i in range(d.ncon):
                k = d.contact[i]
                names = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in (k.geom1, k.geom2)}
                if "wall" in names:
                    on = True
                    f = np.zeros(6)
                    mujoco.mj_contactForce(m, d, i, f)
                    other = (names - {"wall"}).pop()
                    c.peak[other] = max(c.peak.get(other, 0.0), float(f[0]))
            if on:
                touching.append(float(d.time))
        fallen = make_fallen(obstacles=("wall",))
        duration = 1.0 + 1.5 / v + 3.0
        log, fell, _ = run(seed, events=drive(1.0, v), duration=duration, spec_edit=spec_edit,
                           fallen=lambda m, d: (contacts(m, d), fallen(m, d))[1])
        hits = ", ".join(f"{k} {f:.0f} N" for k, f in sorted(c.peak.items(), key=lambda kv: -kv[1])[:3])
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s; wall contact: {hits or 'none'}"
        pitch = np.abs(log[:, 5]).max()
        off = not touching or touching[-1] < duration - 1.5
        gap = 1.45 - log[-1, 1]
        return result(off and pitch < 30, False,
                      f"{'backed off' if off else 'still on the wall'}, base {gap:.2f} m from it, max |pitch| "
                      f"{pitch:.0f} deg; wall contact: {hits or 'none'}")
    scenario.__name__ = f"wall @{v:.1f} m/s"
    return scenario


def drop(h_cm):
    def init(m, d):
        d.qpos[2] += h_cm / 100.0

    def scenario(seed):
        c = Contacts()
        fallen = make_fallen(c)
        log, fell, _ = run(seed, duration=4.0, init=init, fallen=fallen)
        w = c.wheels()
        return result(w < TIRE_RADIAL_LIMIT, fell, f"peak wheel force {w:.0f} N (GIM4305 radial {TIRE_RADIAL_LIMIT:.0f} N)")
    scenario.__name__ = f"drop {h_cm} cm"
    return scenario


def bad_landing(kind):
    """Jump landing on a cross slope, or with one wheel landing mid-way along a 15 mm plate (only the landing counts:
    driving on and off the plate has its own measurement, `plate_edge`)."""
    def scenario(seed):
        if kind == "slope":
            spec_edit, init = tilt_world((1, 0, 0), 8.0)
            ev = [(3.0, "c")]
        else:
            def spec_edit(spec):
                add_box(spec, "plate", (0.6, 0.2, 0.0075), (1.4, 0.23, 0.0075))
            init = None
            ev = drive(1.0, 0.6) + [(3.5, "c"), (5.0, "space")]
        c = Contacts()
        count = (lambda m, d: c(m, d) if d.time >= 3.5 and d.time < 5.0 else None) if kind == "plate" else c
        log, fell, _ = run(seed, events=ev, duration=7.0, spec_edit=spec_edit, init=init, fallen=make_fallen(count))
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.1f} s"
        return result(c.wheels() < TIRE_RADIAL_LIMIT, False,
                      f"landed, peak wheel force {c.wheels():.0f} N, max |pitch| {np.abs(log[:, 5]).max():.0f} deg")
    scenario.__name__ = f"jump landing {'8 deg cross slope' if kind == 'slope' else 'one wheel on 15 mm plate'}"
    return scenario


# ---------------------------------------------------------------- head
def plate_edge(seed):
    """One wheel driving on and off a 15 mm plate at 0.6 m/s: the wheel hits a square edge. A short contact spike, and
    the sim's tyre is stiffer than the real TPU, so this is a measurement rather than a limit."""
    def spec_edit(spec):
        add_box(spec, "plate", (0.6, 0.2, 0.0075), (1.4, 0.23, 0.0075))
    c = Contacts()
    log, fell, _ = run(seed, events=drive(1.0, 0.6) + [(5.0, "space")], duration=7.0, spec_edit=spec_edit,
                       fallen=make_fallen(c))
    return None, False, f"{'fell, ' if fell else ''}peak wheel force {c.wheels():.0f} N (GIM4305 radial {TIRE_RADIAL_LIMIT:.0f} N)"


def head_sweep(seed):
    """Commands the head over its whole range (neck, head pitch, yaw, roll extremes) while balancing; the neck
    envelope clip must keep the head and antennas off the trunk."""
    grid = [(n, h, y, r) for n in (-0.30, 0.15, 0.90) for h in (-0.70, 0.0, 0.70)
            for y in (-1.5, 0.0, 1.5) for r in (-0.45, 0.45)]
    hits = {}

    def hook(m, d, ctl):
        k = int(d.time / 0.4)
        ctl.head_pose = grid[k % len(grid)] if k < len(grid) else None
        ctl.antenna_cmd = 0.78 if (k // 2) % 2 else -0.78
        head = body_ids(m, HEAD_BODIES)
        trunk = body_ids(m, ("trunk_assembly",))
        for i in range(d.ncon):
            c = d.contact[i]
            b = {m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]}
            if b & head and b & trunk:
                key = tuple(sorted(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in (c.geom1, c.geom2)))
                hits[key] = min(hits.get(key, 0.0), c.dist)
    log, fell, _ = run(seed, duration=0.4 * len(grid) + 1.0, hook=hook)
    msg = (f"{len(grid)} head poses, head/trunk contacts: "
           + (", ".join(f"{a}/{b} {d * 1000:.1f} mm" for (a, b), d in hits.items()) or "none"))
    return result(not hits, fell, msg)


# ---------------------------------------------------------------- endurance
def duty_cycle(minutes):
    """60 s block: weave at 0.6 m/s, brake, hold the deepest crouch, jump, 1.2 m/s with 30 deg turns, brake, stand."""
    ev = []
    for k in range(minutes):
        t = 60.0 * k
        ev += drive(t + 1.0, 0.6) + [(t + 4.0 + 3.0 * j + 0.01 * i, "left" if j % 2 == 0 else "right")
                                     for j in range(3) for i in range(3)]
        ev += [(t + 15.0, "space")] + [(t + 18.0 + 0.01 * i, "z") for i in range(3)] + [(t + 28.0 + 0.01 * i, "x") for i in range(3)]
        ev += [(t + 31.0, "c")] + drive(t + 35.0, 1.2) + [(t + 38.0 + 0.01 * i, "left") for i in range(2)]
        ev += [(t + 41.0 + 0.01 * i, "right") for i in range(2)]
        ev += [(t + 46.0, "space")]
    return ev


def rainflow(series):
    """Rainflow cycle ranges (ASTM E1049 three-point method) of a load history."""
    pts = [series[0]]
    for v in series[1:]:   # reversals only
        if len(pts) >= 2 and (pts[-1] - pts[-2]) * (v - pts[-1]) >= 0:
            pts[-1] = v
        elif v != pts[-1]:
            pts.append(v)
    stack, cycles = [], []
    for p in pts:
        stack.append(p)
        while len(stack) >= 3:
            x, y = abs(stack[-1] - stack[-2]), abs(stack[-2] - stack[-3])
            if x < y:
                break
            if len(stack) == 3:
                cycles.append((y, 0.5))
                stack.pop(0)
            else:
                cycles.append((y, 1.0))
                del stack[-3:-1]
    cycles += [(abs(stack[i + 1] - stack[i]), 0.5) for i in range(len(stack) - 1)]
    return cycles


def endurance(minutes=20):
    def scenario(seed):
        torques = []
        loads = {}
        holder = {}

        def hook(m, d, ctl):
            if "j" not in holder:
                holder["j"] = [j for j in range(m.njnt) if m.jnt_type[j] == 3 and "antenna" not in
                               mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)]
            if int(round(d.time / m.opt.timestep)) % 5:   # 100 Hz is plenty for load cycles
                return
            for j in holder["j"]:
                b = m.jnt_bodyid[j]
                f, tq = d.cfrc_int[b, 3:], d.cfrc_int[b, :3]
                p, a = d.xanchor[j], d.xaxis[j]
                tq_p = tq + np.cross(d.subtree_com[m.body_rootid[b]] - p, f)
                ma = tq_p @ a
                n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
                loads.setdefault(n, []).append((np.linalg.norm(f - (f @ a) * a), np.linalg.norm(tq_p - ma * a)))
        log, fell, _ = T.run(duty_cycle(minutes), 60.0 * minutes, realism_seed=seed, torques=torques, hook=hook,
                             fallen=make_fallen())
        if fell:
            return False, True, f"fell at t={log[-1, 0]:.0f} s"
        tq = np.array(torques)
        m = mujoco.MjModel.from_xml_path(T.XML)
        parts, worst = [], 0.0
        for joint, cols in T.MOTOR_GROUPS.items():
            rated = T.RATED_TORQUE[round(float(m.actuator_forcerange[cols[0], 1]), 2)]
            win = []
            for t0 in np.arange(0, 60.0 * minutes - 299, 60.0):
                w = tq[(tq[:, 0] >= t0) & (tq[:, 0] < t0 + 300), 1:]
                win.append(max(np.sqrt(np.mean(w[:, c] ** 2)) for c in cols) / rated)
            whole = max(np.sqrt(np.mean(tq[:, 1 + c] ** 2)) for c in cols) / rated
            worst = max(worst, max(win))
            parts.append(f"{joint} {100 * max(win):.0f}% (whole run {100 * whole:.0f}%)")
        write_fatigue(loads, minutes)
        return worst < 0.7, False, ", ".join(parts) + " of rated, worst 5 min; -> cad/fatigue_cycles.md"
    scenario.__name__ = f"endurance {minutes} min"
    return scenario


def write_fatigue(loads, minutes):
    """Rainflow cycle counts per joint (radial force, bending moment), scaled to cycles per hour."""
    bins_f = [5, 10, 20, 40, 80, 160, 320]
    bins_m = [0.25, 0.5, 1, 2, 4, 8, 16]
    out, L = {}, ["# Joint Load Cycles (fatigue)", "",
                  f"Generated by `test_robustness.py -k endurance` from a {minutes} min mixed duty cycle (weave 0.6 m/s, "
                  "brake, deepest crouch hold, a jump, 1.2 m/s with turns, every minute), ideal sensors. Rainflow cycle "
                  "counts of each joint's radial force and bending moment (`export_design_loads.py` terms), as cycles "
                  "per hour of that duty, at or above each range. Use with the printed parts' S-N data and "
                  "`design_loads.md` peaks.", ""]
    scale = 60.0 / minutes
    for name, rows in loads.items():
        arr = np.array(rows)
        cf, cm = rainflow(list(arr[:, 0])), rainflow(list(arr[:, 1]))
        out[name] = {"radial_N": {b: round(scale * sum(c for r, c in cf if r >= b)) for b in bins_f},
                     "bend_Nm": {b: round(scale * sum(c for r, c in cm if r >= b)) for b in bins_m}}
    L += ["| Joint | Radial force range ≥ " + " / ".join(f"{b} N" for b in bins_f) + " | Bending range ≥ "
          + " / ".join(f"{b} N·m" for b in bins_m) + " |", "| :--- | :--- | :--- |"]
    for name, v in out.items():
        L.append(f"| `{name}` | " + " / ".join(str(x) for x in v["radial_N"].values()) + " | "
                 + " / ".join(str(x) for x in v["bend_Nm"].values()) + " |")
    (ROOT / "cad" / "fatigue_cycles.json").write_text(json.dumps(out, indent=1) + "\n")
    (ROOT / "cad" / "fatigue_cycles.md").write_text("\n".join(L) + "\n")


# ---------------------------------------------------------------- suite
WHEEL_L = (10,)
SCENARIOS = (
    [slope(d, s) for s in ("up", "down", "cross") for d in (5, 10, 15)]
    + [ramp(10, True), ramp(10, False), ramp(15, True), ramp(15, False)]
    + [threshold(h) for h in (5, 10, 15, 20, 25)] + [threshold(15, v=1.0), threshold(10, side="left"),
                                                     threshold(20, side="left")]
    + [threshold(h, curb=True) for h in (10, 20, 30)]
    + [uneven(a) for a in (5, 10, 20)]
    + [turn_at_speed(v, d) for v in (0.6, 1.2, 1.5, 1.8, 2.2, 2.5) for d in (45, 90)]
    + [cruise_push(v, f) for v in (0.6, 1.05, 1.5, 1.8, 2.5) for f in (20, 30, -30)]
    + [low_grip(mu, b) for mu in (0.3, 0.4, 0.5, 0.6) for b in (T.drive_turn_brake, T.push(30), T.jump)]
    + [battery(v, b) for v in (25.2, 22.2, 21.1) for b in (top_speed, T.push(40), T.jump)]
    + [low_battery(0.6), low_battery(1.5), low_battery(2.5)]
    + [estop(v) for v in (0.0, 0.6, 1.8, 2.5)]
    + [can_fault("CAN left wheel frames lost 20 ms @0.6", drive(1.0, 0.6), 7.0, hold=[(4.0, 4.02, WHEEL_L)]),
       can_fault("CAN left wheel frames lost 100 ms @0.6", drive(1.0, 0.6), 7.0, hold=[(4.0, 4.1, WHEEL_L)]),
       can_fault("CAN left bus lost 50 ms @0.6", drive(1.0, 0.6), 7.0, hold=[(4.0, 4.05, LEFT_BUS)]),
       can_fault("CAN left bus lost 200 ms standing", [], 6.0, hold=[(3.0, 3.2, LEFT_BUS)]),
       can_fault("driver timeout left wheel limp 100 ms @0.6", drive(1.0, 0.6), 7.0, limp=[(4.0, 4.1, WHEEL_L)]),
       can_fault("driver timeout left wheel limp 300 ms standing", [], 6.0, limp=[(3.0, 3.3, WHEEL_L)]),
       can_fault("driver timeout left knee limp 300 ms standing", [], 6.0, limp=[(3.0, 3.3, (2,))]),
       can_fault("driver timeout left leg limp 300 ms standing", [], 6.0, limp=[(3.0, 3.3, (0, 1, 2))]),
       can_fault("feedback stale 20 ms @0.6", drive(1.0, 0.6), 7.0, stale=[(4.0, 4.02)]),
       can_fault("feedback stale 50 ms @0.6", drive(1.0, 0.6), 7.0, stale=[(4.0, 4.05)])]
    + [imu_fault(f"IMU mounted {d:+d} deg off", imu_offset_deg=d) for d in (1, 2, 3, -3)]
    + [imu_fault("gyro bias drift 0.05 deg/s per s", gyro_drift=0.05),
       imu_fault("vibration noise @1.0 m/s", vib=(3.0, 0.5), speed=1.05)]
    + [fall_at_speed(v) for v in (0.0, 0.6, 1.8, 2.5)] + [wall(0.6), wall(1.2)]
    + [drop(h) for h in (3, 6, 10)] + [bad_landing("slope"), bad_landing("plate"), plate_edge]
    + [head_sweep, endurance(20)]
)


def _run_one(job):
    name, seed = job
    sc = next(s for s in SCENARIOS if s.__name__ == name)
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            return sc(seed)
        except Exception as e:   # a broken scenario must not hide the others
            return False, False, f"ERROR {type(e).__name__}: {e}"


def main():
    from concurrent.futures import ProcessPoolExecutor
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-k", default="", help="only run scenarios whose name contains this text")
    ap.add_argument("--realism", type=int, default=0, metavar="N", help="run each scenario on N randomized robots")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--report", action="store_true", help="also write reports/robustness.md")
    ap.add_argument("-j", type=int, default=0, help="parallel workers (default: all cores)")
    args = ap.parse_args()
    scenarios = [s for s in SCENARIOS if args.k in s.__name__]
    if args.list:
        print("\n".join(s.__name__ for s in scenarios))
        return
    seeds = list(range(args.realism)) if args.realism else [None]
    jobs = [(s.__name__, seed) for s in scenarios for seed in seeds]
    with ProcessPoolExecutor(args.j or None) as ex:
        flat = list(ex.map(_run_one, jobs))
    lines = []
    for k, sc in enumerate(scenarios):
        res = flat[k * len(seeds):(k + 1) * len(seeds)]
        if res[0][0] is None:
            for seed, (_, _, msg) in zip(seeds, res):
                lines.append(("REPORT", sc.__name__, msg + (f"  [seed {seed}]" if seed is not None else "")))
        elif seeds == [None]:
            ok, fell, msg = res[0]
            lines.append(("PASS" if ok else "FAIL", sc.__name__, ("(FELL) " if fell else "") + msg))
        else:
            n_ok, n_fell = sum(r[0] for r in res), sum(r[1] for r in res)
            msg = f"{n_ok}/{len(seeds)} pass, {n_fell} falls" + "".join(
                f"; seed {seed}: {m}" for seed, (ok, _, m) in zip(seeds, res) if not ok)
            lines.append(("PASS" if n_ok == len(seeds) else "FAIL", sc.__name__, msg))
    for tag, name, msg in lines:
        print(f"{tag:6s} {name:46s} {msg}")
    if args.report:
        n = {t: sum(1 for x in lines if x[0] == t) for t in ("PASS", "FAIL", "REPORT")}
        L = ["# Robustness Report", "",
             f"Generated by `test_robustness.py{' --realism ' + str(args.realism) if args.realism else ''}` on the "
             f"`{pathlib.Path(T.XML).name}` model{', ideal sensors' if not args.realism else ''}: "
             f"{n['PASS']} pass, {n['FAIL']} fail, {n['REPORT']} measurements.", "",
             "| Result | Scenario | Details |", "| :--- | :--- | :--- |"]
        L += [f"| {'✅' if t == 'PASS' else '❌' if t == 'FAIL' else '📏'} {t} | {nm} | {m} |" for t, nm, m in lines]
        out = ROOT / "reports" / "robustness.md"
        out.parent.mkdir(exist_ok=True)
        out.write_text("\n".join(L) + "\n")
        print(f"-> {out.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
