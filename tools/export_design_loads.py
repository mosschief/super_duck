#!/usr/bin/env python3
"""
Design loads for CAD: the force and moment every joint transmits (the load path through each motor's output bearing
and the printed brackets on both sides of it), from the scripted scenarios with ideal sensors.

Per joint and 2 ms step, from MuJoCo's body interaction forces (`cfrc_int`, what the parent applies to the child),
moved to the joint anchor and split along the joint axis. Fixed mounts (bodies welded to their parent, like the hip
mounts) are reported the same way at the body origin, with "axis" = the body's z axis (the bolting direction):
  F_axial  force along the axis        F_radial  force across it
  M_axis   moment about the axis (motor torque + joint friction/damping)   M_bend  moment across it (tilts the bearing)
Reported as peak and 99.5th percentile (the peak of a contact impact depends on simulated contact stiffness; use p99.5
for strength with a safety factor, and the peak as a sanity check). The fall scenario is indicative only: the shells
are boxes and the floor is rigid.

Writes cad/design_loads.json and cad/design_loads.md.
Usage: python3 tools/export_design_loads.py [--robot gim6010]
"""
import argparse
import contextlib
import io
import json
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import simulate_wheeled_duck as swd  # noqa: E402
import test_scenarios as T  # noqa: E402

# GIM4305-10 output bearing limits (cad/purchased_parts.json, listing). GIM6010-8: not published.
BEARING_LIMITS = {3.47: {"motor": "GIM4305-10", "axial_N": 125.0, "radial_N": 500.0}}
DRIVE_FIRST = [(1.0 + 0.01 * i, "up") for i in range(4)]
SCENARIOS = {
    "stand": ([], [], 6.0),
    "drive 0.6 m/s weaving": (T.DRIVE_EVENTS, [], 24.0),
    "crouch / stand cycles": ([(1.0 + 3.0 * k + 0.01 * i, "z" if k % 2 == 0 else "x") for k in range(6) for i in range(3)],
                              [], 20.0),
    "jump (standing)": ([(2.0, "c")], [], 5.0),
    "jump while driving": (DRIVE_FIRST + [(4.0, "c")], [], 7.0),
    "push +40 N": ([], [(2.0, 40.0)], 6.0),
    "push -40 N": ([], [(2.0, -40.0)], 6.0),
    "fall (push +90 N, indicative)": ([], [(2.0, 90.0)], 5.0),
}


def simulate(m, events, pushes, duration):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    ctl = swd.ScaledWheeledDuckController(T.PROFILE)
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    joints = [j for j in range(m.njnt) if m.jnt_type[j] == 3]
    fixed = [b for b in range(2, m.nbody) if m.body_jntnum[b] == 0 and m.body_mass[m.body_parentid[b]] > 0]
    events = sorted(events)
    rows, contact = [], []
    with contextlib.redirect_stdout(io.StringIO()):
        for _ in range(int(duration / m.opt.timestep)):
            while events and d.time >= events[0][0]:
                key = events.pop(0)[1]
                ctl.handle_keypress(T.KEYS[key] if key in T.KEYS else ord(key.upper()))
            d.xfrc_applied[base] = 0
            for t_push, force in pushes:
                if t_push <= d.time < t_push + 0.1:
                    d.xfrc_applied[base, 0] = force
            swd.step_robot(m, d, ctl)
            # mj_step's forward pass already ran mj_rnePostConstraint (the accelerometer needs it), so cfrc_int,
            # xanchor and subtree_com all describe the same state.
            r = []
            for j in joints:
                b = m.jnt_bodyid[j]
                f, tq = d.cfrc_int[b, 3:], d.cfrc_int[b, :3]
                p, a = d.xanchor[j], d.xaxis[j]
                tq_p = tq + np.cross(d.subtree_com[m.body_rootid[b]] - p, f)
                fa, ma = f @ a, tq_p @ a
                r.append((abs(fa), np.linalg.norm(f - fa * a), abs(ma), np.linalg.norm(tq_p - ma * a)))
            for b in fixed:
                f, tq = d.cfrc_int[b, 3:], d.cfrc_int[b, :3]
                p, a = d.xpos[b], d.xmat[b].reshape(3, 3)[:, 2]
                tq_p = tq + np.cross(d.subtree_com[m.body_rootid[b]] - p, f)
                fa, ma = f @ a, tq_p @ a
                r.append((abs(fa), np.linalg.norm(f - fa * a), abs(ma), np.linalg.norm(tq_p - ma * a)))
            rows.append(r)
            contact.append((float(d.sensor("left_wheel_touch").data[0]), float(d.sensor("right_wheel_touch").data[0])))
    labels = ([mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in joints]
              + [f"fixed: {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)}" for b in fixed])
    return labels, np.array(rows), np.array(contact)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT)
    args = ap.parse_args()
    T.PROFILE = swd.PROFILES[args.robot]
    xml = pathlib.Path(swd.profile_xml(T.PROFILE))
    m = mujoco.MjModel.from_xml_path(str(xml))
    act_of = {int(m.actuator_trnid[a, 0]): a for a in range(m.nu)}
    names = ("F_axial_N", "F_radial_N", "M_axis_Nm", "M_bend_Nm")

    out = {"model": xml.name, "weight_N": round(float(m.body_mass.sum()) * 9.81, 1), "scenarios": {}}
    for label, (events, pushes, dur) in SCENARIOS.items():
        labels, rows, contact = simulate(m, events, pushes, dur)
        sc = {"wheel_contact_peak_N": round(float(contact.max()), 1), "joints": {}}
        for k, jn in enumerate(labels):
            sc["joints"][jn] = {n: {"peak": round(float(rows[:, k, i].max()), 2),
                                    "p99.5": round(float(np.percentile(rows[:, k, i], 99.5)), 2)}
                                for i, n in enumerate(names)}
        out["scenarios"][label] = sc
        print(f"  {label}: wheel contact peak {sc['wheel_contact_peak_N']} N")

    # Envelope over the non-fall scenarios, plus bearing checks
    normal = [s for k, s in out["scenarios"].items() if not k.startswith("fall")]
    env = {}
    for jn in normal[0]["joints"]:
        e = {n: max(s["joints"][jn][n]["p99.5"] for s in normal) for n in names}
        e["peak"] = {n: max(s["joints"][jn][n]["peak"] for s in normal) for n in names}
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        a = act_of.get(j) if j >= 0 else None
        lim = BEARING_LIMITS.get(round(float(m.actuator_forcerange[a, 1]), 2)) if a is not None else None
        if a is not None and e["peak"]["M_axis_Nm"] > 1.05 * m.actuator_forcerange[a, 1]:
            # More moment about the axis than the actuator can make: a joint-limit hit, or the reflected rotor inertia
            # (armature) being stopped suddenly, which drives torque through the gearbox
            e["over_actuator"] = "hard stop or rotor-inertia shock" if m.jnt_limited[j] else "rotor-inertia shock"
        if lim:
            e["bearing"] = {"motor": lim["motor"], "axial_used": round(e["peak"]["F_axial_N"] / lim["axial_N"], 2),
                            "radial_used": round(e["peak"]["F_radial_N"] / lim["radial_N"], 2)}
        env[jn] = e
    out["design_envelope (non-fall scenarios)"] = env
    (ROOT / "cad" / "design_loads.json").write_text(json.dumps(out, indent=1) + "\n")

    L = [f"# Design Loads: `{xml.name}`", "",
         "Generated by `tools/export_design_loads.py` (ideal sensors). Loads each joint transmits from parent to child "
         "at its anchor: through the motor's output bearing and into the brackets either side. **F_axial / F_radial**: "
         "force along / across the joint axis. **M_axis**: moment about the axis (motor torque). **M_bend**: moment "
         "across the axis (tilts the bearing; what the bracket and the motor flange bolts carry).", "",
         f"Robot weight {out['weight_N']} N. Values are the **99.5th percentile** over the scenario (peak in brackets). "
         "For printed parts apply a safety factor of at least 2 on p99.5 (impacts, fatigue, layer adhesion), and check "
         "the peak too. Where the M_axis peak exceeds the actuator's own limit, the extra comes from the joint hitting its "
         "limit (the hard stop carries it) or from the motor's reflected rotor inertia being stopped suddenly (a "
         "shock load through the gearbox, e.g. a spinning wheel touching down).", "",
         "## Design envelope (all scenarios except the fall)", "",
         "| Joint | F_axial (N) | F_radial (N) | M_axis (N·m) | M_bend (N·m) | Output bearing (peak / limit) |",
         "| :--- | ---: | ---: | ---: | ---: | :--- |"]
    for jn, e in env.items():
        b = e.get("bearing")
        bear = (f"{b['motor']}: axial {100 * b['axial_used']:.0f}%, radial {100 * b['radial_used']:.0f}%"
                + (" **OVER**" if max(b["axial_used"], b["radial_used"]) > 1 else "")) if b else "limits not published"
        if jn.startswith("fixed: "):
            bear = "fixed mount: bracket + bolts to the parent (axis = mount z)"
        elif "antenna" in jn:
            bear = "micro servo"
        elif jn.startswith(("neck", "head")):
            bear = "STS3215 servo"
        if e.get("over_actuator"):
            bear += f"; **M_axis peak > actuator limit**: {e['over_actuator']}"
        L.append(f"| `{jn}` | " + " | ".join(f"{e[n]:.1f} ({e['peak'][n]:.1f})" if n[0] == "F" else
                                            f"{e[n]:.2f} ({e['peak'][n]:.2f})" for n in names) + f" | {bear} |")
    L += ["", "## Per scenario (p99.5)", ""]
    for label, sc in out["scenarios"].items():
        L += [f"### {label}", "", f"Wheel contact peak (touch sensor): {sc['wheel_contact_peak_N']} N", "",
              "| Joint | F_axial (N) | F_radial (N) | M_axis (N·m) | M_bend (N·m) |", "| :--- | ---: | ---: | ---: | ---: |"]
        for jn, v in sc["joints"].items():
            if "antenna" in jn:
                continue
            L.append(f"| `{jn}` | " + " | ".join(f"{v[n]['p99.5']:.1f}" if n[0] == "F" else f"{v[n]['p99.5']:.2f}"
                                                 for n in names) + " |")
        L.append("")
    (ROOT / "cad" / "design_loads.md").write_text("\n".join(L) + "\n")
    print("-> cad/design_loads.json, cad/design_loads.md")


if __name__ == "__main__":
    main()
