#!/usr/bin/env python3
"""
FEA load cases for the printed leg parts: the wrench each part takes at its loaded interface, as a vector in the
part's own body frame (the frame of cad/freecad/printed/<part>.step), at the worst instants of the scenarios that
load the legs hardest (landings, drops, thresholds, pushes, fast turns; falls excluded: indicative only).

Each part is held at the interface towards the trunk and loaded at the one towards the wheel:
  hip mount   held: trunk floor bolts            loaded: hip-roll GIM4305 housing (the roll joint's wrench)
  hip yoke    held: hip-roll GIM4305 output      loaded: hip-pitch GIM6010 housing (the hip-pitch joint)
  thigh       held: hip-pitch GIM6010 output     loaded: knee GIM6010 housing (the knee joint)
  shin        held: knee GIM6010 output          loaded: wheel GIM4305 housing (the wheel joint)
  wheel hub   held: wheel GIM4305 output         loaded: the tire's ground contact (on the rim)
The right leg's wrenches are mirrored into the left parts' frames, so every case is a left-part case.

Writes cad/fea_loads.json (cases per part: the instants of peak force and of peak moment, per scenario).
Usage: python3 tools/export_fea_loads.py
"""
import contextlib
import io
import json
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import test_robustness as R  # noqa: E402
import test_scenarios as T  # noqa: E402

PARTS = {  # part: (its body (left, right), joint to the child (left, right))
    "L_hip_mount": (("hip_roll_assembly", "hip_roll_assembly_2"), ("left_hip_roll", "right_hip_roll")),
    "L_hip_roll_yoke": (("left_roll_to_pitch_assembly", "right_roll_to_pitch_assembly"),
                        ("left_hip_pitch", "right_hip_pitch")),
    "L_thigh": (("knee_and_ankle_assembly", "knee_and_ankle_assembly_3"), ("left_knee", "right_knee")),
    "L_shin": (("knee_and_ankle_assembly_2", "knee_and_ankle_assembly_4"), ("left_wheel", "right_wheel")),
}
TIRES = ("left_foot_bottom_tpu", "right_foot_bottom_tpu")
FOOT = ("foot_assembly", "foot_assembly_2")
SCENARIOS = [T.jump, T.jump_driving, T.push(40), T.push(-40), T.crouch_stand, T.drive_turn_brake,
             R.drop(6), R.drop(10), R.bad_landing("plate"), R.bad_landing("slope"), R.threshold(15),
             R.threshold(15, v=1.0), R.threshold(20, side="left"), R.uneven(20), R.turn_at_speed(1.2, 45),
             R.threshold(10, curb=True)]
MIRROR_F = np.array([1.0, -1.0, 1.0])    # a vector mirrored in y
MIRROR_M = np.array([-1.0, 1.0, -1.0])   # a moment (pseudo-vector) mirrored in y


def capture():
    """Hook recording, per part and leg, the wrench on the part at its loaded interface (part frame, mm / N / N*m)."""
    rec = {k: [] for k in list(PARTS) + ["L_wheel_hub"]}
    ids = {}

    def hook(m, d, ctl):
        if not ids:
            for part, (bodies, joints) in PARTS.items():
                ids[part] = [(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b),
                              mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)) for b, j in zip(bodies, joints)]
            ids["tire"] = [(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g),
                            mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b)) for g, b in zip(TIRES, FOOT)]
        if d.time < 0.02:   # the keyframe's first contact settling
            return
        for part, pairs in ids.items():
            if part == "tire":
                continue
            for side, (pb, j) in enumerate(pairs):
                cb = m.jnt_bodyid[j]
                f, tq = d.cfrc_int[cb, 3:], d.cfrc_int[cb, :3]
                p = d.xanchor[j]
                tq_p = tq + np.cross(d.subtree_com[m.body_rootid[cb]] - p, f)
                Rm = d.xmat[pb].reshape(3, 3)
                F, M = Rm.T @ -f, Rm.T @ -tq_p   # what the child applies to this part, part frame
                pt = Rm.T @ (p - d.xpos[pb]) * 1000
                if side:
                    F, M, pt = F * MIRROR_F, M * MIRROR_M, pt * MIRROR_F
                rec[part].append((d.time, F, M, pt))
        cf = np.zeros(6)
        tot = [[np.zeros(3), np.zeros(3), 0.0], [np.zeros(3), np.zeros(3), 0.0]]
        for i in range(d.ncon):
            c = d.contact[i]
            for side, (g, fb) in enumerate(ids["tire"]):
                if g in (c.geom1, c.geom2):
                    mujoco.mj_contactForce(m, d, i, cf)
                    fw = c.frame.reshape(3, 3).T @ cf[:3]          # contact frame -> world (force on geom2)
                    if c.geom1 == g:
                        fw = -fw
                    tot[side][0] += fw
                    tot[side][1] += np.asarray(c.pos) * np.linalg.norm(fw)
                    tot[side][2] += np.linalg.norm(fw)
        for side, (fw, pw, w) in enumerate(tot):
            if w <= 0:
                continue
            fb = ids["tire"][side][1]
            Rm = d.xmat[fb].reshape(3, 3)
            F = Rm.T @ fw
            pt = Rm.T @ (pw / w - d.xpos[fb]) * 1000
            if side:
                F, pt = F * MIRROR_F, pt * MIRROR_F
            rec["L_wheel_hub"].append((d.time, F, np.zeros(3), pt))
    return rec, hook


def main():
    cases = {k: [] for k in list(PARTS) + ["L_wheel_hub"]}
    for sc in SCENARIOS:
        rec, hook = capture()
        R.EXTRA_HOOK = hook
        with contextlib.redirect_stdout(io.StringIO()):
            ok, fell, msg = sc(None)
        R.EXTRA_HOOK = None
        for part, rows in rec.items():
            if not rows:
                continue
            fn = np.array([np.linalg.norm(r[1]) for r in rows])
            mn = np.array([np.linalg.norm(r[2]) for r in rows])
            for kind, i in (("peak force", int(fn.argmax())), ("peak moment", int(mn.argmax()))):
                t, F, M, pt = rows[i]
                cases[part].append({"scenario": sc.__name__, "kind": kind, "t": round(float(t), 3),
                                    "fell": bool(fell), "F_N": [round(float(x), 2) for x in F],
                                    "M_Nm": [round(float(x), 3) for x in M], "at_mm": [round(float(x), 1) for x in pt]})
        print(f"{sc.__name__:40s} {'(fell) ' if fell else ''}"
              + ", ".join(f"{p} {max(np.linalg.norm(r[1]) for r in rows):.0f} N" for p, rows in rec.items() if rows))
    out = {"_about": __doc__.strip().split("\n\n")[0], "frame": "each part's body frame (printed/<part>.step), mm",
           "cases": cases}
    (ROOT / "cad" / "fea_loads.json").write_text(json.dumps(out, indent=1) + "\n")
    print("-> cad/fea_loads.json")


if __name__ == "__main__":
    main()
