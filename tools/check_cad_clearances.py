#!/usr/bin/env python3
"""
Checks the MJCF's part envelopes (including visual-only motor canisters, PCBs and the battery) for interference over
the poses the robot actually reaches, before CAD.

  Operating poses (FAIL on interference): what the controller can command. Legs: crouch delta -0.46 (jump take-off)
    .. +0.24 (deepest crouch) x hip roll -0.30 / 0 / +0.30 rad (roll_comp clip); hip yaw is held at 0. Head: neck
    -0.30..0.90, head pitch / yaw / roll at +/-0.70 / 1.50 / 0.45 (controller clips) x antennas +/-45 deg.
  Full joint ranges (WARN): each joint swept to its MJCF limits with the rest at home. These need hard stops or
    tighter limits if they interfere.

Pairs: geoms on the same body are skipped. Parent/child pairs overlap by design where a motor mounts to its bracket,
so they only count if the overlap grows by more than 1 mm beyond the home pose. Any other pair counts if it
penetrates; under 3 mm of gap is reported as tight.

Geoms are either real-part envelopes (motors, tires, PCBs, battery, servos, power parts: their sizes are locked to
cad/purchased_parts.json) or stand-ins for printed parts (shells, brackets, links) that CAD will redesign. A clash
between two real parts is a layout error (FAIL). A clash involving a stand-in is a CAD constraint: the printed part
must be shaped to clear that motion. Also reports ground clearance, the hip-roll scrub torque, and whether model
envelopes match cad/purchased_parts.json.

Writes cad/clearance_report.md. Exit code 1 if two real parts interfere in an operating pose.
Usage: python3 tools/check_cad_clearances.py [--robot gim6010]
"""
import argparse
import itertools
import json
import math
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import simulate_wheeled_duck as swd  # noqa: E402

TIGHT = 0.003      # m: report gaps below this
GROWTH = 0.001     # m: parent/child overlap allowed to grow beyond home
TIRES = ("left_foot_bottom_tpu", "right_foot_bottom_tpu")
WHEELS = ("foot_assembly", "foot_assembly_2")
REAL_PARTS = ("_gim4305", "_gim6010", "_foot_bottom_tpu", "battery_pack_6s", "raspberry_pi_5", "ai_hat_plus_26tops",
              "teensy_40_dual_can", "_rpi_cam3_pcb", "_servo_9g", "speaker_40mm_3w", "max98357a_i2s_amp", "bms_6s_100a",
              "main_fuse_40a", "antispark_switch_120a", "peripheral_5v_buck", "estop_", "microphone_inmp441")


def leg_pose(delta, roll=0.0):
    q = {}
    for side in ("left", "right"):
        q[f"{side}_hip_pitch"] = -0.50 - delta
        q[f"{side}_knee"] = 1.00 + 2.0 * delta
        q[f"{side}_hip_roll"] = roll
    return q


def operating_poses():
    for delta, roll in itertools.product((-0.46, -0.25, 0.0, 0.12, 0.24), (-0.30, 0.0, 0.30)):
        yield f"legs delta {delta:+.2f}, hip_roll {roll:+.2f}", leg_pose(delta, roll)
    for neck, hp, hy, hr, ant in itertools.product((-0.30, 0.15, 0.90), (-0.70, 0.0, 0.70), (-1.50, 0.0, 1.50),
                                                    (-0.45, 0.0, 0.45), (-0.785, 0.785)):
        yield (f"head neck {neck:+.2f} pitch {hp:+.2f} yaw {hy:+.2f} roll {hr:+.2f} antennas {ant:+.2f}",
               {"neck_pitch": neck, "head_pitch": hp, "head_yaw": hy, "head_roll": hr,
                "left_antenna": ant, "right_antenna": ant})


def range_poses(m):
    for j in range(m.njnt):
        if m.jnt_type[j] != 3 or not m.jnt_limited[j]:
            continue
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        for q in np.linspace(*m.jnt_range[j], 9):
            yield f"{n} {math.degrees(q):+.0f} deg", {n: float(q)}


class Checker:
    def __init__(self, m):
        self.m, self.d = m, mujoco.MjData(m)
        self.geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0 and m.geom_type[g] != 0]
        self.gname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        par = m.body_parentid
        self.pairs = []
        for a, b in itertools.combinations(self.geoms, 2):
            ba, bb = m.geom_bodyid[a], m.geom_bodyid[b]
            if ba != bb:
                self.pairs.append((a, b, par[ba] == bb or par[bb] == ba))
        self.pa = np.array([p[0] for p in self.pairs])
        self.pb = np.array([p[1] for p in self.pairs])
        self.set_pose({})
        self.home = {(a, b): d for a, b, d in self.distances()}

    def set_pose(self, q):
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        for n, v in q.items():
            self.d.joint(n).qpos[0] = v
        mujoco.mj_kinematics(self.m, self.d)

    def distances(self, cutoff=0.02):
        m, d = self.m, self.d
        gap = (np.linalg.norm(d.geom_xpos[self.pa] - d.geom_xpos[self.pb], axis=1)
               - m.geom_rbound[self.pa] - m.geom_rbound[self.pb])
        out = []
        for i in np.nonzero(gap < cutoff)[0]:
            a, b = int(self.pa[i]), int(self.pb[i])
            out.append((a, b, mujoco.mj_geomDistance(m, d, a, b, cutoff, None)))
        return out

    def check(self, poses):
        """Returns {pair: (worst metric, pose, kind)} for interferences and tight gaps."""
        hits = {}
        for label, q in poses:
            self.set_pose(q)
            for a, b, dist in self.distances():
                adjacent = self.pairs_adj(a, b)
                if adjacent:
                    base = min(self.home.get((a, b), 0.02), 0.0)
                    if dist < base - GROWTH:
                        self._keep(hits, (a, b), dist - base, label, "parent/child overlap grows")
                elif dist < 0:
                    self._keep(hits, (a, b), dist, label, "interference")
                elif dist < TIGHT:
                    self._keep(hits, (a, b), dist, label, "tight")
        return hits

    def pairs_adj(self, a, b):
        par = self.m.body_parentid
        ba, bb = self.m.geom_bodyid[a], self.m.geom_bodyid[b]
        return par[ba] == bb or par[bb] == ba

    @staticmethod
    def _keep(hits, key, metric, label, kind):
        if key not in hits or metric < hits[key][0]:
            hits[key] = (metric, label, kind)

    def ground_clearance(self, poses):
        m, d = self.m, self.d
        tires = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in TIRES]
        wheels = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) for n in WHEELS}
        worst = (1.0, "", "")
        for label, q in poses:
            self.set_pose(q)
            floor = min(d.geom_xpos[t][2] - m.geom_size[t][0] for t in tires)
            for g in self.geoms:
                if m.geom_bodyid[g] in wheels:
                    continue
                c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
                corners = np.array(list(itertools.product((-1, 1), repeat=3))) * h + c
                z = float((corners @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min()) - floor
                if z < worst[0]:
                    worst = (z, self.gname(g), label)
        return worst


SHELLS = {"trunk_assembly": "trunk_collision", "head_assembly": "head_collision"}
ONE_PART = {frozenset(("estop_bezel", "estop_mushroom"))}  # several geoms modelling one part


def packing_checks(c, real):
    """Home pose, same body: real parts overlapping each other, and real parts sticking out of the body's shell box."""
    m, d = c.m, c.d
    c.set_pose({})
    clashes, protrude = [], []
    by_body = {}
    for g in c.geoms:
        if real(g):
            by_body.setdefault(int(m.geom_bodyid[g]), []).append(g)
    for body, gs in by_body.items():
        for a, b in itertools.combinations(gs, 2):
            if frozenset((c.gname(a), c.gname(b))) in ONE_PART:
                continue
            dist = mujoco.mj_geomDistance(m, d, a, b, 0.02, None)
            if dist < TIGHT:
                clashes.append((c.gname(a), c.gname(b), dist))
        shell = SHELLS.get(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, body))
        if shell is None:
            continue
        sh = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, shell)
        half, R_s, p_s = m.geom_size[sh], d.geom_xmat[sh].reshape(3, 3), d.geom_xpos[sh]
        for g in gs:
            ctr, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
            corners = np.array(list(itertools.product((-1, 1), repeat=3))) * h + ctr
            world = corners @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            local = (world - p_s) @ R_s
            out = np.max(np.abs(local) - half, axis=0)
            if out.max() > 0:
                axis = "xyz"[int(np.argmax(out))]
                protrude.append((c.gname(g), shell, float(out.max()), axis))
    return clashes, protrude


def envelope_checks(m):
    parts = json.loads((ROOT / "cad" / "purchased_parts.json").read_text())["parts"]
    rows = []
    for p in parts:
        mc = p.get("model_check")
        if not mc:
            continue
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, mc["geom"])
        if g < 0:
            rows.append((p["name"], mc["geom"], "-", mc["dims_mm"], "geom missing"))
            continue
        s = m.geom_size[g] * 1000
        model = sorted([2 * s[0], 2 * s[1], 2 * s[2]] if m.geom_type[g] == 6 else [2 * s[0], 2 * s[1]])
        real = sorted(mc["dims_mm"])
        ok = all(abs(x - y) <= 1.5 for x, y in zip(model, real))
        rows.append((p["name"], mc["geom"], ", ".join(f"{x:g}" for x in np.round(model, 1)),
                     ", ".join(f"{x:g}" for x in real), "ok" if ok else "MISMATCH"))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT)
    args = ap.parse_args()
    xml = pathlib.Path(swd.profile_xml(swd.PROFILES[args.robot]))
    m = mujoco.MjModel.from_xml_path(str(xml))
    c = Checker(m)

    real = lambda g: any(t in c.gname(g) for t in REAL_PARTS)
    op = c.check(operating_poses())
    full = c.check(range_poses(m))
    full = {k: v for k, v in full.items() if k not in op}
    op_real = {k: v for k, v in op.items() if real(k[0]) and real(k[1])}
    op_cad = {k: v for k, v in op.items() if k not in op_real}
    leg_ops = [p for p in operating_poses() if p[0].startswith("legs")]
    clear = c.ground_clearance(leg_ops)

    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    wheel_y = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "foot_assembly")][1]
    roll_y = d.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_hip_roll")][1]
    scrub = wheel_y - roll_y
    roll_torque = 0.5 * m.body_mass.sum() * 9.81 * abs(scrub)
    env = envelope_checks(m)
    clashes, protrude = packing_checks(c, real)

    def table(hits, title):
        L = [f"## {title}", ""]
        if not hits:
            return L + ["None.", ""]
        L += ["| Geom A | Geom B | Kind | Worst (mm) | Pose |", "| :--- | :--- | :--- | ---: | :--- |"]
        tag = lambda g: f"`{c.gname(g)}`" + ("" if real(g) else " *(stand-in)*")
        for (a, b), (metric, label, kind) in sorted(hits.items(), key=lambda kv: kv[1][0]):
            L.append(f"| {tag(a)} | {tag(b)} | {kind} | {metric * 1000:+.1f} | {label} |")
        return L + [""]

    n_op_fail = sum(1 for v in op_real.values() if v[2] != "tight")
    n_cad = sum(1 for v in op_cad.values() if v[2] != "tight")
    L = [f"# Clearance Report: `{xml.name}`", "",
         "Generated by `tools/check_cad_clearances.py`. Negative = penetration; for parent/child pairs, how much the "
         "overlap grew beyond the home pose.", "",
         f"- Operating poses, real part vs real part: **{n_op_fail} interferences** (layout errors), "
         f"{len(op_real) - n_op_fail} tight gaps (< {TIGHT * 1000:.0f} mm)",
         f"- Operating poses involving a stand-in: **{n_cad} CAD constraints** (printed parts must clear these motions), "
         f"{len(op_cad) - n_cad} tight",
         f"- Full joint ranges: {sum(1 for v in full.values() if v[2] != 'tight')} more (need hard stops or tighter limits)",
         f"- Ground clearance (lowest part not on a wheel, operating leg poses): **{clear[0] * 1000:.1f} mm** (`{clear[1]}`, {clear[2]})",
         f"- Hip-roll scrub offset {scrub * 1000:+.1f} mm -> standing roll torque {roll_torque:.2f} N·m per hip roll motor",
         f"- Packing (home pose, same body): **{sum(1 for x in clashes if x[2] < 0)} real-part overlaps**, "
         f"{sum(1 for x in clashes if x[2] >= 0)} tight; {len(protrude)} parts stick out of their shell box",
         f"- Model envelopes vs real parts: {sum(1 for r in env if r[4] != 'ok')} mismatches", ""]
    L += table(op_real, "Layout errors: real parts interfering in operating poses")
    L += table(op_cad, "CAD constraints: operating-pose clashes involving stand-in shapes")
    L += table(full, "Full joint ranges only (hard stops or tighter limits)")
    L += ["## Packing inside the trunk and head (home pose)", ""]
    if clashes:
        L += ["| Part A | Part B | Gap (mm) |", "| :--- | :--- | ---: |"]
        L += [f"| `{a}` | `{b}` | {dist * 1000:+.1f} |" for a, b, dist in sorted(clashes, key=lambda x: x[2])]
    else:
        L.append("No real-part overlaps.")
    L += ["", "Parts sticking out of the shell stand-in box (the printed shell must enclose them, or they mount through it):", ""]
    L += ["| Part | Shell box | Protrudes (mm) | Along |", "| :--- | :--- | ---: | :--- |"]
    L += [f"| `{g}` | `{sh}` | {out * 1000:.1f} | {ax} |" for g, sh, out, ax in sorted(protrude, key=lambda x: -x[2])]
    L += ["", "## Model envelopes vs `purchased_parts.json`", "",
          "| Part | Geom | Model (mm, sorted) | Real (mm, sorted) | Result |", "| :--- | :--- | :--- | :--- | :--- |"]
    L += [f"| {n} | `{g}` | {mo} | {re} | {r} |" for n, g, mo, re, r in env]
    (ROOT / "cad" / "clearance_report.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[4:12]))
    print("-> cad/clearance_report.md")
    sys.exit(1 if n_op_fail or any(x[2] < 0 for x in clashes) else 0)


if __name__ == "__main__":
    main()
