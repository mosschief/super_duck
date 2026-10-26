#!/usr/bin/env python3
"""
Checks the sim's contact geometry (the CAD parts' convex hulls, exported by cad/freecad/export_sim.py) for self-contact
over the poses the robot actually reaches: these are the contacts the sim will produce.

  Operating poses: what the controller can command. Legs: crouch delta -0.46 (jump take-off) .. +0.24 (deepest crouch)
    x hip roll -0.30 / 0 / +0.30 rad (roll_comp clip): FAIL on any contact. Head: neck -0.30..0.90, head pitch / yaw /
    roll at +/-0.70 / 1.50 / 0.45 (controller clips) x antennas +/-45 deg: head / trunk contacts outside the neck
    envelope are expected (cad/freecad/neck_envelope.md) until the controller clips head commands to it.
  Full joint ranges (WARN): each joint swept to its MJCF limits with the rest at home.

Pairs MuJoCo collides: different bodies, not parent / child, not in the MJCF's <contact><exclude>. Hulls fill
concave parts, so a hull contact the exact CAD sweeps (sweep_legs.py, sweep_neck.py) don't show is a hull artefact:
exclude that body pair in export_sim.py. Also reports ground clearance and the hip-roll scrub torque. Packing and
part envelopes are checked on the exact CAD (cad/freecad/fit_report.md).

Writes cad/clearance_report.md. Exit code 1 on a contact in an operating leg pose.
Usage: python3 tools/check_cad_clearances.py [--robot gim6010]
"""
import argparse
import itertools
import math
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import simulate_wheeled_duck as swd  # noqa: E402

TIGHT = 0.003      # m: report gaps below this
TIRES = ("left_foot_bottom_tpu", "right_foot_bottom_tpu")
WHEELS = ("foot_assembly", "foot_assembly_2")


def leg_pose(delta, roll=0.0):
    q = {}
    for side in ("left", "right"):
        q[f"{side}_hip_pitch"] = 0.50 + delta
        q[f"{side}_knee"] = -1.00 - 2.0 * delta
        q[f"{side}_hip_roll"] = roll
    return q


def leg_poses():
    for delta, roll in itertools.product((-0.46, -0.25, 0.0, 0.12, 0.24), (-0.30, 0.0, 0.30)):
        yield f"legs delta {delta:+.2f}, hip_roll {roll:+.2f}", leg_pose(delta, roll)


def head_poses():
    for neck, hp, hy, hr, ant in itertools.product((-0.30, 0.15, 0.90), (-0.70, 0.0, 0.70), (-1.50, 0.0, 1.50),
                                                    (-0.45, 0.0, 0.45), (-0.785, 0.785)):
        yield (f"head neck {neck:+.2f} pitch {hp:+.2f} yaw {hy:+.2f} roll {hr:+.2f} antennas {ant:+.2f}",
               {"neck_pitch": neck, "head_pitch": hp, "head_yaw": hy, "head_roll": hr,
                "left_antenna": ant, "right_antenna": ant})


def operating_poses():
    yield from leg_poses()
    yield from head_poses()


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
        self.gname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        self.bname = lambda b: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        self.geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] != 0]
        coll = [g for g in self.geoms if m.geom_contype[g] or m.geom_conaffinity[g]]
        par = m.body_parentid
        excluded = {int(s) for s in m.exclude_signature[:m.nexclude]}
        self.pairs = []
        for a, b in itertools.combinations(coll, 2):
            ba, bb = int(m.geom_bodyid[a]), int(m.geom_bodyid[b])
            if ba == bb or par[ba] == bb or par[bb] == ba:
                continue
            if ((min(ba, bb) << 16) + max(ba, bb)) in excluded:
                continue
            if not (m.geom_contype[a] & m.geom_conaffinity[b] or m.geom_contype[b] & m.geom_conaffinity[a]):
                continue
            self.pairs.append((a, b))
        self.pa = np.array([p[0] for p in self.pairs])
        self.pb = np.array([p[1] for p in self.pairs])

    def set_pose(self, q):
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        for n, v in q.items():
            self.d.joint(n).qpos[0] = v
        mujoco.mj_kinematics(self.m, self.d)

    def check(self, poses):
        """Returns {pair: (worst distance, pose)} for contacts and gaps under TIGHT."""
        m, d = self.m, self.d
        hits = {}
        for label, q in poses:
            self.set_pose(q)
            gap = (np.linalg.norm(d.geom_xpos[self.pa] - d.geom_xpos[self.pb], axis=1)
                   - m.geom_rbound[self.pa] - m.geom_rbound[self.pb])
            for i in np.nonzero(gap < TIGHT)[0]:
                a, b = int(self.pa[i]), int(self.pb[i])
                dist = mujoco.mj_geomDistance(m, d, a, b, TIGHT, None)
                if dist < TIGHT and ((a, b) not in hits or dist < hits[(a, b)][0]):
                    hits[(a, b)] = (dist, label)
        return hits

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
                if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                    mid = m.geom_dataid[g]
                    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                else:
                    c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
                    v = np.array(list(itertools.product((-1, 1), repeat=3))) * h + c
                z = float((v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min()) - floor
                if z < worst[0]:
                    worst = (z, self.gname(g), label)
        return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT)
    args = ap.parse_args()
    xml = pathlib.Path(swd.profile_xml(swd.PROFILES[args.robot]))
    m = mujoco.MjModel.from_xml_path(str(xml))
    c = Checker(m)

    legs = c.check(leg_poses())
    head = c.check(head_poses())
    full = {k: v for k, v in c.check(range_poses(m)).items() if k not in legs and k not in head}
    clear = c.ground_clearance(list(leg_poses()))

    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    wheel_y = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "foot_assembly")][1]
    roll_y = d.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "left_hip_roll")][1]
    scrub = wheel_y - roll_y
    roll_torque = 0.5 * m.body_mass.sum() * 9.81 * abs(scrub)

    def table(hits, title):
        L = [f"## {title}", ""]
        if not hits:
            return L + ["None.", ""]
        L += ["| Geom A | Geom B | Worst (mm) | Pose |", "| :--- | :--- | ---: | :--- |"]
        for (a, b), (dist, label) in sorted(hits.items(), key=lambda kv: kv[1][0]):
            L.append(f"| `{c.gname(a)}` | `{c.gname(b)}` | {dist * 1000:+.1f} | {label} |")
        return L + [""]

    n = lambda hits: sum(1 for v in hits.values() if v[0] < 0)
    excl = [f"`{c.bname(int(s) >> 16)}` / `{c.bname(int(s) & 0xFFFF)}`" for s in m.exclude_signature[:m.nexclude]]
    L = [f"# Clearance Report: `{xml.name}`", "",
         "Generated by `tools/check_cad_clearances.py` on the CAD's collision hulls (`cad/freecad/export_sim.py`). "
         "Negative = penetration, i.e. a contact the sim will produce.", "",
         f"- Operating leg poses: **{n(legs)} contacts**, {len(legs) - n(legs)} gaps under {TIGHT * 1000:.0f} mm",
         f"- Operating head poses (controller clip ranges): {n(head)} contacts (expected outside the neck envelope, "
         "`cad/freecad/neck_envelope.md`)",
         f"- Full joint ranges: {n(full)} more contacts",
         f"- Ground clearance (lowest part not on a wheel, operating leg poses): **{clear[0] * 1000:.1f} mm** "
         f"(`{clear[1]}`, {clear[2]})",
         f"- Hip-roll scrub offset {scrub * 1000:+.1f} mm -> standing roll torque {roll_torque:.2f} N·m per hip roll motor",
         f"- {len(c.pairs)} geom pairs collide; body pairs excluded in the MJCF: {', '.join(excl) or 'none'}", ""]
    L += table(legs, "Operating leg poses")
    L += table(head, "Operating head poses")
    L += table(full, "Full joint ranges only")
    (ROOT / "cad" / "clearance_report.md").write_text("\n".join(L) + "\n")
    print("\n".join(L[4:11]))
    print("-> cad/clearance_report.md")
    sys.exit(1 if n(legs) else 0)


if __name__ == "__main__":
    main()
