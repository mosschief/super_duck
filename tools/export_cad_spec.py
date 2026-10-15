#!/usr/bin/env python3
"""
Exports the CAD spec from an MJCF: every joint (anchor, axis, range, actuator), body frame, part placement (geoms),
camera and light, in millimetres, plus the derived key dimensions CAD must hold (link lengths, track, scrub offset,
crouch heights). Also renders cad/purchased_parts.md from cad/purchased_parts.json.

Frames: world at the `home_balanced` keyframe, floor at z = 0, x forward, y left, z up. "local" values are in the
parent body's frame, as in the MJCF. The model is the source of truth: change it, re-run this, then update CAD.

Writes cad/hardpoints.json, cad/hardpoints.md, cad/purchased_parts.md.
Usage: python3 tools/export_cad_spec.py [--robot gim6010]
"""
import argparse
import json
import math
import pathlib
import sys

import mujoco
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import simulate_wheeled_duck as swd  # noqa: E402

CAD = ROOT / "cad"
GEOM_TYPES = {0: "plane", 2: "sphere", 3: "capsule", 5: "cylinder", 6: "box"}
JOINT_TYPES = {0: "free", 3: "hinge"}


def mm(v):
    return [round(float(x) * 1000, 1) for x in np.atleast_1d(v)]


def name(m, obj, i):
    return mujoco.mj_id2name(m, obj, i) or f"#{i}"


def geom_dims(m, g):
    """Full dimensions (mm): box x/y/z, cylinder/capsule [diameter, length], sphere [diameter]."""
    s, t = m.geom_size[g], m.geom_type[g]
    if t == 6:
        return mm(2 * s)
    if t in (3, 5):
        return mm([2 * s[0], 2 * s[1]])
    if t == 2:
        return mm([2 * s[0]])
    return mm(s)


def pose_legs(m, d, delta, key=0):
    """Sets the keyframe with both legs at crouch `delta` (controller kinematics) and runs kinematics."""
    mujoco.mj_resetDataKeyframe(m, d, key)
    for side in ("left", "right"):
        d.joint(f"{side}_hip_pitch").qpos[0] = -0.50 - delta
        d.joint(f"{side}_knee").qpos[0] = 1.00 + 2.0 * delta
    mujoco.mj_forward(m, d)  # kinematics + camera/light poses


def geom_top(m, d, g):
    """Highest world z of a geom's oriented bounding box."""
    c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    return float((corners @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].max())


def key_dimensions(m, d):
    jt = lambda n: d.xanchor[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    body = lambda n: d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)]
    geom = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    pose_legs(m, d, 0.0)
    lw, rw = body("foot_assembly"), body("foot_assembly_2")
    tire = geom("left_foot_bottom_tpu")
    vis = [g for g in range(m.ngeom) if m.geom_type[g] != 0]
    zmax = max(geom_top(m, d, g) for g in vis)
    out = {
        "wheel_radius": mm(m.geom_size[tire][0])[0],
        "tire_width": mm(2 * m.geom_size[tire][1])[0],
        "wheel_track (wheel centre to centre)": mm(lw[1] - rw[1])[0],
        "hip_yaw axes, lateral spacing": mm(jt("left_hip_yaw")[1] - jt("right_hip_yaw")[1])[0],
        "thigh length (hip_pitch axis to knee axis)": mm(np.linalg.norm(jt("left_knee") - jt("left_hip_pitch")))[0],
        "shin length (knee axis to wheel axle, in the leg plane)": mm(np.linalg.norm((lw - jt("left_knee"))[[0, 2]]))[0],
        "wheel axle lateral offset from the knee axis origin (inboard routing)": mm(lw[1] - jt("left_knee")[1])[0],
        "hip_roll axis to wheel centre, lateral (scrub offset)": mm(lw[1] - jt("left_hip_roll")[1])[0],
        "hip_roll axis height": mm(jt("left_hip_roll")[2])[0],
        "base origin height (home)": mm(d.xpos[1][2])[0],
        "overall height (home)": mm(zmax)[0],
        "eye camera baseline": mm(np.linalg.norm(
            d.cam_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "left_eye_cam")]
            - d.cam_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "right_eye_cam")]))[0],
        "IMU site (base frame)": mm(m.site_pos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "imu")]),
    }
    crouch = {}
    for label, delta in (("jump take-off (delta -0.46)", -0.46), ("tallest drive (delta -0.25)", -0.25),
                         ("home (delta 0)", 0.0), ("deepest crouch / jump wind-up (delta +0.24)", 0.24)):
        pose_legs(m, d, delta)
        crouch[label] = {"hip_pitch_deg": round(math.degrees(-0.50 - delta), 1),
                         "knee_deg": round(math.degrees(1.00 + 2 * delta), 1),
                         "hip_pitch axis above wheel axle": mm(jt("left_hip_pitch")[2] - body("foot_assembly")[2])[0],
                         "wheel axle fore/aft of hip_pitch axis": mm(body("foot_assembly")[0] - jt("left_hip_pitch")[0])[0]}
    out["leg poses (hip = -0.50 - delta, knee = 1.00 + 2*delta)"] = crouch
    return out


def export(m, d, xml):
    pose_legs(m, d, 0.0)
    act_of = {int(m.actuator_trnid[a, 0]): a for a in range(m.nu)}
    joints = []
    for j in range(m.njnt):
        if m.jnt_type[j] == 0:
            continue
        b = m.jnt_bodyid[j]
        a = act_of.get(j)
        joints.append({
            "name": name(m, mujoco.mjtObj.mjOBJ_JOINT, j), "type": JOINT_TYPES.get(int(m.jnt_type[j]), "?"),
            "body": name(m, mujoco.mjtObj.mjOBJ_BODY, b), "parent": name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[b]),
            "anchor_local_mm": mm(m.jnt_pos[j]), "axis_local": [round(float(x), 4) for x in m.jnt_axis[j]],
            "anchor_world_mm": mm(d.xanchor[j]), "axis_world": [round(float(x), 4) for x in d.xaxis[j]],
            "range_deg": ([round(math.degrees(x), 1) for x in m.jnt_range[j]] if m.jnt_limited[j] else None),
            "actuator": name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) if a is not None else None,
            "force_limit_Nm": round(float(m.actuator_forcerange[a, 1]), 2) if a is not None else None,
        })
    bodies = [{
        "name": name(m, mujoco.mjtObj.mjOBJ_BODY, b), "parent": name(m, mujoco.mjtObj.mjOBJ_BODY, m.body_parentid[b]),
        "pos_local_mm": mm(m.body_pos[b]), "quat_local_wxyz": [round(float(x), 5) for x in m.body_quat[b]],
        "origin_world_mm": mm(d.xpos[b]), "mass_g": round(float(m.body_mass[b]) * 1000, 1),
        "com_local_mm": mm(m.body_ipos[b]),
    } for b in range(1, m.nbody)]
    geoms = [{
        "name": name(m, mujoco.mjtObj.mjOBJ_GEOM, g), "body": name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]),
        "type": GEOM_TYPES.get(int(m.geom_type[g]), str(m.geom_type[g])), "dims_mm": geom_dims(m, g),
        "pos_local_mm": mm(m.geom_pos[g]), "quat_local_wxyz": [round(float(x), 5) for x in m.geom_quat[g]],
        "centre_world_mm": mm(d.geom_xpos[g]),
        "role": "collision" if m.geom_contype[g] or m.geom_conaffinity[g] else "visual",
    } for g in range(m.ngeom) if m.geom_bodyid[g] != 0]
    cams = [{"name": name(m, mujoco.mjtObj.mjOBJ_CAMERA, c), "body": name(m, mujoco.mjtObj.mjOBJ_BODY, m.cam_bodyid[c]),
             "pos_local_mm": mm(m.cam_pos[c]), "quat_local_wxyz": [round(float(x), 5) for x in m.cam_quat[c]],
             "pos_world_mm": mm(d.cam_xpos[c]), "fovy_deg": round(float(m.cam_fovy[c]), 1)} for c in range(m.ncam)]
    sites = [{"name": name(m, mujoco.mjtObj.mjOBJ_SITE, s), "body": name(m, mujoco.mjtObj.mjOBJ_BODY, m.site_bodyid[s]),
              "pos_local_mm": mm(m.site_pos[s]), "pos_world_mm": mm(d.site_xpos[s])} for s in range(m.nsite)]
    lights = [{"name": name(m, mujoco.mjtObj.mjOBJ_LIGHT, i), "body": name(m, mujoco.mjtObj.mjOBJ_BODY, m.light_bodyid[i]),
               "pos_local_mm": mm(m.light_pos[i]), "dir_local": [round(float(x), 3) for x in m.light_dir[i]],
               "cutoff_deg": round(float(m.light_cutoff[i]), 1)} for i in range(m.nlight) if m.light_bodyid[i] != 0]
    return {
        "model": xml.name, "units": "mm, deg, g, N*m",
        "frame": "world at keyframe home_balanced; floor z = 0; x forward, y left, z up",
        "total_mass_g": round(float(m.body_mass.sum()) * 1000, 1),
        "key_dimensions": key_dimensions(m, d), "joints": joints, "bodies": bodies, "geoms": geoms,
        "cameras": cams, "sites": sites, "lights": lights,
    }


def fmt(v):
    return ", ".join(f"{x:g}" for x in v) if isinstance(v, list) else str(v)


def hardpoints_md(spec):
    L = [f"# CAD Hardpoints: `{spec['model']}`", "",
         "Generated by `tools/export_cad_spec.py` from the MJCF. Do not edit by hand: change the model and re-run.", "",
         f"Units: {spec['units']}. Frame: {spec['frame']}. Total mass {spec['total_mass_g'] / 1000:.3f} kg.", "",
         "## Key dimensions", "", "| Dimension | Value (mm) |", "| :--- | ---: |"]
    for k, v in spec["key_dimensions"].items():
        if not isinstance(v, dict):
            L.append(f"| {k} | {fmt(v)} |")
    L += ["", "### Leg poses the structure must clear", "",
          "| Pose | hip_pitch (deg) | knee (deg) | hip_pitch axis above axle | axle fore/aft of hip_pitch axis |",
          "| :--- | ---: | ---: | ---: | ---: |"]
    for k, v in spec["key_dimensions"]["leg poses (hip = -0.50 - delta, knee = 1.00 + 2*delta)"].items():
        L.append(f"| {k} | {v['hip_pitch_deg']} | {v['knee_deg']} | {v['hip_pitch axis above wheel axle']} | "
                 f"{v['wheel axle fore/aft of hip_pitch axis']} |")
    L += ["", "## Joints", "",
          "| Joint | Child body | Parent | Anchor (child frame, mm) | Axis (child frame) | Anchor (world, mm) | Range (deg) | Actuator / limit |",
          "| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |"]
    for j in spec["joints"]:
        rng = fmt(j["range_deg"]) if j["range_deg"] else "continuous"
        act = f"{j['actuator']} ±{j['force_limit_Nm']} N·m" if j["actuator"] else "-"
        L.append(f"| `{j['name']}` | `{j['body']}` | `{j['parent']}` | {fmt(j['anchor_local_mm'])} | {fmt(j['axis_local'])} | "
                 f"{fmt(j['anchor_world_mm'])} | {rng} | {act} |")
    L += ["", "## Body frames", "", "| Body | Parent | Origin in parent (mm) | Quat in parent (wxyz) | Origin (world, mm) | Mass (g) |",
          "| :--- | :--- | :--- | :--- | :--- | ---: |"]
    for b in spec["bodies"]:
        L.append(f"| `{b['name']}` | `{b['parent']}` | {fmt(b['pos_local_mm'])} | {fmt(b['quat_local_wxyz'])} | "
                 f"{fmt(b['origin_world_mm'])} | {b['mass_g']} |")
    L += ["", "## Cameras, sites and lights", "", "| Name | Kind | Body | Position (local, mm) | Position (world, mm) | Notes |",
          "| :--- | :--- | :--- | :--- | :--- | :--- |"]
    for c in spec["cameras"]:
        L.append(f"| `{c['name']}` | camera | `{c['body']}` | {fmt(c['pos_local_mm'])} | {fmt(c['pos_world_mm'])} | fovy {c['fovy_deg']}° |")
    for s in spec["sites"]:
        L.append(f"| `{s['name']}` | site | `{s['body']}` | {fmt(s['pos_local_mm'])} | {fmt(s['pos_world_mm'])} | |")
    for i in spec["lights"]:
        L.append(f"| `{i['name']}` | light | `{i['body']}` | {fmt(i['pos_local_mm'])} | | dir {fmt(i['dir_local'])}, cutoff {i['cutoff_deg']}° |")
    L += ["", "## Part placement (geoms)", "",
          "Dims: box = x, y, z; cylinder / capsule = Ø, length (along the geom's local z); sphere = Ø. Position and "
          "orientation are in the owning body's frame. Full data incl. world centres: `hardpoints.json`.", "",
          "| Geom | Body | Type | Dims (mm) | Position (local, mm) | Quat (local, wxyz) | Role |",
          "| :--- | :--- | :--- | :--- | :--- | :--- | :--- |"]
    for g in spec["geoms"]:
        L.append(f"| `{g['name']}` | `{g['body']}` | {g['type']} | {fmt(g['dims_mm'])} | {fmt(g['pos_local_mm'])} | "
                 f"{fmt(g['quat_local_wxyz'])} | {g['role']} |")
    return "\n".join(L) + "\n"


def parts_md(parts):
    L = ["# Purchased Parts: Real Dimensions", "",
         "Generated by `tools/export_cad_spec.py` from `purchased_parts.json` (edit the JSON, not this file). "
         "Units mm / g. Confidence: **drawing** = official mechanical drawing, **step** = measured from the vendor STEP, "
         "**datasheet** = manufacturer spec, **listing** = retailer listing, **estimate** = not yet sourced. "
         "Download vendor drawings/STEP with `tools/fetch_vendor_cad.sh`.", ""]
    for p in parts["parts"]:
        L += [f"## {p['name']}", "",
              f"- **Qty** {p['qty']} · **Mass** {p['mass_g']} g · **Confidence** {p['confidence']}"
              + (f" · **Model** {', '.join(f'`{u}`' for u in p['used_for'])}" if p["used_for"] else ""),
              f"- **Envelope**: {p['envelope']}"]
        L += [f"- **Mounting**: {x}" for x in p.get("mounting", [])]
        if p.get("loads"):
            L.append(f"- **Loads**: {p['loads']}")
        if p.get("verify"):
            L.append(f"- **Verify**: {p['verify']}")
        if p.get("sources"):
            L.append("- **Sources**: " + ", ".join(f"[{i + 1}]({u})" for i, u in enumerate(p["sources"])))
        L.append("")
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(swd.PROFILES), default=swd.DEFAULT_ROBOT)
    args = ap.parse_args()
    xml = pathlib.Path(swd.profile_xml(swd.PROFILES[args.robot]))
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    spec = export(m, d, xml)
    CAD.mkdir(exist_ok=True)
    (CAD / "hardpoints.json").write_text(json.dumps(spec, indent=1, ensure_ascii=False) + "\n")
    (CAD / "hardpoints.md").write_text(hardpoints_md(spec))
    (CAD / "purchased_parts.md").write_text(parts_md(json.loads((CAD / "purchased_parts.json").read_text())))
    print(f"{xml.name}: {len(spec['joints'])} joints, {len(spec['bodies'])} bodies, {len(spec['geoms'])} geoms "
          f"-> cad/hardpoints.json, cad/hardpoints.md, cad/purchased_parts.md")
    for k, v in spec["key_dimensions"].items():
        if not isinstance(v, dict):
            print(f"  {k}: {fmt(v)}")


if __name__ == "__main__":
    main()
