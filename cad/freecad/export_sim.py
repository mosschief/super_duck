"""Export the FreeCAD assembly into the gim6010 sim model (open_duck_wheeled_v2_gim6010.xml), so the sim is the CAD.

From super_duck.FCStd (and parts_manifest.json, both written by build_super_duck.py) this writes:
  - assets/cad/<body>__<material>.stl: every part of each body, merged per colour (visual geoms)
  - assets/cad/<part>_hull.stl: the parts that take contacts (MuJoCo collides their convex hulls)
  - mass_properties.json: mass, CoM and full inertia of every body, in the MJCF body frames (mm, g)
and rewrites, in the MJCF: each body's inertial and geoms, the neck / head body positions and the antenna axes
(Open Duck Mini v2 neck, 1:1), the eye / nav cameras, the projector light and the IMU site.

Mass model: printed PETG at its walls + infill (structural parts 6 walls / 45 %, shells 3 walls / 18 %, the rest
4 walls / 30 %), purchased parts at their real masses (cad/purchased_parts.json), each motor in the body that holds
its stator, plus screw and wiring allowances.

Vendor STEP models (motors, Pi 5) are not redistributed (tools/fetch_vendor_cad.sh): their meshes use the
duck_parts envelope models, at the same placement. The tires stay MuJoCo cylinders (Ø150 x 32, the CAD tire without
its R4 crown): cylinder-on-plane contact is analytic, a mesh tire would roll on facets.

Usage: freecadcmd cad/freecad/export_sim.py   (after build_super_duck.py), then re-run tools/export_cad_spec.py,
tools/derive_lqr_params.py and tools/fit_balance_trim.py gim6010.
"""
import json
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET

import FreeCAD as App
import Mesh
import MeshPart
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import duck_parts as dp  # noqa: E402

V = App.Vector
XML = os.path.join(ROOT, "open_duck_wheeled_v2_gim6010.xml")
ASSETS = os.path.join(ROOT, "assets", "cad")

doc = App.openDocument(os.path.join(HERE, "super_duck.FCStd"))
MANIFEST = json.load(open(os.path.join(HERE, "parts_manifest.json")))

# ---------------------------------------------------------------- mass model
RHO = 1.27  # PETG g/cm3
STRUCT = ("hip_mount", "yoke", "thigh_outer", "thigh_inner", "_shin", "wheel_hub")
SHELL = ("trunk_tub", "trunk_cover", "trunk_lid", "head_shell")
FIXED = {"battery_6s2p_cells": 828, "battery_6s2p_wrap": 42, "bms_6s_100a": 60, "main_fuse_40a": 10,
         "antispark_switch_120a": 60, "buck_7v4_10a_servos": 7, "buck_5v1_5a_pi": 7, "buck_5v_5a_peripherals": 7,
         "estop_22mm": 25, "charge_port_gx16": 15, "fan_40x10_5v": 9, "raspberry_pi_5": 46, "pi_active_cooler": 25,
         "hat_standoffs_16mm": 0.1, "ai_hat_plus_26tops": 41, "teensy_carrier_pcb": 5, "teensy_40": 3,
         "can_transceivers": 4, "teensy_terminals": 4, "imu_icm42688p": 2, "bus_servo_adapter": 10,
         "max98357a_i2s_amp": 2, "speaker_40mm_3w": 27.3, "usb_nav_cam": 8,
         "sts3215_neck_pitch": 55, "sts3215_head_pitch": 55, "sts3215_head_yaw": 55, "sts3215_head_roll": 55,
         "L_antenna_sg90": 9, "R_antenna_sg90": 9, "L_eye_cam_module3": 4, "R_eye_cam_module3": 4,
         "L_eye_led_ring": 2, "R_eye_led_ring": 2, "microphone_inmp441": 2, "projector_led_1w": 2,
         "projector_mosfet": 2}


def mass_of(lab, sh):
    v, a = sh.Volume / 1000.0, sh.Area / 100.0
    if lab in FIXED:
        return FIXED[lab]
    if "gim6010" in lab:
        return 388
    if "gim4305" in lab:
        return 150
    if "tire" in lab:
        return 90
    if "clamp_ring" in lab:
        return v * 2.70  # aluminium
    if "bearing" in lab:
        return v * 7.85  # steel
    if "grommet" in lab:
        return v * 1.2   # rubber
    if "harness" in lab:
        return v * 0.85  # braided copper bundle
    if any(k in lab for k in SHELL):
        t, f = 0.126, 0.18
    elif any(k in lab for k in STRUCT):
        t, f = 0.252, 0.45
    else:
        t, f = 0.168, 0.30
    w = min(v, a * t)
    return RHO * (w + (v - w) * f)


SCREWS = {"Trunk": 30, "L_HipMount": 10, "L_HipRoll": 10, "L_Thigh": 10, "L_Shin": 8, "L_Wheel": 6,
          "NeckPitch": 4, "HeadPitch": 3, "HeadYaw": 4, "Head": 8, "L_Antenna": 0.5}
SCREWS.update({k.replace("L_", "R_"): v for k, v in SCREWS.items() if k.startswith("L_")})
WIRING = {"Trunk": (120, V(-26, 0, -10)), "Head": (15, None)}  # g, point (None: the body's CoM)


def mass_properties(part):
    items = []
    for o in part.Group:
        if o.TypeId != "Part::Feature" or o.Shape.isNull() or not o.Shape.Solids:
            continue
        sols = o.Shape.Solids
        vol = sum(s.Volume for s in sols)
        m = mass_of(o.Label, o.Shape)
        for s in sols:
            k = m * s.Volume / vol
            items.append((k, s.CenterOfMass, s.MatrixOfInertia, s.Volume, o.Label))
    M = sum(i[0] for i in items)
    c0 = sum((cm * k for k, cm, _, _, _ in items), V(0, 0, 0)) * (1.0 / M)
    ext = [(SCREWS[part.Label], c0)]
    if part.Label in WIRING:
        m_, pt = WIRING[part.Label]
        ext.append((m_, pt if pt is not None else c0))
    Mt = M + sum(e[0] for e in ext)
    c = (c0 * M + sum((pt * m_ for m_, pt in ext), V(0, 0, 0))) * (1.0 / Mt)
    I = [[0.0] * 3 for _ in range(3)]

    def add(m, cm, Mi, vol):
        r = cm - c
        rr = [r.x, r.y, r.z]
        for a in range(3):
            for b in range(3):
                base = Mi.A[a * 4 + b] * (m / vol) if Mi is not None else 0.0
                I[a][b] += base + m * ((r.Length ** 2 if a == b else 0) - rr[a] * rr[b])

    for k, cm, Mi, vol, _ in items:
        add(k, cm, Mi, vol)
    for m_, pt in ext:
        add(m_, pt, None, 1)
    by = {}
    for k, _, _, _, lab in items:
        by[lab] = by.get(lab, 0) + k
    return dict(mass_g=Mt, com_mm=[c.x, c.y, c.z], I_gmm2=I, parts={k: round(v, 1) for k, v in by.items()},
                extra=[e[0] for e in ext])


# ---------------------------------------------------------------- vendor parts -> envelope models
def _vendor_kind(label):
    for k in ("gim6010", "gim4305", "raspberry_pi_5"):
        if k in label:
            return k
    return None


_FACTORY = {"gim6010": dp.gim6010, "gim4305": dp.gim4305, "raspberry_pi_5": dp.raspberry_pi_5}
_VENDOR_PL = {k: f().Placement for k, f in _FACTORY.items()}  # placement baked into the vendor shape
dp.VENDOR = os.path.join(HERE, "no-vendor")                   # now the factories return the envelope models
dp._CACHE.clear()
_ENVELOPE = {k: f() for k, f in _FACTORY.items()}


def sim_shape(o):
    """The part's shape in its body frame; vendor models replaced by their envelope model."""
    k = _vendor_kind(o.Label)
    if k is None:
        return o.Shape.copy()
    s = _ENVELOPE[k].copy()
    s.Placement = o.Placement.multiply(_VENDOR_PL[k].inverse()).multiply(s.Placement)
    return s


# ---------------------------------------------------------------- meshes
def tessellate(shape, deflection):
    return MeshPart.meshFromShape(Shape=shape, LinearDeflection=deflection, AngularDeflection=0.5, Relative=False)


def finish(m, max_facets, tolerance):
    """Decimate to max_facets (the Open Duck Mini meshes are dense), mm -> m."""
    if m.CountFacets > max_facets:
        m.decimate(tolerance, 1.0 - max_facets / m.CountFacets)
    m.transform(App.Matrix(0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 1))
    return m


def convex_hull(P, eps=1e-4):
    """Convex hull of points P (n x 3, mm): (vertices, outward triangles). Incremental (quickhull-style)."""
    P = np.unique(np.round(P, 5), axis=0)
    i0 = int(np.argmin(P[:, 0]))
    i1 = int(np.argmax(np.linalg.norm(P - P[i0], axis=1)))
    d01 = (P[i1] - P[i0]) / np.linalg.norm(P[i1] - P[i0])
    rel = P - P[i0]
    i2 = int(np.argmax(np.linalg.norm(rel - np.outer(rel @ d01, d01), axis=1)))
    n = np.cross(P[i1] - P[i0], P[i2] - P[i0])
    i3 = int(np.argmax(np.abs(rel @ n)))
    centre = P[[i0, i1, i2, i3]].mean(axis=0)

    def plane(f):
        a, b, c = P[list(f)]
        nn = np.cross(b - a, c - a)
        nn /= np.linalg.norm(nn)
        return nn, nn @ a

    faces = {}
    for f in ((i0, i1, i2), (i0, i1, i3), (i0, i2, i3), (i1, i2, i3)):
        nn, off = plane(f)
        if nn @ centre - off > 0:
            f = (f[0], f[2], f[1])
        faces[f] = None
    todo = np.arange(len(P))

    def assign(fs, idx):
        for f in fs:
            nn, off = plane(f)
            dist = P[idx] @ nn - off
            out = dist > eps
            faces[f] = (idx[out], dist[out]) if out.any() else None
            idx = idx[~out]

    assign(list(faces), todo)
    while True:
        f0 = next((f for f, o in faces.items() if o is not None), None)
        if f0 is None:
            break
        idx, dist = faces[f0]
        p = int(idx[np.argmax(dist)])
        visible = []
        for f in faces:
            nn, off = plane(f)
            if P[p] @ nn - off > eps:
                visible.append(f)
        edges = set()
        for a, b, c in visible:
            edges ^= {(a, b), (b, c), (c, a)}
        horizon = [(a, b) for a, b in edges if (b, a) not in edges]
        orphans = [faces[f][0] for f in visible if faces[f] is not None]
        for f in visible:
            del faces[f]
        new = [(a, b, p) for a, b in horizon]
        for f in new:
            faces[f] = None
        if orphans:
            idx = np.unique(np.concatenate(orphans))
            assign(new, idx[idx != p])
    verts = sorted({i for f in faces for i in f})
    remap = {v: k for k, v in enumerate(verts)}
    return P[verts], [tuple(remap[i] for i in f) for f in faces]


def hull_mesh(m, ndir=400):
    """Convex hull of a mesh, from its support points in ndir directions (Fibonacci sphere), as a Mesh."""
    P = np.array([[p.x, p.y, p.z] for p in m.Points])
    k = np.arange(ndir) + 0.5
    phi, th = np.arccos(1 - 2 * k / ndir), math.pi * (1 + 5 ** 0.5) * k
    D = np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], axis=1)
    sup = np.unique(np.argmax(P @ D.T, axis=0))
    verts, tris = convex_hull(P[sup])
    return Mesh.Mesh([[App.Vector(*verts[i]) for i in t] for t in tris])


def write_stl(path, mesh):
    pts, faces = mesh.Topology
    with open(path, "wb") as f:
        f.write(b"export_sim.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            pa, pb, pc = pts[a], pts[b], pts[c]
            n = (pb - pa).cross(pc - pa)
            n = n * (1.0 / n.Length) if n.Length > 0 else n
            f.write(struct.pack("<12fH", n.x, n.y, n.z, *pa, *pb, *pc, 0))
    return len(faces)


# CAD colour (build_super_duck.py) -> MJCF material; alpha < 1 only where the real part is translucent (the
# CAD's display transparency is a view choice), emission on the LEDs
MATERIALS = {(0.96, 0.62, 0.16): "cad_print", (0.98, 0.85, 0.3): "cad_shell", (0.22, 0.23, 0.26): "cad_motor",
             (0.1, 0.12, 0.18): "cad_servo", (0.05, 0.45, 0.2): "cad_pcb", (0.12, 0.12, 0.12): "cad_chip",
             (0.8, 0.15, 0.12): "cad_power", (0.2, 0.42, 0.85): "cad_cell", (0.15, 0.15, 0.6): "cad_cell_wrap",
             (0.07, 0.07, 0.07): "cad_rubber", (0.13, 0.13, 0.14): "cad_harness", (0.75, 0.75, 0.78): "cad_metal",
             (0.95, 0.75, 0.1): "cad_antenna", (0.5, 0.1, 0.5): "cad_imu", (0.1, 0.25, 0.55): "cad_servo_adapter",
             (0.2, 0.2, 0.5): "cad_amp", (0.95, 0.95, 0.7): "cad_led", (0.9, 0.9, 0.9): "cad_led_ring",
             (1.0, 1.0, 1.0): "cad_diffuser"}
ALPHA = {"cad_diffuser": 0.5}
EMISSION = {"cad_led": 0.8, "cad_led_ring": 0.6}


def material(info):
    rgb = tuple(round(c, 2) for c in info["rgb"])
    name = MATERIALS.get(rgb, "cad_" + "".join(f"{int(round(c * 255)):02x}" for c in rgb))
    return name, rgb, ALPHA.get(name, 1.0)


# Parts that take contacts, as convex hulls (several labels in one entry = one hull). Not: the harness (it gives),
# parts enclosed by a hull already, the tires (MuJoCo cylinders, kept in the MJCF).
COLLIDE = {
    "Trunk": [("trunk_shell", ("trunk_tub", "trunk_cover", "trunk_lid")), ("estop", ("estop_22mm",)),
              ("charge_port", ("charge_port_gx16",))],
    "NeckPitch": [("neck_left_sheet", ("neck_left_sheet",)), ("neck_right_sheet", ("neck_right_sheet",)),
                  ("sts3215_head_pitch", ("sts3215_head_pitch",))],
    "HeadPitch": [("head_pitch_to_yaw", ("head_pitch_to_yaw001",))],
    "HeadYaw": [("head_yaw_to_roll", ("head_yaw_to_roll",)), ("sts3215_head_yaw", ("sts3215_head_yaw",))],
    "Head": [("head_shell", ("head_shell",)), ("projector_pod", ("projector_pod",))],
}
for P in ("L", "R"):
    COLLIDE[f"{P}_HipMount"] = [(f"{P}_hip_mount", (f"{P}_hip_mount",)),
                                (f"{P}_hip_roll_gim4305", (f"{P}_hip_roll_gim4305", f"{P}_hip_roll_end_cap"))]
    COLLIDE[f"{P}_HipRoll"] = [(f"{P}_hip_roll_yoke", (f"{P}_hip_roll_yoke",)),
                               (f"{P}_hip_pitch_gim6010", (f"{P}_hip_pitch_gim6010",))]
    COLLIDE[f"{P}_Thigh"] = [(f"{P}_thigh_outer", (f"{P}_thigh_outer",)), (f"{P}_thigh_inner", (f"{P}_thigh_inner",)),
                             (f"{P}_knee_gim6010", (f"{P}_knee_gim6010",))]
    COLLIDE[f"{P}_Shin"] = [(f"{P}_shin", (f"{P}_shin",)),
                            (f"{P}_wheel_gim4305", (f"{P}_wheel_gim4305", f"{P}_wheel_end_cap"))]
    COLLIDE[f"{P}_Antenna"] = [(f"{P}_antenna", (f"{P}_antenna", ("left" if P == "L" else "right") + "_antenna_holder"))]
NO_VISUAL = ("tire_vytaflex40",)  # drawn by the MJCF tire geom

VIS_DEFLECTION, HULL_DEFLECTION = 0.2, 0.8  # mm
VIS_FACETS = 3000                           # per visual mesh file

# ---------------------------------------------------------------- export
os.makedirs(ASSETS, exist_ok=True)
for f in os.listdir(ASSETS):
    if f.endswith(".stl"):
        os.remove(os.path.join(ASSETS, f))

parts = {p.Name: p for p in doc.Objects if p.TypeId == "App::Part" and p.Label in SCREWS}
mass, visuals, hulls, mats = {}, {}, {}, {}
n_vis = 0
for body, part in parts.items():
    mass[part.Label] = mass_properties(part)
    objs = {o.Label: o for o in part.Group if o.TypeId == "Part::Feature" and not o.Shape.isNull()}
    groups = {}
    for label, o in objs.items():
        if any(k in label for k in NO_VISUAL):
            continue
        info = MANIFEST.get(label, {"rgb": [0.5, 0.5, 0.5]})
        name, rgb, alpha = material(info)
        mats[name] = (rgb, alpha)
        groups.setdefault(name, []).append(sim_shape(o))
    visuals[body] = []
    for name, shapes in sorted(groups.items()):
        m = tessellate(shapes[0], VIS_DEFLECTION)
        for s in shapes[1:]:
            m.addMesh(tessellate(s, VIS_DEFLECTION))
        fn = f"{body}__{name}.stl"
        n_vis += write_stl(os.path.join(ASSETS, fn), finish(m, VIS_FACETS, 0.5))
        visuals[body].append((fn[:-4], name))
    hulls[body] = []
    for hname, labels in COLLIDE.get(part.Label, []):
        missing = [lb for lb in labels if lb not in objs]
        if missing:
            print("export_sim: no part", missing, "in", part.Label)
            continue
        m = tessellate(sim_shape(objs[labels[0]]), HULL_DEFLECTION)
        for lb in labels[1:]:
            m.addMesh(tessellate(sim_shape(objs[lb]), HULL_DEFLECTION))
        fn = f"{hname}_hull.stl"
        write_stl(os.path.join(ASSETS, fn), finish(hull_mesh(m), 10 ** 6, 0))
        hulls[body].append(fn[:-4])
    if part.Label.endswith("_Wheel"):
        bb = objs[part.Label[0] + "_tire_vytaflex40"].Shape.BoundBox
        assert abs(bb.XLength - 150) < 0.5 and abs(bb.YLength - 32) < 0.5, "tire no longer Ø150 x 32: update the MJCF"
json.dump(mass, open(os.path.join(HERE, "mass_properties.json"), "w"), indent=1)
print(f"export_sim: {n_vis} visual triangles, {sum(len(v) for v in hulls.values())} hulls")

# ---------------------------------------------------------------- sensors / cameras / light (body frames, mm)
def obj(label):
    return doc.getObjectsByLabel(label)[0]


def xyaxes(rot):
    """MJCF camera axes for a lens looking along the part's +x with +z up: x = right (-y), y = up (+z)."""
    r, u = rot.multVec(V(0, -1, 0)), rot.multVec(V(0, 0, 1))
    return " ".join(f"{v:.4f}" for v in (r.x, r.y, r.z, u.x, u.y, u.z))


def m3(v):
    """mm vector -> MJCF metres."""
    return " ".join(f"{x / 1000:.6g}" if abs(x) > 1e-4 else "0" for x in (v.x, v.y, v.z))


CAMERAS = {}
for cam, label in (("left_eye_cam", "L_eye_cam_module3"), ("right_eye_cam", "R_eye_cam_module3"),
                   ("trunk_nav_cam", "usb_nav_cam")):
    p = obj(label).Placement
    CAMERAS[cam] = dict(pos=m3(p.Base), xyaxes=xyaxes(p.Rotation))
led = obj("projector_led_1w").Placement
LIGHT = dict(pos=m3(led.multVec(V(5.2, 0, 0))), dir=" ".join(f"{c:.4f}" for c in led.Rotation.multVec(V(1, 0, 0))))
IMU = m3(obj("imu_icm42688p").Placement.multVec(V(0, 0, 2.05)))  # the ICM-42688-P package centre

NECK_BODIES = ("head_pitch_to_yaw", "neck_yaw_assembly", "head_assembly", "left_antenna", "right_antenna")
ANTENNA_AXIS = {}
try:
    import odm
    if odm.available():
        _, joints, _ = odm.load()
        for j in ("left_antenna", "right_antenna"):
            a = joints[j][1]
            a = a * (-1.0 if a.y < 0 else 1.0)  # our sign: positive about +y, like the other pitch joints
            ANTENNA_AXIS[j] = " ".join(f"{c:.4f}" for c in (a.x, a.y, a.z))
except ImportError:
    pass

# ---------------------------------------------------------------- MJCF
tree = ET.parse(XML, ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
root = tree.getroot()
asset = root.find("asset")
ASSET_GEN = " CAD meshes and colours: generated by cad/freecad/export_sim.py "
for el in list(asset):
    if ((el.tag == "material" and (el.get("name") or "").startswith("cad_")) or
            (el.tag == "mesh" and (el.get("file") or "").startswith("cad/")) or
            (el.tag is ET.Comment and el.text == ASSET_GEN)):
        asset.remove(el)
asset.append(ET.Comment(ASSET_GEN))
for name, (rgb, alpha) in sorted(mats.items()):
    att = {"name": name, "rgba": " ".join(f"{c:g}" for c in rgb) + f" {alpha:g}", "specular": "0.3", "shininess": "0.4"}
    if name in EMISSION:
        att["emission"] = f"{EMISSION[name]:g}"
    ET.SubElement(asset, "material", att)
for body in visuals:
    for mesh, _ in visuals[body]:
        ET.SubElement(asset, "mesh", {"name": mesh, "file": f"cad/{mesh}.stl", "inertia": "shell"})
    for mesh in hulls[body]:
        ET.SubElement(asset, "mesh", {"name": mesh, "file": f"cad/{mesh}.stl", "inertia": "convex"})

bodies = {b.get("name"): b for b in root.iter("body")}


def fmt(x):
    return f"{x:.6g}" if abs(x) > 1e-12 else "0"


GEN = " CAD geoms: generated by cad/freecad/export_sim.py "
for body, part in parts.items():
    el = bodies[body]
    r = mass[part.Label]
    I = [[v * 1e-9 for v in row] for row in r["I_gmm2"]]  # g mm^2 -> kg m^2
    inertial = el.find("inertial")
    inertial.attrib.clear()
    inertial.attrib.update({"pos": " ".join(f"{c / 1000:.4f}" for c in r["com_mm"]), "mass": f"{r['mass_g'] / 1000:.4f}",
                            "fullinertia": " ".join(fmt(v) for v in (I[0][0], I[1][1], I[2][2], I[0][1], I[0][2], I[1][2]))})
    for g in list(el):
        if g.tag == "geom" and g.get("class") != "wheel_tire":
            el.remove(g)
        elif g.tag is ET.Comment and (g.text or "").strip() == GEN.strip():
            el.remove(g)
    i = list(el).index(inertial) + 1
    new = [ET.Comment(GEN)]
    for mesh, mat in visuals[body]:
        new.append(ET.Element("geom", {"name": mesh, "class": "visual", "type": "mesh", "mesh": mesh,
                                       "material": mat}))
    for mesh in hulls[body]:
        new.append(ET.Element("geom", {"name": mesh.replace("_hull", "_collision"), "class": "collision",
                                       "type": "mesh", "mesh": mesh}))
    for k, g in enumerate(new):
        el.insert(i + k, g)
    if body in NECK_BODIES:
        el.set("pos", m3(part.Placement.Base))
    if body in ANTENNA_AXIS:
        el.find("joint").set("axis", ANTENNA_AXIS[body])

# Body pairs whose hulls overlap where the real parts don't (a hull fills a cavity the other part sits in)
EXCLUDE = [("head_pitch_to_yaw", "head_assembly"),    # the head-yaw bracket rises through the head's bottom sheet
           ("neck_pitch_assembly", "head_assembly")]  # sweep_neck.py: the neck clears the head everywhere; the
                                                      # head hull closes the recess round the head-pitch servo
contact = root.find("contact")
if contact is None:
    contact = ET.Element("contact")
    root.insert(list(root).index(root.find("actuator")), contact)
for ex in list(contact):
    if ex.tag == "exclude":
        contact.remove(ex)
for b1, b2 in EXCLUDE:
    ET.SubElement(contact, "exclude", {"body1": b1, "body2": b2})

for cam in root.iter("camera"):
    if cam.get("name") in CAMERAS:
        cam.attrib.pop("euler", None)
        cam.attrib.update(CAMERAS[cam.get("name")])
for light in root.iter("light"):
    if light.get("name") == "projector":
        light.attrib.update(LIGHT)
for site in root.iter("site"):
    if site.get("name") == "imu":
        site.set("pos", IMU)

ET.indent(tree, "  ")
out = []
for line in ET.tostring(root, encoding="unicode").replace(" />", "/>").split("\n"):
    depth = len(line) - len(line.lstrip(" "))
    tag, prev = line.lstrip(), out[-1].lstrip() if out else ""
    if prev and not prev.startswith("<body") and (depth == 2 and tag.startswith("<") and not tag.startswith("</") or
                                                  tag.startswith("<!--") and not prev.startswith("<!--")):
        out.append("")  # blank line before each section and comment
    out.append(line)
with open(XML, "w") as f:
    f.write("\n".join(out) + "\n")
for b, r in mass.items():
    print("export_sim:", b, f"{r['mass_g']:.1f} g", [round(x, 1) for x in r["com_mm"]])
print("export_sim: total", f"{sum(r['mass_g'] for r in mass.values()):.0f} g ->", XML)
