"""Open Duck Mini v2 neck + head, read from its own MJCF (cad/vendor/odm/robot.xml, fetched by
tools/fetch_vendor_cad.sh). Gives the zero-pose frame of every ODM body and every mesh geom in the ODM
trunk frame (mm), so the build script can place the ODM's own STLs unchanged.
"""
import math
import os
import xml.etree.ElementTree as ET

import FreeCAD as App

V = App.Vector
ODM_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vendor", "odm")

NECK_BODIES = ("neck_pitch_assembly", "head_pitch_to_yaw", "neck_yaw_assembly", "head_assembly",
               "left_antenna_holder", "right_antenna_holder")


def _pl(pos, quat):
    w, x, y, z = quat
    return App.Placement(V(*[p * 1000 for p in pos]), App.Rotation(x, y, z, w))


def _vec(s, default):
    return [float(v) for v in (s or default).split()]


def load():
    """Returns (bodies, joints, meshes), all in the ODM trunk frame at zero pose, mm:
    bodies {name: Placement}, joints {name: (Placement of the joint frame, axis Vector)},
    meshes [(body, mesh name, Placement)] for the neck/head bodies and the trunk's neck-pitch servo."""
    root = ET.parse(os.path.join(ODM_DIR, "robot.xml")).getroot()
    bodies, joints, meshes = {}, {}, []

    def walk(el, parent_pl, owner):
        for b in el.findall("body"):
            p = parent_pl.multiply(_pl(_vec(b.get("pos"), "0 0 0"), _vec(b.get("quat"), "1 0 0 0")))
            name = b.get("name")
            bodies[name] = p
            j = b.find("joint")
            if j is not None:
                joints[j.get("name")] = (p, p.Rotation.multVec(V(*_vec(j.get("axis"), "0 0 1"))))
            own = name if name in NECK_BODIES or name == "trunk_assembly" else owner
            if own:
                for g in b.findall("geom"):
                    if g.get("mesh"):
                        meshes.append((own, g.get("mesh"),
                                       p.multiply(_pl(_vec(g.get("pos"), "0 0 0"), _vec(g.get("quat"), "1 0 0 0")))))
            walk(b, p, own if own in NECK_BODIES else None)

    walk(root.find("worldbody"), App.Placement(), None)
    to_trunk = bodies["trunk_assembly"].inverse()
    bodies = {k: to_trunk.multiply(v) for k, v in bodies.items()}
    joints = {k: (to_trunk.multiply(p), to_trunk.Rotation.multVec(a)) for k, (p, a) in joints.items()}
    meshes = [(b, m, to_trunk.multiply(p)) for b, m, p in meshes]
    # trunk: keep only the neck-pitch servo (the STS3215 case + horns whose case sits on the neck axis)
    np_axis = joints["neck_pitch"][0].Base
    keep = []
    for b, m, p in meshes:
        if b == "trunk_assembly":
            if m == "trunk_top":
                keep.append((b, m, p))
                continue
            if not (m.startswith("wj-wk00") or m.endswith("palonier")):
                continue
            if abs(p.Base.x - np_axis.x) > 5 or abs(p.Base.z - np_axis.z) > 40:
                continue
        keep.append((b, m, p))
    return bodies, joints, keep


_MESH_CACHE = {}


MAX_FACETS = 30000  # only the STS3215 case meshes (~40k / ~100k facets) are decimated, to ~6k


def mesh_solid(name):
    """ODM mesh (metres) as a Part solid in mm, in the mesh's own frame. Converted solids are cached in
    cad/vendor/odm/cache/ (.brep), since conversion of the dense meshes takes minutes."""
    import Mesh
    import Part
    if name not in _MESH_CACHE:
        src = os.path.join(ODM_DIR, name + ".stl")
        cache = os.path.join(ODM_DIR, "cache", f"{name}.{MAX_FACETS}.brep")
        if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(src):
            s = Part.Shape()
            s.read(cache)
        else:
            m = Mesh.Mesh(src)
            m.transform(App.Matrix(1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1))
            if m.CountFacets > MAX_FACETS:
                m.decimate(0.05, 1.0 - 6000.0 / m.CountFacets)
            m.harmonizeNormals()
            sh = Part.Shape()
            sh.makeShapeFromMesh(m.Topology, 0.05)
            s = Part.makeSolid(sh)
            try:
                s = s.removeSplitter()
            except Exception:
                pass
            os.makedirs(os.path.dirname(cache), exist_ok=True)
            s.exportBrep(cache)
        _MESH_CACHE[name] = s
    return _MESH_CACHE[name].copy()


def available():
    return os.path.exists(os.path.join(ODM_DIR, "robot.xml"))
