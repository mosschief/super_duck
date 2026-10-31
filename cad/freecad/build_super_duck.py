"""Build the Super Duck FreeCAD assembly from cad/hardpoints.json.

Run inside FreeCAD (the MCP execute_code tool, or the Python console):
    exec(open("<repo>/cad/freecad/build_super_duck.py").read(),
         {"__file__": "<repo>/cad/freecad/build_super_duck.py", "__name__": "__main__"})
or headless:
    freecadcmd cad/freecad/build_super_duck.py

The document mirrors the MJCF body tree: one App::Part per body, nested, with the joint angles of
the home_balanced keyframe. Re-pose a joint by editing that App::Part's rotation. Writes
super_duck.FCStd (git-ignored: it embeds the vendor STEP models), printed/*.step (printable parts
in their body frame) and fit_report.md (interference check + printed-part volumes).

Frame: trunk-local unless stated (trunk origin = base, world z 370.1 at home), mm, x fwd, y left.
"""
import importlib
import time
import json
import math
import os
import sys

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import duck_parts as dp  # noqa: E402

importlib.reload(dp)
from duck_parts import V, box, cbox, cyl, tube, fuse, bolt_circle, rounded_plate_xz  # noqa: E402

GUI = App.GuiUp
ODM_LABELS = set()
_T0 = time.time()


def tick(msg):
    print(f"[{time.time() - _T0:6.1f} s] {msg}", flush=True)


HP = json.load(open(os.path.join(CAD, "hardpoints.json")))
BODIES = {b["name"]: b for b in HP["bodies"]}
JOINTS = {j["name"]: j for j in HP["joints"]}

# ---------------------------------------------------------------- colours
C_PRINT = (0.96, 0.62, 0.16)
C_SHELL = (0.98, 0.85, 0.30)
C_MOTOR = (0.22, 0.23, 0.26)
C_SERVO = (0.10, 0.12, 0.18)
C_PCB = (0.05, 0.45, 0.20)
C_CHIP = (0.12, 0.12, 0.12)
C_POWER = (0.80, 0.15, 0.12)
C_CELL = (0.20, 0.42, 0.85)
C_TIRE = (0.07, 0.07, 0.07)
C_STANDIN = (0.6, 0.6, 0.6)
C_METAL = (0.75, 0.75, 0.78)


def pl(x=0, y=0, z=0, axis=None, deg=0.0):
    rot = App.Rotation(V(*axis), deg) if axis else App.Rotation()
    return App.Placement(V(x, y, z), rot)


def pl_axes(origin, xdir, zdir, shift=0.0):
    """Placement whose rotation maps local x -> xdir, z -> zdir; origin moved by shift along zdir."""
    x, z = V(*xdir), V(*zdir)
    rot = App.Rotation(x, z.cross(x), z, "ZXY")
    o = V(*origin) + z * shift
    return App.Placement(o, rot)


def mirror_y(shape):
    return shape.mirror(V(0, 0, 0), V(0, 1, 0))


# ---------------------------------------------------------------- joint angles at home
def _pitch_angle(a, b):
    """Rotation about +y that takes (0,0,-L) to b - a (in the x-z plane)."""
    return math.degrees(math.atan2(-(b[0] - a[0]), -(b[2] - a[2])))


HOME = {}
for side, knee_j, hip_j, foot in (("left", "left_knee", "left_hip_pitch", "foot_assembly"),
                                  ("right", "right_knee", "right_hip_pitch", "foot_assembly_2")):
    hip = JOINTS[hip_j]["anchor_world_mm"]
    knee = JOINTS[knee_j]["anchor_world_mm"]
    wheel = BODIES[foot]["origin_world_mm"]
    th = _pitch_angle(hip, knee)
    HOME[hip_j] = th
    HOME[knee_j] = _pitch_angle(knee, wheel) - th


# Neck and head follow the Open Duck Mini v2 at 1:1 (same STS3215 servos): the ODM's joint spacing and
# parts, read from its robot.xml, hung from our neck-pitch joint. These body origins differ from the MJCF
# (carry them back into the model: see README).
import odm as odm_mod  # noqa: E402

importlib.reload(odm_mod)
ODM_OK = odm_mod.available()
BODY_POS_OVERRIDE = {}
NECK_W = {}
if ODM_OK:
    ODM_BODIES, ODM_JOINTS, ODM_MESHES = odm_mod.load()
    _np = BODIES["neck_pitch_assembly"]["pos_local_mm"]
    _o = ODM_JOINTS["neck_pitch"][0].Base
    ODM_T = V(_np[0] - _o.x, -ODM_JOINTS["head_yaw"][0].Base.y, _np[2] - _o.z)   # ODM trunk -> our trunk

    def _joint_w(name):
        p = ODM_JOINTS[name][0].Base + ODM_T
        return V(p.x, 0.0, p.z) if name in ("neck_pitch", "head_pitch") else p   # pitch axes: mid-plane

    for body, joint, parent in (("neck_pitch_assembly", "neck_pitch", None),
                                ("head_pitch_to_yaw", "head_pitch", "neck_pitch_assembly"),
                                ("neck_yaw_assembly", "head_yaw", "head_pitch_to_yaw"),
                                ("head_assembly", "head_roll", "neck_yaw_assembly"),
                                ("left_antenna", "left_antenna", "head_assembly"),
                                ("right_antenna", "right_antenna", "head_assembly")):
        NECK_W[body] = _joint_w(joint)
        BODY_POS_OVERRIDE[body] = tuple(NECK_W[body] - (NECK_W[parent] if parent else V(0, 0, 0)))

# ---------------------------------------------------------------- document / body tree
class Builder:
    def __init__(self, name="SuperDuck"):
        if name in App.listDocuments():
            App.closeDocument(name)
        self.doc = App.newDocument(name)
        self.parts = {}      # body name -> App::Part
        self.world = {}      # body name -> world Placement
        self.sim_world = {}  # same, at the MJCF body positions (for the sim stand-ins)
        self.items = []      # (label, body, tag, local Placement, shape)
        self.meta = {}       # label -> (tag, colour, transparency), for the sim export (parts_manifest.json)
        self.objs = {}

    def body(self, name, parent=None):
        b = BODIES[name]
        p = App.Placement(V(*b["pos_local_mm"]), App.Rotation(*(b["quat_local_wxyz"][1:] + b["quat_local_wxyz"][:1])))
        for j in HP["joints"]:
            if j["body"] == name and j["name"] in HOME:
                p = p.multiply(App.Placement(V(*j["anchor_local_mm"]),
                                             App.Rotation(V(*j["axis_local"]), HOME[j["name"]]),
                                             V(0, 0, 0)))
        sim_p = p
        if name in BODY_POS_OVERRIDE:
            p = App.Placement(V(*BODY_POS_OVERRIDE[name]), p.Rotation)
        part = self.doc.addObject("App::Part", name)
        part.Placement = p
        if parent:
            self.parts[parent].addObject(part)
            self.world[name] = self.world[parent].multiply(p)
            self.sim_world[name] = self.sim_world[parent].multiply(sim_p)
        else:
            self.world[name] = p
            self.sim_world[name] = sim_p
        self.parts[name] = part
        return part

    def add(self, body, label, shape, color, tag, placement=None, transparency=0):
        placement = placement or App.Placement()
        obj = self.doc.addObject("Part::Feature", label)
        obj.Shape = shape
        obj.Placement = placement.multiply(shape.Placement)
        self.parts[body].addObject(obj)
        if GUI:
            vo = obj.ViewObject
            vo.ShapeColor = color
            vo.Transparency = transparency
            if hasattr(vo, "LineColor"):
                vo.LineColor = tuple(c * 0.6 for c in color)
        self.items.append((label, body, tag, placement, shape))
        self.meta[label] = (tag, tuple(color), transparency)
        self.objs[label] = obj
        return obj

    def world_shape(self, body, placement, shape):
        s = shape.copy()
        s.transformShape(self.world[body].multiply(placement).toMatrix())
        return s


B = Builder()
doc = B.doc

# Root: world -> base -> trunk_assembly. base sits at its home height.
B.body("base")
B.body("trunk_assembly", "base")
for name in ("hip_roll_assembly", "hip_roll_assembly_2", "neck_pitch_assembly"):
    B.body(name, "trunk_assembly")
for chain in (("left_roll_to_pitch_assembly", "hip_roll_assembly"),
              ("knee_and_ankle_assembly", "left_roll_to_pitch_assembly"),
              ("knee_and_ankle_assembly_2", "knee_and_ankle_assembly"),
              ("foot_assembly", "knee_and_ankle_assembly_2"),
              ("right_roll_to_pitch_assembly", "hip_roll_assembly_2"),
              ("knee_and_ankle_assembly_3", "right_roll_to_pitch_assembly"),
              ("knee_and_ankle_assembly_4", "knee_and_ankle_assembly_3"),
              ("foot_assembly_2", "knee_and_ankle_assembly_4"),
              ("head_pitch_to_yaw", "neck_pitch_assembly"),
              ("neck_yaw_assembly", "head_pitch_to_yaw"),
              ("head_assembly", "neck_yaw_assembly"),
              ("left_antenna", "head_assembly"),
              ("right_antenna", "head_assembly")):
    B.body(*chain)
for name, label in (("base", "SuperDuck"), ("trunk_assembly", "Trunk"),
                    ("hip_roll_assembly", "L_HipMount"), ("hip_roll_assembly_2", "R_HipMount"),
                    ("left_roll_to_pitch_assembly", "L_HipRoll"), ("right_roll_to_pitch_assembly", "R_HipRoll"),
                    ("knee_and_ankle_assembly", "L_Thigh"), ("knee_and_ankle_assembly_3", "R_Thigh"),
                    ("knee_and_ankle_assembly_2", "L_Shin"), ("knee_and_ankle_assembly_4", "R_Shin"),
                    ("foot_assembly", "L_Wheel"), ("foot_assembly_2", "R_Wheel"),
                    ("neck_pitch_assembly", "NeckPitch"), ("head_pitch_to_yaw", "HeadPitch"),
                    ("neck_yaw_assembly", "HeadYaw"), ("head_assembly", "Head"),
                    ("left_antenna", "L_Antenna"), ("right_antenna", "R_Antenna")):
    B.parts[name].Label = label

T = "trunk_assembly"
HEAD = "head_assembly"

tick("trunk structure")
# ================================================================ TRUNK: printed structure
# Outside: the Open Duck Mini v2 body (its body_front / body_back / body_middle_* shells) scaled x1.15 about the
# neck-pitch axis and 3 mm lower, so the battery clears its steep bottom chamfers. The ODM shell is a convex
# polytope: an octagonal prism (126 wide, 45 deg top chamfers, steep bottom chamfers), flat underside to
# x -63.5 then sloping up 23.6 deg (the ODM's 16.6 deg slope starts at x -27, but our battery
# runs back to x -62.5) to the ODM's tail end face, a 45 deg tapered tail to x -142, flat front cap at x 88, and an opening in
# the top round the neck. The ODM's side windows, where its legs come out, are left closed. The planes below are measured from the
# ODM meshes (see README). Inside is ours: a solid floor, the old front and rear walls kept as bulkheads,
# the lid as a deck carrying the neck. Split like the ODM at z 12.3 into a lower tub and a top cover.
DROP = 12.0
FLOOR_TOP, FLOOR_BOT, WALL_TOP = -66.0 - DROP, -70.0 - DROP, 54.0
BATT_Z = -32.0 - DROP
ODM_K, ODM_DZ = 1.15, -3.0
SPLIT_Z = 12.3
LID_SCREWS = ((67.5, 40), (67.5, -40), (-67.5, 40), (-67.5, -52))   # lid + cover screws, into the bulkheads
BODY_BOT = -83.88 + ODM_DZ                         # underside of the body

# (point on the plane, outward normal), body frame before the -3 mm shift
_BODY_PLANES = [((87.75, 0, 0), (1, 0, 0)), ((-142.25, 0, 0), (-1, 0, 0)),
                ((0, 63.25, 0), (0, 1, 0)), ((0, -63.25, 0), (0, -1, 0)),
                ((0, 0, 80.57), (0, 0, 1)), ((0, 0, -83.88), (0, 0, -1)),
                ((0, 63.25, 55.9), (0, 1, 1)), ((0, -63.25, 55.9), (0, -1, 1)),                  # top chamfers
                ((0, 63.2, -42.8), (0, 41.1, -19.1)), ((0, -63.2, -42.8), (0, -41.1, -19.1)),    # bottom chamfers
                ((-63.5, 0, -83.88), (-34.48, 0, -78.75)),                                       # underside slope
                ((-96.2, 63.2, 0), (-45.9, 46, 0)), ((-96.2, -63.2, 0), (-45.9, -46, 0)),         # tail sides
                ((-96.2, 0, 80.57), (-57.5, 0, 46))]                                             # tail top


def odm_body(inset=0.0):
    """The scaled ODM body as a solid, every face moved inward by `inset` (mm)."""
    s = box(-400, 400, -400, 400, -400, 400)
    cuts = []
    for p, n in _BODY_PLANES:
        n = V(*n).normalize()
        h = Part.makeBox(2000, 2000, 1000, V(-1000, -1000, 0))
        h.Placement = App.Placement(V(*p) - n * inset + V(0, 0, ODM_DZ), App.Rotation(V(0, 0, 1), n))
        cuts.append(h)
    return s.cut(cuts)


body_out = odm_body()
front = [e for e in body_out.Edges if abs(e.BoundBox.XMin - 87.75) < 0.01 and abs(e.BoundBox.XMax - 87.75) < 0.01]
body_out = body_out.makeChamfer(3.4, front)                       # the ODM front cap's bevelled edge
body_in = odm_body(3.0)                                           # 3 mm walls
fill_in = odm_body(2.0)                                           # internal parts: 1 mm into the walls, so they fuse
shell = body_out.cut(body_in)
top_open = box(-49.1, 65.9, -28.75, 28.75, 60, 100)               # opening round the neck
top_open = dp.round_corners(top_open, [((x, y, None), 6) for x in (-49.1, 65.9) for y in (-28.75, 28.75)])
shell = shell.cut(top_open)       # closed sides: the ODM's leg openings aren't needed, our hip motors are outside
# E-stop guard: a ring round the mushroom, 2 mm prouder than it, so a fall onto the tail lands on the ring (the sim
# put up to 1.1 kN through the bare button). 4 mm clear of the head all round, so a palm still reaches it; open at
# the bottom, clear of the charge port's cap. The tail end face is only 34 mm wide, so the ring sits on the tapered
# tail sides. Its top crosses the split: that bit is the cover's.
ESTOP_C = V(-142.25, 0, -8)
estop_guard = tube(48, 40, 34, (-130.25, 0, ESTOP_C.z), (-1, 0, 0)).cut(
    [box(-170, -120, -12, 12, -60, ESTOP_C.z - 12), body_in])
shell = fuse([shell, estop_guard])


def vent_slot(w, z0, z1, depth=12):
    """Rounded vertical slot w wide from z0 to z1, centred on the origin in x, through y -depth/2..depth/2."""
    return fuse([box(-w / 2, w / 2, -depth / 2, depth / 2, z0 + w / 2, z1 - w / 2)] +
                [cyl(w, depth, (0, -depth / 2, z), (0, 1, 0)) for z in (z0 + w / 2, z1 - w / 2)])


# Ventilation, ~10 W typical / 20 W peak inside (Pi 5, AI HAT+, bucks): a low inlet through the floor (4 slots,
# 7.5 cm2) and slots high on both side walls beside the Pi, under the lid (9 each, 8.4 cm2), so it breathes by
# convection; the 40 mm fan on the rear bulkhead pulls through the bay and blows out of the tail's tapered
# sides (6 slots each, 6.0 cm2).
VENTS = []
for s_ in (1, -1):
    for x in range(-40, 9, 6):                                   # side slots, in the top cover
        v_ = vent_slot(4, 22, 46)
        v_.translate(V(x, s_ * 63.25, 0))
        VENTS.append(v_)
for y in (-9.75, -3.25, 3.25, 9.75):                             # low inlet: floor slots between the hip-mount
    VENTS.append(fuse([box(22, 66, y - 2, y + 2, -100, FLOOR_TOP + 1)] +    # shelves, under the servo buck
                      [cyl(4, 100 + FLOOR_TOP + 1, (x, y, -100)) for x in (22, 66)]))
_tt = V(-46, -45.9, 0).normalize()                               # along the left tail side, toward the end
for u in range(14, 47, 6):                                       # fan exhaust in the tail sides
    v_ = vent_slot(4, -22, 6)
    v_.rotate(V(0, 0, 0), V(0, 0, 1), math.degrees(math.atan2(_tt.y, _tt.x)))
    v_.translate(V(-96.2, 63.2, 0) + _tt * u)
    VENTS += [v_, mirror_y(v_)]
FAN_Y, FAN_Z = -38.0, 6.0                                          # clear of the cable hole and the cover lip
FAN_PL = pl(-74, FAN_Y, FAN_Z, (0, 1, 0), -90)                   # on the bulkhead's tail face, blowing into the tail

# Leg harness entry: a grommeted hole in each side wall ahead of the battery, into the free front corner of the bay
# (behind the hip-roll motor the BMS and the pack leave no room to turn a cable)
# Harness size: set CABLE_D to the sleeved bundle's measured diameter and everything follows (clip and post holes,
# grommet, the run's offsets from the parts it passes, the twist-point gaps); re-run sweep_legs.py after a change.
# 8.0 = the 2x 16 AWG + CAN pair bundle (~6 mm) in 6 mm expandable PET braid.
CABLE_D = 8.0
CR = CABLE_D / 2
CLIP_D = CABLE_D + 0.8                    # holes in clips / posts: 0.4 mm clearance all round
DROP_D = CABLE_D - 2.0                    # the short sleeved leads to each motor
GAP = CABLE_D + 1.0                       # a run stops this far short of the axis where it meets the other side's ball
GROM_X, GROM_Z = 28.0, -35.0
RING_T = 3.0                              # aluminium clamp ring under the thigh's hip-output screws
GROM_D, GROM_ID, GROM_FL = CABLE_D + 6.0, CABLE_D + 1.0, CABLE_D + 10.0   # hole, grommet bore, flange
C_CABLE = (0.13, 0.13, 0.14)

ESTOP_PL = pl(-142.25, 0, -8, (0, 0, 1), 180)                      # tail end face, under the split
CHARGE_PL = pl(-142.25, 0, -38, (0, 0, 1), 180)
_tn = V(-57.5, 0, 46).normalize()                                  # tail top face normal
SPK_C = V(-120, 0, 77.57 + ODM_DZ - 1.25 * 23.8) - _tn * 3.0        # speaker flange on its inner face
SPK_PL = App.Placement(SPK_C, App.Rotation(V(1, 0, 0), _tn))

below = box(-200, 200, -100, 100, -200, SPLIT_Z)
tub = shell.common(below)
adds = [fill_in.common(box(-200, 200, -100, 100, -200, FLOOR_TOP)),                     # solid floor
        fuse([fill_in.common(box(71, 74, -100, 100, -200, SPLIT_Z)),                     # front bulkhead
              odm_body(3.2).common(box(71, 74, -100, 100, SPLIT_Z - 1, WALL_TOP))]),     # (clear of the cover)
        fuse([fill_in.common(box(-74, -71, -100, 100, -200, SPLIT_Z)),                   # rear bulkhead
              odm_body(3.2).common(box(-74, -71, -100, 100, SPLIT_Z - 1, WALL_TOP))])]
lip = fuse([odm_body(2.0).cut(odm_body(5.0)).common(box(-200, 200, -100, 100, SPLIT_Z - 3, SPLIT_Z)),
            odm_body(3.2).cut(odm_body(5.0)).common(box(-200, 200, -100, 100, SPLIT_Z, SPLIT_Z + 4))])
adds.append(lip.cut([box(-27, 55, -100, 100, 0, 30)]))                                  # cover locating lip
# lid bosses on the bulkheads
adds += [cyl(8, 8, (x, y, WALL_TOP - 8)) for x, y in LID_SCREWS]
# battery tray rails (pack 73 x 89 at x -62.5..10.5, 1 mm clearance)
adds += [box(-65.5, -63.5, -30, 30, FLOOR_TOP, FLOOR_TOP + 8), box(11.5, 13.5, -30, 30, FLOOR_TOP, FLOOR_TOP + 8)]
adds += [box(-55, 3, s * 45.5, s * 47.5, FLOOR_TOP, FLOOR_TOP + 8) if s > 0 else
         box(-55, 3, -47.5, -45.5, FLOOR_TOP, FLOOR_TOP + 8) for s in (1, -1)]
# ledges and posts carrying the Pi deck frame (rests on the pack through 0.5 mm foam)
adds += [box(-49, -41, 55, 61, BATT_Z + 30.5, BATT_Z + 34.5), box(-49, -41, -61, -55, BATT_Z + 30.5, BATT_Z + 34.5)]
adds += [cyl(8, BATT_Z + 34.5 - FLOOR_TOP, (13, y, FLOOR_TOP)) for y in (-50, 50)]
# servo-buck standoffs, Pi-buck and bus-adapter bulkhead bosses
adds += [tube(6, 2.5, 5, (37 + dx, dy, FLOOR_TOP)) for dx in (-13, 13) for dy in (-26, 26)]
adds += [tube(6, 2.5, 2, (69, y, z), (1, 0, 0)) for y in (20, 52) for z in (-36, -17)]   # clear of the nav-cam window
adds += [tube(6, 2.5, 2, (69, y, z), (1, 0, 0)) for y in (-13, 13) for z in (8.5, 43.5)]
# tail shelf for the fuse, anti-spark switch and peripheral buck (on the sloping underside)
TAIL_SHELF_Z = -60.0
adds += [fill_in.common(box(-125, -72.5, -100, 100, TAIL_SHELF_Z - 4, TAIL_SHELF_Z))] + \
        [fill_in.common(box(-125, -72.5, y - 1.5, y + 1.5, -200, TAIL_SHELF_Z)) for y in (-40, -20, 0, 20, 40)]
# + 5 ribs: the pockets under the shelf are closed, so it is printed without support there; the ribs keep its
# bridges to ~18 mm
tub = fuse([tub] + [a.common(fill_in) if i > 2 else a for i, a in enumerate(adds)])
# holes through the walls are teardrops (45 deg roof, flat top), so they print upright without support
cuts = [dp.teardrop(22.5, 10, (-146, 0, -8), (1, 0, 0)), dp.teardrop(16.2, 10, (-146, 0, -38), (1, 0, 0)),  # e-stop,
        dp.teardrop(40, 10, (-78, 0, -25), (1, 0, 0))]                     # charge port, rear bulkhead cable hole
HM_BOLTS_X = (-50, 0, 55)   # hip-mount shelf bolts (y ±30); the front one's counterbore stays 3 mm inside the shelf's end
for x in HM_BOLTS_X:                          # hip-mount shelf: M4 heat inserts, blind, from below
    for s in (1, -1):
        cuts.append(cyl(5.6, 6.5, (x, s * 30, BODY_BOT - 0.1)))
for x, y in LID_SCREWS:                       # lid screws into the bulkhead bosses
    cuts.append(cyl(2.8, 9, (x, y, WALL_TOP - 8.5)))
for y in (-26, 26):                           # servo-buck standoff holes through the floor
    for dx in (-13, 13):
        cuts.append(cyl(2.5, 30, (37 + dx, y, -100)))
cuts += [dp.teardrop(GROM_D, 12, (GROM_X, y_, GROM_Z), (0, 1, 0)) for y_ in (57, -69)]  # leg harness grommets
cuts += [dp.teardrop(38, 10, (-78, FAN_Y, FAN_Z), (1, 0, 0))]                            # fan bore
cuts += [cyl(3.4, 10, (-78, FAN_Y + dy, FAN_Z + dz), (1, 0, 0)) for dy in (-16, 16) for dz in (-16, 16)]
# Nav camera (the sim's trunk_nav_cam): behind the front cap, its axis 20 deg below horizontal like the sim's,
# lens 1 mm inside the cap's outer face. Tilted, the 32 mm board needs 12.4 mm of the 10.75 mm gap in front of
# the front bulkhead, so its lower edge and USB plug pass through a window in the bulkhead (also the screw
# access). It sits on 4 posts off the cap: M2 self-tapping at the bottom two, locating pins at the top two.
# Camera-local frame: lens on +x, PCB back at x -12.
_na = V(math.cos(math.radians(20)), 0, -math.sin(math.radians(20)))
NAVCAM_PL = App.Placement(V(87.75, 0, -14) - _na * 1.0, App.Rotation(V(0, 1, 0), 20))


def _navcam(shapes):
    out = []
    for sh in shapes:
        sh = sh.copy()
        sh.Placement = NAVCAM_PL.multiply(sh.Placement)
        out.append(sh)
    return out


cuts += _navcam([box(-12.5, -9.9, -16.5, 16.5, -16.5, 16.5),               # board
                 box(-10.9, -3.5, -7.5, 7.5, -7.5, 7.5),                    # lens holder
                 cyl(13, 7, (-4.5, 0, 0), (1, 0, 0)),                       # lens, through the cap
                 Part.makeCone(6.5, 15, 6, V(0, 0, 0), V(1, 0, 0))])        # field of view (>= 100 deg)
cuts.append(box(68.5, 75, -16.5, 16.5, -29, -14))                           # front bulkhead window
tub = tub.cut(cuts + VENTS)
navcam_posts = _navcam([cyl(5, 20, (-10.4, y, z), (1, 0, 0)) for y in (-14, 14) for z in (-14, 14)])
navcam_pins = _navcam([cyl(1.9, 2.2, (-12.6, y, 14), (1, 0, 0)) for y in (-14, 14)])
tub = fuse([tub] + [p_.common(fill_in) for p_ in navcam_posts] + navcam_pins)
tub = tub.cut(_navcam([cyl(1.8, 7, (-10.5, y, -14), (1, 0, 0)) for y in (-14, 14)]))   # M2 pilot holes
B.add(T, "trunk_tub", tub, C_SHELL, "printed", transparency=65)

# top cover: the ODM shell above the split, screwed down through four posts into the lid
cover = shell.cut(below)
cover = fuse([cover] + [cyl(8, 30, (x, y, WALL_TOP + 4)).common(fill_in) for x, y in LID_SCREWS])
grille = []
for gy in range(-4, 5):                       # speaker grille on the tail's top face, hex pattern inside r15
    for gz in range(-4, 5):
        y, w = gy * 3.6 + (1.8 if gz % 2 else 0), gz * 3.1
        if math.hypot(y, w) < 15:
            grille.append(Part.makeCylinder(1.3, 12, SPK_PL.multVec(V(-4, y, w)), _tn))
cover = cover.cut(grille + VENTS + [cyl(3.4, 40, (x, y, WALL_TOP)) for x, y in LID_SCREWS])
B.add(T, "trunk_cover", cover, C_SHELL, "printed", transparency=65)

lid = body_in.common(box(-74, 74, -100, 100, WALL_TOP, WALL_TOP + 4))
lid = lid.cut([box(4, 68, -26, 26, WALL_TOP - 1, WALL_TOP + 20)] +   # ODM neck-pitch servo + swept neck sheets
              [cyl(3.4, 10, (x, y, WALL_TOP - 1)) for x, y in LID_SCREWS] +
              [cyl(3.4, 10, (x, -24, WALL_TOP - 1)) for x in (4, 44)])
B.add(T, "trunk_lid", lid, C_SHELL, "printed", transparency=65)

# Pi deck frame: clamps the battery (0.5 mm foam) and carries the Pi directly (it sits 4.5 mm lower than
# the sim so it clears the ODM neck-pitch servo, which hangs 35 mm below the neck axis)
DECK_Z = (BATT_Z + 34.5, BATT_Z + 37.5)          # 3 mm bars, 0.5 mm above the pack
PI_Z = DECK_Z[1] + 2.0                            # 2 mm standoffs: the Pi has parts on its underside
deck = [box(-49, -41, -60, 60, *DECK_Z), box(9, 17, -60, 60, *DECK_Z),
        box(-49, 17, 33, 39, *DECK_Z), box(-49, 17, -39, -33, *DECK_Z)]
deck += [cyl(6, 2, (x - 6, y, DECK_Z[1])) for x in (-39, 19) for y in (-24.5, 24.5)]
deck = fuse(deck).cut([cyl(2.5, 12, (x - 6, y, 0)) for x in (-39, 19) for y in (-24.5, 24.5)] +
                      [cyl(3.4, 12, (x, y, 0)) for x in (-45,) for y in (-57.5, 57.5)] +
                      [cyl(3.4, 12, (13, y, 0)) for y in (-50, 50)])
deck = deck.common(body_in)
B.add(T, "pi_deck_frame", deck, C_PRINT, "printed")

# Teensy / IMU shelf, cantilevered off the rear bulkhead
shelf = fuse([box(-71, -38, -57, 20, 34, 37), box(-71, -66, -57, 20, 24, 34)])
shelf = shelf.cut([cyl(2.5, 6, (x, y, 33)) for x, y in ((-67, -44), (-43, -44), (-67, -8), (-43, -8),
                                                        (-64.5, 8), (-51.5, 8))])
B.add(T, "electronics_shelf", shelf, C_PRINT, "printed")

tick("trunk electronics")
# ================================================================ TRUNK: electronics & power
cells, wrap = dp.battery_6s2p()
B.add(T, "battery_6s2p_cells", cells, C_CELL, "power", pl(-26, 0, BATT_Z))
B.add(T, "battery_6s2p_wrap", wrap, (0.15, 0.15, 0.6), "power", pl(-26, 0, BATT_Z), transparency=70)
B.add(T, "bms_6s_100a", dp.bms_6s(), C_POWER, "power", pl(16, 0, -37.5, (0, 0, 1), 90))   # on edge across the bay, ahead of the pack
# fuse, anti-spark switch and peripheral buck on the shelf in the tail
B.add(T, "main_fuse_40a", dp.blade_fuse_holder(), C_POWER, "power", pl(-90, 34, TAIL_SHELF_Z + 8))
B.add(T, "antispark_switch_120a", dp.antispark_switch(), C_POWER, "power", pl(-91, 0, TAIL_SHELF_Z + 8, (0, 0, 1), 90))
B.add(T, "buck_7v4_10a_servos", dp.pololu_d36v50(), C_POWER, "power", pl(37, 0, FLOOR_TOP + 5))
B.add(T, "buck_5v1_5a_pi", dp.pololu_d36v50(), C_POWER, "power", pl(69, 36, -26.5, (0, 1, 0), -90))
B.add(T, "buck_5v_5a_peripherals", dp.pololu_d36v50(), C_POWER, "power", pl(-90, -40.5, TAIL_SHELF_Z))
B.add(T, "estop_22mm", dp.estop_22mm(), C_POWER, "power", ESTOP_PL)
B.add(T, "charge_port_gx16", dp.gx16_charge_port(), C_METAL, "power", CHARGE_PL)
B.add(T, "fan_40x10_5v", dp.fan_40x10(), C_CHIP, "electronics", FAN_PL)

PI_PL = pl(-6, 0, PI_Z)
pi = dp.raspberry_pi_5()
B.add(T, "raspberry_pi_5", pi, C_PCB, "electronics", PI_PL)
B.add(T, "pi_active_cooler", dp.active_cooler(), C_METAL, "electronics", PI_PL)
B.add(T, "hat_standoffs_16mm", dp.standoffs(16), C_METAL, "electronics", pl(-6, 0, PI_Z + 1.4))
hat = dp.ai_hat_plus()
# put the HAT's stacking header over the Pi's GPIO header, whichever edge the STEP has it on
gpio = [s for s in pi.Solids if 48 < s.BoundBox.XLength < 54 and s.BoundBox.YLength < 7 and s.BoundBox.ZMin > 1]
if gpio and gpio[0].BoundBox.Center.y < 0:
    hat = mirror_y(hat)
B.add(T, "ai_hat_plus_26tops", hat, C_PCB, "electronics", pl(-6, 0, PI_Z + 17.4))

tc = dp.teensy_carrier()
for k, s in tc.items():
    B.add(T, k, s, C_CHIP if k == "teensy_40" else C_PCB, "electronics", pl(-55, -26, 39))
B.add(T, "imu_icm42688p", dp.imu_breakout(), (0.5, 0.1, 0.5), "electronics", pl(-58, 8, 39))
B.add(T, "bus_servo_adapter", dp.bus_servo_adapter(), (0.1, 0.25, 0.55), "electronics", pl(69, 0, 26, (0, 1, 0), -90))
B.add(T, "max98357a_i2s_amp", dp.max98357a(), (0.2, 0.2, 0.5), "electronics", pl(-69, 40, 0, (0, 1, 0), 90))
B.add(T, "speaker_40mm_3w", dp.speaker_40mm(), C_CHIP, "electronics", SPK_PL)
B.add(T, "usb_nav_cam", dp.usb_cam_module(), C_PCB, "electronics", NAVCAM_PL)
tick("legs")
# ================================================================ LEGS
for s, hb, rb, tb, sb, fb in ((1, "hip_roll_assembly", "left_roll_to_pitch_assembly", "knee_and_ankle_assembly",
                               "knee_and_ankle_assembly_2", "foot_assembly"),
                              (-1, "hip_roll_assembly_2", "right_roll_to_pitch_assembly", "knee_and_ankle_assembly_3",
                               "knee_and_ankle_assembly_4", "foot_assembly_2")):
    P = "L" if s > 0 else "R"
    side = (lambda shape: shape) if s > 0 else mirror_y       # trunk-side parts: built for the left leg

    # --- fixed hip mount (built in trunk-local, then placed in the hip body). Hangs under the body like the
    # ODM's leg arm: a shelf screwed into the floor from below, a web hugging the bottom chamfer, a rib out to
    # the GIM4305 rear-face plate. The plate's outboard end is round, R 29.5 about the roll axis, so the
    # hip-pitch GIM6010 cover (never closer than 30 mm to the roll axis) clears it through the full roll range.
    hp, rp = BODIES[hb]["pos_local_mm"], BODIES[rb]["pos_local_mm"]
    AY, AZ = s * (hp[1] + rp[1]), hp[2] + rp[2]                    # roll axis in the trunk frame (left leg)
    PZ = AZ - 29.5                                                  # plate bottom, flush with the rounded end
    MB = BODY_BOT - 6                                               # underside of the shelf
    PT = -17.5                                                      # plate's rear face (7 mm plate)
    # The hip-pitch reaction twists the rib about y, so from the body out to the roll axis the rib and the plate
    # are one solid block: the plate's outline extruded back to the rib's rear face (above the rib only to y 96 and
    # z -55, clear of the harness and the end cap's cable drop; beyond the roll axis only 10 mm deep, as the yoke's
    # motor lead swings past behind it at -25 deg roll): a closed section (walls + infill when printed). The end-cap seat is
    # bored through it (Ø56), so the end cap and the motor's rear screws are as reachable as before.
    # A drop used to break the open cross where a shorter block stopped at y 78 (FEA: safety factor 1.0).
    hm = fuse([box(-58, 62, 14, 63.25, MB, -40),                             # shelf + web under the chamfer
               box(PT, -10.5, 44, AY - 22, PZ, -50), box(PT, -10.5, 44, AY, PZ, AZ),   # GIM4305 plate
               dp.rounded_plate_yz([(AY - 22, -60, 10), (AY, AZ, 29.5)], PT, -10.5),     # top rounds into the end
               fuse([box(-35, PT, 44, AY - 22, PZ, -50),                     # block behind it: the plate's outline
                     dp.rounded_plate_yz([(AY - 22, -60, 10), (AY, AZ, 29.5)], -35, PT).common(fuse([
                         box(-35, PT, 44, AY, PZ, MB), box(-28, PT, AY, AY + 30, PZ, MB),   # under the rib,
                         box(-35, PT, 44, AY - 12, MB, -55)]))]).cut(        # above it to y 96, under the harness
                   cyl(56, 20, (-36, AY, AZ), (1, 0, 0))),                   # and the end cap's cable drop
               box(-35, -10.5, 44, 125, MB, BODY_BOT),                       # rib
               ]).removeSplitter()
    hm = dp.round_corners(hm, [((x, y, None), 12) for x in (-58, 62) for y in (14, 63.25)] +
                          [((None, 44, PZ), 10), ((-35, 125, None), 8)])
    hm = dp.round_corners(hm, [((PT, None, MB), 4)], strict=False)
    hm_full = hm
    hm = hm.cut(odm_body(-0.5))                                     # hugs the body: 0.5 mm off its skin...
    hm = fuse([hm, hm_full.common(box(-100, 100, 0, 200, BODY_BOT - 0.6, BODY_BOT)).cut(odm_body())])  # ...shelf flush
    hm = hm.cut([cyl(41, 20, (-20, AY, AZ), (1, 0, 0)), cyl(56, 9.6, (PT - 9.5, AY, AZ), (1, 0, 0))] +   # + end-cap seat
                bolt_circle(47.5, 6, 2.7, 20, (-20, AY, AZ), "x", 30) +
                [cyl(4.5, 20, (x, 30, MB - 5)) for x in HM_BOLTS_X] +
                [cyl(8, 3, (x, 30, MB - 0.01)) for x in HM_BOLTS_X])        # counterbores

    # --- leg harness (one 6 mm bundle per leg: 2x 16 AWG power + the CAN twisted pair, a short drop to each
    # motor). It crosses the hip-roll, hip-pitch and knee joints on their axes, where a joint only twists the
    # cable (±25 / -30..+70 / ±90 deg) instead of bending it: each crossing is a ball on one side and the
    # other side's run stopping 7 mm short. Fixed part, trunk frame: from the bay's front corner out through the
    # side-wall grommet, back along the body side over the hip-mount plate, down behind the end cap (the rib's
    # cable clip) to the roll axis 20 mm behind the hip mount.
    RX = -40.0                                                      # where the harness crosses the roll axis
    hx = dp.cable([(-35, 46, 2), (26, 46, 2), (GROM_X, 50, -24), (GROM_X, 55, GROM_Z), (GROM_X, 66, GROM_Z),
                   (GROM_X, 72, -40), (-13, 72, -40), (-36, 76, -40), (RX, AY - 10, -50), (RX, AY, AZ + 6),
                   (RX, AY, AZ)], d=CABLE_D, balls=(-1,))
    hx = fuse([hx, dp.cable([(PT - 1, AY, AZ + 27), (PT - 10.5, AY, AZ + 27), (RX + 1, AY - 5.4, AZ + 24)], d=DROP_D)])
    clip = box(RX - CR - 2, -35, AY - 16, AY - 2, MB, AZ + 10)     # zip-tie anchor on the rib's end
    hm = fuse([hm, clip.cut([dp.cable([(RX, AY - 10, -50), (RX, AY, AZ + 6), (RX, AY, AZ)], d=CLIP_D, balls=(-1,)),
                             box(RX - 7, -34, AY - 12, AY - 8, AZ - 2, AZ + 2)])])   # + zip-tie slot
    hm = hm.common(box(-100, 100, 0, 200, -200, 200))
    to_hip = pl(-hp[0], -hp[1], -hp[2])
    B.add(hb, f"{P}_hip_mount", side(hm), C_PRINT, "printed", to_hip)
    B.add(hb, f"{P}_hip_roll_gim4305", dp.gim4305(), C_MOTOR, "motor", pl(*rp))
    # end cap over the hip-roll motor's rear boss (Impulse actuator back plate), cable notch on top
    cap = dp.along_x(dp.end_cap(52, 3, 1.5, 46, notch_deg=180), PT - 4.5)
    cap.translate(V(0, AY, AZ))
    B.add(hb, f"{P}_hip_roll_end_cap", side(cap), C_PRINT, "printed", to_hip)
    grm = dp.grommet(GROM_D, 3.0, GROM_ID, GROM_FL)
    grm.translate(V(GROM_X, 60.25, GROM_Z))
    B.add(hb, f"{P}_harness_grommet", side(grm), C_TIRE, "cable", to_hip)
    B.add(hb, f"{P}_harness_body", side(hx), C_CABLE, "cable", to_hip)

    # Leg styling follows the Impulse robot: slotted motor cans, twin side plates on the thigh. Built for the
    # left leg, mirrored for the right.

    # --- hip-roll yoke = hip casing (roll body frame, origin on the roll axis): GIM4305 output -> GIM6010 housing.
    # A slotted can around the GIM6010 hip-pitch housing (bolted to its inboard face, Ø74) and a slotted wrap
    # around the GIM4305 hip-roll motor (it turns with the roll output, 2.5 mm radial clearance).
    # top / bottom walls 1 mm clear of the Ø60 bore (no feather edge), 6 mm thick (outer corners at r 43 off the
    # hip-pitch axis, inside the can's R44, so the thigh's R47 scallop still clears them), run out flush with the
    # GIM6010 plate's back edge; 5 mm gussets in the inside corners where they meet the two motor plates (print it
    # on the GIM4305 output plate: see README). 3 mm walls stopping short at x -8 gave a safety factor of 1.5 in a drop.
    def gusset(tri, d):
        return Part.Face(Part.makePolygon([V(*p) for p in tri + [tri[0]]])).extrude(V(*d))
    gus = []
    for sz in (1, -1):
        gus += [gusset([(16.5, 0, sz * 31), (11.5, 0, sz * 31), (16.5, 0, sz * 26)], (0, 36, 0)),
                gusset([(-10, 36, sz * 31), (-10, 31, sz * 31), (-10, 36, sz * 26)], (26.5, 0, 0))]
    # GIM4305 output plate 9 mm (x 16.5..25.5; it bent round the motor's output flange): corners r 45 off the
    # hip-pitch axis, 2 mm inside the thigh's R47 scallop
    yk = fuse([cyl(66, 9, (16.5, 0, 0), (1, 0, 0)), box(16.5, 25.5, 0, 41, -37, 37),  # GIM4305 output plate
               cyl(88, 5, (0, 36, 0), (0, 1, 0)), box(-10, 22.5, 36, 41, -37, 37),
               box(-10, 22.5, 0, 41, 31, 37), box(-10, 22.5, 0, 41, -37, -31)] + gus + [
               dp.along_y(dp.slotted_can(88, 81.5, 25, 8, 9, 15, phase=22.5), 41),   # GIM6010 can, y 41..66
               dp.along_x(dp.slotted_can(66, 58, 24.5, 6, 8, 12, phase=30), -8)])    # GIM4305 wrap, x -8..16.5
    yk = yk.cut([cyl(60, 15, (0, 28, 0), (0, 1, 0)), cyl(12, 14, (14, 0, 0), (1, 0, 0))] +
                bolt_circle(27, 3, 3.4, 14, (14, 0, 0), "x", 90) +
                bolt_circle(74, 8, 3.4, 10, (0, 33, 0), "y", 22.5))
    # harness on the yoke: from the roll axis behind the hip mount, out and up a post off the yoke's top wall,
    # over the hip-pitch motor (a clip on its can) and down the thigh's outer face to the hip-pitch axis. The
    # hip-pitch motor's drop leaves its driver cover at the back. All of it stays clear of the thigh's inner
    # sheet, which sweeps the back of the can.
    TO = 29 + RING_T + 1.0 + CR                                     # harness plane, 1 mm off the thigh's hip clamp ring
    HO = 52 + TO                                                    # (roll frame y)
    TOP = 44 + 3 + CR                                               # over the can (R44)
    yx_pts = [(RX, GAP, 0), (RX, 25, 0), (RX, 25, 30), (-30, 29, TOP - 2), (-8, 40, TOP), (0, 60, TOP), (0, HO, TOP),
              (0, HO, GAP)]
    yx = fuse([dp.cable(yx_pts, d=CABLE_D), dp.cable([(-32.5, 32.5, 0), (RX + 1, 25.5, 0)], d=DROP_D)])
    post = fuse([box(RX - CR - 2.5, RX + CR + 2.5, 25 - CR - 2, 31, 8, 33), box(RX - 5, -7, 19, 29, 30, 34),
                 box(-6, 6, 54, 60, 43, TOP + CR + 2)])             # post + arm to the top wall, clip on the can
    yk = fuse([yk, post.cut([dp.cable(yx_pts, d=CLIP_D)])])
    B.add(rb, f"{P}_hip_roll_yoke", side(yk), C_PRINT, "printed")
    B.add(rb, f"{P}_harness_yoke", side(yx), C_CABLE, "cable")
    B.add(rb, f"{P}_hip_pitch_gim6010", dp.gim6010(), C_MOTOR, "motor", pl(0, s * 52, 0, (0, 0, 1), 90 * s))

    # --- thigh (thigh body frame, origin on the hip-pitch axis, knee at z -118). Twin side sheets like the ODM's
    # leg links, the same outline on both sides: the outer one on the hip output, the inner one (y -20..-15) on
    # the knee housing's output face, scalloped R 47 round the hip so it clears the roll yoke. Front and back walls
    # join them between the motors (the ODM's thigh cover). The knee GIM6010 sits in a slotted can (8x M3 on Ø74
    # through both). The wheel is 74 mm inboard of the outer sheet, and the hip-pitch motor has no far-side
    # bearing (the hip-roll motor's wrap is 3 mm inboard of the yoke plate on that axis), so from the hip to the
    # walls the outer sheet alone carries the leg's side bending: it is 13 mm thick (y 16..29) outside the motor's
    # output hub (R 23.5 at y 15..18). Printed as two pieces, both flat without support inside: the outer sheet
    # (on its y 16 face, clips up) and the inner tray (inner sheet, walls, can and the ring the knee housing
    # bolts to, on its inner sheet). The knee housing screws clamp them together; 4 M3 screws into heat inserts
    # in the walls join them along the length.
    KZ = -118
    TH = [(0, 0, 32), (0, KZ, 46)]                                 # thigh outline: hull of these circles
    TY = 16.0                                                       # split face: outer sheet y 16..29

    outer = fuse([rounded_plate_xz(TH, 22, 29, fillet=2),
                  rounded_plate_xz(TH, TY, 22.5).cut([cyl(51, 8, (0, TY - 1, 0), (0, 1, 0)),
                                                       cyl(61, 8, (0, TY - 1, KZ), (0, 1, 0))])])
    hub_win = dp.radial_windows((0, KZ), 11, 28, 6, 22, 3.5, 21, 30, phase=30)   # show the knee motor cover
    cable_exit = box(24, 50, 10, 23, KZ - 9, KZ + 9)
    outer = outer.cut(hub_win + [cable_exit] +
                      [cyl(8, 12, (0, 20, 0), (0, 1, 0)), cyl(20, 12, (0, 20, KZ), (0, 1, 0))] +
                      bolt_circle(25, 6, 3.4, 15, (0, 20, 0), "y") +
                      bolt_circle(20.4, 3, 4, 15, (0, 20, 0), "y", 30))
    inner = rounded_plate_xz(TH, -20, -15, fillet=2).cut([cyl(94, 10, (0, -21, 0), (0, 1, 0)),
                                                         cyl(50, 8, (0, -21, KZ), (0, 1, 0))])
    walls = rounded_plate_xz(TH, -15, TY).cut(rounded_plate_xz([(0, 0, 29), (0, KZ, 43)], -16, TY + 1))
    walls = walls.common(box(-60, 60, -16, TY + 1, -80, -33))      # from r 47 off the hip axis (yoke can R44)
    INS = [(sx * 32.2, -42) for sx in (1, -1)] + [(sx * 36.4, -75) for sx in (1, -1)]   # heat-insert bosses
    walls = fuse([walls] + [cyl(8, TY + 15, (x, -15, z), (0, 1, 0)).common(rounded_plate_xz(TH, -16, TY + 1))
                            for x, z in INS])
    can = dp.along_y(dp.slotted_can(88, 81.5, 26, 8, 9, 14, phase=22.5), -15)
    can.translate(V(0, 0, KZ))
    ring = tube(88, 66, TY - 11, (0, 11, KZ), (0, 1, 0)).cut(cable_exit)   # the knee housing face, y 11
    # harness down the outer face, hip axis to knee axis (balls on both axes), in three clips; the knee motor's
    # drop comes out of its driver cover through the hub window at 30 deg. The hip-output screws are countersunk
    # into an aluminium clamp ring on the outer face (not into the plastic); the knee-housing ones under the
    # knee crossing are countersunk into the sheet.
    clips = [box(-CR - 3, CR + 3, 28.5, TO + CR + 2.5, z - 4, z + 4) for z in (-36, -60, KZ + 38)]
    tx = fuse([dp.cable([(0, TO, 0), (0, TO, KZ)], d=CABLE_D, balls=(0, -1)),
               dp.cable([(10.5, 23, KZ + 18.2), (10.5, 30.5, KZ + 18.2), (2.5, TO, KZ + 28)], d=DROP_D)])
    tho = fuse([outer] + [c.cut(cyl(CLIP_D, 400, (0, TO, -300))) for c in clips])
    thi = fuse([inner, walls, can, ring])
    knee_bolts = bolt_circle(74, 8, 3.4, 52, (0, -21, KZ), "y", 22.5)          # through both sheets
    tho = tho.cut(knee_bolts + [Part.makeCone(1.7, 3.8, 2.1, V(37 * math.cos(math.radians(a_)), 27.0,
                                                                KZ + 37 * math.sin(math.radians(a_))), V(0, 1, 0))
                                for a_ in (22.5 + 45 * i for i in range(8))] +             # 90 deg countersinks
                  [cyl(3.4, 20, (x, TY - 1, z), (0, 1, 0)) for x, z in INS] +
                  [cyl(6.5, 4, (x, 25.5, z), (0, 1, 0)) for x, z in INS])                  # M3 socket heads
    thi = thi.cut(knee_bolts + [cyl(4.2, 6, (x, TY - 5.8, z), (0, 1, 0)) for x, z in INS] +   # M3 heat inserts
                  [Part.makeCone(1.7, 3.8, 2.1, V(37 * math.cos(math.radians(a_)), -17.9,
                                                  KZ + 37 * math.sin(math.radians(a_))), V(0, -1, 0))
                   for a_ in (22.5 + 45 * i for i in range(8))])   # countersunk: the shin passes 2 mm off this face
    B.add(tb, f"{P}_thigh_outer", side(tho), C_PRINT, "printed")
    B.add(tb, f"{P}_thigh_inner", side(thi), C_PRINT, "printed")
    hring = tube(56, 8, RING_T, (0, 29, 0), (0, 1, 0)).cut(
        bolt_circle(20.4, 3, 4, 10, (0, 25, 0), "y", 30) +
        [Part.makeCone(1.7, 3.8, 2.1, V(12.5 * math.cos(math.radians(60 * i)), 29 + RING_T - 2.1,
                                        12.5 * math.sin(math.radians(60 * i))), V(0, 1, 0)) for i in range(6)] +
        bolt_circle(25, 6, 3.4, 10, (0, 25, 0), "y"))
    B.add(tb, f"{P}_hip_clamp_ring", side(hring), C_METAL, "metal")
    B.add(tb, f"{P}_harness_thigh", side(tx), C_CABLE, "cable")
    B.add(tb, f"{P}_knee_gim6010", dp.gim6010(), C_MOTOR, "motor", pl(0, 0, KZ, (0, 0, 1), -90 * s))

    # --- shin (shin body frame, origin on the knee axis): twin sheets of one outline, like the ODM's shin. The
    # outer one (y -28..-22) on the knee output; the inner one (y -18.5..-12.5) on the wheel motor's rear face,
    # scalloped R 54 so it clears the thigh at full knee bend (the thigh's side edges reach r 52 at the shin's
    # angles). The gap between them is filled (no support trapped in it), so below the scallop the shin is one
    # 15.5 mm slab, pocketed for the wheel motor.
    # From the knee to the scallop the outer sheet is 14 mm (y -36..-22) wherever it clears the tyre by 3.5 mm:
    # the wheel sits 23 mm inboard, so that stretch carries the side bending alone.
    # The outer sheet alone is wider at the knee (R40, the inner one R28): between the end of the 14 mm stretch
    # (the tyre circle) and the scallop it was a bare 6 mm sheet (safety factor 1.5 in a drop); wider, the 14 mm
    # edges run on down outside the tyre circle as flanges. Its layer (y -36..-22) has only the tyre beside it.
    SH = [(0, 0, 28), (0, KZ, 32)]
    SHA = [(0, 0, 40), (0, KZ, 32)]
    TYRE_CLR = 78.5                                                 # tyre R75 + 3.5
    a = fuse([rounded_plate_xz(SHA, -28, -22, fillet=2),
              rounded_plate_xz(SHA, -36, -27.5).cut(cyl(2 * TYRE_CLR, 10, (0, -37, KZ), (0, 1, 0)))])
    a = a.cut([cyl(57, 10.5, (0, -29, KZ), (0, 1, 0))])
    b = fuse([rounded_plate_xz(SH, -18.5, -12.5, fillet=2), rounded_plate_xz(SH, -22.5, -18)])
    b = b.cut([cyl(108, 9, (0, -20.5, 0), (0, 1, 0)), cyl(57, 10.5, (0, -29, KZ), (0, 1, 0)),
               Part.makeCone(50.5, 54.0, 2.0, V(0, -22.5, 0), V(0, 1, 0))])   # tapered into the scallop: no notch
    # harness on the shin: from the knee axis on the thigh's outer face (stopping 7 mm short) it spans free
    # round the thigh's knee end (r >= 54 wherever it is inside the thigh's width, thigh R46-52) to one clip on
    # the inner sheet's outer face, then down that face to the wheel motor's end-cap notch. The span is anchored
    # on the knee axis at the thigh end, so the knee only twists it; leave ~15 mm of slack in it when tying off
    SB = -12.5 + 1 + CR                                             # 1 mm off the inner sheet's outer face
    sx_pts = [(0, TO, -GAP), (0, TO, -40), (0, 30, -54), (0, 14, -60), (0, SB, -64), (0, SB, -93)]
    sx = dp.cable(sx_pts, d=CABLE_D)
    clip = box(-CR - 3, CR + 3, -13, SB + CR + 2.5, -76, -68).cut(cyl(CLIP_D, 20, (0, SB, -80), (0, 0, 1)))
    sh = fuse([a, b, clip])
    sh = sh.cut([cyl(8, 20, (0, -38, 0), (0, 1, 0)), cyl(41, 12, (0, -20, KZ), (0, 1, 0))] +
                bolt_circle(25, 6, 3.4, 20, (0, -38, 0), "y") +
                [cyl(6.5, 7, (12.5 * math.cos(math.radians(60 * i)), -38, 12.5 * math.sin(math.radians(60 * i))),
                     (0, 1, 0)) for i in range(6)] +                   # counterbores: M3x14 socket heads at y -31
                bolt_circle(20.4, 3, 4, 20, (0, -38, 0), "y", 30) +
                bolt_circle(47.5, 6, 2.7, 12, (0, -20, KZ), "y", 30))
    B.add(sb, f"{P}_shin", side(sh), C_PRINT, "printed")
    B.add(sb, f"{P}_harness_shin", side(sx), C_CABLE, "cable")
    B.add(sb, f"{P}_wheel_gim4305", dp.gim4305(), C_MOTOR, "motor", pl(0, -29 * s, KZ, (0, 0, 1), -90 * s))
    # end cap over the wheel motor's rear boss, inside the shin lip, cable notch toward the knee
    wcap = mirror_y(dp.along_y(dp.end_cap(50, 2, 1, 44, notch_deg=-90)))
    wcap.translate(V(0, -9.5, KZ))
    B.add(sb, f"{P}_wheel_end_cap", side(wcap), C_PRINT, "printed")

    # --- wheel
    B.add(fb, f"{P}_wheel_hub", side(dp.wheel_hub()), C_PRINT, "printed")
    B.add(fb, f"{P}_tire_vytaflex40", dp.tire(), C_TIRE, "tire")

tick("neck + head")
# ================================================================ NECK + HEAD (Open Duck Mini v2, 1:1)
ODM_DIR = os.path.join(CAD, "vendor", "odm")
ODM_OURS = {"trunk_assembly": T, "left_antenna_holder": "left_antenna", "right_antenna_holder": "right_antenna"}
SERVO_LABEL = {T: "sts3215_neck_pitch", "neck_pitch_assembly": "sts3215_head_pitch",
               "neck_yaw_assembly": "sts3215_head_yaw", HEAD: "sts3215_head_roll"}
PRINTED_ODM = {"neck_left_sheet", "neck_right_sheet", "head_pitch_to_yaw", "head_yaw_to_roll",
               "head_roll_mount", "head_bot_sheet", "left_antenna_holder", "right_antenna_holder", "antenna"}
SKIP_ODM = {"raspberrypizerow", "board"}   # the ODM's Pi Zero + its board: our compute is in the trunk


def odm_local(odm_body, plc):
    """ODM trunk-frame placement -> (our body, placement in that body's frame)."""
    ours = ODM_OURS.get(odm_body, odm_body)
    origin = V(0, 0, 0) if ours == T else NECK_W[ours]
    return ours, App.Placement(plc.Base + ODM_T - origin, plc.Rotation)


def span(shape, p0, d, length):
    """Solid intervals of shape along the ray p0 + t*d, t in [0, length] (wall thicknesses)."""
    probe = Part.makeCylinder(0.15, length, p0, d)
    out = []
    for sol in shape.common(probe).Solids:
        ts = sorted((v.Point - p0).dot(d) for v in sol.Vertexes)
        out.append((ts[0], ts[-1]))
    return sorted(out)


if not ODM_OK:
    print("cad/vendor/odm/ missing: run tools/fetch_vendor_cad.sh (neck and head are skipped)")
else:
    servos, head_mesh_pl = {}, None
    _n0 = len(B.items)
    for odm_body, mesh, plc in ODM_MESHES:
        if mesh in SKIP_ODM:
            continue
        body, lp = odm_local(odm_body, plc)
        if mesh.startswith("wj-wk00") or mesh.endswith("palonier"):
            s = odm_mod.mesh_solid(mesh)
            s.Placement = lp
            servos.setdefault(body, []).append(s)
        elif mesh == "head":
            head_mesh_pl = lp
        elif mesh == "trunk_top":
            # the ODM's neck-pitch servo pocket, cut out of its trunk top plate; bolts into our lid opening
            crop = odm_mod.mesh_solid(mesh)
            crop.Placement = plc
            crop = crop.common(box(-15.4, 19, -25, 25, 40, 90))  # ODM trunk frame
            crop.translate(ODM_T)
            B.add(T, "neck_servo_pocket", crop, C_PRINT, "printed")
        elif mesh == "sg90":
            B.add(body, ("L" if plc.Base.y > 0 else "R") + "_antenna_sg90", odm_mod.mesh_solid(mesh), C_SERVO, "servo", lp)
        elif mesh == "roll_bearing":
            B.add(body, "head_roll_bearing", odm_mod.mesh_solid(mesh), C_METAL, "bearing", lp)
        else:
            label = mesh if mesh != "antenna" else ("L" if body == "left_antenna" else "R") + "_antenna"
            B.add(body, label, odm_mod.mesh_solid(mesh), (0.95, 0.75, 0.1) if "antenna" in mesh else C_PRINT,
                  "printed" if mesh in PRINTED_ODM else "odm", lp)
    for body, parts in servos.items():
        B.add(body, SERVO_LABEL[body], Part.makeCompound(parts), C_SERVO, "servo")
    ODM_LABELS = {it[0] for it in B.items[_n0:]} - {"neck_servo_pocket"}
    ODM_LABELS |= {"head_shell", "projector_pod", "projector_reflector"}

    # ---- head shell: the ODM head.stl, with our stereo cameras, LED rings, mic and projector pod.
    # Everything below is in ODM head-mesh coordinates (mm, axes = ours); head_mesh_pl places it.
    FACE_X, EYE_Z = 120.0, 139.4
    EYES = [(1, 40.0, EYE_Z), (-1, -40.0, EYE_Z)]          # ODM eye centres: 80 mm baseline
    shell = odm_mod.mesh_solid("head")
    face_w = span(shell, V(FACE_X + 1, 0, EYE_Z), V(-1, 0, 0), 20)[0]
    INNER_X = FACE_X - face_w[1] + 1                       # inner face of the front wall
    top_w = span(shell, V(110, 0, 180), V(0, 0, -1), 40)[0]
    TOP_IN = 180 - top_w[1]
    cups = []
    for s_, y, z in EYES:
        cup = fuse([tube(44, 38.5, 13.5 + 0.5, (INNER_X - 13.5, y, z), (1, 0, 0)),
                    cyl(44, 2, (INNER_X - 15.5, y, z), (1, 0, 0))] +
                   [cyl(4.5, 1.2, (INNER_X - 13.5, y + dy, z + dz), (1, 0, 0))
                    for dy in (-10.5, 10.5) for dz in (7.53, -4.97)])
        cup = cup.cut([box(INNER_X - 16.5, INNER_X - 12.5, y - 11, y + 11, z - 16.5, z - 9)] +
                      [cyl(1.8, 6, (INNER_X - 16.5, y + dy, z + dz), (1, 0, 0))
                       for dy in (-10.5, 10.5) for dz in (7.53, -4.97)])
        cups.append(cup)
    tick("head: cups")
    shell = fuse([shell] + cups)
    hcuts = [cyl(36, 12, (INNER_X - 4, y, z), (1, 0, 0)) for s_, y, z in EYES]   # eye windows (Ø37 rings)
    hcuts += [cyl(39.2, 2.2, (INNER_X - 1.7, y, z), (1, 0, 0)) for s_, y, z in EYES]   # diffuser pockets
    MIC_XY = (110.0, 0.0)                                                             # between the eye cups
    hcuts.append(cyl(2.5, 20, (MIC_XY[0], MIC_XY[1], TOP_IN - 5)))                    # mic port

    # projector pod (ODM print/ STLs share these coordinates), raised 4 mm up the cheek
    ODM_FRONT_YZ = [(-99.0, 109.7), (99.0, 109.7), (77.4, 169.1), (-77.4, 169.1)]   # head.stl front silhouette
    _a, _b = ODM_FRONT_YZ[1], ODM_FRONT_YZ[2]
    _L = math.hypot(_b[0] - _a[0], _b[1] - _a[1])
    POD_N = V(0, (_b[1] - _a[1]) / _L, -(_b[0] - _a[0]) / _L)
    POD_UP = V(0, (_b[0] - _a[0]) / _L, (_b[1] - _a[1]) / _L)
    POD_RAISE = 4.0
    POD_D = POD_UP * POD_RAISE + POD_N * 0.05

    def cheek(x, z):
        return V(x, _a[0] + (_b[0] - _a[0]) * (z - _a[1]) / (_b[1] - _a[1]), z)

    old = [(cheek(100, 135), 5.4)] + [(cheek(x, z), 2.6) for x in (90, 110) for z in (125.7, 144.5)]
    shell = fuse([shell] + [Part.makeCylinder(d / 2, 1.5, p - POD_N * 1.5, POD_N) for p, d in old])  # plug ODM holes
    hcuts += [Part.makeCylinder(2.5, 12, p + POD_D - POD_N * 6, POD_N) for p, d in old[:1]]
    hcuts += [Part.makeCylinder(1.1, 12, p + POD_D - POD_N * 6, POD_N) for p, d in old[1:]]
    tick("head: cut windows + holes")
    shell = shell.cut(hcuts)
    tick("head: done")
    B.add(HEAD, "head_shell", shell, C_SHELL, "printed", head_mesh_pl, transparency=60)

    def hp(x, y, z, axis=None, deg=0.0):
        """Placement in ODM head-mesh coordinates -> our head frame."""
        return head_mesh_pl.multiply(pl(x, y, z, axis, deg))

    for label, f in (("projector_pod", "flash_light_module.stl"), ("projector_reflector", "flash_reflector_interface.stl")):
        B.add(HEAD, label, dp.stl_solid(os.path.join(ODM_DIR, f)), C_PRINT, "printed", hp(*POD_D))
    PROJ_LED = V(101.3, 99.09, 144.54) + POD_D
    B.add(HEAD, "projector_led_1w", dp.projector_led(), (0.95, 0.95, 0.7), "electronics",
          head_mesh_pl.multiply(App.Placement(PROJ_LED, App.Rotation(V(1, 0, 0), POD_N, POD_UP, "ZXY"))))
    print("projector LED (head frame):", head_mesh_pl.multVec(PROJ_LED))

    for s_, y, z in EYES:
        tag = "L" if s_ > 0 else "R"
        B.add(HEAD, f"{tag}_eye_cam_module3", dp.camera_module_3(), C_PCB, "electronics", hp(INNER_X - 1, y, z))
        B.add(HEAD, f"{tag}_eye_led_ring", dp.ws2812_ring_12(), (0.9, 0.9, 0.9), "electronics", hp(INNER_X - 5, y, z))
        B.add(HEAD, f"{tag}_eye_diffuser", tube(38.4, 12, 1.5, (0, 0, 0), (1, 0, 0)), (1, 1, 1), "printed",
              hp(INNER_X - 1.5, y, z), transparency=50)
    B.add(HEAD, "microphone_inmp441", dp.inmp441(), C_PCB, "electronics", hp(MIC_XY[0], MIC_XY[1], TOP_IN, (1, 0, 0), 180))
    _bs = [o for o in B.items if o[0] == "head_bot_sheet"][0]
    _bsw = _bs[4].copy()
    _bsw.Placement = head_mesh_pl.inverse().multiply(_bs[3].multiply(_bsw.Placement))
    _segs = span(_bsw, V(95, 55, 180), V(0, 0, -1), 120)
    _sheet_top = 180 - _segs[0][0] if _segs else EYE_Z - 25
    B.add(HEAD, "projector_mosfet", dp.mosfet_board(), C_PCB, "electronics", hp(95, 55, _sheet_top))


def geom_shape(g):
    d = g["dims_mm"]
    if g["type"] == "box":
        s = cbox(0, 0, 0, *d)
    elif g["type"] == "cylinder":
        s = cyl(d[0], d[1], (0, 0, -d[1] / 2))
    elif g["type"] == "capsule":
        r = d[0] / 2
        s = fuse([cyl(d[0], d[1], (0, 0, -d[1] / 2)), Part.makeSphere(r, V(0, 0, d[1] / 2)),
                  Part.makeSphere(r, V(0, 0, -d[1] / 2))])
    else:
        s = Part.makeSphere(d[0] / 2)
    q = g["quat_local_wxyz"]
    s.Placement = App.Placement(V(*g["pos_local_mm"]), App.Rotation(q[1], q[2], q[3], q[0]))
    return s


tick("sim stand-ins")
# Sim stand-ins (hidden): the MJCF's primitive geoms, for comparison (its mesh geoms are exported from this
# document by export_sim.py, so they match by construction)
sim = doc.addObject("App::Part", "Sim_Standins")
for g in HP["geoms"]:
    if g["type"] not in ("box", "cylinder", "capsule", "sphere"):
        continue
    s = geom_shape(g)
    wp = B.sim_world[g["body"]].multiply(s.Placement)
    s.Placement = App.Placement()
    o = doc.addObject("Part::Feature", "sim_" + g["name"])
    o.Shape = s
    o.Placement = wp
    sim.addObject(o)
    if GUI:
        o.ViewObject.ShapeColor = C_STANDIN
        o.ViewObject.Transparency = 80
if GUI:
    sim.ViewObject.Visibility = False

doc.recompute()

tick("fit check")
# ================================================================ fit check + exports
REPORT = [
    "# FreeCAD Fit Report",
    "",
    "Generated by `cad/freecad/build_super_duck.py` (home_balanced pose). Do not edit by hand.",
    "",
]
world = []
for label, body, tag, plc, shape in B.items:
    ws = B.world_shape(body, plc, shape)
    world.append((label, tag, ws, ws.BoundBox))

# parts that legitimately overlap at home (mating connectors, mounted-through-the-panel parts)
EXPECTED = {frozenset(p) for p in [("raspberry_pi_5", "ai_hat_plus_26tops"),
                                   ("raspberry_pi_5", "pi_active_cooler"),
                                   ("battery_6s2p_cells", "battery_6s2p_wrap"), ("trunk_lid", "neck_servo_pocket"),
                                   ("L_antenna", "L_antenna_servo_mg90s"), ("R_antenna", "R_antenna_servo_mg90s")]}
clashes = []
for i in range(len(world)):
    a = world[i]
    for j in range(i + 1, len(world)):
        b = world[j]
        if frozenset((a[0], b[0])) in EXPECTED or not a[3].intersect(b[3]):
            continue
        t_ = time.time()
        try:
            v = a[2].common(b[2]).Volume
        except Exception:
            continue
        if time.time() - t_ > 2:
            tick(f"  slow pair {a[0]} / {b[0]}: {time.time() - t_:.1f} s")
        if v > 1.0:
            clashes.append((v, a[0], b[0]))
clashes.sort(reverse=True)
odm_native = [c for c in clashes if c[1] in ODM_LABELS and c[2] in ODM_LABELS]
clashes = [c for c in clashes if c not in odm_native]
REPORT += [f"## Interferences at home: {len(clashes)}", ""]
if clashes:
    REPORT += ["| Part A | Part B | Overlap (mm³) |", "| :--- | :--- | ---: |"]
    REPORT += [f"| `{a}` | `{b}` | {v:.0f} |" for v, a, b in clashes]
else:
    REPORT.append("None.")
REPORT += ["", f"### Between Open Duck Mini parts at their ODM positions: {len(odm_native)}", "",
           "Press fits / pockets in the ODM design, or STL-to-solid conversion artefacts; not layout errors.", ""]
if odm_native:
    REPORT += ["| Part A | Part B | Overlap (mm³) |", "| :--- | :--- | ---: |"]
    REPORT += [f"| `{a}` | `{b}` | {v:.0f} |" for v, a, b in odm_native]

tick("exports")
out_dir = os.path.join(HERE, "printed")
os.makedirs(out_dir, exist_ok=True)
REPORT += ["", "## Printed parts", "",
           "Exported to `printed/<name>.step` in the part's body frame. Mass at 100% infill, PETG 1.27 g/cm³.", "",
           "| Part | Body | Volume (cm³) | Mass @100% (g) |", "| :--- | :--- | ---: | ---: |"]
for label, body, tag, plc, shape in B.items:
    if tag != "printed":
        continue
    s = shape.copy()
    s.Placement = plc.multiply(s.Placement)
    s.exportStep(os.path.join(out_dir, label + ".step"))
    REPORT.append(f"| `{label}` | `{body}` | {shape.Volume / 1000:.1f} | {shape.Volume / 1000 * 1.27:.0f} |")

# part list for the sim export (export_sim.py): body, tag, colour of every part
with open(os.path.join(HERE, "parts_manifest.json"), "w") as f:
    json.dump({B.objs[label].Label: {"body": body, "part": B.parts[body].Label, "tag": B.meta[label][0],
                       "rgb": [round(c, 3) for c in B.meta[label][1]], "transparency": B.meta[label][2]}
               for label, body, tag, plc, shape in B.items}, f, indent=1)
with open(os.path.join(HERE, "fit_report.md"), "w") as f:
    f.write("\n".join(REPORT) + "\n")
doc.saveAs(os.path.join(HERE, "super_duck.FCStd"))
tick("saved")
print("\n".join(REPORT[:8 + len(clashes)]))
