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
# Interior envelope: x -71..71, y -60..60, floor top z -78, wall top z 54, lid 54..58.
# The floor is 20 mm lower than the sim's trunk box (-58) and the battery and Pi stack sit 18 mm lower
# than the model (battery centre z -44, not -26): that closes the 2 mm battery/Pi clash flagged in
# clearance_report and puts the Pi under the ODM neck-pitch servo pocket.
# DROP: a further 12 mm so the Pi stack fits under the ODM neck-pitch servo pocket (which reaches z 14.8).
DROP = 12.0
FLOOR_TOP, FLOOR_BOT, WALL_TOP = -66.0 - DROP, -70.0 - DROP, 54.0
BATT_Z = -32.0 - DROP

ESTOP_YZ = (44.0, 30.0)
CHARGE_YZ = (-44.0, 20.0)
SPEAKER_YZ = (0.0, 12.0)

outer = box(-74, 74, -63, 63, FLOOR_BOT, WALL_TOP)
outer = outer.makeFillet(6, [e for e in outer.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1])
inner = box(-71, 71, -60, 60, FLOOR_TOP, WALL_TOP + 10)
inner = inner.makeFillet(4, [e for e in inner.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1])
tub = outer.cut(inner)
adds = []
# lid screw bosses in the corners
for x in (-66, 66):
    for y in (-55, 55):
        adds.append(cyl(8, 8, (x, y, WALL_TOP - 8)))
# battery tray rails (pack 73 x 89 at x -62.5..10.5, 1 mm clearance)
adds += [box(-65.5, -63.5, -30, 30, FLOOR_TOP, FLOOR_TOP + 8), box(11.5, 13.5, -30, 30, FLOOR_TOP, FLOOR_TOP + 8)]
adds += [box(-55, 3, s * 45.5, s * 47.5, FLOOR_TOP, FLOOR_TOP + 8) if s > 0 else
         box(-55, 3, -47.5, -45.5, FLOOR_TOP, FLOOR_TOP + 8) for s in (1, -1)]
# ledges carrying the Pi deck frame (rests on the pack through 0.5 mm foam)
for x0, x1 in ((-49, -41), (9, 17)):
    adds += [box(x0, x1, 55, 60, BATT_Z + 30.5, BATT_Z + 34.5), box(x0, x1, -60, -55, BATT_Z + 30.5, BATT_Z + 34.5)]
# servo-buck standoffs, Pi-buck and bus-adapter wall bosses
adds += [tube(6, 2.5, 5, (37 + dx, dy, FLOOR_TOP)) for dx in (-13, 13) for dy in (-26, 26)]
adds += [tube(6, 2.5, 2, (69, y, z), (1, 0, 0)) for y in (14, 52) for z in (-36, -17)]
adds += [tube(6, 2.5, 2, (69, y, z), (1, 0, 0)) for y in (-13, 13) for z in (8.5, 43.5)]
# chest camera posts (outside the front wall, inside the breast cover)
adds += [tube(6, 2, 9, (74, y, -14 + z), (1, 0, 0)) for y in (-14, 14) for z in (-14, 14)]
# speaker clamp bosses
adds += [tube(6, 2.5, 4, (-71, SPEAKER_YZ[0] + y, SPEAKER_YZ[1]), (1, 0, 0)) for y in (-26, 26)]
tub = fuse([tub] + adds)
cuts = []
cuts.append(cyl(22.5, 10, (-78, ESTOP_YZ[0], ESTOP_YZ[1]), (1, 0, 0)))
cuts.append(cyl(16.2, 10, (-78, CHARGE_YZ[0], CHARGE_YZ[1]), (1, 0, 0)))
for gy in range(-4, 5):                       # speaker grille, hex pattern inside r15
    for gz in range(-4, 5):
        y = gy * 3.6 + (1.8 if gz % 2 else 0)
        z = gz * 3.1
        if math.hypot(y, z) < 15:
            cuts.append(cyl(2.6, 10, (-78, SPEAKER_YZ[0] + y, SPEAKER_YZ[1] + z), (1, 0, 0)))
cuts.append(box(70, 76, -22, -2, -28, -14))   # chest-cam USB cable pass
for y in (-38, 38):                           # breast-cover screws
    for z in (-28, 36):
        cuts.append(cyl(3.4, 10, (68, y, z), (1, 0, 0)))
for s in (1, -1):                             # hip-mount flange bolts + leg cable grommets + Pi vents
    for x in (-28, 28):
        for z in (-60, -35):
            cuts.append(cyl(4.5, 10, (x, s * 58, z), (0, 1, 0)))
    cuts.append(cyl(12, 10, (45, s * 58, -45), (0, 1, 0)))
    for i in range(8):
        cuts.append(box(-40 + i * 9, -37 + i * 9, s * 58 - 5, s * 58 + 5, 12, 26))
for x in (-50, 0, 58):                        # hip-mount shelf bolts through the floor
    for s in (1, -1):
        cuts.append(cyl(4.5, 10, (x, s * 54, FLOOR_BOT - 3)))
for x in (-66, 66):
    for y in (-55, 55):
        cuts.append(cyl(2.8, 9, (x, y, WALL_TOP - 8.5)))
for y in (-26, 26):                           # servo-buck standoff holes through the floor
    for dx in (-13, 13):
        cuts.append(cyl(2.5, 10, (37 + dx, y, FLOOR_BOT - 1)))
tub = tub.cut(cuts)
B.add(T, "trunk_tub", tub, C_SHELL, "printed", transparency=65)

lid = box(-74, 74, -63, 63, WALL_TOP, WALL_TOP + 4)
lid = lid.makeFillet(6, [e for e in lid.Edges if abs(e.Vertexes[0].Point.z - e.Vertexes[1].Point.z) > 1])
lid = lid.cut([box(4, 68, -26, 26, WALL_TOP - 1, WALL_TOP + 20)] +   # ODM neck-pitch servo + swept neck sheets
              [cyl(3.4, 10, (x, y, WALL_TOP - 1)) for x in (-66, 66) for y in (-55, 55)] +
              [cyl(3.4, 10, (x, -24, WALL_TOP - 1)) for x in (4, 44)])
B.add(T, "trunk_lid", lid, C_SHELL, "printed", transparency=65)

# Front breast cover (the BDX chest bulge): R58 cylinder ahead of the front wall, carries the chest cam
bo = cyl(116, 76, (40, 0, -34)).common(box(74, 100, -63, 63, -34, 42))
bi = cyl(110, 70, (40, 0, -31)).common(box(70, 96, -63, 63, -31, 39))
breast = bo.cut(bi)
lugs = [box(74, 80, y - 4, y + 4, z - 3, z + 3) for y in (-38, 38) for z in (-28, 36)]
breast = fuse([breast] + [lg.common(bo) for lg in lugs])
breast = breast.cut([cyl(16, 20, (85, 0, -14), (1, 0, 0))] +
                    [cyl(3.4, 12, (72, y, z), (1, 0, 0)) for y in (-38, 38) for z in (-28, 36)])
B.add(T, "breast_cover", breast, C_SHELL, "printed", transparency=50)

# Pi deck frame: clamps the battery (0.5 mm foam) and carries the Pi directly (it sits 4.5 mm lower than
# the sim so it clears the ODM neck-pitch servo, which hangs 35 mm below the neck axis)
DECK_Z = (BATT_Z + 34.5, BATT_Z + 37.5)          # 3 mm bars, 0.5 mm above the pack
PI_Z = DECK_Z[1] + 2.0                            # 2 mm standoffs: the Pi has parts on its underside
deck = [box(-49, -41, -60, 60, *DECK_Z), box(9, 17, -60, 60, *DECK_Z),
        box(-49, 17, 33, 39, *DECK_Z), box(-49, 17, -39, -33, *DECK_Z)]
deck += [cyl(6, 2, (x - 6, y, DECK_Z[1])) for x in (-39, 19) for y in (-24.5, 24.5)]
deck = fuse(deck).cut([cyl(2.5, 12, (x - 6, y, 0)) for x in (-39, 19) for y in (-24.5, 24.5)] +
                      [cyl(3.4, 12, (x, y, 0)) for x in (-45, 13) for y in (-57.5, 57.5)])
B.add(T, "pi_deck_frame", deck, C_PRINT, "printed")

# Teensy / IMU shelf on the upper rear wall, tied to the right side wall
shelf = fuse([box(-71, -38, -60, 20, 34, 37), box(-66, -38, -60, -56, 24, 34)])
shelf = shelf.cut([cyl(2.5, 6, (x, y, 33)) for x, y in ((-67, -44), (-43, -44), (-67, -8), (-43, -8),
                                                        (-64.5, 8), (-51.5, 8))])
shelf = shelf.common(inner)
B.add(T, "electronics_shelf", shelf, C_PRINT, "printed")

tick("trunk electronics")
# ================================================================ TRUNK: electronics & power
cells, wrap = dp.battery_6s2p()
B.add(T, "battery_6s2p_cells", cells, C_CELL, "power", pl(-26, 0, BATT_Z))
B.add(T, "battery_6s2p_wrap", wrap, (0.15, 0.15, 0.6), "power", pl(-26, 0, BATT_Z), transparency=70)
B.add(T, "bms_6s_100a", dp.bms_6s(), C_POWER, "power", pl(-26, 53, -26 - DROP))
B.add(T, "main_fuse_40a", dp.blade_fuse_holder(), C_POWER, "power", pl(-26, -52.5, FLOOR_TOP + 8))
B.add(T, "antispark_switch_120a", dp.antispark_switch(), C_POWER, "power", pl(37, -47, FLOOR_TOP + 8))
B.add(T, "buck_7v4_10a_servos", dp.buck(34, 60, 18), C_POWER, "power", pl(37, 0, FLOOR_TOP + 5))
B.add(T, "buck_5v1_5a_pi", dp.buck(27, 46, 14), C_POWER, "power", pl(69, 33, -26.5, (0, 1, 0), -90))
B.add(T, "buck_5v_3a_peripherals", dp.buck(22, 16, 12), C_POWER, "power", pl(30, 48, FLOOR_TOP + 3))
B.add(T, "estop_22mm", dp.estop_22mm(), C_POWER, "power", pl(-74, ESTOP_YZ[0], ESTOP_YZ[1], (0, 0, 1), 180))
B.add(T, "charge_port_gx16", dp.gx16_charge_port(), C_METAL, "power", pl(-74, CHARGE_YZ[0], CHARGE_YZ[1], (0, 0, 1), 180))

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
B.add(T, "max98357a_i2s_amp", dp.max98357a(), (0.2, 0.2, 0.5), "electronics", pl(-10, 59, 39, (1, 0, 0), 90))
B.add(T, "speaker_40mm_3w", dp.speaker_40mm(), C_CHIP, "electronics", pl(-71, *SPEAKER_YZ, axis=(0, 0, 1), deg=180))
B.add(T, "usb_chest_cam", dp.usb_cam_module(), C_PCB, "electronics", pl(95, 0, -14))
tick("legs")
# ================================================================ LEGS
for s, hb, rb, tb, sb, fb in ((1, "hip_roll_assembly", "left_roll_to_pitch_assembly", "knee_and_ankle_assembly",
                               "knee_and_ankle_assembly_2", "foot_assembly"),
                              (-1, "hip_roll_assembly_2", "right_roll_to_pitch_assembly", "knee_and_ankle_assembly_3",
                               "knee_and_ankle_assembly_4", "foot_assembly_2")):
    P = "L" if s > 0 else "R"
    side = (lambda shape: shape) if s > 0 else mirror_y

    # --- fixed hip mount (built in trunk-local, then placed in the hip body)
    hm = fuse([box(-15.5, -10.5, 63, 136, -120, -70),                 # motor plate (GIM4305 rear face)
               box(-35, 35, 63, 69, FLOOR_BOT, -25),                   # flange on the trunk side wall
               box(-58, 62, 44, 69, FLOOR_BOT - 6, FLOOR_BOT),         # shelf under the floor
               box(-35, -17, 69, 125, FLOOR_BOT - 6, FLOOR_BOT)])      # top rib, clear of the motor boss
    hm = hm.cut([cyl(41, 20, (-20, 108, -88), (1, 0, 0))] +
                bolt_circle(47.5, 6, 2.7, 20, (-20, 108, -88), "x", 30) +
                [cyl(4.5, 10, (x, 60, z), (0, 1, 0)) for x in (-28, 28) for z in (-60, -35)] +
                [cyl(4.5, 10, (x, 54, FLOOR_BOT - 8)) for x in (-50, 0, 58)])
    to_hip = pl(0, -s * 84, 44)
    B.add(hb, f"{P}_hip_mount", side(hm), C_PRINT, "printed", to_hip)
    B.add(hb, f"{P}_hip_roll_gim4305", dp.gim4305(), C_MOTOR, "motor", pl(0, s * 24, -44))

    # --- hip-roll yoke (roll body frame, origin on the roll axis): GIM4305 output -> GIM6010 housing
    yk = fuse([box(16.5, 22.5, -22, 41, -33, 33),
               cyl(88, 5, (0, 36, 0), (0, 1, 0)), box(-10, 22.5, 36, 41, -33, 33),
               box(-8, 22.5, -22, 41, 29, 33), box(-8, 22.5, -22, 41, -33, -29)])
    yk = yk.cut([cyl(60, 15, (0, 28, 0), (0, 1, 0)), cyl(12, 10, (14, 0, 0), (1, 0, 0))] +
                bolt_circle(27, 3, 3.4, 10, (14, 0, 0), "x", 90) +
                bolt_circle(74, 8, 3.4, 10, (0, 33, 0), "y", 22.5))
    B.add(rb, f"{P}_hip_roll_yoke", side(yk), C_PRINT, "printed")
    B.add(rb, f"{P}_hip_pitch_gim6010", dp.gim6010(), C_MOTOR, "motor", pl(0, s * 52, 0, (0, 0, 1), 90 * s))

    # --- thigh (thigh body frame, origin on the hip-pitch axis): hip output (y=22) -> knee housing (y=11)
    th = fuse([rounded_plate_xz([(0, 0, 24), (0, -118, 44)], 22, 30),
               tube(84, 60, 11, (0, 11, -118), (0, 1, 0))])
    slot = rounded_plate_xz([(0, -32, 8), (0, -78, 10)], 20, 32)
    th = th.cut([slot, cyl(8, 12, (0, 20, 0), (0, 1, 0)), cyl(20, 12, (0, 20, -118), (0, 1, 0)),
                 box(22, 46, 10, 23, -128, -108)] +
                bolt_circle(25, 6, 3.4, 12, (0, 20, 0), "y") +
                bolt_circle(20.4, 3, 4, 12, (0, 20, 0), "y", 30) +
                bolt_circle(74, 8, 3.4, 22, (0, 10, -118), "y", 22.5))
    B.add(tb, f"{P}_thigh", side(th), C_PRINT, "printed")
    B.add(tb, f"{P}_knee_gim6010", dp.gim6010(), C_MOTOR, "motor", pl(0, 0, -118, (0, 0, 1), -90 * s))

    # --- shin (shin body frame, origin on the knee axis): knee output (y=-22) -> wheel motor rear (y=-18.5)
    sh = fuse([rounded_plate_xz([(0, 0, 24), (0, -62, 16)], -28, -22),
               rounded_plate_xz([(0, -66, 16), (0, -118, 32)], -18.5, -12.5),
               box(-16, 16, -28, -12.5, -74, -56)])
    sh = sh.cut([cyl(8, 12, (0, -30, 0), (0, 1, 0)), cyl(41, 12, (0, -20, -118), (0, 1, 0))] +
                bolt_circle(25, 6, 3.4, 12, (0, -30, 0), "y") +
                bolt_circle(20.4, 3, 4, 12, (0, -30, 0), "y", 30) +
                bolt_circle(47.5, 6, 2.7, 12, (0, -20, -118), "y", 30))
    B.add(sb, f"{P}_shin", side(sh), C_PRINT, "printed")
    B.add(sb, f"{P}_wheel_gim4305", dp.gim4305(), C_MOTOR, "motor", pl(0, -29 * s, -118, (0, 0, 1), -90 * s))

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
# Sim stand-ins (hidden): every visual/collision geom of the MJCF, for comparison
sim = doc.addObject("App::Part", "Sim_Standins")
for g in HP["geoms"]:
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

with open(os.path.join(HERE, "fit_report.md"), "w") as f:
    f.write("\n".join(REPORT) + "\n")
doc.saveAs(os.path.join(HERE, "super_duck.FCStd"))
tick("saved")
print("\n".join(REPORT[:8 + len(clashes)]))
