"""Part generators for the Super Duck FreeCAD assembly.

Every function returns a Part.Shape in the part's own frame (mm). build_super_duck.py places
them. Purchased parts follow cad/purchased_parts.json; vendor STEP files (cad/vendor/, fetched by
tools/fetch_vendor_cad.sh) are used for the GIM motors and the Pi 5 when present, with a
parametric stand-in otherwise.
"""
import math
import os

import FreeCAD as App
import Part

V = App.Vector
VENDOR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vendor")


# ---------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1):
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cbox(cx, cy, cz, dx, dy, dz):
    """Box centred on (cx, cy, cz)."""
    return box(cx - dx / 2, cx + dx / 2, cy - dy / 2, cy + dy / 2, cz - dz / 2, cz + dz / 2)


def cyl(d, h, base=(0, 0, 0), axis=(0, 0, 1)):
    return Part.makeCylinder(d / 2, h, V(*base), V(*axis))


def tube(od, id_, h, base=(0, 0, 0), axis=(0, 0, 1)):
    return cyl(od, h, base, axis).cut(cyl(id_, h, base, axis))


def fuse(shapes):
    shapes = [s for s in shapes if s is not None]
    out = shapes[0]
    if len(shapes) > 1:
        out = out.fuse(shapes[1:])
    return out.removeSplitter()


def bolt_circle(pcd, n, d, h, base, axis, phase=0.0):
    """n holes of diameter d on a pitch circle, normal to a principal axis ('x', 'y' or 'z')."""
    holes = []
    for i in range(n):
        a = math.radians(phase + 360.0 * i / n)
        u, w = pcd / 2 * math.cos(a), pcd / 2 * math.sin(a)
        if axis == "x":
            p, ax = (base[0], base[1] + u, base[2] + w), (1, 0, 0)
        elif axis == "y":
            p, ax = (base[0] + u, base[1], base[2] + w), (0, 1, 0)
        else:
            p, ax = (base[0] + u, base[1] + w, base[2]), (0, 0, 1)
        holes.append(cyl(d, h, p, ax))
    return holes


def rounded_plate_xz(points, y0, y1):
    """Convex hull of circles [(x, z, r), ...] extruded along y from y0 to y1 (a link plate)."""
    pts = [(x, z, r) for x, z, r in points]
    solids = [cyl(2 * r, y1 - y0, (x, y0, z), (0, 1, 0)) for x, z, r in pts]
    # bridge consecutive circles with their tangent hull
    for (xa, za, ra), (xb, zb, rb) in zip(pts, pts[1:]):
        dx, dz = xb - xa, zb - za
        L = math.hypot(dx, dz)
        nx, nz = -dz / L, dx / L
        poly = [V(xa + nx * ra, y0, za + nz * ra), V(xb + nx * rb, y0, zb + nz * rb),
                V(xb - nx * rb, y0, zb - nz * rb), V(xa - nx * ra, y0, za - nz * ra)]
        face = Part.Face(Part.makePolygon(poly + [poly[0]]))
        solids.append(face.extrude(V(0, y1 - y0, 0)))
    return fuse(solids)


def _read_step(name):
    path = os.path.join(VENDOR, name)
    if not os.path.exists(path):
        return None
    s = Part.Shape()
    s.read(path)
    return s


# ---------------------------------------------------------------- actuators
_CACHE = {}


def gim6010():
    """GIM6010-8 + GDS68 driver. Axis +x, centred: driver cover x -22..-11, housing faces at
    x = -11 and +15 (8x M3 on Ø74), output hub face at x = +22 (6x M3 on Ø25)."""
    if "6010" not in _CACHE:
        s = _read_step("GIM6010-8.step")
        if s is not None:
            s.translate(V(-11, 0, 0))  # vendor: cover face x=-11, output face x=33
        else:
            s = fuse([cyl(57, 11, (-22, 0, 0), (1, 0, 0)), cyl(80, 26, (-11, 0, 0), (1, 0, 0)),
                      cyl(47, 3.5, (15, 0, 0), (1, 0, 0)), cyl(40, 3.5, (18.5, 0, 0), (1, 0, 0))])
        _CACHE["6010"] = s
    return _CACHE["6010"].copy()


def gim4305():
    """GIM4305-10SHS. Axis +x, centred: rear driver boss x -16.5..-10.5 (Ø40), housing to
    x = +15.5 (rear face 6x M2.5 on Ø47.5 at x = -10.5), output step face at x = +16.5
    (3x M3 on Ø27)."""
    if "4305" not in _CACHE:
        s = _read_step("GIM4305-10.step")
        if s is not None:
            s.translate(V(-120.21 - 16.5, 0, 0))  # vendor: output x=120.21, boss end x=153.21
            s.rotate(V(0, 0, 0), V(0, 0, 1), 180)
        else:
            s = fuse([cyl(40, 6, (-16.5, 0, 0), (1, 0, 0)), cyl(53, 26, (-10.5, 0, 0), (1, 0, 0)),
                      cyl(37, 1, (15.5, 0, 0), (1, 0, 0))])
        _CACHE["4305"] = s
    return _CACHE["4305"].copy()


def sts3215():
    """Feetech STS3215. Output spline on +z at the origin; body 45.2 x 24.7 x 35 below it
    (listing envelope, ears included), the shaft 11 mm from the -x end."""
    body = box(-11, 34.2, -12.35, 12.35, -35, -3)
    cap = box(-11, 34.2, -12.35, 12.35, -3, 0)
    spline = cyl(6, 4, (0, 0, 0))
    horn = cyl(20, 2.5, (0, 0, 4))
    return fuse([body, cap, spline, horn])


def micro_servo():
    """MG90S/SG90. Shaft on +z at the origin, body 22.8 x 12.2 x 22.7 below it (shaft 6 mm
    from the -x end), tabs 32.5 across at 4.5 mm below the top."""
    body = box(-6, 16.8, -6.1, 6.1, -22.7, 0)
    tabs = box(-10.85, 21.65, -6.1, 6.1, -7, -4.5)
    gear = cyl(11.8, 4, (0, 0, 0))
    horn = box(-2.5, 14, -2.5, 2.5, 4, 5.5)
    return fuse([body, tabs, gear, horn])


# ---------------------------------------------------------------- compute
def raspberry_pi_5():
    """Pi 5 in board frame: PCB bottom at z=0, PCB centre at the origin, USB/Ethernet end +x,
    mounting holes at x = -39 / +19, y = ±24.5."""
    s = _read_step("Raspberry_Pi_5.step")
    if s is not None:
        s.rotate(V(0, 0, 0), V(1, 0, 0), 90)  # vendor: PCB in x-z, components on +y
        s.translate(V(0, 0, -0.1))
        return s
    pcb = box(-42.5, 42.5, -28, 28, 0, 1.4)
    usb = box(28, 45.8, -27, 7, -1.8, 16)
    eth = box(24, 45.5, 10, 26, 1.4, 15.4)
    soc = cbox(-8, 0, 2.4, 15, 15, 2)
    return pcb.cut(pi_holes(-1, 4)).fuse([usb, eth, soc])


def pi_holes(z0, h, d=2.7):
    return [cyl(d, h, (x, y, z0)) for x in (-39, 19) for y in (-24.5, 24.5)]


def active_cooler():
    """Raspberry Pi Active Cooler on the Pi 5 (board frame): fins + blower, ~9 mm tall."""
    base = box(-35, 8, -22, 22, 1.4, 4)
    fins = [box(-35, -6, y - 0.6, y + 0.6, 4, 10.4) for y in range(-20, 21, 4)]
    fan = cyl(30, 6.5, (-22, 8, 4))
    return fuse([base, fan] + fins)


def ai_hat_plus():
    """AI HAT+ 26 TOPS (board frame of the HAT: PCB bottom at z=0, hole pattern centred on
    the Pi's): 65.1 x 56.7 x 1.6 PCB, Hailo-8 module with heat spreader, 40-pin header."""
    pcb = box(-42.5, 22.6, -28.35, 28.35, 0, 1.6).cut(pi_holes(-1, 4))
    hailo = cbox(-8, 0, 1.6 + 1.9, 22, 30, 3.8)
    header = box(-36, 15, 23.5, 28.5, -8.5, 0)  # 2x20 stacking header (female underneath)
    pcie = box(18, 22.6, -9, 9, 1.6, 4)
    return fuse([pcb, hailo, header, pcie])


def standoffs(length, d=5, x=(-39, 19), y=(-24.5, 24.5)):
    return fuse([tube(d, 2.7, length, (xx, yy, 0)) for xx in x for yy in y])


def teensy_carrier():
    """Teensy 4.0 + 2x SN65HVD230 CAN transceivers on a 42 x 30 carrier (board frame: PCB
    bottom z=0, long side along y). Returns dict of named shapes."""
    pcb = box(-15, 15, -21, 21, 0, 1.6)
    holes = [cyl(2.7, 4, (x, y, -1)) for x in (-12, 12) for y in (-18, 18)]
    pcb = pcb.cut(holes)
    headers = fuse([box(-9.5, -6.9, -17.8, 17.8, 1.6, 9.6), box(6.9, 9.5, -17.8, 17.8, 1.6, 9.6)])
    teensy = fuse([box(-8.9, 8.9, -17.8, 17.8, 9.6, 11.2),
                   cbox(0, 1, 12.2, 10, 10, 2),          # i.MX RT1062
                   box(-4, 4, 15.8, 19.6, 11.2, 14.4)])  # micro-USB
    can1 = fuse([box(10.5, 14.5, -16, -2, 1.6, 9), box(10.5, 14.5, -16, -2, 9, 10.6)])
    can2 = fuse([box(10.5, 14.5, 2, 16, 1.6, 9), box(10.5, 14.5, 2, 16, 9, 10.6)])
    term = box(-15, -11, -16, 16, 1.6, 11)  # CAN / power screw terminals
    return {"teensy_carrier_pcb": pcb, "teensy_40": fuse([headers, teensy]),
            "can_transceivers": fuse([can1, can2]), "teensy_terminals": term}


def imu_breakout():
    """ICM-42688-P SPI breakout, ~18 x 13 x 1.6 with 2 mounting holes."""
    pcb = box(-9, 9, -6.5, 6.5, 0, 1.6).cut([cyl(2.2, 4, (x, 0, -1)) for x in (-6.5, 6.5)])
    return fuse([pcb, cbox(0, 0, 2.05, 3, 2.5, 0.9), box(-2, 6, 4, 6.5, 1.6, 3.5)])


def bus_servo_adapter():
    """Waveshare Bus Servo Adapter (A): estimated 42 x 33 board (verify), PCB bottom z=0."""
    pcb = box(-21, 21, -16.5, 16.5, 0, 1.6).cut(
        [cyl(3.2, 4, (x, y, -1)) for x in (-17.5, 17.5) for y in (-13, 13)])
    usb = box(-21, -13, -5, 5, 1.6, 5.5)
    jacks = box(8, 19, -14, 14, 1.6, 8)  # servo bus 3-pin headers
    dc = box(-10, 2, 6, 16, 1.6, 12)    # power terminal
    return fuse([pcb, usb, jacks, dc])


def camera_module_3():
    """Pi Camera Module 3 (standard). Lens axis = +x at the origin, PCB back face at
    x = -11.3, 25 (y) x 23.86 (z) PCB with the lens 2.4 mm above its centre; 4x M2 holes
    21 apart (y) x 12.5 (z). Same PCB and holes as Camera Module 2."""
    zc = -2.4  # PCB centre relative to the lens axis
    pcb = box(-11.3, -10.2, -12.5, 12.5, zc - 11.93, zc + 11.93)
    holes = [cyl(2.2, 3, (-12, y, z), (1, 0, 0)) for y in (-10.5, 10.5)
             for z in (zc + 11.93 - 2, zc + 11.93 - 14.5)]
    pcb = pcb.cut(holes)
    holder = box(-10.2, -3, -5.4, 5.4, -5.4, 5.4)
    lens = cyl(5.75, 3, (-3, 0, 0), (1, 0, 0))
    fpc = box(-10.2, -8.2, -10.5, 10.5, zc - 11.93, zc - 7)
    return fuse([pcb, holder, lens, fpc])


def usb_cam_module():
    """Wide-angle USB UVC camera board, 32 x 32 (common Arducam size: verify), lens on +x at
    the origin, PCB back at x = -12."""
    pcb = box(-12, -10.4, -16, 16, -16, 16).cut(
        [cyl(2.2, 3, (-13, y, z), (1, 0, 0)) for y in (-14, 14) for z in (-14, 14)])
    holder = box(-10.4, -4, -7, 7, -7, 7)
    lens = cyl(12, 4, (-4, 0, 0), (1, 0, 0))
    usb = box(-16, -12, -5, 5, -16, -10)
    return fuse([pcb, holder, lens, usb])


def ws2812_ring_12():
    """12-LED ring, Ø37 / Ø23, LEDs facing +x, PCB back at x = 0."""
    ring = tube(37, 23, 1.6, (0, 0, 0), (1, 0, 0))
    leds = []
    for i in range(12):
        a = 2 * math.pi * i / 12
        leds.append(cbox(1.6 + 0.8, 15 * math.cos(a), 15 * math.sin(a), 1.6, 5, 5))
    return fuse([ring] + leds)


def inmp441():
    """INMP441 breakout, Ø14 round PCB, acoustic port on the PCB bottom (faces -z here)."""
    pcb = cyl(14, 1.2, (0, 0, 0)).cut(cyl(1.2, 3, (0, 0, -1)))
    return fuse([pcb, box(-5, 5, 4, 6.5, 1.2, 3.5)])


def max98357a():
    """MAX98357A breakout, 18 x 19 PCB, terminal block on one edge (board frame)."""
    pcb = box(-9, 9, -9.5, 9.5, 0, 1.6)
    return fuse([pcb, box(-9, 9, -9.5, -3.5, 1.6, 8.5), cbox(0, 3, 2.2, 3, 3, 1.2)])


def speaker_40mm():
    """40 mm thin-profile speaker, cone facing +x, flange at x = 0, 6 mm deep."""
    frame = tube(40, 34, 1.5, (-1.5, 0, 0), (1, 0, 0))
    cone = Part.makeCone(17, 9, 4, V(-1.5, 0, 0), V(-1, 0, 0))
    magnet = cyl(18, 1.5, (-5.5, 0, 0), (-1, 0, 0))
    tabs = fuse([cbox(-0.75, y, 0, 1.5, 4, 8) for y in (-21, 21)])
    return fuse([frame, cone, magnet, tabs])


def projector_led():
    """1 W LED on a 15 x 12 x 1.4 board: the Open Duck Mini reflector insert's pocket (15 long along
    local y, 12 along local z). Beam along +x, board back at x = 0; the emitter goes through the
    insert's Ø8 throat."""
    board = box(0, 1.4, -7.5, 7.5, -6, 6)
    body = cyl(6, 1.2, (1.4, 0, 0), (1, 0, 0))
    dome = Part.makeSphere(2.6, V(2.6, 0, 0)).common(cyl(6, 3, (2.6, 0, 0), (1, 0, 0)))
    return fuse([board, body, dome])


def stl_solid(path):
    """An STL (e.g. an Open Duck Mini printed part) as a Part solid, in the STL's own coordinates."""
    import Mesh
    m = Mesh.Mesh(path)
    m.harmonizeNormals()
    sh = Part.Shape()
    sh.makeShapeFromMesh(m.Topology, 0.05)
    s = Part.makeSolid(sh)
    try:
        s = s.removeSplitter()
    except Exception:
        pass
    return s


def mosfet_board():
    return fuse([box(-8, 8, -6, 6, 0, 1.6), cbox(0, 0, 2.6, 6.5, 6, 2)])


# ---------------------------------------------------------------- power
def battery_6s2p():
    """12x 21700 (Ø21.55 x 70.15) lying along x, 4 across y x 3 high, centred on the origin.
    Returns (cells, wrap)."""
    cells = []
    for iy in range(4):
        for iz in range(3):
            y = -32.325 + iy * 21.55
            z = -21.55 + iz * 21.55
            cells.append(cyl(21.55, 70.15, (-35.075, y, z), (1, 0, 0)))
    cells = fuse(cells)
    wrap = cbox(0, 0, 0, 73, 89, 68).cut(cbox(0, 0, 0, 70, 86, 65))
    return cells, wrap


def bms_6s():
    """6S BMS, modeled 60 x 40 board on edge (normal +y), 8 thick incl. FETs and heatsink."""
    pcb = box(-30, 30, -4, -2.4, -20, 20)
    sink = box(-26, 26, -2.4, 4, -16, 8)
    pads = box(-30, -22, -2.4, 0, -20, 20)
    return fuse([pcb, sink, pads])


def blade_fuse_holder():
    holder = cbox(0, 0, 0, 24, 10, 16)
    fuse_ = cbox(0, 0, 10, 19, 5, 4)
    return fuse([holder, fuse_])


def antispark_switch():
    """Anti-spark switch body 40 x 20 x 16 (estimate) with the remote-switch lead exit."""
    return fuse([cbox(0, 0, 0, 40, 20, 16), cyl(6, 6, (20, 0, 0), (1, 0, 0)),
                 cyl(6, 6, (-20, 0, 0), (-1, 0, 0))])


def buck(dx, dy, dz, n_caps=2):
    """Generic buck module, PCB bottom z=0, footprint dx x dy, overall height dz."""
    pcb = box(-dx / 2, dx / 2, -dy / 2, dy / 2, 0, 1.6)
    parts = [pcb, cbox(0, 0, 1.6 + (dz - 1.6) * 0.3, dx * 0.3, dy * 0.4, (dz - 1.6) * 0.6)]
    for i in range(n_caps):
        x = -dx / 2 + dx * (i + 0.5) / (n_caps + 1.5)
        parts.append(cyl(min(dy * 0.35, 10), dz - 1.6, (x, dy * 0.2, 1.6)))
    parts.append(box(dx / 2 - 6, dx / 2, -dy / 2, dy / 2, 1.6, min(dz, 10)))  # terminals
    return fuse(parts)


def estop_22mm():
    """22 mm latching mushroom e-stop. Panel front face at x = 0, mushroom on +x, body and
    NC contact block behind (to x = -45)."""
    mush = fuse([cyl(32, 8, (12, 0, 0), (1, 0, 0)), cyl(24, 12, (0, 0, 0), (1, 0, 0))])
    nut = cyl(26, 4, (-7, 0, 0), (1, 0, 0))
    body = cyl(22, 18, (-25, 0, 0), (1, 0, 0))
    block = cbox(-35, 0, 0, 20, 28, 28)
    return fuse([mush, nut, body, block])


def gx16_charge_port():
    """GX16-2 aviation panel socket (charge input). Panel front face at x = 0, thread Ø16,
    flange on the front, solder cup body to x = -30, dust cap on the front."""
    flange = cyl(20, 3, (0, 0, 0), (1, 0, 0))
    cap = cyl(18, 10, (3, 0, 0), (1, 0, 0))
    thread = cyl(16, 18, (-18, 0, 0), (1, 0, 0))
    nut = cyl(22, 3, (-6, 0, 0), (1, 0, 0))
    cups = cyl(12, 12, (-30, 0, 0), (1, 0, 0))
    return fuse([flange, cap, thread, nut, cups])


# ---------------------------------------------------------------- wheel
def wheel_hub():
    """Printed wheel hub, foot frame (origin = wheel centre, axis y). The motor output face is
    at y = +2.5 (left wheel); web y -3.5..+2.5, rim Ø138/Ø128 y -16..+16."""
    web = cyl(130, 6, (0, -3.5, 0), (0, 1, 0))
    rim = tube(138, 128, 32, (0, -16, 0), (0, 1, 0))
    hub = fuse([web, rim])
    cuts = bolt_circle(27, 3, 3.4, 10, (0, -5, 0), "y", 90)
    cuts += [cyl(10, 10, (0, -5, 0), (0, 1, 0))]
    cuts += bolt_circle(92, 6, 24, 10, (0, -5, 0), "y", 30)        # lightening
    cuts += bolt_circle(133, 12, 5, 40, (0, -20, 0), "y", 15)       # tread key holes
    for y in (-8, 0, 8):                                             # key grooves
        cuts.append(tube(140, 136, 1.2, (0, y - 0.6, 0), (0, 1, 0)))
    return hub.cut(cuts)


def tire():
    """Cast VytaFlex 40 tread: Ø150 x 32, 6 thick, crowned edges R4."""
    t = tube(150, 138, 32, (0, -16, 0), (0, 1, 0))
    try:
        edges = [e for e in t.Edges if abs(e.Curve.Radius - 75) < 0.1]
        t = t.makeFillet(4, edges)
    except Exception:
        pass
    return t
