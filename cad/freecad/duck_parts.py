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


def cable(points, d=6.0, balls=()):
    """A cable of diameter d along the polyline `points`: a cylinder per segment, a ball at each inner corner
    and at the ends listed in `balls` (0 and/or -1)."""
    pts = [V(*p) for p in points]
    parts = [Part.makeCylinder(d / 2, (b - a).Length, a, b - a) for a, b in zip(pts, pts[1:]) if (b - a).Length > 1e-6]
    parts += [Part.makeSphere(d / 2, pts[i]) for i in list(range(1, len(pts) - 1)) + [i % len(pts) for i in balls]]
    return fuse(parts)


def grommet(hole_d, wall, id_, flange_d, flange_t=1.5):
    """Rubber grommet along +y for a hole_d hole in a wall y 0..wall: core plus a flange each side."""
    return fuse([tube(hole_d, id_, wall, (0, 0, 0), (0, 1, 0)),
                 tube(flange_d, id_, flange_t, (0, -flange_t, 0), (0, 1, 0)),
                 tube(flange_d, id_, flange_t, (0, wall, 0), (0, 1, 0))])


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


def round_corners(shape, picks, strict=True):
    """Fillet straight edges picked by their midpoint: picks = [((x, y, z), r), ...], None = any coordinate.
    strict=False: a pick whose fillet fails (or matches no edge) is skipped and reported instead of raising."""
    for r in sorted({r for _, r in picks}, reverse=True):
        want = [p for p, rr in picks if rr == r]
        groups = [want] if strict else [[p] for p in want]
        for g in groups:
            edges = [e for e in shape.Edges if e.Curve.TypeId == "Part::GeomLine" and any(
                all(c is None or abs(m - c) < 0.01 for m, c in zip(e.CenterOfMass, p)) for p in g)]
            if strict:
                shape = shape.makeFillet(r, edges)
                continue
            try:
                if not edges:
                    raise ValueError("no edge")
                shape = shape.makeFillet(r, edges)
            except Exception as ex:
                print(f"round_corners: skipped R{r} at {g[0]}: {ex}")
    return shape


def teardrop(d, h, base, axis, flat=1.15):
    """Horizontal hole of diameter d, length h from base along axis ((1, 0, 0) or (0, 1, 0)), with a 45 deg
    roof toward +z truncated at flat * r above the centre, so it prints without support."""
    r = d / 2.0
    k, t = r / math.sqrt(2), (math.sqrt(2) - flat) * r
    pts2 = [(-k, k), (-t, flat * r), (t, flat * r), (k, k), (0, 0)]
    if abs(axis[0]) > 0.5:
        pts = [V(base[0], base[1] + u, base[2] + w) for u, w in pts2]
    else:
        pts = [V(base[0] + u, base[1], base[2] + w) for u, w in pts2]
    roof = Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(*axis) * h)
    return fuse([cyl(d, h, base, axis), roof])


def rounded_plate_xz(points, y0, y1, fillet=0.0):
    """Convex hull of circles [(x, z, r), ...] extruded along y from y0 to y1 (a link plate). fillet > 0
    rounds the outline edges of both faces."""
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
    plate = fuse(solids).removeSplitter()
    if fillet > 0:
        plate = plate.makeFillet(fillet, [e for e in plate.Edges if e.BoundBox.YLength < 1e-6])
    return plate


def rounded_plate_yz(points, x0, x1):
    """Convex hull of circles [(y, z, r), ...] extruded along x from x0 to x1."""
    solids = [cyl(2 * r, x1 - x0, (x0, y, z), (1, 0, 0)) for y, z, r in points]
    for (ya, za, ra), (yb, zb, rb) in zip(points, points[1:]):
        dy, dz = yb - ya, zb - za
        L = math.hypot(dy, dz)
        # outer tangents of two circles: normal n with n.(b - a) = ra - rb
        c = (ra - rb) / L
        sn = math.sqrt(max(0.0, 1 - c * c))
        ux, uz = dy / L, dz / L
        poly = []
        for sg in (1, -1):
            ny, nz = c * ux - sg * sn * uz, c * uz + sg * sn * ux
            poly.append((V(x0, ya + ny * ra, za + nz * ra), V(x0, yb + ny * rb, zb + nz * rb)))
        pts = [poly[0][0], poly[0][1], poly[1][1], poly[1][0]]
        solids.append(Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(x1 - x0, 0, 0)))
    return fuse(solids).removeSplitter()


def truss_cutouts(a, b, u0, u1, w0, w1, n, web, y0, y1, r=2.5, w_in=None):
    """Warren-truss triangular windows (the Impulse leg look) along the link a -> b, points (x, z).
    The band runs from u0 to u1 mm along the link with half-width w0 -> w1 (the plate's edge minus its
    rim); n triangles alternate apex up / down, struts `web` wide, corners rounded r. With w_in, the nodes
    alternate between the edge (w0 -> w1, signed) and the line w_in instead: one row of a two-row truss
    either side of a spine. Returns prisms y0..y1."""
    ax, az = a
    L = math.hypot(b[0] - ax, b[1] - az)
    ux, uz = (b[0] - ax) / L, (b[1] - az) / L
    nx, nz = -uz, ux
    d = (u1 - u0) / (n + 1)

    def node(i):
        u = u0 + i * d
        w = w0 + (w1 - w0) * (u - u0) / (u1 - u0)
        if w_in is None:
            w = w if i % 2 == 0 else -w
        elif i % 2:
            w = w_in
        return V(ax + ux * u + nx * w, y0, az + uz * u + nz * w)

    out = []
    for i in range(n):
        face = Part.Face(Part.makePolygon([node(i), node(i + 1), node(i + 2), node(i)]))
        face = face.makeOffset2D(-(web / 2 + r), 0).makeOffset2D(r, 0)
        out.append(face.extrude(V(0, y1 - y0, 0)))
    return out


def slotted_can(od, id_, h, n, slot_w, slot_l, phase=0.0, skip=()):
    """Motor can (the Impulse hip-casing look) along +z from z = 0 to h, with n stadium windows
    slot_w wide and slot_l long (along z) centred at h / 2; window i sits at phase + 360 i / n deg from +x."""
    can = tube(od, id_, h)
    cuts = []
    for i in range(n):
        if i in skip:
            continue
        x0, x1 = id_ / 2 - 3, od / 2 + 3
        e = (slot_l - slot_w) / 2
        s = fuse([box(x0, x1, -slot_w / 2, slot_w / 2, h / 2 - e, h / 2 + e),
                  cyl(slot_w, x1 - x0, (x0, 0, h / 2 - e), (1, 0, 0)),
                  cyl(slot_w, x1 - x0, (x0, 0, h / 2 + e), (1, 0, 0))])
        s.rotate(V(0, 0, 0), V(0, 0, 1), phase + 360.0 * i / n)
        cuts.append(s)
    return can.cut(cuts) if cuts else can


def along_y(shape, y0=0.0):
    """Turn a part built along +z (z = 0..h) so it runs along +y from y = y0 (x is kept)."""
    s = shape.copy()
    s.rotate(V(0, 0, 0), V(1, 0, 0), -90)
    s.translate(V(0, y0, 0))
    return s


def along_x(shape, x0=0.0):
    """Turn a part built along +z (z = 0..h) so it runs along +x from x = x0."""
    s = shape.copy()
    s.rotate(V(0, 0, 0), V(0, 1, 0), 90)
    s.translate(V(x0, 0, 0))
    return s


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
    """Adafruit 3968 40 mm 4 ohm speaker (Ø40 x 20 overall, no mounting tabs: clamped by its rim), cone facing
    +x, rim front at x = 0. Rim 2 mm, basket tapering to a Ø22 magnet cup at the back."""
    rim = tube(40, 34, 2, (-2, 0, 0), (1, 0, 0))
    basket = Part.makeCone(20, 13, 9, V(-2, 0, 0), V(-1, 0, 0))
    magnet = cyl(22, 9, (-20, 0, 0), (1, 0, 0))
    return fuse([rim, basket, magnet])


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
    """Heltec 6S 80 A split-port BMS, 83 x 55 x 8 board on edge (normal +y): PCB at y -4..-2.4, FETs and
    their sink to y +4; B- / P- / C- pads at the -x end."""
    pcb = box(-41.5, 41.5, -4, -2.4, -27.5, 27.5)
    sink = box(-30, 38, -2.4, 4, -24, 20)
    pads = box(-41.5, -32, -2.4, 0, -27.5, 27.5)
    return fuse([pcb, sink, pads])


def blade_fuse_holder():
    holder = cbox(0, 0, 0, 24, 10, 16)
    fuse_ = cbox(0, 0, 10, 19, 5, 4)
    return fuse([holder, fuse_])


def antispark_switch():
    """Flipsky Antispark Switch Pro V3.0, heatsink version: 42.2 x 33 x 26.6 with heat shrink, 12 AWG leads
    out of both ends along x (lead stubs Ø6). Bottom at z -8 (placed as the old 16 mm box was)."""
    return fuse([box(-21.1, 21.1, -16.5, 16.5, -8, 18.6), cyl(6, 6, (21.1, 0, 0), (1, 0, 0)),
                 cyl(6, 6, (-21.1, 0, 0), (-1, 0, 0))])


def pololu_d36v50():
    """Pololu D36V50Fx step-down regulator (F5 = 5 V, F7 = 7.5 V): 25.4 x 25.4 x 9.5, PCB bottom z = 0,
    3x M2 holes (Ø2.18) in three corners, terminals along the -x edge."""
    pcb = box(-12.7, 12.7, -12.7, 12.7, 0, 1.6).cut(
        [cyl(2.18, 3, (x, y, -0.5)) for x, y in ((-10.2, 10.2), (-10.2, -10.2), (10.2, -10.2))])
    ind = box(-6, 7, -1, 12, 1.6, 9.5)                                   # 4.7 uH inductor
    caps = [cyl(8, 8, (8, -3, 1.6)), cyl(8, 8, (3, -9, 1.6))]
    return fuse([pcb, ind] + caps)


def buck(dx, dy, dz, n_caps=2):
    """Generic buck module, PCB bottom z=0, footprint dx x dy, overall height dz."""
    pcb = box(-dx / 2, dx / 2, -dy / 2, dy / 2, 0, 1.6)
    parts = [pcb, cbox(0, 0, 1.6 + (dz - 1.6) * 0.3, dx * 0.3, dy * 0.4, (dz - 1.6) * 0.6)]
    for i in range(n_caps):
        x = -dx / 2 + dx * (i + 0.5) / (n_caps + 1.5)
        parts.append(cyl(min(dy * 0.35, 10), dz - 1.6, (x, dy * 0.2, 1.6)))
    parts.append(box(dx / 2 - 6, dx / 2, -dy / 2, dy / 2, 1.6, min(dz, 10)))  # terminals
    return fuse(parts)


def fan_40x10():
    """40 x 40 x 10 axial fan (e.g. Noctua NF-A4x10 5V PWM), axis z, frame z 0..10, 4x M3 holes on 32 mm."""
    frame = box(-20, 20, -20, 20, 0, 10).cut([cyl(38, 12, (0, 0, -1))] +
                                             [cyl(3.4, 12, (x, y, -1)) for x in (-16, 16) for y in (-16, 16)])
    hub = cyl(15, 9, (0, 0, 0.5))
    spokes = []
    for ang, (z0, z1, r1) in [(45 + 51.4 * i, (1.5, 8.5, 18.5)) for i in range(7)] + \
                             [(45 + 90 * i, (0, 1.5, 19.5)) for i in range(4)]:   # blades, then struts
        sp = box(7, r1, -1, 1, z0, z1)
        sp.rotate(V(0, 0, 0), V(0, 0, 1), ang)
        spokes.append(sp)
    return fuse([frame, hub] + spokes)


def estop_22mm():
    """RAFI LUMOTAST 22 latching e-stop (Ø22.3 cutout, 17.9 mm behind the panel, 0.8 mm tab terminals).
    Panel front face at x = 0, mushroom on +x. Head size not published: modelled Ø32 x 20 proud (verify)."""
    mush = fuse([cyl(32, 8, (12, 0, 0), (1, 0, 0)), cyl(24, 12, (0, 0, 0), (1, 0, 0))])
    nut = cyl(26, 4, (-7, 0, 0), (1, 0, 0))
    body = cyl(22, 10.9, (-17.9, 0, 0), (1, 0, 0))
    return fuse([mush, nut, body])


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
    cuts += bolt_circle(133, 12, 3, 40, (0, -20, 0), "y", 15)       # tread key holes (1 mm wall each side)
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


def radial_windows(c, r0, r1, n, half_deg, web, y0, y1, r=2.0, phase=0.0):
    """n triangular windows around a hub at c = (x, z) in an x-z plate: apex at radius r0, base at r1
    spanning +-half_deg, struts `web` wide, corners rounded r. Returns prisms y0..y1."""
    out = []
    for i in range(n):
        a = math.radians(phase + 360.0 * i / n)
        h = math.radians(half_deg)
        pts = [V(c[0] + r0 * math.sin(a), y0, c[1] + r0 * math.cos(a)),
               V(c[0] + r1 * math.sin(a - h), y0, c[1] + r1 * math.cos(a - h)),
               V(c[0] + r1 * math.sin(a + h), y0, c[1] + r1 * math.cos(a + h))]
        face = Part.Face(Part.makePolygon(pts + [pts[0]]))
        face = face.makeOffset2D(-(web / 2 + r), 0).makeOffset2D(r, 0)
        out.append(face.extrude(V(0, y1 - y0, 0)))
    return out


def end_cap(od, t, rim_h, rim_id, n=6, notch_deg=None, notch_w=10.0):
    """Slotted motor end cap (the Impulse actuator back plate) along +z: a disc z 0..t with n kidney slots,
    and a rim ring (od / rim_id) from z t to t + rim_h that seats on the part behind it. notch_deg: angle in
    the x-y plane (from +x) of a cable notch through the rim and the disc edge."""
    disc = cyl(od, t)
    r0, r1 = 0.24 * od, 0.40 * od
    slots = []
    for i in range(n):
        a0 = math.radians(360.0 * i / n + 8)
        a1 = math.radians(360.0 * (i + 1) / n - 8)
        pts = [V(r0 * math.cos(a0 + (a1 - a0) * k / 8), r0 * math.sin(a0 + (a1 - a0) * k / 8), -1) for k in range(9)]
        pts += [V(r1 * math.cos(a1 - (a1 - a0) * k / 8), r1 * math.sin(a1 - (a1 - a0) * k / 8), -1) for k in range(9)]
        face = Part.Face(Part.makePolygon(pts + [pts[0]])).makeOffset2D(-1.5, 0).makeOffset2D(1.5, 0)
        slots.append(face.extrude(V(0, 0, t + 2)))
    cap = fuse([disc.cut(slots), tube(od, rim_id, rim_h, (0, 0, t))])
    if notch_deg is not None:
        n_ = box(od / 2 - 6, od / 2 + 2, -notch_w / 2, notch_w / 2, -1, t + rim_h + 1)
        n_.rotate(V(0, 0, 0), V(0, 0, 1), notch_deg)
        cap = cap.cut(n_)
    return cap
