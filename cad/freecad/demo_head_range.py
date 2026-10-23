"""Animate the head through its clash-free range (see neck_envelope.md) in the FreeCAD GUI.

Run in FreeCAD after build_super_duck.py (the MCP execute_code tool, or the Python console):
    exec(open("<repo>/cad/freecad/demo_head_range.py").read())

Every pose stays inside the envelope: neck_pitch + head_pitch in [-15, +65] deg, head_pitch <= +15 deg,
yaw within +-94.5, roll within +-30. Each move's name shows in the status bar. Stop early with
head_demo.stop(); the head goes back to its home pose at the end either way.
"""
import FreeCAD as App
import FreeCADGui as Gui
from PySide import QtCore

V = App.Vector
DOC = App.getDocument("SuperDuck")
JOINTS = [("neck_pitch_assembly", "neck_pitch", V(0, 1, 0)), ("head_pitch_to_yaw", "head_pitch", V(0, 1, 0)),
          ("neck_yaw_assembly", "head_yaw", V(0, 0, 1)), ("head_assembly", "head_roll", V(1, 0, 0))]

# (label, (neck_pitch, head_pitch, head_yaw, head_roll) deg, seconds to get there)
MOVES = [
    ("home", (0, 0, 0, 0), 0.8),
    ("look left: yaw +94.5", (0, 0, 94.5, 0), 1.6),
    ("look right: yaw -94.5", (0, 0, -94.5, 0), 2.6),
    ("centre", (0, 0, 0, 0), 1.3),
    ("tilt: roll +30", (0, 0, 0, 30), 1.0),
    ("tilt: roll -30", (0, 0, 0, -30), 1.6),
    ("centre", (0, 0, 0, 0), 0.8),
    ("neck back, look up: neck -15", (-15, 0, 0, 0), 1.0),
    ("neck forward, look up: neck +30, head -45", (30, -45, 0, 0), 1.6),
    ("nod down: neck +65", (65, 0, 0, 0), 1.8),
    ("head down: neck +50, head +15", (50, 15, 0, 0), 1.0),
    ("curious: neck +20, head -10, yaw +70, roll +25", (20, -10, 70, 25), 1.8),
    ("curious: neck +20, head -10, yaw -70, roll -25", (20, -10, -70, -25), 2.4),
    ("look up and around: neck +40, head -45, yaw +90", (40, -45, 90, 0), 1.8),
    ("look down and around: neck +55, head +10, yaw -90", (55, 10, -90, 0), 2.4),
    ("home", (0, 0, 0, 0), 1.6),
]
DT = 0.03


class HeadDemo:
    def __init__(self):
        self.parts = [DOC.getObject(name) for name, _, _ in JOINTS]
        self.home = [p.Placement for p in self.parts]
        self.frames = []
        cur = MOVES[0][1]
        for label, target, secs in MOVES:
            n = max(1, int(secs / DT))
            for i in range(1, n + 1):
                t = i / n
                t = t * t * (3 - 2 * t)                        # ease in/out
                self.frames.append((label, [a + (b - a) * t for a, b in zip(cur, target)]))
            cur = target
        self.i = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.step)

    def pose(self, ang):
        for p, h, (_, _, axis), a in zip(self.parts, self.home, JOINTS, ang):
            p.Placement = App.Placement(h.Base, App.Rotation(axis, a))

    def step(self):
        if self.i >= len(self.frames):
            return self.stop()
        label, ang = self.frames[self.i]
        self.pose(ang)
        Gui.getMainWindow().statusBar().showMessage(
            f"Head range demo: {label}   [neck {ang[0]:+.0f}  head {ang[1]:+.0f}  yaw {ang[2]:+.0f}  roll {ang[3]:+.0f}]")
        self.i += 1

    def start(self):
        self.timer.start(int(DT * 1000))

    def stop(self):
        self.timer.stop()
        for p, h in zip(self.parts, self.home):
            p.Placement = h
        Gui.getMainWindow().statusBar().showMessage("Head range demo: done", 5000)


try:
    head_demo.stop()            # noqa: F821  (a previous run still going)
except NameError:
    pass
head_demo = HeadDemo()
head_demo.start()
