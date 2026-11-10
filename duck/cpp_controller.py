"""The C++ controller (firmware/controller/) as a drop-in for `ScaledWheeledDuckController` in the sim.

`CppController` has the Python controller's interface (constructor, `compute_control`, `handle_keypress`, the
attributes the sim and tests set and read), so `test_scenarios.py --controller cpp` runs the firmware's control
code against the MuJoCo plant. The shared library is built on first use into `build/` (needs a C++17 compiler).

`config_values` is the one place the C++ `Config` gets its numbers: from the Python controller class, its profile
and its LQR gains. tools/export_firmware_config.py writes the same values into the firmware header.
"""

import ctypes
import math
import os
import pathlib
import subprocess
import tempfile

import numpy as np

from .controller import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_UP, ScaledWheeledDuckController
from .robot import DEFAULT_ROBOT, PROFILES, RobotProfile, ScaledWheeledDuckParams

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "firmware" / "controller"
SOURCES = [SRC_DIR / "duck_controller.cpp", SRC_DIR / "duck_capi.cpp"]
LIB_PATH = ROOT / "build" / "libduck_controller.so"
# -ffp-contract=off: no fused multiply-adds, so the arithmetic rounds like Python's
CXXFLAGS = ["-std=c++17", "-O2", "-fPIC", "-shared", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror"]

PHASES = ("idle", "crouch", "thrust", "flight", "land", "drop")      # duck::JumpPhase order
THRUST_MODES = {"lqr": 0.0, "attitude": 1.0, "zero": 2.0}
OPTIONAL = ("yaw_ref", "bus_voltage", "filtered_voltage", "slope_obs", "slope_before")   # None <-> has_<name> false
ARRAYS = {"head_pose": (4, True), "gyro_bias": (3, False), "lean_filter": (3, True)}   # name: (length, optional)
# duck::Command values, by the keys `ScaledWheeledDuckController.handle_keypress` takes
KEY_COMMANDS = {}
for _cmd, _keys in enumerate([(KEY_UP, "Ii", "Ww"), (KEY_DOWN, "Kk", "Ss"), (KEY_LEFT, "Jj", "Aa"),
                              (KEY_RIGHT, "Ll", "Dd"), (KEY_SPACE,), ("Zz",), ("Xx",), ("Uu",), ("Oo",), ("Cc",),
                              ("Qq",), ("Ee",), ("Pp",), ("Rr",)]):
    for _k in _keys:
        for _code in ([_k] if isinstance(_k, int) else [ord(ch) for ch in _k]):
            KEY_COMMANDS[_code] = _cmd


def build(force: bool = False) -> pathlib.Path:
    """Compiles the shared library if it is missing or older than its sources."""
    headers = list(SRC_DIR.glob("*.hpp"))
    newest = max(p.stat().st_mtime for p in SOURCES + headers)
    if not force and LIB_PATH.exists() and LIB_PATH.stat().st_mtime >= newest:
        return LIB_PATH
    LIB_PATH.parent.mkdir(exist_ok=True)
    cxx = os.environ.get("CXX", "c++")
    # Build to a temporary name and rename: parallel test workers may all try at once
    fd, tmp = tempfile.mkstemp(dir=LIB_PATH.parent, suffix=".so")
    os.close(fd)
    try:
        subprocess.run([cxx, *CXXFLAGS, *map(str, SOURCES), "-o", tmp], check=True)
        os.replace(tmp, LIB_PATH)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return LIB_PATH


_lib = None
_LOG_FN = ctypes.CFUNCTYPE(None, ctypes.c_char_p)
_log_callback = _LOG_FN(lambda msg: print(msg.decode()))   # module-level: must outlive every controller


def lib() -> ctypes.CDLL:
    global _lib
    if _lib is None:
        L = ctypes.CDLL(str(build()))
        vp, d, i = ctypes.c_void_p, ctypes.c_double, ctypes.c_int
        dp = ctypes.POINTER(ctypes.c_double)
        for name, res, args in [
            ("duck_config_count", i, []), ("duck_config_name", ctypes.c_char_p, [i]),
            ("duck_state_count", i, []), ("duck_state_name", ctypes.c_char_p, [i]), ("duck_state_kind", i, [i]),
            ("duck_create", vp, [dp, _LOG_FN]), ("duck_destroy", None, [vp]),
            ("duck_state_get", d, [vp, i]), ("duck_state_set", None, [vp, i, d]),
            ("duck_compute", None, [vp, dp, dp]), ("duck_command", None, [vp, i]), ("duck_reset_state", None, [vp]),
            ("duck_speed_cap", d, [vp]), ("duck_leg_servo_kd", d, [vp]),
            ("duck_deliverable_wheel_torque", d, [vp, d, d]),
        ]:
            fn = getattr(L, name)
            fn.restype, fn.argtypes = res, args
        L.config_names = [L.duck_config_name(k).decode() for k in range(L.duck_config_count())]
        L.state_index = {L.duck_state_name(k).decode(): k for k in range(L.duck_state_count())}
        L.state_kind = {n: L.duck_state_kind(k) for n, k in L.state_index.items()}
        _lib = L
    return _lib


def config_values(profile: RobotProfile, params: ScaledWheeledDuckParams = None,
                  cls=ScaledWheeledDuckController) -> dict:
    """{C++ Config field: value}, in field order, from the Python controller `cls` built for `profile`."""
    py = cls(profile, params)
    P, prof = py.params, py.profile
    curve = prof.wheel_torque_curve
    special = {
        "posture_a": prof.posture_coeffs[0], "posture_b": prof.posture_coeffs[1],
        "initial_pitch_trim": prof.initial_pitch_trim, "jump_thrust_time": prof.jump_thrust_time,
        "thrust_wheel_mode": THRUST_MODES[prof.thrust_wheel_mode], "has_wheel_curve": float(curve is not None),
        "wheel_curve_stall": curve[0] if curve else 0.0, "wheel_curve_no_load_rpm": curve[1] if curve else 0.0,
        "wheel_curve_rpm_per_nm": curve[2] if curve else 0.0,
        "lean_c_x": py.lean_accel_coeffs[0], "lean_c_p": py.lean_accel_coeffs[1],
        **{f"k_lqr_{k}": py.k_lqr[k] for k in range(4)},
    }
    out = {}
    for name in lib().config_names:
        if name in special:
            v = special[name]
        elif hasattr(P, name):
            v = getattr(P, name)
        elif name.endswith(("_LO", "_HI")) and isinstance(getattr(cls, name[:-3], None), tuple):
            v = getattr(cls, name[:-3])[0 if name.endswith("_LO") else 1]
        else:
            v = getattr(cls, name)   # AttributeError: the C++ Config has a field the Python controller doesn't
        out[name] = float(v)
    return out


def python_constants(cls=ScaledWheeledDuckController) -> set:
    """The Python controller's tunables, as C++ Config names (to check the C++ side has every one)."""
    names = set()
    for name in dir(cls):
        if name.isupper() and not name.startswith("_"):
            v = getattr(cls, name)
            names |= {name + "_LO", name + "_HI"} if isinstance(v, tuple) else {name}
    return names


class CppController:
    """`ScaledWheeledDuckController`'s interface, running the C++ controller."""

    def __init__(self, profile: RobotProfile = None, params: ScaledWheeledDuckParams = None):
        L = lib()
        profile = profile or PROFILES[DEFAULT_ROBOT]
        values = config_values(profile, params)
        arr = (ctypes.c_double * len(values))(*values.values())
        object.__setattr__(self, "_lib", L)
        object.__setattr__(self, "_h", L.duck_create(arr, _log_callback))
        object.__setattr__(self, "profile", profile)
        object.__setattr__(self, "params", params or profile.params)
        object.__setattr__(self, "config", values)
        object.__setattr__(self, "k_lqr", np.array([values[f"k_lqr_{k}"] for k in range(4)]))
        object.__setattr__(self, "lean_accel_coeffs", (values["lean_c_x"], values["lean_c_p"]))
        object.__setattr__(self, "_sensors", (ctypes.c_double * 15)())
        object.__setattr__(self, "_out", (ctypes.c_double * 22)())

    def __del__(self):
        h = self.__dict__.get("_h")
        if h:
            self._lib.duck_destroy(h)

    # --- state fields, by their Python names ---
    def _get(self, name):
        L = self._lib
        return L.duck_state_get(self._h, L.state_index[name])

    def _set(self, name, value):
        L = self._lib
        L.duck_state_set(self._h, L.state_index[name], float(value))

    def __getattr__(self, name):
        L = self.__dict__.get("_lib")
        if L is None:
            raise AttributeError(name)
        if name in ARRAYS:
            n, optional = ARRAYS[name]
            if optional and not self._get("has_" + name):
                return None
            vals = [self._get(f"{name}_{k}") for k in range(n)]
            return tuple(vals) if name == "head_pose" else (list(vals) if name == "lean_filter" else np.array(vals))
        if name in OPTIONAL:
            return self._get(name) if self._get("has_" + name) else None
        if name in L.state_index:
            v = self._get(name)
            kind = L.state_kind[name]
            return bool(v) if kind == 1 else PHASES[int(v)] if kind == 2 else v
        if name.isupper():   # class constants
            return getattr(ScaledWheeledDuckController, name)
        raise AttributeError(name)

    def __setattr__(self, name, value):
        L = self._lib
        if name in ARRAYS:
            n, optional = ARRAYS[name]
            if optional:
                self._set("has_" + name, value is not None)
            if value is not None:
                for k in range(n):
                    self._set(f"{name}_{k}", value[k])
        elif name in OPTIONAL:
            self._set("has_" + name, value is not None)
            if value is not None:
                self._set(name, value)
        elif name in L.state_index:
            self._set(name, PHASES.index(value) if L.state_kind[name] == 2 else value)
        else:
            object.__setattr__(self, name, value)

    # --- methods ---
    def compute_control(self, quat_wxyz, gyro_xyz, left_wheel_pos, right_wheel_pos, left_wheel_vel, right_wheel_vel,
                        leg_fold=None, leg_fold_rate=0.0, accel_mag=None, dt=0.002):
        s = self._sensors
        s[0:4] = [float(v) for v in quat_wxyz]
        s[4:7] = [float(v) for v in gyro_xyz]
        s[7:15] = [float(left_wheel_pos), float(right_wheel_pos), float(left_wheel_vel), float(right_wheel_vel),
                   math.nan if leg_fold is None else float(leg_fold), float(leg_fold_rate),
                   math.nan if accel_mag is None else float(accel_mag), float(dt)]
        self._lib.duck_compute(self._h, s, self._out)
        o = self._out
        ctrl = np.array(o[0:14])
        roll, pitch, yaw, x_pos, x_vel, tau_left, tau_right, balance_pitch = o[14:22]
        telemetry = {
            "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch), "yaw_deg": math.degrees(yaw),
            "x_pos": x_pos, "x_vel": x_vel, "cmd_vel": self.cmd_vel,
            "target_yaw_deg": math.degrees(self.target_yaw), "tau_left": tau_left, "tau_right": tau_right,
            "pitch_trim_deg": math.degrees(balance_pitch), "slope_deg": math.degrees(math.asin(self.slope_sin)),
            "jump_phase": self.jump_phase, "fallen": self.fallen,
        }
        return ctrl, telemetry

    def handle_keypress(self, keycode: int) -> None:
        cmd = KEY_COMMANDS.get(keycode)
        if cmd is not None:
            self._lib.duck_command(self._h, cmd)

    def reset_state(self) -> None:
        self._lib.duck_reset_state(self._h)

    def speed_cap(self) -> float:
        return self._lib.duck_speed_cap(self._h)

    def leg_servo_kd(self) -> float:
        return self._lib.duck_leg_servo_kd(self._h)

    def deliverable_wheel_torque(self, tau: float, omega: float) -> float:
        return self._lib.duck_deliverable_wheel_torque(self._h, float(tau), float(omega))

    def posture_trim(self, fold: float) -> float:
        a, b = self.profile.posture_coeffs
        return a * fold + b * fold * fold

    def posture_trim_slope(self, fold: float) -> float:
        a, b = self.profile.posture_coeffs
        return a + 2.0 * b * fold


class CompareController:
    """Runs the Python and the C++ controller side by side on the same sensor readings: the sim is driven by the
    Python one, and every tick each command and every state field of the two must agree to `TOL`. The first
    disagreement raises, naming the field and the sim time. (`--controller compare` in the sim and test suites.)"""

    TOL = 1e-9

    def __init__(self, profile: RobotProfile = None, params: ScaledWheeledDuckParams = None):
        missing = python_constants() - set(lib().config_names)
        if missing:
            raise AssertionError(f"Python controller constants missing from the C++ Config: {sorted(missing)}")
        object.__setattr__(self, "py", ScaledWheeledDuckController(profile, params))
        object.__setattr__(self, "cpp", CppController(profile, params))
        object.__setattr__(self, "t", 0.0)
        object.__setattr__(self, "max_diff", 0.0)
        L = lib()
        array_parts = {f"{n}_{k}" for n, (size, _) in ARRAYS.items() for k in range(size)}
        names = [n for n in L.state_index if not n.startswith("has_") and n not in array_parts]
        object.__setattr__(self, "_fields", [n for n in names if n not in OPTIONAL] + list(OPTIONAL) + list(ARRAYS))

    def __getattr__(self, name):
        return getattr(self.__dict__["py"], name)

    def __setattr__(self, name, value):
        setattr(self.py, name, value)
        setattr(self.cpp, name, value)

    def _check(self, what, a, b):
        if a is None or b is None or isinstance(a, (str, bool)) or isinstance(b, (str, bool)):
            ok = a == b if not isinstance(a, bool) else bool(a) == bool(b)
            diff = 0.0
        else:
            diff = float(np.max(np.abs(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))))
            ok = diff <= self.TOL
        if not ok:
            raise AssertionError(f"C++ controller differs at t={self.t:.3f} s: {what} python={a!r} cpp={b!r}")
        object.__setattr__(self, "max_diff", max(self.max_diff, diff))

    def compute_control(self, *args, **kwargs):
        ctrl, info = self.py.compute_control(*args, **kwargs)
        ctrl_c, info_c = self.cpp.compute_control(*args, **kwargs)
        object.__setattr__(self, "t", self.t + kwargs.get("dt", 0.002))
        self._check("ctrl", ctrl, ctrl_c)
        for k, v in info.items():
            self._check(f"telemetry[{k}]", v, info_c[k])
        for name in self._fields:
            self._check(name, getattr(self.py, name), getattr(self.cpp, name))
        self._check("leg_servo_kd", self.py.leg_servo_kd(), self.cpp.leg_servo_kd())
        return ctrl, info

    def handle_keypress(self, keycode: int) -> None:
        self.py.handle_keypress(keycode)
        self.cpp.handle_keypress(keycode)

    def reset_state(self) -> None:
        self.py.reset_state()
        self.cpp.reset_state()


CONTROLLERS = {"py": ScaledWheeledDuckController, "cpp": CppController, "compare": CompareController}


def make_controller(profile: RobotProfile = None, kind: str = None):
    """The controller the sim and tests run: `kind`, else $DUCK_CONTROLLER, else "py" (the Python reference)."""
    kind = kind or os.environ.get("DUCK_CONTROLLER", "py")
    return CONTROLLERS[kind](profile)
