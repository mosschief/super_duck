#!/usr/bin/env python3
"""
Active Balancing, Posture & Live Keyboard Driving Controller for the 1.35x Scaled
Wheeled Open Duck Mini v2 (based on `apirrone/Open_Duck_Mini`).

Robot profiles (`--robot`): `gim6010` (default, the build) = SteadyWin GIM6010-8 on hip-pitch and knees,
6S2P battery (`open_duck_wheeled_v2_gim6010.xml`); `gim4305` = the original all-GIM4305-10 design with a 6S1P pack
(`open_duck_wheeled_v2.xml`).

Live Keyboard Driving Controls (inside the 3D MuJoCo Viewer Window):
  - Up Arrow    or 'I' / 'W' : Increase forward speed (+0.15 m/s per press, up to +1.50 m/s)
  - Down Arrow  or 'K' / 'S' : Decrease speed / reverse (-0.15 m/s per press, down to -1.50 m/s)
  - Left Arrow  or 'J' / 'A' : Steer left (+15 deg heading step per press)
  - Right Arrow or 'L' / 'D' : Steer right (-15 deg heading step per press)
  - Spacebar                 : Brake to a full stop (zero speed & hold current position)
  - 'Z' / 'X'                : Crouch lower ('Z') or stand taller ('X') using symmetric hip/knee kinematics
  - 'U' / 'O'                : Pan the BDX binocular head left ('U') / right ('O')
  - 'C'                      : Jump (deep crouch -> explosive leg thrust -> flight -> soft landing)
  - 'Q' / 'E'                : Raise ('Q') / lower ('E') the antennas (Open Duck Mini: gamepad triggers)
  - 'P'                      : Toggle the head projector light (Open Duck Mini: X button)
  - 'R'                      : Reset robot to the home balanced keyframe at the origin

Usage:
  pip install mujoco scipy numpy
  python3 simulate_wheeled_duck.py                    # default robot (gim6010)
  python3 simulate_wheeled_duck.py --robot gim4305    # original GIM4305-only design
  python3 simulate_wheeled_duck.py --realism 3        # randomized robot, Teensy 4.0 timing, fast IMU
  python3 simulate_wheeled_duck.py --realism 3 --arch pi --imu bno055  # compare architectures / IMUs
"""

import argparse
import math
import pathlib
import time
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from scipy.linalg import solve_continuous_are

# GLFW Keycode Constants Used by MuJoCo Viewer `key_callback`
KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_SPACE = 32


@dataclass
class ScaledWheeledDuckParams:
    """Physical parameters for LQR synthesis. Body terms default to the stock GIM4305 model; each `RobotProfile`
    sets its own (derived from its MJCF with the wheels excluded, as the wheeled-inverted-pendulum model requires)."""
    body_mass: float = 3.385        # 1.35x scaled trunk (1.05 kg) + head/neck (0.705 kg) + 2x upper/lower legs (1.63 kg)
    wheel_mass: float = 0.255       # Single 75 mm wheel + GIM4305-10 hub mass (kg)
    wheel_radius: float = 0.075     # 1.35x scaled wheel tire radius (m)
    wheel_track: float = 0.324      # Lateral distance between left and right wheel centers (m)
    com_height: float = 0.2555      # Body (excl. wheels) CoM height above the axle at `home_balanced` (m). The old 0.222
                                    # was the whole-robot CoM *including* the wheels, which the WIP model excludes.
    body_pitch_inertia: float = 0.0559  # Body (excl. wheels) pitch moment of inertia about its CoM (kg*m^2)
    wheel_inertia: float = 0.00452  # Single wheel inertia (0.00072) + GIM4305 reflected armature (0.0038) (kg*m^2)
    gravity: float = 9.81           # Gravitational acceleration (m/s^2)
    max_wheel_torque: float = 3.47  # GIM4305-10 peak stall torque per wheel (N*m)
    max_joint_vel: float = 38.33    # GIM4305-10 max speed after 10:1 reduction (366 RPM = 38.33 rad/s)
    wheel_viscous: float = 0.05     # Wheel joint viscous damping (N*m*s/rad), matches `gim4305_wheel` damping
    wheel_coulomb: float = 0.015    # Wheel joint Coulomb friction (N*m), matches `gim4305_wheel` frictionloss


# LQR state weights: [x_pos (m), x_vel (m/s), pitch (rad), pitch_rate (rad/s)]
# Stiffened from (5, 22, 280, 32): those soft position/velocity weights let a 30 N shove carry the robot 1.5-2.5 m
# and could not hold position against a +/-8 mm CoM offset on randomized robots (see test_scenarios.py --realism).
LQR_Q = (15.0, 60.0, 280.0, 32.0)


@dataclass(frozen=True)
class RobotProfile:
    """Everything that changes with the leg motors: model file, LQR body model, balance-lean fit, jump tuning.

    Keeping these together means a robot's tuning can't be half-applied (the trim, posture fit and jump timing
    were all measured on a specific model and are wrong for the other one).
    """
    name: str
    xml: str                        # MJCF file next to this script
    params: ScaledWheeledDuckParams  # body (excluding wheels) mass / CoM height above axle / pitch inertia about CoM
    # Balance lean (rad) = initial_pitch_trim + a*fold + b*fold^2, fitted against the *measured* leg fold
    # (hip_pitch + knee, minus nominal) from 80 s holds at crouch commands -0.24..+0.24 (fit error < 0.02 deg).
    # Re-fit whenever trunk mass moves (adding the AI HAT+ / Feather shifted it ~0.13 deg = ~3 cm of drift).
    # Fitting against the commanded crouch was wrong: legs sag under load, so a 0.24 command measures 0.27-0.29.
    initial_pitch_trim: float
    posture_coeffs: Tuple[float, float]
    jump_thrust_time: float         # s
    thrust_wheel_mode: str          # wheel control during jump thrust: "lqr", "attitude" or "zero"
    # Leg (hip-pitch + knee) motor torque-speed curve: torque <= min(stall, (no_load_rpm - rpm) / rpm_per_nm).
    # None = not modelled (MuJoCo's forcerange is otherwise speed-independent).
    leg_torque_curve: Tuple[float, float, float] = None   # (stall N*m, no-load rpm, rpm drop per N*m)
    curve_actuators: Tuple[int, ...] = ()                 # actuator indices the curve applies to


PROFILES = {
    # Build configuration (default): SteadyWin GIM6010-8 on hip-roll, hip-pitch and knees (GIM4305-10 on hip-yaw and
    # wheels), 6S2P 21700 battery. Specs: SteadyWin GIM6010-8 manual rev1.2. Body params: tools/derive_lqr_params.py.
    "gim6010": RobotProfile(
        name="gim6010",
        xml="open_duck_wheeled_v2_gim6010.xml",
        params=ScaledWheeledDuckParams(body_mass=4.733, wheel_track=0.224, com_height=0.2467, body_pitch_inertia=0.0789),
        initial_pitch_trim=-0.0631,
        posture_coeffs=(-0.1305, -0.0263),
        # CAD-ready zero-scrub geometry: 4x GIM6010-8 on hip-pitch + knee, 6x GIM4305-10 on hip-yaw + hip-roll + wheels
        # (saves 476 g; wheel aligned 4 mm from hip-roll axis -> 0.10 N*m standing roll torque), 6S2P battery shifted
        # to x = -26 mm. Trim/posture re-fit with tools/fit_balance_trim.py (standing lean -3.7 deg).
        jump_thrust_time=0.05,
        thrust_wheel_mode="zero",
        leg_torque_curve=(11.0, 285.0, 17.0),   # manual sec 1.2 characteristic curve (24 V)
        curve_actuators=(1, 2, 4, 5),           # hip pitch, knee (both legs)
    ),
    # Original design: SteadyWin GIM4305-10 on every joint (open_duck_wheeled_v2.xml as first downloaded).
    "gim4305": RobotProfile(
        name="gim4305",
        xml="open_duck_wheeled_v2.xml",
        # Original LQR body numbers. com_height/inertia here are the whole-robot values *including* the wheels
        # (body-only: 0.2555 m / 0.0559 kg*m^2), but this robot's trim/posture fit and tuning were validated with
        # them, and the body-only values scored slightly worse on it (randomized crouch 7/12 vs 9/12).
        params=ScaledWheeledDuckParams(body_mass=3.361, com_height=0.222, body_pitch_inertia=0.0310),
        initial_pitch_trim=-0.0730,
        posture_coeffs=(-0.1013, -0.0324),
        # On these weaker legs the LQR's wheel kick during thrust is (accidentally) part of the jump: with the
        # wheels limp the jump shrinks to ~7 mm, so this profile keeps "lqr".
        jump_thrust_time=0.12,
        thrust_wheel_mode="lqr",
        leg_torque_curve=None,   # GIM4305-10 curve not found; its jump (16-17 mm) is likely optimistic
    ),
}
DEFAULT_ROBOT = "gim6010"


def compute_wip_lqr_gains(params: ScaledWheeledDuckParams) -> np.ndarray:
    """
    Derives the linearized 4-state Wheeled Inverted Pendulum (WIP) model around upright equilibrium
    and solves the Continuous Algebraic Riccati Equation (CARE) for optimal LQR state-feedback gains.

    State vector: x_state = [x_pos, x_vel, pitch_angle, pitch_rate]^T
    Input: u = total_wheel_torque = tau_left + tau_right (N*m)
    """
    M = params.body_mass
    m_w = params.wheel_mass
    R = params.wheel_radius
    L = params.com_height
    I_b = params.body_pitch_inertia
    I_w = params.wheel_inertia
    g = params.gravity

    M_eff = M + 2.0 * m_w + 2.0 * I_w / (R ** 2)
    I_eff = I_b + M * (L ** 2)
    coupling = M * L
    denom = M_eff * I_eff - (coupling ** 2)

    A = np.zeros((4, 4), dtype=float)
    A[0, 1] = 1.0
    A[1, 2] = -(coupling ** 2) * g / denom
    A[2, 3] = 1.0
    A[3, 2] = M_eff * M * g * L / denom

    B = np.zeros((4, 1), dtype=float)
    B[1, 0] = (I_eff / R + coupling) / denom
    B[3, 0] = -(M_eff + coupling / R) / denom

    Q = np.diag(LQR_Q)
    R_cost = np.array([[1.0]])

    P = solve_continuous_are(A, B, Q, R_cost)
    K = np.linalg.inv(R_cost) @ (B.T @ P)
    return K.flatten()


def quat_to_euler_zyx(quat_wxyz: np.ndarray) -> Tuple[float, float, float]:
    """Converts MuJoCo quaternion [w, x, y, z] into (roll, pitch, yaw) in radians."""
    w, x, y, z = quat_wxyz
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ScaledWheeledDuckController:
    """Full-body balance, 4-DOF leg posture, stereo-camera horizon, and live keyboard driving controller."""

    # Deepest crouch delta: knee = 1.00 + 2*delta must stay clear of the knee's +1.5708 rad (90 deg) joint limit,
    # *including* servo sag under load (~0.01-0.02 rad). On the stop, the hip keeps folding, the knee = -2*hip
    # geometry breaks, and the balance lean jumps (at 0.28 the knee rested on the stop and the robot rolled
    # ~18 cm). 0.24 leaves ~0.09 rad of margin and matches the range `posture_trim` was measured over.
    MAX_CROUCH = 0.24
    RAMP_JERK = 8.0        # m/s^3, how quickly the speed ramp's acceleration may fade out near the target speed
    REANCHOR_DIST = 0.30   # m; pushed further than this from the hold point -> hold wherever it comes to rest

    def __init__(self, profile: RobotProfile = None, params: ScaledWheeledDuckParams = None):
        self.profile = profile or PROFILES[DEFAULT_ROBOT]
        self.params = params or self.profile.params
        self.k_lqr = compute_wip_lqr_gains(self.params)
        self.target_x = 0.0
        self.target_vel = 0.0
        self.cmd_vel = 0.0
        self.target_yaw = 0.0
        self.crouch_delta = 0.0       # Adjusts crouch depth while keeping wheels under hips
        self.head_yaw_offset = 0.0    # Manual head pan offset (rad)
        self.antenna_cmd = 0.0        # Antenna servo target (rad, + = tips swung forward/upright), both sides
        self.projector_on = False
        self.nominal_hip_pitch = -0.50
        self.nominal_knee_pitch = 1.00
        self.commanded_leg_fold = self.nominal_hip_pitch + self.nominal_knee_pitch
        # Static CoM forward offset compensation (rad). The measured CoM sits ~14 mm ahead of the axle,
        # so the trunk must lean back ~4.5 deg to balance. This is the initial guess; it is refined online
        # by a slow integrator (see `compute_control`) so crouch changes / payloads are absorbed automatically.
        # Starting value from the profile's fit; each 1 deg of trim error parks the robot ~25 cm off its hold point,
        # so it matters (on hardware, persist the learned trim between runs).
        self.pitch_trim = self.profile.initial_pitch_trim
        self.trim_adapt_gain = 0.01   # (rad/s) per (m of position-equivalent error)
        self.reset_requested = False

        # Jump state machine: "idle" -> "crouch" -> "thrust" -> "flight" -> "land" -> "idle"
        self.jump_phase = "idle"
        self.jump_timer = 0.0
        self.jump_requested = False
        self.jump_takeoff_vel = 0.0
        self.braking = False
        self.settling = False

    def reset_state(self) -> None:
        """Clears all driving/posture/jump references (used on 'R' reset)."""
        self.target_x = 0.0
        self.target_vel = 0.0
        self.cmd_vel = 0.0
        self.target_yaw = 0.0
        self.crouch_delta = 0.0
        self.head_yaw_offset = 0.0
        self.antenna_cmd = 0.0
        self.pitch_trim = self.profile.initial_pitch_trim
        self.jump_phase = "idle"
        self.jump_timer = 0.0
        self.jump_requested = False
        self.jump_takeoff_vel = 0.0
        self.braking = False
        self.settling = False
        self.reset_requested = False

    def handle_keypress(self, keycode: int) -> None:
        """Processes live keyboard commands from the MuJoCo viewer window."""
        if keycode in (KEY_UP, ord("I"), ord("i"), ord("W"), ord("w")):
            self.cmd_vel = float(np.clip(self.cmd_vel + 0.15, -1.50, 1.50))
            print(f"[Drive] Target Speed: {self.cmd_vel:+.2f} m/s")
        elif keycode in (KEY_DOWN, ord("K"), ord("k"), ord("S"), ord("s")):
            self.cmd_vel = float(np.clip(self.cmd_vel - 0.15, -1.50, 1.50))
            print(f"[Drive] Target Speed: {self.cmd_vel:+.2f} m/s")
        elif keycode in (KEY_LEFT, ord("J"), ord("j"), ord("A"), ord("a")):
            self.target_yaw += math.radians(15.0)
            print(f"[Steer] Target Heading: {math.degrees(self.target_yaw):+.1f} deg")
        elif keycode in (KEY_RIGHT, ord("L"), ord("l"), ord("D"), ord("d")):
            self.target_yaw -= math.radians(15.0)
            print(f"[Steer] Target Heading: {math.degrees(self.target_yaw):+.1f} deg")
        elif keycode == KEY_SPACE:
            # Ramp down at a higher decel (see `compute_control`) rather than stepping target_vel to 0,
            # which used to leave the robot rolling back ~0.25 m to the point where Space was pressed.
            self.cmd_vel = 0.0
            self.braking = True
            print("[Brake] Full Stop (Holding Position)")
        elif keycode in (ord("Z"), ord("z")):
            self.crouch_delta = float(np.clip(self.crouch_delta + 0.08, -0.25, self.MAX_CROUCH))
            print(f"[Posture] Crouch Deeper (delta = {self.crouch_delta:+.2f} rad)")
        elif keycode in (ord("X"), ord("x")):
            self.crouch_delta = float(np.clip(self.crouch_delta - 0.08, -0.25, self.MAX_CROUCH))
            print(f"[Posture] Stand Taller (delta = {self.crouch_delta:+.2f} rad)")
        elif keycode in (ord("U"), ord("u")):
            self.head_yaw_offset = float(np.clip(self.head_yaw_offset + 0.20, -1.20, 1.20))
            print(f"[Head] Look Left ({math.degrees(self.head_yaw_offset):+.1f} deg)")
        elif keycode in (ord("O"), ord("o")):
            self.head_yaw_offset = float(np.clip(self.head_yaw_offset - 0.20, -1.20, 1.20))
            print(f"[Head] Look Right ({math.degrees(self.head_yaw_offset):+.1f} deg)")
        elif keycode in (ord("C"), ord("c")):
            if self.jump_phase == "idle":
                self.jump_requested = True
                print("[Jump] Crouching for jump...")
        elif keycode in (ord("Q"), ord("q")):
            self.antenna_cmd = float(np.clip(self.antenna_cmd + 0.20, -0.78, 0.78))
            print(f"[Antennas] Raise ({math.degrees(self.antenna_cmd):+.0f} deg)")
        elif keycode in (ord("E"), ord("e")):
            self.antenna_cmd = float(np.clip(self.antenna_cmd - 0.20, -0.78, 0.78))
            print(f"[Antennas] Lower ({math.degrees(self.antenna_cmd):+.0f} deg)")
        elif keycode in (ord("P"), ord("p")):
            self.projector_on = not self.projector_on
            print(f"[Projector] {'On' if self.projector_on else 'Off'}")
        elif keycode in (ord("R"), ord("r")):
            self.reset_requested = True
            print("[Reset] Resetting robot to home balanced pose...")

    def compute_control(
        self,
        quat_wxyz: np.ndarray,
        gyro_xyz: np.ndarray,
        left_wheel_pos: float,
        right_wheel_pos: float,
        left_wheel_vel: float,
        right_wheel_vel: float,
        leg_fold: float = None,
        leg_fold_rate: float = 0.0,
        dt: float = 0.002,
    ) -> Tuple[np.ndarray, dict]:
        """
        `leg_fold` / `leg_fold_rate` are the measured hip_pitch + knee angle (and its rate), averaged over both legs,
        from the GIM4305 encoders. If omitted, the commanded value is used (no servo lag compensation).

        Computes the 14-element MuJoCo `data.ctrl` vector (no hip yaw: the hips are fixed mounts, steering is by
        wheel torque difference):
          [0:3]   left leg targets (left_hip_roll, left_hip_pitch, left_knee)
          [3:6]   right leg targets (right_hip_roll, right_hip_pitch, right_knee)
          [6:10]  neck & head STS3215 targets (neck_pitch, head_pitch, head_yaw, head_roll)
          [10:12] wheel GIM4305-10 torques (left_wheel_motor, right_wheel_motor)
          [12:14] antenna 9 g micro servo targets (left_antenna, right_antenna)
        """
        roll, pitch, yaw = quat_to_euler_zyx(quat_wxyz)
        roll_rate, pitch_rate, yaw_rate = gyro_xyz

        # Smoothly ramp target velocity toward keyboard-commanded speed (limits pitch jerk). The accel limit also
        # tapers as sqrt(2*J*|dv|) near the target (an "S-curve" arrival): the robot must lean to accelerate, so
        # an abrupt end to the ramp leaves it leaning, and un-leaning makes it roll back (~10 cm after a brake).
        max_accel = 2.50 if self.braking else 1.20  # m/s^2
        vel_diff = self.cmd_vel - self.target_vel
        max_accel = min(max_accel, math.sqrt(2.0 * self.RAMP_JERK * abs(vel_diff)))
        self.target_vel += float(np.clip(vel_diff, -max_accel * dt, max_accel * dt))
        if self.braking and abs(self.target_vel) < 1e-6:
            self.braking = False
            self.settling = True

        R = self.params.wheel_radius
        # Wheel odometry. The wheel encoder measures rotation relative to the *foot*, and the foot's pitch is
        # trunk pitch + hip_pitch + knee. Ignoring the leg term made every crouch/jump look like wheel rolling
        # (the 'Z' crouch walked the robot ~0.5 m, and the jump thrust faked a -1.3 m/s velocity spike).
        if leg_fold is None:
            leg_fold, leg_fold_rate = self.commanded_leg_fold, 0.0
        fold = leg_fold - (self.nominal_hip_pitch + self.nominal_knee_pitch)
        mean_wheel_angle = 0.5 * (left_wheel_pos + right_wheel_pos) + pitch + fold
        mean_wheel_omega = 0.5 * (left_wheel_vel + right_wheel_vel) + pitch_rate + leg_fold_rate
        x_pos = R * mean_wheel_angle
        x_vel = R * mean_wheel_omega

        # 0. Jump State Machine ('C' key): crouch -> explosive thrust -> flight (wheels unloaded) -> soft landing
        leg_delta, airborne = self._update_jump(x_pos, x_vel, dt)

        # Advance target_x reference when driving; clamp windup so braking is immediate
        if abs(self.cmd_vel) > 1e-3 or abs(self.target_vel) > 1e-3:
            self.target_x = float(np.clip(self.target_x + self.target_vel * dt, x_pos - 0.25, x_pos + 0.25))
            self.settling = False
        elif (abs(x_pos - self.target_x) > self.REANCHOR_DIST and abs(x_vel) < 0.05
              and self.jump_phase == "idle"):
            # Shoved well away from the hold point (a big push) and its run-out has just stopped: hold here rather
            # than crawl all the way back at the clamped position-error speed. Waiting for the stop (instead of
            # re-anchoring as soon as it is displaced) keeps the position term helping to brake the run-out.
            self.target_x = x_pos
        if self.settling and abs(self.cmd_vel) <= 1e-3 and abs(self.target_vel) <= 1e-3:
            # After a brake, keep the hold point under the robot until it has physically stopped, then hold there
            self.target_x = x_pos
            if abs(x_vel) < 0.03:
                self.settling = False

        # 1. Longitudinal LQR Wheel Balancing Torque
        # Balance lean = learned trim (feedback, absorbs payload / build error) + posture feedforward: crouching
        # swings the trunk mass forward, so the balance lean changes by ~-0.5 deg per 0.1 rad of crouch. Leaving
        # that to the slow trim learner made each 'Z'/'X' press roll the robot ~20 cm while it re-learned.
        balance_pitch = self.pitch_trim + self.posture_trim(fold)
        pitch_err = pitch - balance_pitch
        pos_err = float(np.clip(x_pos - self.target_x, -0.30, 0.30))
        # When actively cruising, soften position-hold weight so velocity tracking dominates cleanly
        pos_weight = 0.25 if abs(self.cmd_vel) > 1e-3 else 1.0
        state_err = np.array([
            pos_weight * pos_err,
            x_vel - self.target_vel,
            pitch_err,
            pitch_rate,
        ])
        total_balance_torque = float(np.dot(-self.k_lqr, state_err))

        # Online CoM-offset (pitch trim) estimator: a persistent position error while parked means the trim is
        # wrong (the LQR is fighting gravity). Slowly integrate it away. Only runs when parked (while driving the
        # robot *must* lean to accelerate, which would corrupt the estimate) and is frozen during jumps.
        # Also frozen while settling after a brake/push: the robot is still rolling to a stop then, and learning
        # from that motion skewed the trim by ~0.5 deg, which shows up as a ~12 cm roll-back once it stops
        # (at rest K_pitch * trim_error = K_x * pos_error, i.e. ~25 cm of offset per degree of trim error).
        # Do not also gate on |x_vel|: a badly wrong trim makes the robot creep at a steady speed, and a speed gate
        # then freezes the very estimator that would fix it (seen as a 0.18 m/s endless creep on a heavier robot).
        parked = abs(self.cmd_vel) < 1e-3 and abs(self.target_vel) < 0.05 and not self.settling
        if self.jump_phase == "idle" and parked:
            trim_err = pos_err + 0.5 * (x_vel - self.target_vel)
            self.pitch_trim = float(np.clip(self.pitch_trim - self.trim_adapt_gain * trim_err * dt, -0.20, 0.10))

        if airborne:
            # Wheels are unloaded, so LQR odometry is meaningless. Use the two hub motors as a reaction wheel
            # to hold trunk attitude (reaction torque on the body is -tau), so we touch down level.
            total_balance_torque = 6.0 * pitch_err + 0.6 * pitch_rate
        elif self.jump_phase == "thrust" and self.profile.thrust_wheel_mode != "lqr":
            # During the leg thrust the wheels unload and the odometry is garbage (est. speed swings +/-1-2 m/s),
            # so the LQR slams the wheels to +/-3.47 N*m and the reaction launches the body spinning nose-down.
            total_balance_torque = (6.0 * pitch_err + 0.6 * pitch_rate) if self.profile.thrust_wheel_mode == "attitude" else 0.0

        # 2. Differential Yaw Steering Torque
        yaw_err = math.atan2(math.sin(yaw - self.target_yaw), math.cos(yaw - self.target_yaw))
        yaw_torque = 0.0 if airborne else (-1.45 * yaw_err - 0.30 * yaw_rate)

        # Per-wheel friction feedforward (viscous + Coulomb) at the reference speed. Without it the LQR needs a
        # persistent velocity error to supply the drag torque, so cruise speed overshot the command by ~17%.
        wheel_ff = 0.0
        if not airborne and abs(self.target_vel) > 0.01:
            wheel_ff = (self.params.wheel_viscous * self.target_vel / R
                        + self.params.wheel_coulomb * math.copysign(1.0, self.target_vel))

        tau_left = np.clip(
            0.5 * total_balance_torque - yaw_torque + wheel_ff,
            -self.params.max_wheel_torque,
            self.params.max_wheel_torque,
        )
        tau_right = np.clip(
            0.5 * total_balance_torque + yaw_torque + wheel_ff,
            -self.params.max_wheel_torque,
            self.params.max_wheel_torque,
        )

        # 3. 4-DOF GIM4305-10 Upper Leg Coordination (Roll Leveling + Turn Banking + Crouch Height)
        turn_bank = np.clip(-0.12 * x_vel * yaw_err, -0.12, 0.12)
        roll_comp = np.clip(-0.55 * (roll - turn_bank) - 0.10 * roll_rate, -0.30, 0.30)
        # NOTE: the previous hip-pitch balance assist (-0.18*pitch_err - 0.04*pitch_rate) was removed: with the
        # compliant kp=32 GIM4305 position loops it coupled into the LQR and produced a saturated ~2.5 Hz
        # bang-bang limit cycle (wheel torque pinned at +/-3.47 N*m and the robot drifting away at 1.6 m/s).
        # Keep knee = -2 * hip_pitch so the wheel axle stays vertically aligned under the hip at any crouch height
        hip_pitch_cmd = self.nominal_hip_pitch - leg_delta
        knee_cmd = self.nominal_knee_pitch + 2.0 * leg_delta
        self.commanded_leg_fold = hip_pitch_cmd + knee_cmd

        # 4. Horizon-Stabilized Stereo Camera Gimbal (Looks into turns + manual pan)
        # TODO: the head hits the trunk at same-direction pitch extremes (cad/freecad/neck_envelope.md). Clip to the
        # coupled CAD envelope, which the MJCF joint ranges can't express:
        #   neck_pitch + head_pitch >= -30 deg,  head_pitch <= +30 deg,  neck_pitch + head_pitch <= +60 deg
        # and enable head/trunk contact in the sim so violations show up.
        neck_pitch_cmd = np.clip(0.15 - 0.5 * pitch, -0.30, 0.90)
        head_pitch_cmd = np.clip(-0.15 - 0.5 * pitch, -0.70, 0.70)
        head_yaw_cmd = np.clip(-0.6 * yaw_err + self.head_yaw_offset, -1.5, 1.5)
        head_roll_cmd = np.clip(-roll, -0.45, 0.45)

        ctrl = np.array([
            # Left leg (hip roll, hip pitch, knee)
            roll_comp,
            hip_pitch_cmd,
            knee_cmd,
            # Right leg
            roll_comp,
            hip_pitch_cmd,
            knee_cmd,
            # Neck & Head (4 STS3215 gimbal joints)
            neck_pitch_cmd,
            head_pitch_cmd,
            head_yaw_cmd,
            head_roll_cmd,
            # Driven Wheels (2 GIM4305-10 hub motors)
            tau_left,
            tau_right,
            # Antennas (2 micro servos)
            self.antenna_cmd,
            self.antenna_cmd,
        ], dtype=float)

        telemetry = {
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
            "x_pos": x_pos,
            "x_vel": x_vel,
            "cmd_vel": self.cmd_vel,
            "target_yaw_deg": math.degrees(self.target_yaw),
            "tau_left": tau_left,
            "tau_right": tau_right,
            "pitch_trim_deg": math.degrees(balance_pitch),
            "jump_phase": self.jump_phase,
        }
        return ctrl, telemetry

    def posture_trim(self, fold: float) -> float:
        """Change in balance lean (rad) vs. zero leg fold, from the profile's fit (see `RobotProfile`).

        Crouching swings the trunk mass forward, so the balance lean changes (~-0.5 deg per 0.1 rad of fold on the
        stock robot). A static CoM-over-axle calculation gives the same shape but ~25% less slope, because it
        ignores the legs sagging under load on their kp=32 servos.
        """
        a, b = self.profile.posture_coeffs
        return a * fold + b * fold * fold

    # Leg servo damping (GIM4305 MIT-mode kd, N*m*s/rad). High damping stops the compliant kp=32 legs from
    # pogoing under hard wheel torque (which bounced the tires off the floor during push recovery), but it also
    # resists the fast leg extension of a jump, so it is dropped for the jump wind-up and thrust.
    LEG_KD_NOMINAL = 4.0
    LEG_KD_JUMP = 1.4

    def leg_servo_kd(self) -> float:
        """Damping gain the leg servos should run with right now (applied by the sim loop each step)."""
        return self.LEG_KD_JUMP if self.jump_phase in ("crouch", "thrust") else self.LEG_KD_NOMINAL

    # Jump trajectory tuning (leg_delta is the symmetric crouch offset: hip = -0.50 - d, knee = 1.00 + 2d)
    JUMP_CROUCH_DELTA = MAX_CROUCH  # Deep wind-up crouch (hip -0.74, knee 1.48)
    JUMP_EXTEND_DELTA = -0.46   # Near-straight legs at take-off (hip -0.04, knee 0.08)
    JUMP_LAND_DELTA = -0.15     # Slightly bent, compliant legs for touchdown
    JUMP_CROUCH_TIME = 0.40     # s
    JUMP_FLIGHT_TIME = 0.20     # s (wheels unloaded; tucks legs toward landing pose)
    JUMP_LAND_TIME = 0.50       # s (blend back to the driver's crouch setting)

    def _update_jump(self, x_pos: float, x_vel: float, dt: float) -> Tuple[float, bool]:
        """Advances the timed jump state machine. Returns (leg_delta, airborne)."""
        if self.jump_requested and self.jump_phase == "idle":
            self.jump_phase, self.jump_timer, self.jump_requested = "crouch", 0.0, False
        if self.jump_phase == "idle":
            return self.crouch_delta, False

        self.jump_timer += dt
        t = self.jump_timer
        if self.jump_phase == "crouch":
            # Smooth cosine wind-up from the current crouch into the deep crouch
            a = 0.5 - 0.5 * math.cos(math.pi * min(t / (0.75 * self.JUMP_CROUCH_TIME), 1.0))
            leg_delta = self.crouch_delta + a * (self.JUMP_CROUCH_DELTA - self.crouch_delta)
            if t >= self.JUMP_CROUCH_TIME:
                self.jump_phase, self.jump_timer = "thrust", 0.0
                print("[Jump] Thrust!")
            return leg_delta, False
        if self.jump_phase == "thrust":
            if t >= self.profile.jump_thrust_time:
                self.jump_phase, self.jump_timer = "flight", 0.0
                self.jump_takeoff_vel = x_vel
            return self.JUMP_EXTEND_DELTA, False
        if self.jump_phase == "flight":
            if t >= self.JUMP_FLIGHT_TIME:
                self.jump_phase, self.jump_timer = "land", 0.0
                # Wheel odometry drifted while unloaded: re-anchor the position hold where we touched down
                self.target_x = x_pos
            return self.JUMP_LAND_DELTA, True
        # "land": blend from the landing crouch back to the driver's crouch setting
        a = min(t / self.JUMP_LAND_TIME, 1.0)
        if t >= self.JUMP_LAND_TIME:
            self.jump_phase, self.jump_timer = "idle", 0.0
            print("[Jump] Landed.")
        return self.JUMP_LAND_DELTA + a * (self.crouch_delta - self.JUMP_LAND_DELTA), False


LEG_ACTUATORS = slice(0, 6)  # ctrl[0:6] are the six leg position servos (hip roll, hip pitch, knee x2)
WHEEL_ACTUATORS = slice(10, 12)
TIRE_GEOMS = ("floor", "left_foot_bottom_tpu", "right_foot_bottom_tpu")


def apply_leg_servo_kd(model, kd: float) -> None:
    """Sets the leg position servos' damping (MuJoCo position actuator: force = kp*(ctrl - q) - kv*qdot)."""
    model.actuator_biasprm[LEG_ACTUATORS, 2] = -kd


@dataclass
class RealismConfig:
    """Sim-to-real gap: sensor imperfections, control-computer timing, and per-robot variation (domain randomization).

    Each range is sampled once per run (a "different robot on a different floor"); noise terms are sampled every step.

    Control architectures (`arch`):
      "mcu":     balance loop on the PJRC Teensy 4.0 (600 MHz Cortex-M7) with Dual 1 Mbps CAN buses (`CAN1` left leg +
                 wheel, `CAN2` right leg + wheel; `can_buses = 2`) and an ICM-42688-P SPI IMU (~0.25-0.55 ms latency).
      "feather": balance loop on the single-bus Adafruit Feather M4 CAN Express (120 MHz Cortex-M4F, `can_buses = 1`,
                 ~0.3-0.8 ms latency).
      "pi":      balance loop in Linux user space on the Raspberry Pi 5: 2-4 ms latency plus occasional scheduling stalls
                 (missed ticks, the drivers keep applying the last command).
    The GIM drivers speak classic CAN (11-bit ID, 8-byte frames, 1 Mbps max; GIM6010-8 manual sec 4.1). On the Teensy
    4.0 (`CAN1` + `CAN2`), the 8 GIM motors split 4-per-bus; on the Feather M4 CAN (`can_buses = 1`), all 8 share one
    bus (`can_bus_load` gives the numbers).
    """
    arch: str = "mcu"
    imu: str = "fast"                           # "fast" (default): raw 1 kHz-class IMU (ICM-42688-P) on the MCU, filtered every
                                                #   tick; "bno055": fused orientation AND gyro held at 100 Hz (fusion rate)
    gyro_noise_dps: float = 0.3                 # gyro white noise, RMS per sample (deg/s)
    gyro_bias_dps: float = 0.5                  # constant per-axis gyro bias, drawn in +/- this range (deg/s)
    orient_noise_deg: float = 0.2               # orientation estimate jitter, RMS per axis (deg)
    bno055_rate_hz: float = 100.0               # BNO055 fusion-mode output rate (orientation and gyro)
    encoder_counts: int = 16384                 # 14-bit encoders (counts per revolution)
    wheel_vel_noise: float = 0.05               # wheel/joint velocity estimate noise, RMS (rad/s)
    mcu_latency_ms: Tuple[float, float] = (0.25, 0.55)  # CAN feedback frame + compute + command frame on the Teensy 4.0
    feather_latency_ms: Tuple[float, float] = (0.3, 0.8)  # ...the same on the Feather M4 CAN
    pi_latency_ms: Tuple[float, float] = (2.0, 4.0)     # Linux user-space loop + SPI CAN adapter
    pi_stall_prob: float = 0.005                # per-tick chance (pi) of a scheduling stall...
    pi_stall_ms: Tuple[float, float] = (4.0, 12.0)      # ...lasting this long (ticks missed, last command held)
    leg_rate_hz: float = 100.0                  # leg setpoint + feedback rate (GIM default encoder feedback: 10 ms)
    can_bitrate: float = 1e6                    # classic CAN, GIM driver maximum
    can_buses: int = 2                          # 2 on Teensy 4.0 (CAN1 left, CAN2 right); 1 on Feather M4 CAN
    can_frame_bits: int = 135                   # 11-bit ID, 8 data bytes, worst-case bit stuffing
    friction: Tuple[float, float] = (0.8, 1.25)          # tire/floor friction (TPU on tile/wood ~0.8, on rubber mat ~1.25)
    trunk_mass_scale: Tuple[float, float] = (0.9, 1.1)   # battery/payload variation
    trunk_com_shift_m: Tuple[float, float] = (-0.008, 0.008)  # fore/aft payload placement
    motor_strength: Tuple[float, float] = (0.9, 1.05)    # GIM4305 unit-to-unit torque constant spread


def can_bus_load(cfg: RealismConfig, wheel_rate_hz: float) -> float:
    """Per-channel fraction of the 1 Mbps CAN bus used: one command + one feedback frame per motor update, plus 10 Hz
    heartbeats. Split across `cfg.can_buses` (2 channels on Teensy 4.0 = 1 wheel + 3 leg motors per bus; 1 on Feather).
    Keep under ~0.7."""
    buses = cfg.can_buses if cfg.arch == "mcu" else 1
    frames_per_s = (2 * 2 * wheel_rate_hz + 6 * 2 * cfg.leg_rate_hz + 8 * 10) / buses
    return frames_per_s * cfg.can_frame_bits / cfg.can_bitrate


class SimToReal:
    """Wraps a MuJoCo model with sensor noise, control timing and randomized physics for one simulated robot."""

    def __init__(self, model, cfg: RealismConfig = RealismConfig(), seed: int = 0):
        import mujoco
        assert cfg.arch in ("mcu", "feather", "pi") and cfg.imu in ("bno055", "fast"), (cfg.arch, cfg.imu)
        self.cfg = cfg
        self.rng = np.random.default_rng(seed)
        u = lambda r: float(self.rng.uniform(*r))
        dt = float(model.opt.timestep)
        self.dt = dt

        # --- Per-robot physical randomization (applied to the model once) ---
        self.friction = u(cfg.friction)
        for name in TIRE_GEOMS:  # MuJoCo uses the max of the two geoms' friction, so set floor and tires together
            model.geom_friction[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name), 0] = self.friction
        trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_assembly")
        self.trunk_mass_scale = u(cfg.trunk_mass_scale)
        model.body_mass[trunk] *= self.trunk_mass_scale
        model.body_inertia[trunk] *= self.trunk_mass_scale
        self.trunk_com_shift = u(cfg.trunk_com_shift_m)
        model.body_ipos[trunk, 0] += self.trunk_com_shift
        self.motor_strength = u(cfg.motor_strength)
        model.actuator_gear[WHEEL_ACTUATORS, 0] *= self.motor_strength
        self.latency_ms = u({"mcu": cfg.mcu_latency_ms, "feather": cfg.feather_latency_ms, "pi": cfg.pi_latency_ms}[cfg.arch])

        # --- Sensor / actuation state ---
        self.gyro_bias = np.radians(self.rng.uniform(-cfg.gyro_bias_dps, cfg.gyro_bias_dps, 3))
        self.imu_period = 1 if cfg.imu == "fast" else max(1, round(1.0 / (cfg.bno055_rate_hz * dt)))
        self.leg_period = max(1, round(1.0 / (cfg.leg_rate_hz * dt)))
        self.held_imu = None
        self.held_legs = None
        self.step_count = 0
        # Delay = n whole ticks + a fraction f. The fraction is applied by blending the last two commands over the
        # step (the old command acts for f of the step, the new one for the rest) - exact for the torque motors.
        delay = self.latency_ms * 1e-3 / dt
        self.delay_whole, self.delay_frac = int(delay), delay - int(delay)
        self.history = []
        self.held_leg_cmd = None
        self.last_out = None
        self.stall_left = 0
        self.stalls = 0

    def describe(self) -> str:
        load = can_bus_load(self.cfg, 1.0 / self.dt)
        return (f"mu={self.friction:.2f} trunk_mass x{self.trunk_mass_scale:.2f} com{self.trunk_com_shift*1000:+.0f}mm "
                f"motor x{self.motor_strength:.2f} | {self.cfg.arch} imu={self.cfg.imu} latency={self.latency_ms:.1f}ms "
                f"legs@{1.0 / (self.leg_period * self.dt):.0f}Hz CAN {100 * load:.0f}%")

    def sense(self, quat, gyro, lw_pos, rw_pos, lw_vel, rw_vel, leg_fold, leg_fold_rate):
        """Corrupts perfect MuJoCo sensor readings the way the IMU and GIM encoders (over CAN) would."""
        import mujoco
        cfg, rng = self.cfg, self.rng
        if self.held_imu is None or self.step_count % self.imu_period == 0:
            # New IMU sample: rotate the true quaternion by a small random error; gyro gets bias + noise
            err = np.radians(rng.normal(0.0, cfg.orient_noise_deg, 3))
            dq = np.array([1.0, *(0.5 * err)])
            dq /= np.linalg.norm(dq)
            q = np.zeros(4)
            mujoco.mju_mulQuat(q, np.asarray(quat, dtype=float), dq)
            g = np.asarray(gyro, dtype=float) + self.gyro_bias + np.radians(rng.normal(0.0, cfg.gyro_noise_dps, 3))
            self.held_imu = (q, g)
        tick = 2.0 * math.pi / cfg.encoder_counts
        if self.held_legs is None or self.step_count % self.leg_period == 0:
            # Leg feedback frames arrive at the leg rate; between them the MCU only has the last values
            self.held_legs = (round(leg_fold / tick) * tick, leg_fold_rate + rng.normal(0.0, cfg.wheel_vel_noise))
        self.step_count += 1
        lw_pos, rw_pos = (round(p / tick) * tick for p in (lw_pos, rw_pos))
        lw_vel, rw_vel = (v + rng.normal(0.0, cfg.wheel_vel_noise) for v in (lw_vel, rw_vel))
        q, g = self.held_imu
        return q.copy(), g.copy(), lw_pos, rw_pos, lw_vel, rw_vel, *self.held_legs

    def actuate(self, ctrl: np.ndarray) -> np.ndarray:
        """Returns the command the motors apply this step: delayed, legs updated only at the leg rate, and (pi)
        frozen during scheduling stalls."""
        ctrl = np.array(ctrl, dtype=float)
        # Leg setpoints only go out at the leg rate (the drivers hold the last one)
        if self.held_leg_cmd is None or (self.step_count - 1) % self.leg_period == 0:
            self.held_leg_cmd = ctrl[LEG_ACTUATORS].copy()
        ctrl[LEG_ACTUATORS] = self.held_leg_cmd

        self.history.append(ctrl)
        del self.history[:-(self.delay_whole + 2)]
        newer = self.history[max(0, len(self.history) - 1 - self.delay_whole)]
        older = self.history[max(0, len(self.history) - 2 - self.delay_whole)]
        out = (1.0 - self.delay_frac) * newer + self.delay_frac * older

        if self.cfg.arch == "pi":
            if self.stall_left == 0 and self.rng.random() < self.cfg.pi_stall_prob:
                self.stall_left = max(1, round(float(self.rng.uniform(*self.cfg.pi_stall_ms)) * 1e-3 / self.dt))
                self.stalls += 1
            if self.stall_left > 0 and self.last_out is not None:
                self.stall_left -= 1
                return self.last_out
        self.last_out = out
        return out


def read_sensors(model, data) -> tuple:
    """Perfect (simulator ground-truth) sensor readings in `compute_control` argument order (incl. leg encoders)."""
    import mujoco

    def s(name: str, dim: int = 1):
        adr = model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)]
        return data.sensordata[adr:adr + dim].copy()

    def joint(name: str):
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        return data.qpos[model.jnt_qposadr[j]], data.qvel[model.jnt_dofadr[j]]

    legs = [joint(f"{side}_{j}") for side in ("left", "right") for j in ("hip_pitch", "knee")]
    leg_fold = 0.5 * sum(q for q, _ in legs)        # mean over the two legs of (hip_pitch + knee)
    leg_fold_rate = 0.5 * sum(v for _, v in legs)

    return (s("orientation", 4), s("gyro", 3), float(s("left_wheel_pos")[0]), float(s("right_wheel_pos")[0]),
            float(s("left_wheel_vel")[0]), float(s("right_wheel_vel")[0]), float(leg_fold), float(leg_fold_rate))


def apply_leg_torque_curve(model, data, curve, actuators) -> None:
    """Caps the GIM6010-8 joints' torque at what the motor can deliver at its current speed.

    MuJoCo's forcerange is speed-independent, but a real motor runs out of voltage as it speeds up: the GIM6010-8
    gives 11 N*m at standstill but only ~6 N*m at the ~185 rpm the jump thrust reaches.
    """
    stall, no_load_rpm, rpm_per_nm = curve
    actuators = list(actuators)
    dof = model.jnt_dofadr[model.actuator_trnid[actuators, 0]]
    rpm = np.abs(data.qvel[dof]) * 60.0 / (2.0 * math.pi)
    limit = np.clip((no_load_rpm - rpm) / rpm_per_nm, 0.5, stall)
    model.actuator_forcerange[actuators, 0] = -limit
    model.actuator_forcerange[actuators, 1] = limit


def step_robot(model, data, controller: "ScaledWheeledDuckController", realism: "SimToReal" = None) -> dict:
    """One 500 Hz control tick: sense -> control -> (delayed) actuate -> physics step. Returns telemetry."""
    import mujoco
    sensors = read_sensors(model, data)
    if realism is not None:
        sensors = realism.sense(*sensors)
    ctrl, info = controller.compute_control(*sensors, dt=float(model.opt.timestep))
    data.ctrl[:] = realism.actuate(ctrl) if realism is not None else ctrl
    apply_leg_servo_kd(model, controller.leg_servo_kd())
    if controller.profile.leg_torque_curve is not None:
        apply_leg_torque_curve(model, data, controller.profile.leg_torque_curve, controller.profile.curve_actuators)
    mujoco.mj_step(model, data)
    return info


def profile_xml(profile: RobotProfile) -> str:
    """Absolute path of a profile's MJCF (it lives next to this script)."""
    return str(pathlib.Path(__file__).with_name(profile.xml))


def print_driving_instructions() -> None:
    """Prints keyboard driving controls to the terminal on startup."""
    print("=" * 68)
    print("  1.35x SCALED WHEELED OPEN DUCK MINI v2 — LIVE KEYBOARD CONTROLS")
    print("=" * 68)
    print("  [Up Arrow]    or [I]/[W] : Drive Forward  (+0.15 m/s step)")
    print("  [Down Arrow]  or [K]/[S] : Drive Backward (-0.15 m/s step)")
    print("  [Left Arrow]  or [J]/[A] : Steer Left     (+15 deg heading)")
    print("  [Right Arrow] or [L]/[D] : Steer Right    (-15 deg heading)")
    print("  [Spacebar]               : Brake / Hold Current Position")
    print("  [Z] / [X]                : Crouch Deeper [Z] / Stand Taller [X]")
    print("  [U] / [O]                : Pan BDX Head Cameras Left [U] / Right [O]")
    print("  [C]                      : Jump (crouch -> thrust -> flight -> land)")
    print("  [Q] / [E]                : Raise [Q] / Lower [E] Antennas")
    print("  [P]                      : Toggle Head Projector Light")
    print("  [R]                      : Reset Robot to Home Balanced Pose")
    print("=" * 68)


def run_mujoco_simulation(xml_path: str = None, duration: float = 120.0, headless: bool = False,
                          realism_seed: int = None, robot: str = DEFAULT_ROBOT,
                          realism_cfg: RealismConfig = RealismConfig()) -> None:
    """Runs the interactive or headless MuJoCo simulation with active LQR wheel balancing & keyboard driving.

    `robot` picks the `RobotProfile` (its MJCF is used unless `xml_path` overrides it). `realism_seed` (optional)
    enables `SimToReal`: noisy sensors, CAN latency and a randomized robot/floor.
    """
    import mujoco

    profile = PROFILES[robot]
    model = mujoco.MjModel.from_xml_path(xml_path or profile_xml(profile))
    data = mujoco.MjData(model)
    realism = SimToReal(model, realism_cfg, seed=realism_seed) if realism_seed is not None else None
    if realism is not None:
        print("[Sim-to-real]", realism.describe())

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home_balanced")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    controller = ScaledWheeledDuckController(profile)
    dt = float(model.opt.timestep)

    if headless:
        steps = int(duration / dt)
        for step in range(steps):
            info = step_robot(model, data, controller, realism)

            if step % 250 == 0:
                print(
                    f"[t={data.time:5.2f}s] pitch={info['pitch_deg']:+6.2f} deg | "
                    f"x={info['x_pos']:+5.3f} m | v={info['x_vel']:+5.3f} m/s (cmd={info['cmd_vel']:+.2f}) | "
                    f"wheel_tau=({info['tau_left']:+5.2f}, {info['tau_right']:+5.2f}) N*m"
                )
        return

    print_driving_instructions()
    import mujoco.viewer
    projector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_LIGHT, "projector")
    with mujoco.viewer.launch_passive(model, data, key_callback=controller.handle_keypress) as viewer:
        start_wall = time.time()
        while viewer.is_running() and (time.time() - start_wall) < duration:
            step_start = time.time()

            if controller.reset_requested:
                if key_id >= 0:
                    mujoco.mj_resetDataKeyframe(model, data, key_id)
                mujoco.mj_forward(model, data)
                controller.reset_state()
            if projector_id >= 0:
                model.light_active[projector_id] = controller.projector_on

            step_robot(model, data, controller, realism)
            viewer.sync()

            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulate & Drive the 1.35x Scaled Wheeled Open Duck Mini v2 in MuJoCo.")
    parser.add_argument("--robot", choices=sorted(PROFILES), default=DEFAULT_ROBOT,
                        help="Leg-motor configuration (default: %(default)s). Selects the model file and tuning.")
    parser.add_argument("--xml", type=str, default=None,
                        help="Override the MJCF file (default: the --robot profile's model).")
    parser.add_argument("--duration", type=float, default=120.0, help="Simulation duration in seconds.")
    parser.add_argument("--headless", action="store_true", help="Run without GUI viewer and print telemetry.")
    parser.add_argument("--realism", type=int, default=None, metavar="SEED",
                        help="Enable sim-to-real mode (sensor noise, CAN latency, randomized robot) with this seed.")
    parser.add_argument("--arch", choices=("mcu", "feather", "pi"), default="mcu",
                        help="With --realism: balance loop on the Teensy 4.0 dual-CAN (mcu), Feather M4 CAN (feather) or in Linux on the Pi (pi).")
    parser.add_argument("--imu", choices=("bno055", "fast"), default="fast",
                        help="With --realism: fast raw IMU filtered on the MCU (default) or BNO055 (100 Hz fused).")
    parser.add_argument("--leg-rate", type=float, default=100.0, help="With --realism: leg CAN update rate (Hz).")
    args = parser.parse_args()

    print(f"Robot '{args.robot}': WIP LQR Feedback Gains K =",
          np.round(compute_wip_lqr_gains(PROFILES[args.robot].params), 3))
    run_mujoco_simulation(args.xml, duration=args.duration, headless=args.headless, realism_seed=args.realism,
                          robot=args.robot,
                          realism_cfg=RealismConfig(arch=args.arch, imu=args.imu, leg_rate_hz=args.leg_rate))