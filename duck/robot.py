"""Robot description for the balance controller: physical parameters, per-robot profiles, and the
wheeled-inverted-pendulum LQR synthesis. No MuJoCo dependency."""

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from scipy.linalg import solve_continuous_are


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
    wheel_torque_curve: Tuple[float, float, float] = None # the same for the wheel motors (at 24 V, like the leg curve)


PROFILES = {
    # Build configuration (default): SteadyWin GIM6010-8 on hip-roll, hip-pitch and knees (GIM4305-10 on hip-yaw and
    # wheels), 6S2P 21700 battery. Specs: SteadyWin GIM6010-8 manual rev1.2. Body params: tools/derive_lqr_params.py.
    "gim6010": RobotProfile(
        name="gim6010",
        xml="open_duck_wheeled_v2_gim6010.xml",
        params=ScaledWheeledDuckParams(body_mass=6.529, wheel_mass=0.251, wheel_track=0.224, com_height=0.2211,
                                       body_pitch_inertia=0.1050, wheel_inertia=0.00477,    # sim = CAD (2026-10-31)
                                       wheel_viscous=0.005),   # = the MJCF's gim4305_wheel joint
        initial_pitch_trim=0.0758,
        posture_coeffs=(-0.1574, 0.0358),
        # CAD-ready zero-scrub geometry: 4x GIM6010-8 on hip-pitch + knee, 6x GIM4305-10 on hip-yaw + hip-roll + wheels
        # (saves 476 g; wheel aligned 4 mm from hip-roll axis -> 0.10 N*m standing roll torque), 6S2P battery shifted
        # to x = -26 mm. Trim/posture re-fit with tools/fit_balance_trim.py after the sim was exported from the CAD (standing lean +4.6 deg).
        jump_thrust_time=0.05,
        thrust_wheel_mode="zero",
        leg_torque_curve=(11.0, 285.0, 17.0),   # manual sec 1.2 characteristic curve (24 V)
        curve_actuators=(1, 2, 4, 5),           # hip pitch, knee (both legs)
        # GIM4305-10: 3.47 N*m stall, 366 rpm no load. Same curve shape as the GIM6010-8 (intercept 1.53 x stall).
        wheel_torque_curve=(3.47, 366.0, 366.0 / (1.53 * 3.47)),
    ),
    # Original design: SteadyWin GIM4305-10 on every joint (open_duck_wheeled_v2.xml as first downloaded).
    "gim4305": RobotProfile(
        name="gim4305",
        xml="open_duck_wheeled_v2.xml",
        # Original LQR body numbers. com_height/inertia here are the whole-robot values *including* the wheels
        # (body-only: 0.2555 m / 0.0559 kg*m^2), but this robot's trim/posture fit and tuning were validated with
        # them, and the body-only values scored slightly worse on it (randomized crouch 7/12 vs 9/12).
        params=ScaledWheeledDuckParams(body_mass=3.361, com_height=0.222, body_pitch_inertia=0.0310),
        initial_pitch_trim=0.0080,
        posture_coeffs=(-0.0714, -0.0064),
        # On these weaker legs the LQR's wheel kick during thrust is (accidentally) part of the jump: with the
        # wheels limp the jump shrinks to ~7 mm, so this profile keeps "lqr".
        jump_thrust_time=0.12,
        thrust_wheel_mode="lqr",
        leg_torque_curve=None,   # GIM4305-10 curve not found; its jump (16-17 mm) is likely optimistic
    ),
}
DEFAULT_ROBOT = "gim6010"


def wip_model(params: ScaledWheeledDuckParams) -> Tuple[np.ndarray, np.ndarray]:
    """
    Linearized 4-state Wheeled Inverted Pendulum (WIP) model around upright equilibrium: x_dot = A x + B u.

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
    return A, B


def compute_wip_lqr_gains(params: ScaledWheeledDuckParams) -> np.ndarray:
    """Solves the Continuous Algebraic Riccati Equation (CARE) on `wip_model` for optimal LQR state-feedback gains."""
    A, B = wip_model(params)
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
