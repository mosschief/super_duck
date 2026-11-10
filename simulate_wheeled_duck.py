#!/usr/bin/env python3
"""
Active Balancing, Posture & Live Keyboard Driving Controller for the 1.35x Scaled
Wheeled Open Duck Mini v2 (based on `apirrone/Open_Duck_Mini`).

Robot profiles (`--robot`): `gim6010` (default, the build) = SteadyWin GIM6010-8 on hip-pitch and knees,
6S2P battery (`open_duck_wheeled_v2_gim6010.xml`); `gim4305` = the original all-GIM4305-10 design with a 6S1P pack
(`open_duck_wheeled_v2.xml`).

Live Keyboard Driving Controls (inside the 3D MuJoCo Viewer Window):
  - Up Arrow    or 'I' / 'W' : Increase forward speed (+0.15 m/s per press, up to +2.50 m/s; less on a low pack)
  - Down Arrow  or 'K' / 'S' : Decrease speed / reverse (-0.15 m/s per press, down to -2.50 m/s)
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
import time

import numpy as np

# Re-exported so `import simulate_wheeled_duck as swd` keeps working in the tests and tools.
from duck.robot import (DEFAULT_ROBOT, LQR_Q, PROFILES, RobotProfile, ScaledWheeledDuckParams,  # noqa: F401
                        compute_wip_lqr_gains, quat_to_euler_zyx, wip_model)
from duck.controller import (KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_UP,  # noqa: F401
                             ScaledWheeledDuckController)
from duck.cpp_controller import CONTROLLERS, make_controller  # noqa: F401
from duck.plant import (LEG_ACTUATORS, TIRE_GEOMS, WHEEL_ACTUATORS, RealismConfig, SimToReal,  # noqa: F401
                        apply_leg_servo_kd, apply_torque_curve, can_bus_load, profile_xml, read_sensors, step_robot)


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
                          realism_cfg: RealismConfig = RealismConfig(), controller_kind: str = None) -> None:
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

    controller = make_controller(profile, controller_kind)
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
    parser.add_argument("--controller", choices=sorted(CONTROLLERS), default=None,
                        help="py: the Python controller (default, or $DUCK_CONTROLLER); cpp: the C++ firmware controller; "
                             "compare: both, checked against each other every tick.")
    args = parser.parse_args()

    print(f"Robot '{args.robot}': WIP LQR Feedback Gains K =",
          np.round(compute_wip_lqr_gains(PROFILES[args.robot].params), 3))
    run_mujoco_simulation(args.xml, duration=args.duration, headless=args.headless, realism_seed=args.realism,
                          robot=args.robot,
                          realism_cfg=RealismConfig(arch=args.arch, imu=args.imu, leg_rate_hz=args.leg_rate),
                          controller_kind=args.controller)
