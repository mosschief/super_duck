"""Balance, posture, jump and driving controller (`ScaledWheeledDuckController`).

Pure Python + NumPy: it sees only sensor values and returns actuator commands, never the MuJoCo model, so it is
the part to port to the Teensy firmware.
"""

import math
from typing import Tuple

import numpy as np

from .robot import (DEFAULT_ROBOT, PROFILES, RobotProfile, ScaledWheeledDuckParams, compute_wip_lqr_gains,
                    quat_to_euler_zyx, wip_model)

# GLFW Keycode Constants Used by MuJoCo Viewer `key_callback`
KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262
KEY_SPACE = 32


class ScaledWheeledDuckController:
    """Full-body balance, 4-DOF leg posture, stereo-camera horizon, and live keyboard driving controller."""

    # Deepest crouch delta: knee = -1.00 - 2*delta must stay clear of the knee's -1.5708 rad (90 deg) joint limit,
    # *including* servo sag under load (~0.01-0.02 rad). On the stop, the hip keeps folding, the knee = -2*hip
    # geometry breaks, and the balance lean jumps (at 0.28 the knee rested on the stop and the robot rolled
    # ~18 cm). 0.24 leaves ~0.09 rad of margin and matches the range `posture_trim` was measured over.
    MAX_CROUCH = 0.24
    RAMP_JERK = 8.0        # m/s^3, how quickly the speed ramp's acceleration may fade out near the target speed
    REANCHOR_DIST = 0.30   # m; pushed further than this from the hold point -> hold wherever it comes to rest
    # m/s. The GIM4305-10's speed-torque curve sets this through `speed_cap` (2.78 m/s on a full pack, 2.43 at 22.2 V,
    # 2.30 at the 21 V cutoff). This caps it below that: at 2.8 m/s a 90 deg turn knocks 3 of 10 randomized
    # robots over (the outer wheel runs out of speed), at 2.5 m/s none. It was 1.8 while the wheel joint
    # double-counted the motor's losses (see the MJCF's `gim4305_wheel`).
    MAX_SPEED = 2.50
    # Wheel torque (N*m) kept free at top speed: drag and the motor's speed-torque curve use up the rest, and a
    # balancing robot with none left can't drive under itself to catch a forward lean.
    SPEED_HEADROOM = 0.25
    # Fall cutoff. Past this lean beyond the balance lean no wheel torque brings the robot back (the worst recovered
    # shove in the test suites peaks at 23 deg), and a fallen robot whose balance loop keeps driving scoots along the
    # floor on its head: 4.4 m after a 30 N shove at 2.5 m/s, 5.4 m at 1.8 m/s (1.4 / 1.6 m with the cutoff). So
    # the wheels go to zero torque and the drive command is cleared until the robot has been stood up again.
    FALL_PITCH = math.radians(40.0)     # rad beyond the balance lean
    FALL_ROLL = math.radians(45.0)      # rad; the wheels can't catch a sideways fall at all (turns bank <= 7 deg)
    UPRIGHT_PITCH = math.radians(10.0)  # rad, held this close to the balance lean (and level) ...
    UPRIGHT_RATE = 0.5                  # rad/s, ... this still ...
    UPRIGHT_TIME = 1.0                  # s, ... this long re-arms the balance loop
    WHEEL_DAMPING = 0.045  # N*m*s/rad, virtual wheel damper about the reference speed (see `compute_control`)
    TURN_RATE_MAX = 2.5    # rad/s, heading reference rate at low speed
    TURN_ACCEL_MAX = 1.5   # m/s^2, sideways: speed x turn rate. About half the 3.3 m/s^2 that tips the robot.
    LOW_BATTERY_V = 21.0   # V, bus voltage (filtered) that parks the robot: below this a 40 N shove knocks it over
    LOW_BATTERY_CLEAR_V = 21.8   # V, only a charged pack clears it (a parked pack recovers a few tenths on its own)
    VOLTAGE_FILTER_S = 2.0  # s, so the sag of a jump or a hard stop doesn't count as a flat pack
    TRIM_LIMIT = 0.30      # rad, standing lean plus a 15 deg slope's lean, should the slope feedforward be off
    # Trim learner while parked: gain = TRIM_GAIN_PARKED + self.TRIM_GAIN_PER_M * |position error|, (rad/s) per m. The gain
    # grows with the error so a big trim error (CoM offset, IMU mounting) is learned in ~5 s, without a high gain stirring
    # the robot up on a flat floor. Slopes are the slope observer's job (`_update_slope`); this takes up what it misses.
    TRIM_GAIN_PARKED = 0.03
    TRIM_GAIN_PER_M = 0.3
    TRIM_VEL_WEIGHT = 1.0  # m of position-equivalent error per m/s
    TRIM_GAIN_CRUISE = 0.02  # (rad/s) per (m/s of speed error), while cruising
    TRIM_HOLDOFF = 1.0     # s after start-up and after a leg move
    # Fast lean learner for the rest of the leg-move holdoff (see `_update_lean_estimate`): it starts once the legs have
    # stopped and converges in ~0.5 s. A crouch changes the lean a trunk CoM offset needs (and the robot may not have
    # learned it yet): a robot with its trunk CoM 8 mm off sat out the holdoff ~0.6 deg off its balance lean and rolled
    # 10-12 cm (crouch_stand: 3/10 randomized robots passed). Not used at start-up: on a slope the robot is still far
    # from its hold point then, and a trim corrected before it gets back made the position learner overshoot.
    LEAN_GAIN = 2.0        # 1/s
    LEAN_FILTER_S = 0.3    # s, low-pass on the pitch error and on the accelerations
    LEAN_SETTLE = 0.3      # s after a leg move (the WIP model doesn't hold while the legs swing)
    LEAN_WINDOW = 1.0      # s the fast learner also runs after the hold point is re-anchored on a newly learned slope
    # Slope observer (see `_update_slope`): momentum-observer bandwidth, the slower low-pass on what is fed forward
    # (a real slope lasts; a shove or a bump doesn't), and the steepest slope it will feed forward.
    SLOPE_OBS_TAU = 0.15   # s
    SLOPE_FF_TAU = 0.5     # s
    SLOPE_MAX_DEG = 20.0
    # deg: estimates well under this are mostly not fed forward (scaled by s^4 / (s^4 + this^4)). On a flat floor the
    # estimate wanders ~+/-0.2 deg (sensor noise, latency, motor torque-constant error while braking), and feeding that
    # forward made the parked robot creep ~3 cm on randomized robots; the trim learner takes up the small slopes it
    # leaves out. A hard deadband instead left 1.5 deg out on every slope (s^2 still ~0.2 deg at 10 deg): braking down
    # 10 deg, the robot then crept at a steady 3-5 cm/s, too fast for a brake to count as settled.
    SLOPE_DEADBAND_DEG = 1.5
    # Lean per unit of slope torque, relative to the rigid-body figure M*R*sin(slope) / (m_b*l). Fitted in the sim on
    # held slopes of 5-15 deg with the wheel friction removed (so the trim learner's leftover is the model error): the
    # rigid-body figure is ~12% short (0.66 deg at 15 deg, a ~16 cm hold offset), presumably the leg servos giving
    # under the wheel torque's reaction. Re-fit on hardware, like `posture_trim`.
    SLOPE_LEAN_GAIN = 1.12
    SLOPE_STICK_SPEED = 1.0  # rad/s: a wheel turning slower than this (relative to its leg) may be held by dry friction
    # deg: the trim learner waits while the fed-forward slope is further than this from the observer's (it is still
    # catching up). Otherwise both learned a fresh slope at once and the trim then spent ~5 s unlearning its share:
    # parked on a 10 deg slope, it rolled 10 cm.
    SLOPE_SETTLED_DEG = 1.0
    GYRO_BIAS_TAU = 30.0   # s, drift-following low-pass
    GYRO_BIAS_PRIOR = 500  # samples: the running average starts as if the first second had read zero
    GYRO_STILL_RATE = 0.05  # rad/s (~3 deg/s), faster than this is real motion
    FREEFALL_ACCEL = 3.0   # m/s^2, IMU acceleration magnitude below this is free fall (standing reads 9.81)
    FREEFALL_TIME = 0.025  # s, it must last this long to count as a drop
    TOUCHDOWN_ACCEL = 15.0  # m/s^2, the first reading above this ends a drop
    # A drop also ends once the IMU has read "supported" (> SUPPORTED_ACCEL) for SUPPORTED_TIME: a wheel bouncing off
    # a curb nose is a few mm of real free fall that lands too softly for TOUCHDOWN_ACCEL, and it used to leave the
    # robot on the ground in "drop" (wheels on attitude hold, not balancing) for the 1 s timeout, enough to fall.
    SUPPORTED_ACCEL = 7.0   # m/s^2 (free fall reads ~0, standing 9.81)
    SUPPORTED_TIME = 0.02   # s
    DROP_LEGS_SETTLE = 0.5  # s after a leg move before free fall counts
    DROP_DELTA = 0.10      # Crouch held through a drop: of the poses tried (-0.15 to 0.20) this lands softest
    # Bump detection (driving into a wall / furniture), see `_detect_bump`. Measured in test_robustness.py: the wall at
    # 0.6 m/s reads 50 m/s^2 and 1.3 rad/s back, at 1.2 m/s 111 m/s^2 and 2.7 rad/s; the biggest spikes of everything
    # else while cruising steadily are 30 (a 30 N shove from behind, trunk pitching forward) and 25 (from the front).
    BUMP_ACCEL = 30.0      # m/s^2, IMU acceleration magnitude that arms the detector...
    BUMP_SPEED_ERR = 0.3   # m/s, ...if the robot was cruising steadily (not landing from a hop over a threshold)
    BUMP_PITCH_RATE = 0.8  # rad/s, then the trunk must rotate back, away from the direction of travel...
    BUMP_CONFIRM = 0.012   # s, ...within this long
    BUMP_MIN_SPEED = 0.25  # m/s, reference speed below which bumps aren't checked (no momentum to worry about)
    BUMP_BACKOFF = 0.10    # m, hold point set this far back from where the bump happened
    BUMP_SLOPE_HOLD = 1.5  # s after a bump the slope observer is frozen (still pressed on / rebounding off the obstacle)

    def __init__(self, profile: RobotProfile = None, params: ScaledWheeledDuckParams = None):
        self.profile = profile or PROFILES[DEFAULT_ROBOT]
        self.params = params or self.profile.params
        self.k_lqr = compute_wip_lqr_gains(self.params)
        # Eliminating the wheel torque from the WIP model's x_accel and pitch_accel rows leaves
        # pitch - balance_lean = c_x * x_accel + c_p * pitch_accel (see `_update_lean_estimate`).
        A, B = wip_model(self.params)
        det = B[3, 0] * A[1, 2] - B[1, 0] * A[3, 2]
        self.lean_accel_coeffs = (B[3, 0] / det, -B[1, 0] / det)
        self.lean_filter = None       # low-passed (pitch error, x_vel, pitch_rate)
        self.lean_est = 0.0           # estimated balance lean - balance_pitch (rad)
        self.target_x = 0.0
        self.target_vel = 0.0
        self.cmd_vel = 0.0
        self.target_yaw = 0.0
        self.crouch_delta = 0.0       # Adjusts crouch depth while keeping wheels under hips
        self.head_yaw_offset = 0.0    # Manual head pan offset (rad)
        self.head_pose = None         # (neck_pitch, head_pitch, head_yaw, head_roll) command, or None = auto gimbal
        self.antenna_cmd = 0.0        # Antenna servo target (rad, + = tips swung forward/upright), both sides
        self.projector_on = False
        self.nominal_hip_pitch = 0.50   # knees bend backward (bird-style)
        self.nominal_knee_pitch = -1.00
        self.commanded_leg_fold = self.nominal_hip_pitch + self.nominal_knee_pitch
        # Static CoM offset compensation (rad). With the knees bent backward the CoM sits behind the axle,
        # so the trunk must lean forward ~3.3 deg to balance. This is the initial guess; it is refined online
        # by a slow integrator (see `compute_control`) so crouch changes / payloads are absorbed automatically.
        # Starting value from the profile's fit; each 1 deg of trim error parks the robot ~25 cm off its hold point,
        # so it matters (on hardware, persist the learned trim between runs).
        self.pitch_trim = self.profile.initial_pitch_trim
        self.trim_adapt_gain = self.TRIM_GAIN_PARKED
        self.run_time = 0.0
        self.legs_moved_age = 10.0    # s since the legs were last told to move (crouch key, jump, drop)
        self.last_crouch = 0.0
        self.yaw_ref = None           # Rate-limited heading the wheels steer to (`target_yaw` is where it's going)
        self.gyro_bias = np.zeros(3)  # Learned while parked: see `_update_gyro_bias`
        self.gyro_samples = 0
        self.bus_voltage = None       # Set by whoever reads the GIM drivers' bus voltage (CAN); None = not measured
        self.filtered_voltage = None
        self.low_battery = False
        self.freefall_time = 0.0
        self.supported_time = 0.0
        self.landing_from_drop = False
        self.bump_age = 10.0          # s since the last detected bump (`_detect_bump`)
        self.bump_armed = 0.0         # s left to confirm an acceleration spike as a bump
        self.slope_at_arm = 0.0       # `slope_sin` when the bump detector armed
        self.lean_window = 0.0        # s left of the fast lean learner's window after a slope re-anchor
        self.reset_requested = False
        self.fallen = False           # Fall cutoff latched: wheels off until stood up (see FALL_PITCH)
        self.upright_time = 0.0

        # Jump state machine: "idle" -> "crouch" -> "thrust" -> "flight" -> "land" -> "idle"
        self.jump_phase = "idle"
        self.jump_timer = 0.0
        self.jump_requested = False
        self.jump_takeoff_vel = 0.0
        self.braking = False
        self.settling = False
        self.slope_obs = None         # Momentum observer state (see `_update_slope`); None = (re)start it
        self.slope_force = 0.0        # N, observed external force along the ground (+ = pushing it forward)
        self.slope_sin = 0.0          # sin(slope), + = uphill ahead, low-passed: what the feedforward uses
        self.slope_gap = 0.0          # sin: observer's slope minus `slope_sin` (how far the feedforward still has to go)
        self.slope_before = None      # `slope_sin` when the feedforward last started catching up while parked
        self.last_wheel_torque = 0.0  # N*m, both wheels, as commanded last tick

    def reset_state(self) -> None:
        """Clears all driving/posture/jump references (used on 'R' reset)."""
        self.target_x = 0.0
        self.target_vel = 0.0
        self.cmd_vel = 0.0
        self.target_yaw = 0.0
        self.crouch_delta = 0.0
        self.head_yaw_offset = 0.0
        self.head_pose = None
        self.antenna_cmd = 0.0
        self.pitch_trim = self.profile.initial_pitch_trim
        self.lean_filter = None
        self.lean_est = 0.0
        self.run_time = 0.0
        self.legs_moved_age = 10.0
        self.last_crouch = 0.0
        self.yaw_ref = None
        self.freefall_time = 0.0
        self.supported_time = 0.0
        self.bump_age = 10.0
        self.bump_armed = 0.0
        self.slope_at_arm = 0.0
        self.lean_window = 0.0
        self.low_battery = False
        self.jump_phase = "idle"
        self.jump_timer = 0.0
        self.jump_requested = False
        self.jump_takeoff_vel = 0.0
        self.braking = False
        self.settling = False
        self.slope_obs = None
        self.slope_force = 0.0
        self.slope_sin = 0.0
        self.slope_gap = 0.0
        self.slope_before = None
        self.last_wheel_torque = 0.0
        self.reset_requested = False
        self.fallen = False
        self.upright_time = 0.0

    def handle_keypress(self, keycode: int) -> None:
        """Processes live keyboard commands from the MuJoCo viewer window."""
        if keycode in (KEY_UP, ord("I"), ord("i"), ord("W"), ord("w")):
            self.cmd_vel = float(np.clip(self.cmd_vel + 0.15, -self.MAX_SPEED, self.MAX_SPEED))
            print(f"[Drive] Target Speed: {self.cmd_vel:+.2f} m/s")
        elif keycode in (KEY_DOWN, ord("K"), ord("k"), ord("S"), ord("s")):
            self.cmd_vel = float(np.clip(self.cmd_vel - 0.15, -self.MAX_SPEED, self.MAX_SPEED))
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
        accel_mag: float = None,
        dt: float = 0.002,
    ) -> Tuple[np.ndarray, dict]:
        """
        `leg_fold` / `leg_fold_rate` are the measured hip_pitch + knee angle (and its rate), averaged over both legs,
        from the GIM4305 encoders. If omitted, the commanded value is used (no servo lag compensation).
        `accel_mag` is the magnitude of the IMU's acceleration reading (m/s^2); without it drops aren't detected.

        Computes the 14-element MuJoCo `data.ctrl` vector (no hip yaw: the hips are fixed mounts, steering is by
        wheel torque difference):
          [0:3]   left leg targets (left_hip_roll, left_hip_pitch, left_knee)
          [3:6]   right leg targets (right_hip_roll, right_hip_pitch, right_knee)
          [6:10]  neck & head STS3215 targets (neck_pitch, head_pitch, head_yaw, head_roll)
          [10:12] wheel GIM4305-10 torques (left_wheel_motor, right_wheel_motor)
          [12:14] antenna 9 g micro servo targets (left_antenna, right_antenna)
        """
        roll, pitch, yaw = quat_to_euler_zyx(quat_wxyz)
        self._update_battery(dt)
        parked_before = abs(self.cmd_vel) < 1e-3 and abs(self.target_vel) < 0.05 and not self.settling
        self._update_gyro_bias(gyro_xyz, parked_before and self.jump_phase == "idle", dt)
        roll_rate, pitch_rate, yaw_rate = np.asarray(gyro_xyz, dtype=float) - self.gyro_bias

        # Smoothly ramp target velocity toward keyboard-commanded speed (limits pitch jerk). The accel limit also
        # tapers as sqrt(2*J*|dv|) near the target (an "S-curve" arrival): the robot must lean to accelerate, so
        # an abrupt end to the ramp leaves it leaning, and un-leaning makes it roll back (~10 cm after a brake).
        max_accel = 2.50 if self.braking else 1.20  # m/s^2
        cap = self.speed_cap()
        vel_diff = float(np.clip(self.cmd_vel, -cap, cap)) - self.target_vel
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
        if self.crouch_delta != self.last_crouch or self.jump_phase != "idle":
            self.legs_moved_age = 0.0
            self.last_crouch = self.crouch_delta
        self.legs_moved_age += dt
        self._detect_drop(accel_mag, dt)
        self._detect_bump(accel_mag, pitch_rate, x_pos, x_vel, dt)
        leg_delta, airborne = self._update_jump(x_pos, x_vel, accel_mag, dt)
        self._update_slope(x_vel, pitch, pitch_rate, left_wheel_vel, right_wheel_vel,
                           frozen=(airborne or self.fallen or self.jump_phase != "idle"
                                   or self.legs_moved_age < self.TRIM_HOLDOFF
                                   or self.bump_age < self.BUMP_SLOPE_HOLD),
                           dt=dt)

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
        # moves the trunk mass fore/aft, so the balance lean changes by ~+0.7 deg per 0.1 rad of crouch. Leaving
        # that to the slow trim learner made each 'Z'/'X' press roll the robot ~20 cm while it re-learned.
        # Plus the slope feedforward (see `_update_slope`): on an incline the wheels must hold M*g*R*sin(slope) and the
        # trunk must lean that far further uphill to carry it. The LQR has no integral term, so without this it only
        # found that torque through standing errors: at 15 deg it crawled up at 0.16 m/s of a 0.45 command, and a brake
        # there rolled it back 33 cm while the trim learner caught up (and then the learned "trim" was wrong at the top).
        M = self.params.body_mass + 2.0 * self.params.wheel_mass
        db = math.sin(math.radians(self.SLOPE_DEADBAND_DEG))
        slope_ff = self.slope_sin ** 5 / (self.slope_sin ** 4 + db ** 4)
        lean_sin = self.SLOPE_LEAN_GAIN * M * R * slope_ff / (self.params.body_mass * self.params.com_height)
        slope_lean = math.asin(float(np.clip(lean_sin, -0.5, 0.5)))
        slope_torque = M * self.params.gravity * R * slope_ff
        balance_pitch = self.pitch_trim + self.posture_trim(fold) + slope_lean
        self._update_lean_estimate(pitch - balance_pitch, x_vel, pitch_rate, dt)
        # ...and while the legs move, the balance lean moves with them: the pitch-rate reference is its rate. With a
        # zero rate reference the LQR fought the trunk's rotation to the new lean, paying for it with a velocity error
        # (K_pitch_rate / K_vel ~ 0.9 m per rad): a crouch-then-stand rolled even the ideal robot 7 cm.
        balance_rate = self.posture_trim_slope(fold) * leg_fold_rate if self.jump_phase == "idle" else 0.0
        pitch_err = pitch - balance_pitch
        self._update_fall(pitch_err, roll, pitch_rate, x_pos, yaw, dt)
        pos_err = float(np.clip(x_pos - self.target_x, -0.30, 0.30))
        # When actively cruising, soften position-hold weight so velocity tracking dominates cleanly
        pos_weight = 0.25 if abs(self.cmd_vel) > 1e-3 else 1.0
        state_err = np.array([
            pos_weight * pos_err,
            x_vel - self.target_vel,
            pitch_err,
            pitch_rate - balance_rate,
        ])
        total_balance_torque = float(np.dot(-self.k_lqr, state_err)) + slope_torque

        # Online CoM-offset (pitch trim) estimator: a persistent position error while parked means the trim is
        # wrong (the LQR is fighting a CoM offset, a mis-mounted IMU or what the slope feedforward missed). Integrate it
        # away. Runs when parked (while the speed ramps the robot *must* lean to accelerate, which would corrupt the estimate) and is frozen in jumps.
        # Also frozen while settling after a brake/push: the robot is still rolling to a stop then, and learning
        # from that motion skewed the trim by ~0.5 deg, which shows up as a ~12 cm roll-back once it stops
        # (at rest K_pitch * trim_error = K_x * pos_error, i.e. ~25 cm of offset per degree of trim error).
        # Do not also gate on |x_vel|: a badly wrong trim makes the robot creep at a steady speed, and a speed gate
        # then freezes the very estimator that would fix it (seen as a 0.18 m/s endless creep on a heavier robot).
        parked = abs(self.cmd_vel) < 1e-3 and abs(self.target_vel) < 0.05 and not self.settling
        # Also held off just after the legs move (a crouch rolls the robot a few cm and back on its own) and at start-up
        # (it's still settling onto its wheels). After a leg move the fast lean learner, which needs no position
        # error, takes over once the legs have stopped.
        self.run_time += dt
        # And while the slope feedforward is still settling onto a new slope (see SLOPE_SETTLED_DEG).
        settled_sin = math.sin(math.radians(self.SLOPE_SETTLED_DEG))
        slope_settled = abs(self.slope_gap) < settled_sin
        if parked and not slope_settled and self.slope_before is None:
            self.slope_before = self.slope_sin
        elif slope_settled and self.slope_before is not None:
            # Parked on a slope it didn't know (switched on there): the robot rolled downhill until the feedforward
            # caught up. That run-out is not a trim error, and returning from it through the trim learner wound the
            # trim up by ~1 deg (it rolled back and forth 7 cm for seconds). Hold where it is instead, like REANCHOR
            # after a big push. A shove's blip in the estimate comes back to where it was and keeps the hold point.
            if parked and abs(self.slope_sin - self.slope_before) > settled_sin:
                self.target_x = x_pos
                pos_err = 0.0
                # That also leaves no position error for the trim learner to learn what the slope estimate left over
                # (a trunk CoM offset, a weak motor misreading the slope): let the fast lean learner take it.
                self.lean_window = self.LEAN_WINDOW
            self.slope_before = None
        self.lean_window = max(0.0, self.lean_window - dt)
        if self.fallen:
            pass   # lying on the floor says nothing about the balance lean
        elif self.lean_window > 0.0 and parked and slope_settled and self.jump_phase == "idle":
            # Just re-anchored on a newly learned slope (see above): the fast learner takes what the slope estimate left
            self.pitch_trim = float(np.clip(self.pitch_trim + self.LEAN_GAIN * self.lean_est * dt,
                                            -self.TRIM_LIMIT, self.TRIM_LIMIT))
        # And after a bump: the rebound off the obstacle is not a trim error.
        elif (parked and slope_settled and self.legs_moved_age > self.TRIM_HOLDOFF
                and self.run_time > self.TRIM_HOLDOFF and self.bump_age > self.TRIM_HOLDOFF):
            trim_err = pos_err + self.TRIM_VEL_WEIGHT * (x_vel - self.target_vel)
            gain = self.trim_adapt_gain + self.TRIM_GAIN_PER_M * abs(pos_err)
            self.pitch_trim = float(np.clip(self.pitch_trim - gain * trim_err * dt,
                                            -self.TRIM_LIMIT, self.TRIM_LIMIT))
        elif (parked and slope_settled and self.jump_phase == "idle" and self.run_time > self.TRIM_HOLDOFF
                and self.LEAN_SETTLE < self.legs_moved_age <= self.TRIM_HOLDOFF):
            # (Not at start-up, nor just after the fall cutoff re-arms, which restarts run_time: the fall's braced landing
            # counts as a leg move, and a robot still being held by hand then read as a 3 deg lean error.)
            self.pitch_trim = float(np.clip(self.pitch_trim + self.LEAN_GAIN * self.lean_est * dt,
                                            -self.TRIM_LIMIT, self.TRIM_LIMIT))
        elif self.jump_phase == "idle" and abs(self.cmd_vel) > 1e-3 and abs(vel_diff) < 1e-3 and slope_settled:
            # Cruising at a steady speed (no ramp, so no lean to accelerate): a speed error means the lean is wrong.
            # Learning it here means the robot doesn't start from scratch at each stop.
            self.pitch_trim = float(np.clip(self.pitch_trim - self.TRIM_GAIN_CRUISE * (x_vel - self.target_vel) * dt,
                                            -self.TRIM_LIMIT, self.TRIM_LIMIT))

        if airborne:
            # Wheels are unloaded, so LQR odometry is meaningless. Use the two hub motors as a reaction wheel
            # to hold trunk attitude (reaction torque on the body is -tau), so we touch down level.
            total_balance_torque = 6.0 * pitch_err + 0.6 * pitch_rate
        elif self.jump_phase == "thrust" and self.profile.thrust_wheel_mode != "lqr":
            # During the leg thrust the wheels unload and the odometry is garbage (est. speed swings +/-1-2 m/s),
            # so the LQR slams the wheels to +/-3.47 N*m and the reaction launches the body spinning nose-down.
            total_balance_torque = (6.0 * pitch_err + 0.6 * pitch_rate) if self.profile.thrust_wheel_mode == "attitude" else 0.0

        # 2. Differential Yaw Steering Torque
        # The heading reference moves at a limited rate: a 90 deg step used to put the whole difference on the wheels at
        # once, which at 1.2 m/s took the torque the balance needed and tipped the robot over.
        if self.yaw_ref is None:
            self.yaw_ref = self.target_yaw
        yaw_todo = math.atan2(math.sin(self.target_yaw - self.yaw_ref), math.cos(self.target_yaw - self.yaw_ref))
        turn_rate_max = min(self.TURN_RATE_MAX, self.TURN_ACCEL_MAX / max(abs(x_vel), 1e-3))
        turn_rate_ref = float(np.clip(yaw_todo / dt, -turn_rate_max, turn_rate_max))
        self.yaw_ref += turn_rate_ref * dt
        yaw_err = math.atan2(math.sin(yaw - self.yaw_ref), math.cos(yaw - self.yaw_ref))
        yaw_torque = 0.0 if airborne else (-1.45 * yaw_err - 0.30 * (yaw_rate - turn_rate_ref))

        # Per-wheel friction feedforward (viscous + Coulomb) at the reference speed. Without it the LQR needs a
        # persistent velocity error to supply the drag torque, so cruise speed overshot the command by ~17%.
        wheel_ff = 0.0
        if not airborne and abs(self.target_vel) > 0.01:
            wheel_ff = (self.params.wheel_viscous * self.target_vel / R
                        + self.params.wheel_coulomb * math.copysign(1.0, self.target_vel))

        # Virtual wheel damper on each wheel's speed (encoder, relative to the foot) about the reference speed. The
        # balance tuning was done with a 0.05 N*m*s/rad passive wheel damper that the feedforward above cancelled at
        # the reference speed, so it acted on exactly this error. Without it, slopes held 2 cm looser and a leg
        # going limp for 300 ms knocked the robot over. Being a damper about the reference, it costs no torque at a
        # steady cruise, where the passive one ate 1.2 N*m per wheel at 1.8 m/s.
        omega_ref = self.target_vel / R
        damp_left = -self.WHEEL_DAMPING * (left_wheel_vel - omega_ref)
        damp_right = -self.WHEEL_DAMPING * (right_wheel_vel - omega_ref)

        tau_left = np.clip(
            0.5 * total_balance_torque - yaw_torque + wheel_ff + damp_left,
            -self.params.max_wheel_torque,
            self.params.max_wheel_torque,
        )
        tau_right = np.clip(
            0.5 * total_balance_torque + yaw_torque + wheel_ff + damp_right,
            -self.params.max_wheel_torque,
            self.params.max_wheel_torque,
        )
        if self.fallen:
            tau_left = tau_right = 0.0

        # 3. 4-DOF GIM4305-10 Upper Leg Coordination (Roll Leveling + Turn Banking + Crouch Height)
        turn_bank = np.clip(-math.atan2(x_vel * turn_rate_ref, self.params.gravity), -0.12, 0.12)
        roll_comp = np.clip(-0.55 * (roll - turn_bank) - 0.10 * roll_rate, -0.30, 0.30)
        # NOTE: the previous hip-pitch balance assist (-0.18*pitch_err - 0.04*pitch_rate) was removed: with the
        # compliant kp=32 GIM4305 position loops it coupled into the LQR and produced a saturated ~2.5 Hz
        # bang-bang limit cycle (wheel torque pinned at +/-3.47 N*m and the robot drifting away at 1.6 m/s).
        # Keep knee = -2 * hip_pitch so the wheel axle stays vertically aligned under the hip at any crouch height
        hip_pitch_cmd = self.nominal_hip_pitch + leg_delta
        knee_cmd = self.nominal_knee_pitch - 2.0 * leg_delta
        self.commanded_leg_fold = hip_pitch_cmd + knee_cmd

        # 4. Horizon-Stabilized Stereo Camera Gimbal (Looks into turns + manual pan), or a commanded head pose
        if self.head_pose is not None:
            neck_pitch_cmd, head_pitch_cmd, head_yaw_cmd, head_roll_cmd = self.head_pose
        else:
            neck_pitch_cmd = 0.15 - 0.5 * pitch
            head_pitch_cmd = -0.15 - 0.5 * pitch
            look = math.atan2(math.sin(yaw - self.target_yaw), math.cos(yaw - self.target_yaw))   # into the turn
            head_yaw_cmd = -0.6 * look + self.head_yaw_offset
            head_roll_cmd = -roll
        neck_pitch_cmd, head_pitch_cmd = self.clip_neck_envelope(neck_pitch_cmd, head_pitch_cmd)
        head_yaw_cmd = float(np.clip(head_yaw_cmd, -1.5, 1.5))
        head_roll_cmd = float(np.clip(head_roll_cmd, -0.45, 0.45))

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

        # What the motors deliver, not what was asked: at speed the speed-torque curve caps the motoring torque, and the
        # slope observer read the shortfall as an uphill (a shove at 1.8 m/s read as a 7 deg slope). On hardware: the
        # drivers' reported current.
        self.last_wheel_torque = (self.deliverable_wheel_torque(tau_left, left_wheel_vel)
                                  + self.deliverable_wheel_torque(tau_right, right_wheel_vel))

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
            "slope_deg": math.degrees(math.asin(self.slope_sin)),
            "jump_phase": self.jump_phase,
            "fallen": self.fallen,
        }
        return ctrl, telemetry

    # Neck envelope (cad/freecad/neck_envelope.md, the CAD's clash-free region for any head yaw / roll): the head
    # hits the trunk when both pitch joints go to their extremes in the same direction. MJCF joint ranges can't
    # express this coupled limit, so every head command goes through `clip_neck_envelope`.
    NECK_RANGE = (-0.30, 0.90)               # neck_pitch command range (rad)
    HEAD_PITCH_RANGE = (-0.70, math.radians(15.0))
    NECK_SUM_RANGE = (math.radians(-15.0), math.radians(65.0))   # neck_pitch + head_pitch

    def clip_neck_envelope(self, neck: float, head: float) -> Tuple[float, float]:
        """Clips (neck_pitch, head_pitch) into the clash-free envelope. The head pitch gives way first (it keeps the
        neck's gaze lift), then the neck."""
        neck = float(np.clip(neck, *self.NECK_RANGE))
        lo, hi = self.NECK_SUM_RANGE
        head = float(np.clip(head, max(self.HEAD_PITCH_RANGE[0], lo - neck), min(self.HEAD_PITCH_RANGE[1], hi - neck)))
        neck = float(np.clip(neck, lo - head, hi - head))
        return neck, head

    def posture_trim(self, fold: float) -> float:
        """Change in balance lean (rad) vs. zero leg fold, from the profile's fit (see `RobotProfile`).

        Crouching moves the trunk mass fore/aft, so the balance lean changes (~-0.7 deg per 0.1 rad of fold on the
        gim6010 build; with backward knees, crouching makes the fold more negative). A static CoM-over-axle calculation gives the same shape but ~25% less slope, because it
        ignores the legs sagging under load on their kp=32 servos.
        """
        a, b = self.profile.posture_coeffs
        return a * fold + b * fold * fold

    def posture_trim_slope(self, fold: float) -> float:
        """d posture_trim / d fold (rad of lean per rad of fold)."""
        a, b = self.profile.posture_coeffs
        return a + 2.0 * b * fold

    # Leg servo damping (GIM4305 MIT-mode kd, N*m*s/rad). High damping stops the compliant kp=32 legs from
    # pogoing under hard wheel torque (which bounced the tires off the floor during push recovery), but it also
    # resists the fast leg extension of a jump, so it is dropped for the jump wind-up and thrust.
    LEG_KD_NOMINAL = 4.0
    LEG_KD_JUMP = 1.4

    def leg_servo_kd(self) -> float:
        """Damping gain the leg servos should run with right now (applied by the sim loop each step)."""
        return self.LEG_KD_JUMP if self.jump_phase in ("crouch", "thrust") else self.LEG_KD_NOMINAL

    # Jump trajectory tuning (leg_delta is the symmetric crouch offset: hip = 0.50 + d, knee = -1.00 - 2d)
    JUMP_CROUCH_DELTA = MAX_CROUCH  # Deep wind-up crouch (hip 0.74, knee -1.48)
    JUMP_EXTEND_DELTA = -0.46   # Near-straight legs at take-off (hip 0.04, knee -0.08)
    JUMP_LAND_DELTA = -0.15     # Slightly bent, compliant legs for touchdown
    JUMP_CROUCH_TIME = 0.40     # s
    JUMP_FLIGHT_TIME = 0.20     # s (wheels unloaded; tucks legs toward landing pose)
    JUMP_LAND_TIME = 0.50       # s (blend back to the driver's crouch setting)

    def speed_cap(self) -> float:
        """Top speed (m/s) that leaves `SPEED_HEADROOM` of wheel torque after drag, at the present bus voltage.

        Available torque = (no-load rpm x V / 24 - rpm) / (rpm drop per N*m), drag = viscous x omega + Coulomb.
        """
        curve = self.profile.wheel_torque_curve
        if curve is None:
            return self.MAX_SPEED
        _, no_load_rpm, rpm_per_nm = curve
        volts = 24.0 if self.filtered_voltage is None else self.filtered_voltage
        rpm_per_rad_s = 60.0 / (2.0 * math.pi)
        free = no_load_rpm * volts / 24.0 / rpm_per_nm - self.params.wheel_coulomb - self.SPEED_HEADROOM
        omega = free / (rpm_per_rad_s / rpm_per_nm + self.params.wheel_viscous)
        return float(np.clip(omega * self.params.wheel_radius, 0.0, self.MAX_SPEED))

    def _update_slope(self, x_vel: float, pitch: float, pitch_rate: float, left_wheel_vel: float,
                      right_wheel_vel: float, frozen: bool, dt: float) -> None:
        """Estimates the slope underfoot from wheel odometry, the gyro and the wheel torque (a momentum observer).

        The robot's momentum along the ground, p = (M + 2*I_w/R^2)*v + m_b*l*cos(pitch)*pitch_rate (wheels' rolling
        speed, plus the trunk swinging over the axle, plus the wheels' spin), changes only through the wheel torque
        and outside forces: dp/dt = tau/R - M*g*sin(slope) (+ a shove). The observer's output r follows the outside
        force with a first-order lag of SLOPE_OBS_TAU without differentiating anything, so it is not noisy, and its
        estimate does not depend on the IMU's mounting (pitch only enters through cos(pitch)). It does see any
        lasting push as a slope, which is what the robot should lean into anyway; brief ones (a shove, a bump) are
        mostly filtered out by the slower SLOPE_FF_TAU low-pass on what is fed forward. Accelerating is explained by
        the momentum change, so the speed ramps and brakes don't read as slopes.

        Torque is what was commanded last tick, less the wheel joints' modelled friction (on hardware: the drivers'
        reported current). A torque-constant error scales the estimate (a 10% weak motor reads 15 deg as 16), and
        the trim learner takes up the lean that leaves. Frozen (held) while a wheel may be unloaded or the legs are
        moving the trunk mass around.
        """
        P = self.params
        R = P.wheel_radius
        M = P.body_mass + 2.0 * P.wheel_mass
        p = (M + 2.0 * P.wheel_inertia / R ** 2) * x_vel + P.body_mass * P.com_height * math.cos(pitch) * pitch_rate
        k = 1.0 / self.SLOPE_OBS_TAU
        if self.slope_obs is None or frozen:
            self.slope_obs = p - self.slope_force / k   # (re)start without a jump in the output
            return
        friction = sum(P.wheel_viscous * w + P.wheel_coulomb * math.tanh(w / 0.5) for w in (left_wheel_vel, right_wheel_vel))
        self.slope_obs += ((self.last_wheel_torque - friction) / R + self.slope_force) * dt
        self.slope_force = k * (p - self.slope_obs)
        s_max = math.sin(math.radians(self.SLOPE_MAX_DEG))
        s = float(np.clip(-self.slope_force / (M * P.gravity), -s_max, s_max))
        self.slope_gap = s - self.slope_sin
        # Backlash of the wheel joints' dry friction: near standstill it takes any value within +/-coulomb per wheel,
        # so the observer can't tell +/-0.33 deg of slope from it. Following that made the estimate and the parked
        # robot swing in an ~8 s limit cycle (0.6 deg, 3.7 cm on a 15 deg slope; none with the friction removed).
        # Only near standstill: while both wheels turn the friction's direction is known (and the estimate then tracks
        # a slope fully instead of stopping a band short of it, which left the robot creeping after a brake).
        stuck = min(abs(left_wheel_vel), abs(right_wheel_vel)) < self.SLOPE_STICK_SPEED
        band = 2.0 * P.wheel_coulomb / R / (M * P.gravity) if stuck else 0.0
        s = self.slope_sin + math.copysign(max(abs(s - self.slope_sin) - band, 0.0), s - self.slope_sin)
        self.slope_sin += (s - self.slope_sin) * dt / self.SLOPE_FF_TAU

    def deliverable_wheel_torque(self, tau: float, omega: float) -> float:
        """Wheel torque the motor can deliver at wheel speed `omega` (rad/s, relative to the foot) and the present pack
        voltage: the speed-torque curve caps torque *with* the motion, the drivers' current limit any torque."""
        curve = self.profile.wheel_torque_curve
        if curve is None:
            return float(tau)
        stall, no_load_rpm, rpm_per_nm = curve
        volts = 24.0 if self.filtered_voltage is None else self.filtered_voltage
        rpm = abs(omega) * 60.0 / (2.0 * math.pi)
        limit = float(np.clip((no_load_rpm * volts / 24.0 - rpm) / rpm_per_nm, 0.05, stall))
        if tau * omega > 0.0:
            return float(math.copysign(min(abs(tau), limit), tau))
        return float(np.clip(tau, -stall, stall))

    def _reset_slope(self, slope_sin: float = 0.0) -> None:
        """Restarts the slope observer at `slope_sin` (see `_update_slope`)."""
        P = self.params
        self.slope_sin = slope_sin
        self.slope_force = -slope_sin * (P.body_mass + 2.0 * P.wheel_mass) * P.gravity
        self.slope_obs = None
        self.slope_gap = 0.0
        self.slope_before = None

    def _update_battery(self, dt: float) -> None:
        """Low battery: stop, crouch to shorten the eventual fall, refuse to drive or jump until the pack is charged."""
        if self.bus_voltage is None:
            return
        if self.filtered_voltage is None:
            self.filtered_voltage = float(self.bus_voltage)
        self.filtered_voltage += (self.bus_voltage - self.filtered_voltage) * dt / self.VOLTAGE_FILTER_S
        if not self.low_battery and self.filtered_voltage < self.LOW_BATTERY_V:
            self.low_battery = True
            self.braking = abs(self.target_vel) > 1e-6
            print(f"[Battery] {self.filtered_voltage:.1f} V: parking")
        elif self.low_battery and self.filtered_voltage > self.LOW_BATTERY_CLEAR_V:
            self.low_battery = False
            print(f"[Battery] {self.filtered_voltage:.1f} V: ok")
        if self.low_battery:
            self.cmd_vel = 0.0
            self.jump_requested = False
            self.crouch_delta = min(self.MAX_CROUCH, self.crouch_delta + 0.10 * dt)

    def _update_fall(self, pitch_err: float, roll: float, pitch_rate: float, x_pos: float, yaw: float, dt: float) -> None:
        """Fall cutoff (see `FALL_PITCH`): latches past the fall lean, re-arms once held upright and still."""
        if not self.fallen:
            if abs(pitch_err) > self.FALL_PITCH or abs(roll) > self.FALL_ROLL:
                self.fallen, self.upright_time = True, 0.0
                self.cmd_vel = self.target_vel = 0.0
                self.braking = self.settling = self.jump_requested = False
                print(f"[Fall] pitch {math.degrees(pitch_err):+.0f} deg past the balance lean, roll "
                      f"{math.degrees(roll):+.0f} deg: wheels off")
            return
        upright = (abs(pitch_err) < self.UPRIGHT_PITCH and abs(roll) < self.UPRIGHT_PITCH
                   and abs(pitch_rate) < self.UPRIGHT_RATE)
        self.upright_time = self.upright_time + dt if upright else 0.0
        self.cmd_vel = self.target_vel = 0.0
        self.target_x = x_pos
        self.target_yaw = self.yaw_ref = yaw   # it may have been stood up facing anywhere
        if self.upright_time >= self.UPRIGHT_TIME:
            self.fallen = False
            self.run_time = 0.0   # trim learner holds off as at start-up: it's still settling onto its wheels
            # Picked up and set down somewhere else: start the slope observer afresh, as at start-up (what it held
            # from the fall, the shove included, says nothing about the floor here).
            self._reset_slope()
            print("[Fall] Upright again: balancing")

    def _update_lean_estimate(self, pitch_err: float, x_vel: float, pitch_rate: float, dt: float) -> None:
        """Balance lean error from accelerations, without waiting for a position error to build up.

        A robot leaning off its balance point accelerates: by the WIP model, pitch - balance_lean = c_x * x_accel +
        c_p * pitch_accel whatever the wheel torque. So balance_lean - balance_pitch = pitch_err - c_x * x_accel -
        c_p * pitch_accel, all low-passed (the accelerations as the derivative of the low-passed rates). Trunk CoM
        offset, payload and IMU mounting error all show up in it. It needs the legs still (the model has fixed legs).
        """
        f = self.lean_filter
        if f is None:
            # The estimate is f[0] - c_x * x_accel - c_p * pitch_accel, so starting f[0] at 0 (not at the pitch error)
            # starts the estimate at 0: the present trim is the prior. Starting it at the start pose's pitch error
            # (the robot isn't leaning in yet) left a bias of -0.6 deg 0.6 s later.
            f = self.lean_filter = [0.0, x_vel, pitch_rate]
        k = dt / self.LEAN_FILTER_S
        x_accel = (x_vel - f[1]) / self.LEAN_FILTER_S
        pitch_accel = (pitch_rate - f[2]) / self.LEAN_FILTER_S
        f[0] += (pitch_err - f[0]) * k
        f[1] += (x_vel - f[1]) * k
        f[2] += (pitch_rate - f[2]) * k
        c_x, c_p = self.lean_accel_coeffs
        self.lean_est = f[0] - c_x * x_accel - c_p * pitch_accel

    def _update_gyro_bias(self, gyro_xyz, parked: bool, dt: float) -> None:
        """A parked robot's mean turn rate is zero, so the gyro's mean is its bias. A running average gets close within
        seconds; after that a slow low-pass follows temperature drift. Any real motion is left out of the average."""
        gyro = np.asarray(gyro_xyz, dtype=float)
        if not parked or np.abs(gyro).max() > self.GYRO_STILL_RATE:
            return
        self.gyro_samples += 1
        k = max(1.0 / (self.gyro_samples + self.GYRO_BIAS_PRIOR), dt / self.GYRO_BIAS_TAU)
        self.gyro_bias = self.gyro_bias + k * (gyro - self.gyro_bias)

    def _detect_drop(self, accel_mag: float, dt: float) -> None:
        """Free fall (off a step, or lifted and let go): straighten the legs for stroke and soften them to take the
        landing. The IMU reads near zero in free fall, whichever way the robot is tilted. A fast crouch reads low too (the
        trunk drops away from the wheels), so it doesn't count just after the legs were told to move."""
        if accel_mag is None or self.jump_phase != "idle" or self.legs_moved_age < self.DROP_LEGS_SETTLE:
            self.freefall_time = 0.0
            return
        self.freefall_time = self.freefall_time + dt if accel_mag < self.FREEFALL_ACCEL else 0.0
        if self.freefall_time >= self.FREEFALL_TIME:
            self.jump_phase, self.jump_timer, self.freefall_time, self.supported_time = "drop", 0.0, 0.0, 0.0
            self.landing_from_drop = True
            print("[Drop] Free fall, bracing")

    def _detect_bump(self, accel_mag: float, pitch_rate: float, x_pos: float, x_vel: float, dt: float) -> None:
        """Driving into a wall: stop driving, hold a little way back from it.

        The head sticks out ahead of the wheels, so on a wall it hits first and stops the top of the robot while the
        wheels roll on: a sharp deceleration (the IMU reads several g) out of a steady cruise, then within a few ms
        the trunk rotating back, away from the direction of travel. A wheel hitting a threshold or curb first pitches
        the trunk forward over the stopped wheel; the back-pitching spikes there come when the robot lands after
        hopping it, running well above the reference speed (hence the steady-cruise condition). A shove reads <= 30
        m/s^2, and a hard brake, a turn or a slope doesn't come near. Only the IMU and wheel encoders are used.
        A wall is caught from ~0.45 m/s; at 0.3 m/s it reads under the threshold and the robot still ends up leaning
        on it (~10 deg, it stays up).
        Without this the speed reference kept running ahead, the LQR drove the robot back into the wall, and the
        cruise trim learner read the stall as "lean further": it ended up pinned to the wall at a 14-22 deg lean.
        The stop clears the speed command (on a gamepad: until the stick is pushed again)."""
        self.bump_age += dt
        self.bump_armed = max(0.0, self.bump_armed - dt)
        v = self.target_vel
        if (accel_mag is None or self.jump_phase != "idle" or self.legs_moved_age < self.DROP_LEGS_SETTLE
                or abs(v) < self.BUMP_MIN_SPEED):
            self.bump_armed = 0.0
            return
        if accel_mag > self.BUMP_ACCEL and abs(x_vel - v) < self.BUMP_SPEED_ERR:
            if self.bump_armed <= 0.0:
                self.slope_at_arm = self.slope_sin   # before the impact's force reaches the slope observer
            self.bump_armed = max(self.bump_armed, self.BUMP_CONFIRM)
        if self.bump_armed <= 0.0 or math.copysign(1.0, v) * pitch_rate > -self.BUMP_PITCH_RATE:
            return
        self.bump_armed = 0.0
        self.cmd_vel = self.target_vel = 0.0
        self.braking = self.settling = False
        self.target_x = x_pos - math.copysign(self.BUMP_BACKOFF, v)
        self.bump_age = 0.0
        # The obstacle's push is not a slope: the slope observer read a wall as a 3 deg upslope, fed that forward
        # (leaning back into the wall) and, once it settled, re-anchored the hold point on the wall. Restore the
        # estimate from before the impact; `_update_slope` stays frozen for BUMP_SLOPE_HOLD while it backs off.
        self._reset_slope(self.slope_at_arm)
        print(f"[Bump] {accel_mag:.0f} m/s^2 at {v:+.2f} m/s: stopping, backing off {self.BUMP_BACKOFF:.2f} m")

    def _update_jump(self, x_pos: float, x_vel: float, accel_mag: float, dt: float) -> Tuple[float, bool]:
        """Advances the timed jump state machine. Returns (leg_delta, airborne)."""
        if self.jump_requested and self.jump_phase == "idle":
            self.jump_phase, self.jump_timer, self.jump_requested = "crouch", 0.0, False
            self.landing_from_drop = False
        if self.jump_phase == "idle":
            return self.crouch_delta, False

        self.jump_timer += dt
        t = self.jump_timer
        if self.jump_phase == "drop":
            # Ends at touchdown: an impact spike, or a soft landing read as `SUPPORTED_TIME` of ~1 g (or after 1 s:
            # nothing falls that long indoors, so the sensor must be wrong)
            supported = accel_mag is not None and accel_mag > self.SUPPORTED_ACCEL
            self.supported_time = self.supported_time + dt if supported else 0.0
            if ((accel_mag is not None and accel_mag > self.TOUCHDOWN_ACCEL) or self.supported_time >= self.SUPPORTED_TIME
                    or t >= 1.0):
                self.supported_time = 0.0
                self.jump_phase, self.jump_timer = "land", 0.0
                self.target_x = x_pos
                return self.DROP_DELTA, False
            return self.DROP_DELTA, True
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
        start = self.DROP_DELTA if self.landing_from_drop else self.JUMP_LAND_DELTA
        return start + a * (self.crouch_delta - start), False
