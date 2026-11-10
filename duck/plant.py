"""The simulated plant around the controller: actuator layout, sim-to-real realism layer (sensor noise, CAN timing,
domain randomization), sensor readout, motor torque-speed curves and the per-tick `step_robot`."""

import math
import pathlib
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .controller import ScaledWheeledDuckController
from .robot import RobotProfile

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent


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
    accel_noise: float = 0.1                    # accelerometer magnitude noise, RMS per sample (m/s^2)
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

    def sense(self, quat, gyro, lw_pos, rw_pos, lw_vel, rw_vel, leg_fold, leg_fold_rate, accel_mag=None):
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
        if accel_mag is not None:
            accel_mag = abs(accel_mag + rng.normal(0.0, cfg.accel_noise))
        return q.copy(), g.copy(), lw_pos, rw_pos, lw_vel, rw_vel, *self.held_legs, accel_mag

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
            float(s("left_wheel_vel")[0]), float(s("right_wheel_vel")[0]), float(leg_fold), float(leg_fold_rate),
            float(np.linalg.norm(s("accelerometer", 3))))


def apply_torque_curve(model, data, curve, actuators, floor: float = 0.5, volts: float = 24.0) -> None:
    """Caps the motors' torque at what they can deliver at their current speed.

    MuJoCo's forcerange is speed-independent, but a real motor runs out of voltage as it speeds up: the GIM6010-8
    gives 11 N*m at standstill but only ~6 N*m at the ~185 rpm the jump thrust reaches.

    The curve only limits torque *with* the motion (motoring). Torque against it (braking) is helped by the back-EMF
    instead of fought by it, so there the drivers' current limit (stall torque) is the only cap. Capping both ways
    let a wheel at 2.5 m/s brake with only 0.7 N*m, and a backward shove at that speed tipped the robot over.
    """
    stall, no_load_rpm, rpm_per_nm = curve
    no_load_rpm *= volts / 24.0   # no-load speed goes with the pack voltage; drivers current-limit the stall torque
    actuators = list(actuators)
    dof = model.jnt_dofadr[model.actuator_trnid[actuators, 0]]
    vel = data.qvel[dof]
    rpm = np.abs(vel) * 60.0 / (2.0 * math.pi)
    limit = np.clip((no_load_rpm - rpm) / rpm_per_nm, floor, stall)
    model.actuator_forcerange[actuators, 0] = np.where(vel < 0, -limit, -stall)
    model.actuator_forcerange[actuators, 1] = np.where(vel > 0, limit, stall)


def step_robot(model, data, controller: "ScaledWheeledDuckController", realism: "SimToReal" = None) -> dict:
    """One 500 Hz control tick: sense -> control -> (delayed) actuate -> physics step. Returns telemetry."""
    import mujoco
    sensors = read_sensors(model, data)
    if realism is not None:
        sensors = realism.sense(*sensors)
    ctrl, info = controller.compute_control(*sensors, dt=float(model.opt.timestep))
    data.ctrl[:] = realism.actuate(ctrl) if realism is not None else ctrl
    apply_leg_servo_kd(model, controller.leg_servo_kd())
    volts = 24.0 if controller.bus_voltage is None else controller.bus_voltage
    if controller.profile.leg_torque_curve is not None:
        apply_torque_curve(model, data, controller.profile.leg_torque_curve, controller.profile.curve_actuators,
                           volts=volts)
    if controller.profile.wheel_torque_curve is not None:
        apply_torque_curve(model, data, controller.profile.wheel_torque_curve, (10, 11), floor=0.05, volts=volts)
    mujoco.mj_step(model, data)
    return info


def profile_xml(profile: RobotProfile) -> str:
    """Absolute path of a profile's MJCF (it lives in the repo root)."""
    return str(REPO_ROOT / profile.xml)
