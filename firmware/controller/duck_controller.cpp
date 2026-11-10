// C++ port of duck/controller.py. See duck_controller.hpp.
//
// Kept line-for-line with the Python, in the same order of operations, so the results agree to rounding: the
// comments on *why* live in the Python, which stays the reference. Python's min(a, b) / max(a, b) / np.clip are
// std::min / std::max / clip with the same argument order (they pick the same operand on ties).
#include "duck_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace duck {
namespace {

constexpr double kPi = 3.141592653589793;   // math.pi

inline double clip(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }   // np.clip
inline double radians(double deg) { return deg * (kPi / 180.0); }                             // math.radians
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

void quat_to_euler_zyx(const double q[4], double& roll, double& pitch, double& yaw) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    const double sinp = 2.0 * (w * y - z * x);
    pitch = std::fabs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

Controller::Controller(const Config& config, LogFn log) : state{}, cfg_(config), log_(log) {
    State& s = state;
    s.nominal_hip_pitch = 0.50;
    s.nominal_knee_pitch = -1.00;
    s.commanded_leg_fold = s.nominal_hip_pitch + s.nominal_knee_pitch;
    s.trim_adapt_gain = cfg_.TRIM_GAIN_PARKED;
    s.gyro_bias_0 = s.gyro_bias_1 = s.gyro_bias_2 = 0.0;
    s.gyro_samples = 0.0;
    s.has_bus_voltage = s.has_filtered_voltage = false;
    s.landing_from_drop = false;
    s.projector_on = false;
    reset_state();
}

void Controller::reset_state() {
    State& s = state;
    s.target_x = s.target_vel = s.cmd_vel = s.target_yaw = 0.0;
    s.crouch_delta = s.head_yaw_offset = 0.0;
    s.has_head_pose = false;
    s.antenna_cmd = 0.0;
    s.pitch_trim = cfg_.initial_pitch_trim;
    s.has_lean_filter = false;
    s.lean_est = 0.0;
    s.run_time = 0.0;
    s.legs_moved_age = 10.0;
    s.last_crouch = 0.0;
    s.has_yaw_ref = false;
    s.freefall_time = s.supported_time = 0.0;
    s.bump_age = 10.0;
    s.bump_armed = s.slope_at_arm = 0.0;
    s.lean_window = 0.0;
    s.low_battery = false;
    s.jump_phase = kIdle;
    s.jump_timer = 0.0;
    s.jump_requested = false;
    s.jump_takeoff_vel = 0.0;
    s.braking = s.settling = false;
    s.has_slope_obs = false;
    s.slope_force = s.slope_sin = s.slope_gap = 0.0;
    s.has_slope_before = false;
    s.last_wheel_torque = 0.0;
    s.reset_requested = false;
    s.fallen = false;
    s.upright_time = 0.0;
}

void Controller::log(const char* fmt, ...) const {
    if (log_ == nullptr) return;
    char buf[160];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, args);
    va_end(args);
    log_(buf);
}

void Controller::command(Command cmd) {
    State& s = state;
    const Config& c = cfg_;
    switch (cmd) {
        case kFaster:
            s.cmd_vel = clip(s.cmd_vel + 0.15, -c.MAX_SPEED, c.MAX_SPEED);
            log("[Drive] Target Speed: %+.2f m/s", s.cmd_vel);
            break;
        case kSlower:
            s.cmd_vel = clip(s.cmd_vel - 0.15, -c.MAX_SPEED, c.MAX_SPEED);
            log("[Drive] Target Speed: %+.2f m/s", s.cmd_vel);
            break;
        case kSteerLeft:
            s.target_yaw += radians(15.0);
            log("[Steer] Target Heading: %+.1f deg", s.target_yaw * (180.0 / kPi));
            break;
        case kSteerRight:
            s.target_yaw -= radians(15.0);
            log("[Steer] Target Heading: %+.1f deg", s.target_yaw * (180.0 / kPi));
            break;
        case kBrake:
            s.cmd_vel = 0.0;
            s.braking = true;
            log("[Brake] Full Stop (Holding Position)");
            break;
        case kCrouchDeeper:
            s.crouch_delta = clip(s.crouch_delta + 0.08, -0.25, c.MAX_CROUCH);
            log("[Posture] Crouch Deeper (delta = %+.2f rad)", s.crouch_delta);
            break;
        case kStandTaller:
            s.crouch_delta = clip(s.crouch_delta - 0.08, -0.25, c.MAX_CROUCH);
            log("[Posture] Stand Taller (delta = %+.2f rad)", s.crouch_delta);
            break;
        case kLookLeft:
            s.head_yaw_offset = clip(s.head_yaw_offset + 0.20, -1.20, 1.20);
            log("[Head] Look Left (%+.1f deg)", s.head_yaw_offset * (180.0 / kPi));
            break;
        case kLookRight:
            s.head_yaw_offset = clip(s.head_yaw_offset - 0.20, -1.20, 1.20);
            log("[Head] Look Right (%+.1f deg)", s.head_yaw_offset * (180.0 / kPi));
            break;
        case kJump:
            if (s.jump_phase == kIdle) {
                s.jump_requested = true;
                log("[Jump] Crouching for jump...");
            }
            break;
        case kAntennasUp:
            s.antenna_cmd = clip(s.antenna_cmd + 0.20, -0.78, 0.78);
            log("[Antennas] Raise (%+.0f deg)", s.antenna_cmd * (180.0 / kPi));
            break;
        case kAntennasDown:
            s.antenna_cmd = clip(s.antenna_cmd - 0.20, -0.78, 0.78);
            log("[Antennas] Lower (%+.0f deg)", s.antenna_cmd * (180.0 / kPi));
            break;
        case kToggleProjector:
            s.projector_on = !s.projector_on;
            log("[Projector] %s", s.projector_on ? "On" : "Off");
            break;
        case kReset:
            s.reset_requested = true;
            log("[Reset] Resetting robot to home balanced pose...");
            break;
    }
}

void Controller::compute(const Inputs& in, Outputs& out) {
    State& s = state;
    const Config& c = cfg_;
    const double dt = in.dt;

    double roll, pitch, yaw;
    quat_to_euler_zyx(in.quat_wxyz, roll, pitch, yaw);
    update_battery(dt);
    const bool parked_before = std::fabs(s.cmd_vel) < 1e-3 && std::fabs(s.target_vel) < 0.05 && !s.settling;
    update_gyro_bias(in.gyro_xyz, parked_before && s.jump_phase == kIdle, dt);
    const double roll_rate = in.gyro_xyz[0] - s.gyro_bias_0;
    const double pitch_rate = in.gyro_xyz[1] - s.gyro_bias_1;
    const double yaw_rate = in.gyro_xyz[2] - s.gyro_bias_2;

    // Speed ramp with an S-curve arrival
    double max_accel = s.braking ? 2.50 : 1.20;
    const double cap = speed_cap();
    const double vel_diff = clip(s.cmd_vel, -cap, cap) - s.target_vel;
    max_accel = std::min(max_accel, std::sqrt(2.0 * c.RAMP_JERK * std::fabs(vel_diff)));
    s.target_vel += clip(vel_diff, -max_accel * dt, max_accel * dt);
    if (s.braking && std::fabs(s.target_vel) < 1e-6) {
        s.braking = false;
        s.settling = true;
    }

    // Wheel odometry, corrected for the measured leg fold
    const double R = c.wheel_radius;
    double leg_fold = in.leg_fold, leg_fold_rate = in.leg_fold_rate;
    if (!in.has_leg_fold) {
        leg_fold = s.commanded_leg_fold;
        leg_fold_rate = 0.0;
    }
    const double fold = leg_fold - (s.nominal_hip_pitch + s.nominal_knee_pitch);
    const double mean_wheel_angle = 0.5 * (in.left_wheel_pos + in.right_wheel_pos) + pitch + fold;
    const double mean_wheel_omega = 0.5 * (in.left_wheel_vel + in.right_wheel_vel) + pitch_rate + leg_fold_rate;
    const double x_pos = R * mean_wheel_angle;
    const double x_vel = R * mean_wheel_omega;

    // 0. Jump state machine, drop and bump detection, slope observer
    if (s.crouch_delta != s.last_crouch || s.jump_phase != kIdle) {
        s.legs_moved_age = 0.0;
        s.last_crouch = s.crouch_delta;
    }
    s.legs_moved_age += dt;
    detect_drop(in.has_accel, in.accel_mag, dt);
    detect_bump(in.has_accel, in.accel_mag, pitch_rate, x_pos, x_vel, dt);
    bool airborne;
    const double leg_delta = update_jump(x_pos, x_vel, in.has_accel, in.accel_mag, dt, airborne);
    update_slope(x_vel, pitch, pitch_rate, in.left_wheel_vel, in.right_wheel_vel,
                 airborne || s.fallen || s.jump_phase != kIdle || s.legs_moved_age < c.TRIM_HOLDOFF ||
                     s.bump_age < c.BUMP_SLOPE_HOLD,
                 dt);

    // Hold point
    if (std::fabs(s.cmd_vel) > 1e-3 || std::fabs(s.target_vel) > 1e-3) {
        s.target_x = clip(s.target_x + s.target_vel * dt, x_pos - 0.25, x_pos + 0.25);
        s.settling = false;
    } else if (std::fabs(x_pos - s.target_x) > c.REANCHOR_DIST && std::fabs(x_vel) < 0.05 && s.jump_phase == kIdle) {
        s.target_x = x_pos;
    }
    if (s.settling && std::fabs(s.cmd_vel) <= 1e-3 && std::fabs(s.target_vel) <= 1e-3) {
        s.target_x = x_pos;
        if (std::fabs(x_vel) < 0.03) s.settling = false;
    }

    // 1. LQR wheel balancing torque, with posture and slope feedforward
    const double M = c.body_mass + 2.0 * c.wheel_mass;
    const double db = std::sin(radians(c.SLOPE_DEADBAND_DEG));
    const double slope_ff = std::pow(s.slope_sin, 5.0) / (std::pow(s.slope_sin, 4.0) + std::pow(db, 4.0));
    const double lean_sin = c.SLOPE_LEAN_GAIN * M * R * slope_ff / (c.body_mass * c.com_height);
    const double slope_lean = std::asin(clip(lean_sin, -0.5, 0.5));
    const double slope_torque = M * c.gravity * R * slope_ff;
    const double balance_pitch = s.pitch_trim + posture_trim(fold) + slope_lean;
    update_lean_estimate(pitch - balance_pitch, x_vel, pitch_rate, dt);
    const double balance_rate = s.jump_phase == kIdle ? posture_trim_slope(fold) * leg_fold_rate : 0.0;
    const double pitch_err = pitch - balance_pitch;
    update_fall(pitch_err, roll, pitch_rate, x_pos, yaw, dt);
    double pos_err = clip(x_pos - s.target_x, -0.30, 0.30);
    const double pos_weight = std::fabs(s.cmd_vel) > 1e-3 ? 0.25 : 1.0;
    const double e0 = pos_weight * pos_err, e1 = x_vel - s.target_vel, e2 = pitch_err, e3 = pitch_rate - balance_rate;
    double total_balance_torque =
        (-c.k_lqr_0 * e0 + -c.k_lqr_1 * e1 + -c.k_lqr_2 * e2 + -c.k_lqr_3 * e3) + slope_torque;

    // Pitch trim learners
    const bool parked = std::fabs(s.cmd_vel) < 1e-3 && std::fabs(s.target_vel) < 0.05 && !s.settling;
    s.run_time += dt;
    const double settled_sin = std::sin(radians(c.SLOPE_SETTLED_DEG));
    const bool slope_settled = std::fabs(s.slope_gap) < settled_sin;
    if (parked && !slope_settled && !s.has_slope_before) {
        s.has_slope_before = true;
        s.slope_before = s.slope_sin;
    } else if (slope_settled && s.has_slope_before) {
        if (parked && std::fabs(s.slope_sin - s.slope_before) > settled_sin) {
            s.target_x = x_pos;
            pos_err = 0.0;
            s.lean_window = c.LEAN_WINDOW;
        }
        s.has_slope_before = false;
    }
    s.lean_window = std::max(0.0, s.lean_window - dt);
    if (s.fallen) {
        // lying on the floor says nothing about the balance lean
    } else if (s.lean_window > 0.0 && parked && slope_settled && s.jump_phase == kIdle) {
        s.pitch_trim = clip(s.pitch_trim + c.LEAN_GAIN * s.lean_est * dt, -c.TRIM_LIMIT, c.TRIM_LIMIT);
    } else if (parked && slope_settled && s.legs_moved_age > c.TRIM_HOLDOFF && s.run_time > c.TRIM_HOLDOFF &&
               s.bump_age > c.TRIM_HOLDOFF) {
        const double trim_err = pos_err + c.TRIM_VEL_WEIGHT * (x_vel - s.target_vel);
        const double gain = s.trim_adapt_gain + c.TRIM_GAIN_PER_M * std::fabs(pos_err);
        s.pitch_trim = clip(s.pitch_trim - gain * trim_err * dt, -c.TRIM_LIMIT, c.TRIM_LIMIT);
    } else if (parked && slope_settled && s.jump_phase == kIdle && s.run_time > c.TRIM_HOLDOFF &&
               c.LEAN_SETTLE < s.legs_moved_age && s.legs_moved_age <= c.TRIM_HOLDOFF) {
        s.pitch_trim = clip(s.pitch_trim + c.LEAN_GAIN * s.lean_est * dt, -c.TRIM_LIMIT, c.TRIM_LIMIT);
    } else if (s.jump_phase == kIdle && std::fabs(s.cmd_vel) > 1e-3 && std::fabs(vel_diff) < 1e-3 && slope_settled) {
        s.pitch_trim = clip(s.pitch_trim - c.TRIM_GAIN_CRUISE * (x_vel - s.target_vel) * dt, -c.TRIM_LIMIT,
                            c.TRIM_LIMIT);
    }

    if (airborne) {
        total_balance_torque = 6.0 * pitch_err + 0.6 * pitch_rate;
    } else if (s.jump_phase == kThrust && c.thrust_wheel_mode != 0.0) {
        total_balance_torque = c.thrust_wheel_mode == 1.0 ? (6.0 * pitch_err + 0.6 * pitch_rate) : 0.0;
    }

    // 2. Differential yaw steering torque, rate-limited heading reference
    if (!s.has_yaw_ref) {
        s.has_yaw_ref = true;
        s.yaw_ref = s.target_yaw;
    }
    const double yaw_todo = wrap(s.target_yaw - s.yaw_ref);
    const double turn_rate_max = std::min(c.TURN_RATE_MAX, c.TURN_ACCEL_MAX / std::max(std::fabs(x_vel), 1e-3));
    const double turn_rate_ref = clip(yaw_todo / dt, -turn_rate_max, turn_rate_max);
    s.yaw_ref += turn_rate_ref * dt;
    const double yaw_err = wrap(yaw - s.yaw_ref);
    const double yaw_torque = airborne ? 0.0 : (-1.45 * yaw_err - 0.30 * (yaw_rate - turn_rate_ref));

    // Wheel friction feedforward and virtual wheel damper
    double wheel_ff = 0.0;
    if (!airborne && std::fabs(s.target_vel) > 0.01) {
        wheel_ff = c.wheel_viscous * s.target_vel / R + c.wheel_coulomb * std::copysign(1.0, s.target_vel);
    }
    const double omega_ref = s.target_vel / R;
    const double damp_left = -c.WHEEL_DAMPING * (in.left_wheel_vel - omega_ref);
    const double damp_right = -c.WHEEL_DAMPING * (in.right_wheel_vel - omega_ref);

    double tau_left = clip(0.5 * total_balance_torque - yaw_torque + wheel_ff + damp_left, -c.max_wheel_torque,
                           c.max_wheel_torque);
    double tau_right = clip(0.5 * total_balance_torque + yaw_torque + wheel_ff + damp_right, -c.max_wheel_torque,
                            c.max_wheel_torque);
    if (s.fallen) tau_left = tau_right = 0.0;

    // 3. Legs: roll leveling, turn banking, crouch height (knee = -2 * hip keeps the wheel under the hip)
    const double turn_bank = clip(-std::atan2(x_vel * turn_rate_ref, c.gravity), -0.12, 0.12);
    const double roll_comp = clip(-0.55 * (roll - turn_bank) - 0.10 * roll_rate, -0.30, 0.30);
    const double hip_pitch_cmd = s.nominal_hip_pitch + leg_delta;
    const double knee_cmd = s.nominal_knee_pitch - 2.0 * leg_delta;
    s.commanded_leg_fold = hip_pitch_cmd + knee_cmd;

    // 4. Head: horizon-stabilized gimbal looking into turns, or a commanded pose
    double neck_pitch_cmd, head_pitch_cmd, head_yaw_cmd, head_roll_cmd;
    if (s.has_head_pose) {
        neck_pitch_cmd = s.head_pose_0;
        head_pitch_cmd = s.head_pose_1;
        head_yaw_cmd = s.head_pose_2;
        head_roll_cmd = s.head_pose_3;
    } else {
        neck_pitch_cmd = 0.15 - 0.5 * pitch;
        head_pitch_cmd = -0.15 - 0.5 * pitch;
        const double look = wrap(yaw - s.target_yaw);
        head_yaw_cmd = -0.6 * look + s.head_yaw_offset;
        head_roll_cmd = -roll;
    }
    clip_neck_envelope(neck_pitch_cmd, head_pitch_cmd);
    head_yaw_cmd = clip(head_yaw_cmd, -1.5, 1.5);
    head_roll_cmd = clip(head_roll_cmd, -0.45, 0.45);

    const double ctrl[kNumCtrl] = {
        roll_comp,      hip_pitch_cmd,  knee_cmd,     roll_comp,    hip_pitch_cmd,  knee_cmd,      // legs L, R
        neck_pitch_cmd, head_pitch_cmd, head_yaw_cmd, head_roll_cmd,                                // neck & head
        tau_left,       tau_right,                                                                  // wheels
        s.antenna_cmd,  s.antenna_cmd,                                                              // antennas
    };
    for (int i = 0; i < kNumCtrl; ++i) out.ctrl[i] = ctrl[i];

    s.last_wheel_torque =
        deliverable_wheel_torque(tau_left, in.left_wheel_vel) + deliverable_wheel_torque(tau_right, in.right_wheel_vel);

    out.roll = roll;
    out.pitch = pitch;
    out.yaw = yaw;
    out.x_pos = x_pos;
    out.x_vel = x_vel;
    out.tau_left = tau_left;
    out.tau_right = tau_right;
    out.balance_pitch = balance_pitch;
}

void Controller::clip_neck_envelope(double& neck, double& head) const {
    const Config& c = cfg_;
    neck = clip(neck, c.NECK_RANGE_LO, c.NECK_RANGE_HI);
    const double lo = c.NECK_SUM_RANGE_LO, hi = c.NECK_SUM_RANGE_HI;
    head = clip(head, std::max(c.HEAD_PITCH_RANGE_LO, lo - neck), std::min(c.HEAD_PITCH_RANGE_HI, hi - neck));
    neck = clip(neck, lo - head, hi - head);
}

double Controller::posture_trim(double fold) const { return cfg_.posture_a * fold + cfg_.posture_b * fold * fold; }

double Controller::posture_trim_slope(double fold) const { return cfg_.posture_a + 2.0 * cfg_.posture_b * fold; }

double Controller::leg_servo_kd() const {
    return (state.jump_phase == kCrouch || state.jump_phase == kThrust) ? cfg_.LEG_KD_JUMP : cfg_.LEG_KD_NOMINAL;
}

double Controller::speed_cap() const {
    const Config& c = cfg_;
    if (c.has_wheel_curve == 0.0) return c.MAX_SPEED;
    const double volts = state.has_filtered_voltage ? state.filtered_voltage : 24.0;
    const double rpm_per_rad_s = 60.0 / (2.0 * kPi);
    const double free = c.wheel_curve_no_load_rpm * volts / 24.0 / c.wheel_curve_rpm_per_nm - c.wheel_coulomb -
                        c.SPEED_HEADROOM;
    const double omega = free / (rpm_per_rad_s / c.wheel_curve_rpm_per_nm + c.wheel_viscous);
    return clip(omega * c.wheel_radius, 0.0, c.MAX_SPEED);
}

void Controller::update_slope(double x_vel, double pitch, double pitch_rate, double left_wheel_vel,
                              double right_wheel_vel, bool frozen, double dt) {
    State& s = state;
    const Config& c = cfg_;
    const double R = c.wheel_radius;
    const double M = c.body_mass + 2.0 * c.wheel_mass;
    const double p =
        (M + 2.0 * c.wheel_inertia / std::pow(R, 2.0)) * x_vel + c.body_mass * c.com_height * std::cos(pitch) * pitch_rate;
    const double k = 1.0 / c.SLOPE_OBS_TAU;
    if (!s.has_slope_obs || frozen) {
        s.has_slope_obs = true;
        s.slope_obs = p - s.slope_force / k;
        return;
    }
    const double friction = (c.wheel_viscous * left_wheel_vel + c.wheel_coulomb * std::tanh(left_wheel_vel / 0.5)) +
                            (c.wheel_viscous * right_wheel_vel + c.wheel_coulomb * std::tanh(right_wheel_vel / 0.5));
    s.slope_obs += ((s.last_wheel_torque - friction) / R + s.slope_force) * dt;
    s.slope_force = k * (p - s.slope_obs);
    const double s_max = std::sin(radians(c.SLOPE_MAX_DEG));
    double est = clip(-s.slope_force / (M * c.gravity), -s_max, s_max);
    s.slope_gap = est - s.slope_sin;
    const bool stuck = std::min(std::fabs(left_wheel_vel), std::fabs(right_wheel_vel)) < c.SLOPE_STICK_SPEED;
    const double band = stuck ? 2.0 * c.wheel_coulomb / R / (M * c.gravity) : 0.0;
    est = s.slope_sin + std::copysign(std::max(std::fabs(est - s.slope_sin) - band, 0.0), est - s.slope_sin);
    s.slope_sin += (est - s.slope_sin) * dt / c.SLOPE_FF_TAU;
}

double Controller::deliverable_wheel_torque(double tau, double omega) const {
    const Config& c = cfg_;
    if (c.has_wheel_curve == 0.0) return tau;
    const double stall = c.wheel_curve_stall;
    const double volts = state.has_filtered_voltage ? state.filtered_voltage : 24.0;
    const double rpm = std::fabs(omega) * 60.0 / (2.0 * kPi);
    const double limit = clip((c.wheel_curve_no_load_rpm * volts / 24.0 - rpm) / c.wheel_curve_rpm_per_nm, 0.05, stall);
    if (tau * omega > 0.0) return std::copysign(std::min(std::fabs(tau), limit), tau);
    return clip(tau, -stall, stall);
}

void Controller::reset_slope(double slope_sin) {
    State& s = state;
    s.slope_sin = slope_sin;
    s.slope_force = -slope_sin * (cfg_.body_mass + 2.0 * cfg_.wheel_mass) * cfg_.gravity;
    s.has_slope_obs = false;
    s.slope_gap = 0.0;
    s.has_slope_before = false;
}

void Controller::update_battery(double dt) {
    State& s = state;
    const Config& c = cfg_;
    if (!s.has_bus_voltage) return;
    if (!s.has_filtered_voltage) {
        s.has_filtered_voltage = true;
        s.filtered_voltage = s.bus_voltage;
    }
    s.filtered_voltage += (s.bus_voltage - s.filtered_voltage) * dt / c.VOLTAGE_FILTER_S;
    if (!s.low_battery && s.filtered_voltage < c.LOW_BATTERY_V) {
        s.low_battery = true;
        s.braking = std::fabs(s.target_vel) > 1e-6;
        log("[Battery] %.1f V: parking", s.filtered_voltage);
    } else if (s.low_battery && s.filtered_voltage > c.LOW_BATTERY_CLEAR_V) {
        s.low_battery = false;
        log("[Battery] %.1f V: ok", s.filtered_voltage);
    }
    if (s.low_battery) {
        s.cmd_vel = 0.0;
        s.jump_requested = false;
        s.crouch_delta = std::min(c.MAX_CROUCH, s.crouch_delta + 0.10 * dt);
    }
}

void Controller::update_fall(double pitch_err, double roll, double pitch_rate, double x_pos, double yaw, double dt) {
    State& s = state;
    const Config& c = cfg_;
    if (!s.fallen) {
        if (std::fabs(pitch_err) > c.FALL_PITCH || std::fabs(roll) > c.FALL_ROLL) {
            s.fallen = true;
            s.upright_time = 0.0;
            s.cmd_vel = s.target_vel = 0.0;
            s.braking = s.settling = s.jump_requested = false;
            log("[Fall] pitch %+.0f deg past the balance lean, roll %+.0f deg: wheels off", pitch_err * (180.0 / kPi),
                roll * (180.0 / kPi));
        }
        return;
    }
    const bool upright =
        std::fabs(pitch_err) < c.UPRIGHT_PITCH && std::fabs(roll) < c.UPRIGHT_PITCH && std::fabs(pitch_rate) < c.UPRIGHT_RATE;
    s.upright_time = upright ? s.upright_time + dt : 0.0;
    s.cmd_vel = s.target_vel = 0.0;
    s.target_x = x_pos;
    s.target_yaw = s.yaw_ref = yaw;
    s.has_yaw_ref = true;
    if (s.upright_time >= c.UPRIGHT_TIME) {
        s.fallen = false;
        s.run_time = 0.0;
        reset_slope(0.0);
        log("[Fall] Upright again: balancing");
    }
}

void Controller::update_lean_estimate(double pitch_err, double x_vel, double pitch_rate, double dt) {
    State& s = state;
    const Config& c = cfg_;
    if (!s.has_lean_filter) {
        s.has_lean_filter = true;
        s.lean_filter_0 = 0.0;
        s.lean_filter_1 = x_vel;
        s.lean_filter_2 = pitch_rate;
    }
    const double k = dt / c.LEAN_FILTER_S;
    const double x_accel = (x_vel - s.lean_filter_1) / c.LEAN_FILTER_S;
    const double pitch_accel = (pitch_rate - s.lean_filter_2) / c.LEAN_FILTER_S;
    s.lean_filter_0 += (pitch_err - s.lean_filter_0) * k;
    s.lean_filter_1 += (x_vel - s.lean_filter_1) * k;
    s.lean_filter_2 += (pitch_rate - s.lean_filter_2) * k;
    s.lean_est = s.lean_filter_0 - c.lean_c_x * x_accel - c.lean_c_p * pitch_accel;
}

void Controller::update_gyro_bias(const double gyro[3], bool parked, double dt) {
    State& s = state;
    const Config& c = cfg_;
    const double peak = std::max(std::max(std::fabs(gyro[0]), std::fabs(gyro[1])), std::fabs(gyro[2]));
    if (!parked || peak > c.GYRO_STILL_RATE) return;
    s.gyro_samples += 1.0;
    const double k = std::max(1.0 / (s.gyro_samples + c.GYRO_BIAS_PRIOR), dt / c.GYRO_BIAS_TAU);
    s.gyro_bias_0 = s.gyro_bias_0 + k * (gyro[0] - s.gyro_bias_0);
    s.gyro_bias_1 = s.gyro_bias_1 + k * (gyro[1] - s.gyro_bias_1);
    s.gyro_bias_2 = s.gyro_bias_2 + k * (gyro[2] - s.gyro_bias_2);
}

void Controller::detect_drop(bool has_accel, double accel_mag, double dt) {
    State& s = state;
    const Config& c = cfg_;
    if (!has_accel || s.jump_phase != kIdle || s.legs_moved_age < c.DROP_LEGS_SETTLE) {
        s.freefall_time = 0.0;
        return;
    }
    s.freefall_time = accel_mag < c.FREEFALL_ACCEL ? s.freefall_time + dt : 0.0;
    if (s.freefall_time >= c.FREEFALL_TIME) {
        s.jump_phase = kDrop;
        s.jump_timer = s.freefall_time = s.supported_time = 0.0;
        s.landing_from_drop = true;
        log("[Drop] Free fall, bracing");
    }
}

void Controller::detect_bump(bool has_accel, double accel_mag, double pitch_rate, double x_pos, double x_vel,
                             double dt) {
    State& s = state;
    const Config& c = cfg_;
    s.bump_age += dt;
    s.bump_armed = std::max(0.0, s.bump_armed - dt);
    const double v = s.target_vel;
    if (!has_accel || s.jump_phase != kIdle || s.legs_moved_age < c.DROP_LEGS_SETTLE || std::fabs(v) < c.BUMP_MIN_SPEED) {
        s.bump_armed = 0.0;
        return;
    }
    if (accel_mag > c.BUMP_ACCEL && std::fabs(x_vel - v) < c.BUMP_SPEED_ERR) {
        if (s.bump_armed <= 0.0) s.slope_at_arm = s.slope_sin;
        s.bump_armed = std::max(s.bump_armed, c.BUMP_CONFIRM);
    }
    if (s.bump_armed <= 0.0 || std::copysign(1.0, v) * pitch_rate > -c.BUMP_PITCH_RATE) return;
    s.bump_armed = 0.0;
    s.cmd_vel = s.target_vel = 0.0;
    s.braking = s.settling = false;
    s.target_x = x_pos - std::copysign(c.BUMP_BACKOFF, v);
    s.bump_age = 0.0;
    reset_slope(s.slope_at_arm);
    log("[Bump] %.0f m/s^2 at %+.2f m/s: stopping, backing off %.2f m", accel_mag, v, c.BUMP_BACKOFF);
}

double Controller::update_jump(double x_pos, double x_vel, bool has_accel, double accel_mag, double dt,
                               bool& airborne) {
    State& s = state;
    const Config& c = cfg_;
    airborne = false;
    if (s.jump_requested && s.jump_phase == kIdle) {
        s.jump_phase = kCrouch;
        s.jump_timer = 0.0;
        s.jump_requested = false;
        s.landing_from_drop = false;
    }
    if (s.jump_phase == kIdle) return s.crouch_delta;

    s.jump_timer += dt;
    const double t = s.jump_timer;
    switch (s.jump_phase) {
        case kDrop: {
            const bool supported = has_accel && accel_mag > c.SUPPORTED_ACCEL;
            s.supported_time = supported ? s.supported_time + dt : 0.0;
            if ((has_accel && accel_mag > c.TOUCHDOWN_ACCEL) || s.supported_time >= c.SUPPORTED_TIME || t >= 1.0) {
                s.supported_time = 0.0;
                s.jump_phase = kLand;
                s.jump_timer = 0.0;
                s.target_x = x_pos;
                return c.DROP_DELTA;
            }
            airborne = true;
            return c.DROP_DELTA;
        }
        case kCrouch: {
            const double a = 0.5 - 0.5 * std::cos(kPi * std::min(t / (0.75 * c.JUMP_CROUCH_TIME), 1.0));
            const double leg_delta = s.crouch_delta + a * (c.JUMP_CROUCH_DELTA - s.crouch_delta);
            if (t >= c.JUMP_CROUCH_TIME) {
                s.jump_phase = kThrust;
                s.jump_timer = 0.0;
                log("[Jump] Thrust!");
            }
            return leg_delta;
        }
        case kThrust:
            if (t >= c.jump_thrust_time) {
                s.jump_phase = kFlight;
                s.jump_timer = 0.0;
                s.jump_takeoff_vel = x_vel;
            }
            return c.JUMP_EXTEND_DELTA;
        case kFlight:
            if (t >= c.JUMP_FLIGHT_TIME) {
                s.jump_phase = kLand;
                s.jump_timer = 0.0;
                s.target_x = x_pos;
            }
            airborne = true;
            return c.JUMP_LAND_DELTA;
        default: {   // kLand: blend from the landing crouch back to the driver's crouch setting
            const double a = std::min(t / c.JUMP_LAND_TIME, 1.0);
            if (t >= c.JUMP_LAND_TIME) {
                s.jump_phase = kIdle;
                s.jump_timer = 0.0;
                log("[Jump] Landed.");
            }
            const double start = s.landing_from_drop ? c.DROP_DELTA : c.JUMP_LAND_DELTA;
            return start + a * (s.crouch_delta - start);
        }
    }
}

}  // namespace duck
