// Super Duck balance, posture, jump and driving controller: a C++ port of duck/controller.py.
//
// Plain C++17 with no heap, exceptions or Arduino code, so the same file builds into the Teensy firmware and into
// a shared library the MuJoCo sim drives (duck/cpp_controller.py). test_cpp_controller.py runs both controllers
// side by side on the same sensor readings and checks every command matches.
//
// The Python controller is the reference: change it first, then mirror the change here. Every number the
// controller uses comes in through `Config`, which is filled from the Python class (in the sim) or from the header
// that tools/export_firmware_config.py generates (on the Teensy), so the two can't disagree on a constant.
#pragma once

#include <cstdint>

namespace duck {

// X(name) for every Config field, in order. All doubles.
#define DUCK_CONFIG_FIELDS(X)                                                                                        \
    /* ScaledWheeledDuckParams */                                                                                    \
    X(body_mass) X(wheel_mass) X(wheel_radius) X(com_height) X(gravity) X(max_wheel_torque) X(wheel_inertia)         \
    X(wheel_viscous) X(wheel_coulomb)                                                                                \
    /* RobotProfile (thrust_wheel_mode: 0 = lqr, 1 = attitude, 2 = zero) */                                          \
    X(initial_pitch_trim) X(posture_a) X(posture_b) X(jump_thrust_time) X(thrust_wheel_mode) X(has_wheel_curve)      \
    X(wheel_curve_stall) X(wheel_curve_no_load_rpm) X(wheel_curve_rpm_per_nm)                                        \
    /* Derived from the WIP model in Python (scipy's Riccati solver): LQR gains and the lean-estimate coefficients */ \
    X(k_lqr_0) X(k_lqr_1) X(k_lqr_2) X(k_lqr_3) X(lean_c_x) X(lean_c_p)                                              \
    /* ScaledWheeledDuckController class constants (tuple constants as NAME_LO / NAME_HI) */                         \
    X(MAX_CROUCH) X(RAMP_JERK) X(REANCHOR_DIST) X(MAX_SPEED) X(SPEED_HEADROOM) X(FALL_PITCH) X(FALL_ROLL)            \
    X(UPRIGHT_PITCH) X(UPRIGHT_RATE) X(UPRIGHT_TIME) X(WHEEL_DAMPING) X(TURN_RATE_MAX) X(TURN_ACCEL_MAX)             \
    X(LOW_BATTERY_V) X(LOW_BATTERY_CLEAR_V) X(VOLTAGE_FILTER_S) X(TRIM_LIMIT) X(TRIM_GAIN_PARKED)                    \
    X(TRIM_GAIN_PER_M) X(TRIM_VEL_WEIGHT) X(TRIM_GAIN_CRUISE) X(TRIM_HOLDOFF) X(LEAN_GAIN) X(LEAN_FILTER_S)          \
    X(LEAN_SETTLE) X(LEAN_WINDOW) X(SLOPE_OBS_TAU) X(SLOPE_FF_TAU) X(SLOPE_MAX_DEG) X(SLOPE_DEADBAND_DEG)            \
    X(SLOPE_LEAN_GAIN) X(SLOPE_STICK_SPEED) X(SLOPE_SETTLED_DEG) X(GYRO_BIAS_TAU) X(GYRO_BIAS_PRIOR)                 \
    X(GYRO_STILL_RATE) X(FREEFALL_ACCEL) X(FREEFALL_TIME) X(TOUCHDOWN_ACCEL) X(SUPPORTED_ACCEL) X(SUPPORTED_TIME)    \
    X(DROP_LEGS_SETTLE) X(DROP_DELTA) X(BUMP_ACCEL) X(BUMP_SPEED_ERR) X(BUMP_PITCH_RATE) X(BUMP_CONFIRM)             \
    X(BUMP_MIN_SPEED) X(BUMP_BACKOFF) X(BUMP_SLOPE_HOLD) X(NECK_RANGE_LO) X(NECK_RANGE_HI) X(HEAD_PITCH_RANGE_LO)    \
    X(HEAD_PITCH_RANGE_HI) X(NECK_SUM_RANGE_LO) X(NECK_SUM_RANGE_HI) X(LEG_KD_NOMINAL) X(LEG_KD_JUMP)                \
    X(JUMP_CROUCH_DELTA) X(JUMP_EXTEND_DELTA) X(JUMP_LAND_DELTA) X(JUMP_CROUCH_TIME) X(JUMP_FLIGHT_TIME)             \
    X(JUMP_LAND_TIME)

struct Config {
#define DUCK_DECLARE_CONFIG(name) double name;
    DUCK_CONFIG_FIELDS(DUCK_DECLARE_CONFIG)
#undef DUCK_DECLARE_CONFIG
};

enum JumpPhase : int32_t { kIdle = 0, kCrouch, kThrust, kFlight, kLand, kDrop };

// The controller's state, named as in the Python controller. A Python `None` is a `has_<name>` flag; Python arrays
// are <name>_0, _1, ... D = double, B = bool, P = JumpPhase.
#define DUCK_STATE_FIELDS(D, B, P)                                                                                   \
    D(lean_est) D(target_x) D(target_vel) D(cmd_vel) D(target_yaw) D(crouch_delta) D(head_yaw_offset)                \
    B(has_head_pose) D(head_pose_0) D(head_pose_1) D(head_pose_2) D(head_pose_3) D(antenna_cmd) B(projector_on)      \
    D(nominal_hip_pitch) D(nominal_knee_pitch) D(commanded_leg_fold) D(pitch_trim) D(trim_adapt_gain) D(run_time)    \
    D(legs_moved_age) D(last_crouch) B(has_yaw_ref) D(yaw_ref) D(gyro_bias_0) D(gyro_bias_1) D(gyro_bias_2)          \
    D(gyro_samples) B(has_bus_voltage) D(bus_voltage) B(has_filtered_voltage) D(filtered_voltage) B(low_battery)     \
    D(freefall_time) D(supported_time) B(landing_from_drop) D(bump_age) D(bump_armed) D(slope_at_arm)                \
    D(lean_window) B(reset_requested) B(fallen) D(upright_time) P(jump_phase) D(jump_timer) B(jump_requested)        \
    D(jump_takeoff_vel) B(braking) B(settling) B(has_slope_obs) D(slope_obs) D(slope_force) D(slope_sin)             \
    D(slope_gap) B(has_slope_before) D(slope_before) D(last_wheel_torque) B(has_lean_filter) D(lean_filter_0)        \
    D(lean_filter_1) D(lean_filter_2)

struct State {
#define DUCK_DECLARE_D(name) double name;
#define DUCK_DECLARE_B(name) bool name;
#define DUCK_DECLARE_P(name) JumpPhase name;
    DUCK_STATE_FIELDS(DUCK_DECLARE_D, DUCK_DECLARE_B, DUCK_DECLARE_P)
#undef DUCK_DECLARE_D
#undef DUCK_DECLARE_B
#undef DUCK_DECLARE_P
};

// One tick's sensor readings, as `compute_control` takes them.
struct Inputs {
    double quat_wxyz[4];
    double gyro_xyz[3];       // rad/s, body frame
    double left_wheel_pos, right_wheel_pos, left_wheel_vel, right_wheel_vel;   // rad, rad/s, relative to the foot
    bool has_leg_fold;        // false: use the commanded fold (no servo-lag compensation)
    double leg_fold, leg_fold_rate;   // hip_pitch + knee, mean of both legs
    bool has_accel;           // false: no drop / bump detection
    double accel_mag;         // m/s^2, IMU acceleration magnitude
    double dt;                // s
};

constexpr int kNumCtrl = 14;

// One tick's commands (the MuJoCo `data.ctrl` layout, see compute_control) and the telemetry the sim logs.
struct Outputs {
    double ctrl[kNumCtrl];
    double roll, pitch, yaw, x_pos, x_vel, tau_left, tau_right, balance_pitch;
};

// Driver commands (the sim's keys, the robot's gamepad).
enum Command : int32_t {
    kFaster = 0, kSlower, kSteerLeft, kSteerRight, kBrake, kCrouchDeeper, kStandTaller, kLookLeft, kLookRight,
    kJump, kAntennasUp, kAntennasDown, kToggleProjector, kReset,
};

using LogFn = void (*)(const char* message);

class Controller {
public:
    explicit Controller(const Config& config, LogFn log = nullptr);

    void compute(const Inputs& in, Outputs& out);
    void command(Command cmd);
    void reset_state();

    double speed_cap() const;
    double deliverable_wheel_torque(double tau, double omega) const;
    double leg_servo_kd() const;
    double posture_trim(double fold) const;
    double posture_trim_slope(double fold) const;
    void clip_neck_envelope(double& neck, double& head) const;

    const Config& config() const { return cfg_; }
    State state;   // public, like the Python controller's attributes: the sim and tests set and read it directly

private:
    void update_slope(double x_vel, double pitch, double pitch_rate, double left_wheel_vel, double right_wheel_vel,
                      bool frozen, double dt);
    void reset_slope(double slope_sin);
    void update_battery(double dt);
    void update_fall(double pitch_err, double roll, double pitch_rate, double x_pos, double yaw, double dt);
    void update_lean_estimate(double pitch_err, double x_vel, double pitch_rate, double dt);
    void update_gyro_bias(const double gyro[3], bool parked, double dt);
    void detect_drop(bool has_accel, double accel_mag, double dt);
    void detect_bump(bool has_accel, double accel_mag, double pitch_rate, double x_pos, double x_vel, double dt);
    double update_jump(double x_pos, double x_vel, bool has_accel, double accel_mag, double dt, bool& airborne);
    void log(const char* fmt, ...) const;

    Config cfg_;
    LogFn log_;
};

void quat_to_euler_zyx(const double q[4], double& roll, double& pitch, double& yaw);

}  // namespace duck
