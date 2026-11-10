// C API over duck::Controller for the sim (duck/cpp_controller.py loads it with ctypes). Not built into the
// firmware. Config and state fields are looked up by name, from the same X-macro lists the structs are made of, so
// the Python side never hard-codes a struct layout.
#include <cstddef>

#include "duck_controller.hpp"

namespace {

enum Kind : int { kDouble = 0, kBool = 1, kPhase = 2 };

struct Field {
    const char* name;
    int kind;
    std::size_t offset;
};

const char* const kConfigNames[] = {
#define DUCK_CONFIG_NAME(name) #name,
    DUCK_CONFIG_FIELDS(DUCK_CONFIG_NAME)
#undef DUCK_CONFIG_NAME
};
constexpr int kNumConfig = sizeof kConfigNames / sizeof kConfigNames[0];
static_assert(sizeof(duck::Config) == kNumConfig * sizeof(double), "Config must be all doubles");

const Field kStateFields[] = {
#define DUCK_FIELD_D(name) {#name, kDouble, offsetof(duck::State, name)},
#define DUCK_FIELD_B(name) {#name, kBool, offsetof(duck::State, name)},
#define DUCK_FIELD_P(name) {#name, kPhase, offsetof(duck::State, name)},
    DUCK_STATE_FIELDS(DUCK_FIELD_D, DUCK_FIELD_B, DUCK_FIELD_P)
#undef DUCK_FIELD_D
#undef DUCK_FIELD_B
#undef DUCK_FIELD_P
};
constexpr int kNumState = sizeof kStateFields / sizeof kStateFields[0];

}  // namespace

extern "C" {

int duck_config_count() { return kNumConfig; }
const char* duck_config_name(int i) { return (i >= 0 && i < kNumConfig) ? kConfigNames[i] : nullptr; }

int duck_state_count() { return kNumState; }
const char* duck_state_name(int i) { return (i >= 0 && i < kNumState) ? kStateFields[i].name : nullptr; }
int duck_state_kind(int i) { return kStateFields[i].kind; }

// `values`: duck_config_count() doubles in duck_config_name order.
void* duck_create(const double* values, duck::LogFn log) {
    duck::Config cfg;
    double* dst = reinterpret_cast<double*>(&cfg);
    for (int i = 0; i < kNumConfig; ++i) dst[i] = values[i];
    return new duck::Controller(cfg, log);
}

void duck_destroy(void* h) { delete static_cast<duck::Controller*>(h); }

double duck_state_get(void* h, int i) {
    const char* p = reinterpret_cast<const char*>(&static_cast<duck::Controller*>(h)->state) + kStateFields[i].offset;
    switch (kStateFields[i].kind) {
        case kBool: return *reinterpret_cast<const bool*>(p) ? 1.0 : 0.0;
        case kPhase: return static_cast<double>(*reinterpret_cast<const duck::JumpPhase*>(p));
        default: return *reinterpret_cast<const double*>(p);
    }
}

void duck_state_set(void* h, int i, double v) {
    char* p = reinterpret_cast<char*>(&static_cast<duck::Controller*>(h)->state) + kStateFields[i].offset;
    switch (kStateFields[i].kind) {
        case kBool: *reinterpret_cast<bool*>(p) = v != 0.0; break;
        case kPhase: *reinterpret_cast<duck::JumpPhase*>(p) = static_cast<duck::JumpPhase>(static_cast<int>(v)); break;
        default: *reinterpret_cast<double*>(p) = v; break;
    }
}

// sensors: quat(4), gyro(3), left/right wheel pos, left/right wheel vel, leg_fold, leg_fold_rate, accel_mag, dt
// (15 doubles); NaN leg_fold / accel_mag = not measured. out: ctrl(14), then roll, pitch, yaw, x_pos, x_vel,
// tau_left, tau_right, balance_pitch (22 doubles).
void duck_compute(void* h, const double* sensors, double* out) {
    duck::Inputs in;
    for (int i = 0; i < 4; ++i) in.quat_wxyz[i] = sensors[i];
    for (int i = 0; i < 3; ++i) in.gyro_xyz[i] = sensors[4 + i];
    in.left_wheel_pos = sensors[7];
    in.right_wheel_pos = sensors[8];
    in.left_wheel_vel = sensors[9];
    in.right_wheel_vel = sensors[10];
    in.has_leg_fold = sensors[11] == sensors[11];
    in.leg_fold = sensors[11];
    in.leg_fold_rate = sensors[12];
    in.has_accel = sensors[13] == sensors[13];
    in.accel_mag = sensors[13];
    in.dt = sensors[14];
    duck::Outputs o;
    static_cast<duck::Controller*>(h)->compute(in, o);
    for (int i = 0; i < duck::kNumCtrl; ++i) out[i] = o.ctrl[i];
    const double tail[] = {o.roll, o.pitch, o.yaw, o.x_pos, o.x_vel, o.tau_left, o.tau_right, o.balance_pitch};
    for (int i = 0; i < 8; ++i) out[duck::kNumCtrl + i] = tail[i];
}

void duck_command(void* h, int cmd) { static_cast<duck::Controller*>(h)->command(static_cast<duck::Command>(cmd)); }
void duck_reset_state(void* h) { static_cast<duck::Controller*>(h)->reset_state(); }
double duck_speed_cap(void* h) { return static_cast<duck::Controller*>(h)->speed_cap(); }
double duck_leg_servo_kd(void* h) { return static_cast<duck::Controller*>(h)->leg_servo_kd(); }
double duck_deliverable_wheel_torque(void* h, double tau, double omega) {
    return static_cast<duck::Controller*>(h)->deliverable_wheel_torque(tau, omega);
}

}  // extern "C"
