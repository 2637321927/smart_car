#include "control_profile.hpp"

#include "drive_by.hpp"
#include "front_ui.hpp"
#include "gyro_yaw_rate_control.hpp"
#include "hardware_test.hpp"
#include "lq_all_demo.hpp"

#include <algorithm>
#include <atomic>

namespace {

ControlProfileValues make_stable_defaults()
{
    return {
        35.0f,
        454.0f,
        14.0f,
        0.0f,
        0.128f,
        1.55f,
        0.25f,
        30,
        4.5f,
        3.3f,
        0.40f,
        0.0f,
        360.0f,
        20.0f,
        500.0f,
        15.0f,
        25.0f,
    };
}

ControlProfileValues make_pro_defaults()
{
    ControlProfileValues values = make_stable_defaults();
    values.target_speed_rps = 45.0f;
    values.direction_p = 0.231f;
    values.direction_d = 3.5f;
    values.aim_m = 0.40f;
    values.speed_slow_ratio = 40;
    values.rescue_target_dps = 650.0f;
    values.rescue_base_rps = 15.0f;
    values.rescue_turn_max_rps = 30.0f;
    return values;
}

std::atomic<int> active_mode(CONTROL_PROFILE_STABLE);

ControlProfile &active_profile()
{
    return active_mode.load(std::memory_order_acquire) == CONTROL_PROFILE_PRO
        ? pro_profile
        : stable_profile;
}

void apply_values(const ControlProfileValues &values)
{
    P = values.speed_p;
    I = values.speed_i;
    D = values.speed_d;
    dir_P = values.direction_p;
    dir_D = values.direction_d;
    AIM = values.aim_m;
    spd_slow_ratio = values.speed_slow_ratio;
    gyro_outer_kp = values.gyro_outer_p;
    gyro_outer_kd = values.gyro_outer_d;
    gyro_inner_kp = values.gyro_inner_p;
    gyro_inner_ki = values.gyro_inner_i;
    gyro_target_max_dps = values.gyro_target_max_dps;
    gyro_turn_max_rps = values.gyro_turn_max_rps;
    vision_y_guard_target_dps = values.rescue_target_dps;
    vision_y_guard_base_rps = values.rescue_base_rps;
    vision_y_guard_turn_max_rps = values.rescue_turn_max_rps;
    drive_by_normal_speed_rps = values.target_speed_rps;
}

bool switch_is_safe()
{
    return !front_ui_is_running() &&
        !drive_by_is_busy() &&
        !hardware_test_is_enabled() &&
        !drive_by_heading_hold_is_enabled() &&
        !front_ui_remote_is_active();
}

} // namespace

ControlProfile stable_profile(make_stable_defaults());
ControlProfile pro_profile(make_pro_defaults());

ControlProfile::ControlProfile(const ControlProfileValues &defaults)
    : defaults_(defaults), session_(defaults)
{
}

const ControlProfileValues &ControlProfile::defaults() const
{
    return defaults_;
}

const ControlProfileValues &ControlProfile::session() const
{
    return session_;
}

void ControlProfile::reset_session()
{
    session_ = defaults_;
}

void ControlProfile::set_session(const ControlProfileValues &values)
{
    session_ = values;
}

void control_profile_init()
{
    stable_profile.reset_session();
    pro_profile.reset_session();
    active_mode.store(CONTROL_PROFILE_STABLE, std::memory_order_release);
    apply_values(stable_profile.session());
}

bool control_profile_switch(ControlProfileMode mode)
{
    if (mode != CONTROL_PROFILE_STABLE && mode != CONTROL_PROFILE_PRO) {
        return false;
    }
    if (mode == control_profile_mode()) {
        return true;
    }
    if (!switch_is_safe()) {
        return false;
    }

    control_profile_capture_active();
    front_ui_stop();
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
    vision_y_guard_active = 0;
    vision_y_guard_turn_sign = 0;
    vision_y_guard_aim_dy_px = 0.0f;
    motor_speed_pid_reset();
    gyro_yaw_rate_control_reset();
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);

    active_mode.store(mode, std::memory_order_release);
    apply_values(active_profile().session());
    return true;
}

ControlProfileMode control_profile_mode()
{
    return static_cast<ControlProfileMode>(
        active_mode.load(std::memory_order_acquire));
}

bool control_profile_is_pro()
{
    return control_profile_mode() == CONTROL_PROFILE_PRO;
}

float control_profile_target_speed_rps()
{
    return active_profile().session().target_speed_rps;
}

float control_profile_set_target_speed_rps(float speed_rps)
{
    if (control_profile_is_pro()) {
        speed_rps = std::max(40.0f, std::min(55.0f, speed_rps));
    } else {
        speed_rps = std::max(0.0f, std::min(200.0f, speed_rps));
    }

    ControlProfileValues values = active_profile().session();
    values.target_speed_rps = speed_rps;
    active_profile().set_session(values);
    drive_by_normal_speed_rps = speed_rps;
    return speed_rps;
}

void control_profile_capture_active()
{
    ControlProfileValues values = active_profile().session();
    values.speed_p = P;
    values.speed_i = I;
    values.speed_d = D;
    values.direction_p = dir_P;
    values.direction_d = dir_D;
    values.aim_m = AIM;
    values.speed_slow_ratio = spd_slow_ratio;
    values.gyro_outer_p = gyro_outer_kp;
    values.gyro_outer_d = gyro_outer_kd;
    values.gyro_inner_p = gyro_inner_kp;
    values.gyro_inner_i = gyro_inner_ki;
    values.gyro_target_max_dps = gyro_target_max_dps;
    values.gyro_turn_max_rps = gyro_turn_max_rps;
    values.rescue_target_dps = vision_y_guard_target_dps;
    values.rescue_base_rps = vision_y_guard_base_rps;
    values.rescue_turn_max_rps = vision_y_guard_turn_max_rps;
    active_profile().set_session(values);
}
