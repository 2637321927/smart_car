#include "hardware_test.hpp"

#include "drive_by.hpp"
#include "front_ui.hpp"
#include "gyro_yaw_rate_control.hpp"
#include "lq_all_demo.hpp"

namespace {

constexpr int kHardwareTestMaxPwm = 5000;

// 工程当前的定时器和主循环普遍通过volatile共享简单状态。这里保持同样
// 的兼容方式，不引入std::atomic，也不让关闭状态承担互斥锁开销。
volatile int g_hardware_test_enabled = 0;
volatile int g_hardware_test_pwm = 0;

int clamp_hardware_test_pwm(int pwm)
{
    if (pwm < 0) return 0;
    if (pwm > kHardwareTestMaxPwm) return kHardwareTestMaxPwm;
    return pwm;
}

void reset_motor_control_state()
{
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
    gyro_yaw_rate_control_reset();
    motor_speed_pid_reset();
}

} // namespace

bool hardware_test_set_enabled(bool enabled)
{
    if (enabled) {
        if (g_hardware_test_enabled) {
            return true;
        }

        // 测试模式只能从完全停车的空闲状态进入。这里拒绝而不是强行打断
        // 其它功能，防止误触开关时改变正常行车、刹车测试或正在执行的绕行动作。
        if (front_ui_is_running() || drive_by_is_busy() ||
            drive_by_brake_test_is_enabled() ||
            front_ui_remote_is_active() ||
            drive_by_heading_hold_is_enabled()) {
            return false;
        }

        // 开启瞬间始终从0开始，绝不复用关闭期间误触滑块留下的数值。
        g_hardware_test_pwm = 0;
        reset_motor_control_state();
        motor_speed_force_pwm(0, 0);
        g_hardware_test_enabled = 1;
        return true;
    }

    // 关闭状态收到重复的#hwTest=0时必须完全无副作用，尤其不能在正常
    // 行车中清PWM或复位PID，这就是“关闭时不影响正常行驶”的关键保证。
    if (!g_hardware_test_enabled) {
        return true;
    }

    g_hardware_test_pwm = 0;
    g_hardware_test_enabled = 0;
    motor_speed_force_pwm(0, 0);
    reset_motor_control_state();
    return true;
}

bool hardware_test_is_enabled()
{
    return g_hardware_test_enabled != 0;
}

void hardware_test_set_pwm(int pwm)
{
    g_hardware_test_pwm = clamp_hardware_test_pwm(pwm);
}

int hardware_test_get_pwm()
{
    return g_hardware_test_pwm;
}

bool hardware_test_update()
{
    // 关闭时只执行一次快速判断，不访问PWM、不复位PID，也不修改任何
    // 正常控制变量，因此正常巡线链路保持原样。
    if (!g_hardware_test_enabled) {
        return false;
    }

    const int requested_pwm = clamp_hardware_test_pwm(g_hardware_test_pwm);
    if (!g_hardware_test_enabled) {
        return false;
    }

    // motor_speed_force_pwm统一处理电机方向脚：正数保证PWM1正向输出，
    // PWM2固定写0表示失能。该路径仍受本模块更严格的5000上限保护。
    motor_speed_force_pwm(requested_pwm, 0);

    // 若关闭命令恰好与本周期并发，关闭方和这里都会再写一次0，避免旧的
    // requested_pwm在关闭之后成为最后一次硬件输出。
    if (!g_hardware_test_enabled) {
        motor_speed_force_pwm(0, 0);
    }
    return true;
}
