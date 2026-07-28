#include "lq_all_demo.hpp"

#include <mutex>

volatile float P = 454.0f;
//volatile float I = 80.0f;old static premeter
volatile float I = 14.0f;
volatile float D = 0.0f;
volatile int current_pwm1 = 0;
volatile int current_pwm2 = 0;

namespace {

constexpr int kMaxForwardPwm = 8000;
constexpr int kMaxReversePwm = 4000;
constexpr int kMaxBrakeReversePwm = 9000;

// 增量式PID不仅依赖当前PWM，还依赖前两次误差。主动制动结束或停车时，
// 两类状态必须一起清零，否则下一次闭环会把制动前后的误差突变再次叠加到PWM。
float g_error_last1 = 0.0f;
float g_error_last2 = 0.0f;
float g_error_prev1 = 0.0f;
float g_error_prev2 = 0.0f;
std::mutex g_motor_speed_mutex;

int clamp_pwm(int pwm)
{
    if (pwm > kMaxForwardPwm) return kMaxForwardPwm;
    if (pwm < -kMaxReversePwm) return -kMaxReversePwm;
    return pwm;
}

int clamp_brake_pwm(int pwm)
{
    // This path is braking-only: forward PWM is rejected rather than clamped.
    if (pwm > 0) return 0;
    if (pwm < -kMaxBrakeReversePwm) return -kMaxBrakeReversePwm;
    return pwm;
}

void write_motor_pwm(int motor1_pwm, int motor2_pwm)
{
    current_pwm1 = motor1_pwm;
    current_pwm2 = motor2_pwm;

    if (current_pwm1 >= 0) {
        polar_pwm1.gpio_level_set(GPIO_HIGH);
        pwm1.atim_pwm_set_duty(current_pwm1);
    } else {
        polar_pwm1.gpio_level_set(GPIO_LOW);
        pwm1.atim_pwm_set_duty(-current_pwm1);
    }

    if (current_pwm2 >= 0) {
        polar_pwm2.gpio_level_set(GPIO_HIGH);
        pwm2.atim_pwm_set_duty(current_pwm2);
    } else {
        polar_pwm2.gpio_level_set(GPIO_LOW);
        pwm2.atim_pwm_set_duty(-current_pwm2);
    }
}

void apply_motor_pwm(int motor1_pwm, int motor2_pwm)
{
    write_motor_pwm(clamp_pwm(motor1_pwm), clamp_pwm(motor2_pwm));
}

void apply_motor_brake_pwm(int motor1_pwm, int motor2_pwm)
{
    write_motor_pwm(clamp_brake_pwm(motor1_pwm),
                    clamp_brake_pwm(motor2_pwm));
}

} // namespace

/************a********************************************************************
 * @brief   电机转速PD闭环控制S
 * @param   目标转速RPS，当前测速值RPS
 * @return  PWM调整量
 * @note    PD算法，带抗噪微分，固定周期调用
 ********************************************************************************/
void calculate_differential_for_motor(
    const float& speed_of_motor1, const float& speed_of_motor2,
    const float target_speed_of_motor1_RPS, const float target_speed_of_motor2_RPS,
    int& pwm1_plusduty, int& pwm2_plusduty)
{
    // 当前误差
    float error_current1 = target_speed_of_motor1_RPS - speed_of_motor1;
    float error_current2 = target_speed_of_motor2_RPS - speed_of_motor2;

    // 真正的增量式 PID
    float p_term1 = P * (error_current1 - g_error_last1);
    float p_term2 = P * (error_current2 - g_error_last2);
    const int max_P=2400;
  if(p_term1>max_P)  p_term1=max_P;
if(p_term1<-max_P) p_term1=-max_P;
if(p_term2>max_P)  p_term2=max_P;
if(p_term2<-max_P) p_term2=-max_P;
    
    float i_term1 = I * error_current1;
    float i_term2 = I * error_current2;

    float d_term1 = D * (error_current1 - 2.0f * g_error_last1 + g_error_prev1);
    float d_term2 = D * (error_current2 - 2.0f * g_error_last2 + g_error_prev2);

    const int max_D=200;

     if(d_term1>max_D)  d_term1=max_D;
if(d_term1<-max_D) d_term1=-max_D;
if(d_term2>max_D)  d_term2=max_D;
if(d_term2<-max_D) d_term2=-max_D;
    float delta_u1 = p_term1 + i_term1 + d_term1;
    float delta_u2 = p_term2 + i_term2 + d_term2;

    pwm1_plusduty = static_cast<int>(delta_u1);
    pwm2_plusduty = static_cast<int>(delta_u2);

    // 调试量
    P1_motor = p_term1;
    P2_motor = p_term2;



    I1_motor = i_term1;
    I2_motor = i_term2;

    // 更新历史误差
    g_error_prev1 = g_error_last1;
    g_error_prev2 = g_error_last2;
    g_error_last1 = error_current1;
    g_error_last2 = error_current2;
}

void motor_speed_pid_reset()
{
    std::lock_guard<std::mutex> lock(g_motor_speed_mutex);
    g_error_last1 = 0.0f;
    g_error_last2 = 0.0f;
    g_error_prev1 = 0.0f;
    g_error_prev2 = 0.0f;
    current_pwm1 = 0;
    current_pwm2 = 0;
    P1_motor = 0.0f;
    P2_motor = 0.0f;
    I1_motor = 0.0f;
    I2_motor = 0.0f;
}

void motor_speed_force_pwm(int motor1_pwm, int motor2_pwm)
{
    // 该接口只供3ms速度线程使用。统一在速度环模块内写方向和占空比，
    // 可避免绕行脚本绕过软件的正反向PWM限幅。
    std::lock_guard<std::mutex> lock(g_motor_speed_mutex);
    apply_motor_pwm(motor1_pwm, motor2_pwm);
}

void motor_speed_force_brake_pwm(int motor1_pwm, int motor2_pwm)
{
    // Only the active-brake state calls this API. Normal PID and force-PWM
    // paths remain limited to 4000 reverse PWM by apply_motor_pwm().
    std::lock_guard<std::mutex> lock(g_motor_speed_mutex);
    apply_motor_brake_pwm(motor1_pwm, motor2_pwm);
}

/********************************************************************************
 * @brief   闭环控制主函数
 * @param   pwm1/pwm2: PWM驱动对象
 *          speed_of_motor1/2: 当前电机转速（输入）
 *          target_speed1/2: 目标转速（输入）
 ********************************************************************************/

void close_circle_control(
    float speed_of_motor1,
    float speed_of_motor2,
    float target_speed_of_motor1_RPS,
    float target_speed_of_motor2_RPS)
{
    std::lock_guard<std::mutex> lock(g_motor_speed_mutex);
    int pwm1_plusduty = 0;
    int pwm2_plusduty = 0;

    // 计算增量式 PID 输出
    calculate_differential_for_motor(
        speed_of_motor1, speed_of_motor2,
        target_speed_of_motor1_RPS, target_speed_of_motor2_RPS,
        pwm1_plusduty, pwm2_plusduty);

    // 增量叠加
    current_pwm1 += pwm1_plusduty;
    current_pwm2 += pwm2_plusduty;

    // 内部状态双向限幅，防止 windup
    apply_motor_pwm(current_pwm1, current_pwm2);
}
