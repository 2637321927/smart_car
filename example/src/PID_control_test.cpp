#include "lq_all_demo.hpp"
 
/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.
            
 ********************************************************************************/
int calculate_diffrential(int error, int expect_error)
{
    const float P = 0.25f; 
    const float D = 0.08f;

    static int error_last = 0;

    int err = error - expect_error;

    // 误差先做滤波，防止跳变
    static int err_filter = 0;
    err_filter = (err_filter * 7 + err) / 8;  // 一阶低通滤波

    int out = err_filter * P + (err_filter - error_last) * D;
    error_last = err_filter;

    return out;
}
void PID_control_test(int error)
{
    // 1. 计算转向差速
    int diffrential = calculate_diffrential(error, 40);

    // 2. 计算期望速度
    int want1 = set_speed_of_motor1_rps + diffrential;
    int want2 = set_speed_of_motor2_rps - diffrential;

    // ===================== 【核心：速度平滑，真正解决卡顿】 =====================
    const int MAX_STEP = 2;   // 每次最多变 2，越小越丝滑

    // 左电机平滑
    if (want1 > pwm1_duty_rps)
        pwm1_duty_rps += std::min(want1 - pwm1_duty_rps, MAX_STEP);
    else if (want1 < pwm1_duty_rps)
        pwm1_duty_rps -= std::min(pwm1_duty_rps - want1, MAX_STEP);

    // 右电机平滑
    if (want2 > pwm2_duty_rps)
        pwm2_duty_rps += std::min(want2 - pwm2_duty_rps, MAX_STEP);
    else if (want2 < pwm2_duty_rps)
        pwm2_duty_rps -= std::min(pwm2_duty_rps - want2, MAX_STEP);
    // ===========================================================================

    // 最后安全限幅
   const int MAX_SPEED = 200;
    const int MIN_SPEED = 0;

    // 左电机限幅
    if (pwm1_duty_rps > MAX_SPEED)
        pwm1_duty_rps = MAX_SPEED;
    else if (pwm1_duty_rps < MIN_SPEED)
        pwm1_duty_rps = MIN_SPEED;

    // 右电机限幅
    if (pwm2_duty_rps > MAX_SPEED)
        pwm2_duty_rps = MAX_SPEED;
    else if (pwm2_duty_rps < MIN_SPEED)
        pwm2_duty_rps = MIN_SPEED;
}