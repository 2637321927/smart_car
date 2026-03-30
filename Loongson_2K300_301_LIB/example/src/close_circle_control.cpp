#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   电机转速PD闭环控制
 * @param   目标转速RPS，当前测速值RPS
 * @return  PWM调整量
 * @note    PD算法，带抗噪微分，固定周期调用
 ********************************************************************************/

void calculate_differential_for_motor(
    const float& speed_of_motor1, const float& speed_of_motor2,
    const int target_speed_of_motor1_RPS, const int target_speed_of_motor2_RPS,
    int& pwm1_plusduty, int& pwm2_plusduty)
{
    // PD参数
    const float P = 6.0f;
    const float D = 0.1f;

    // 静态变量：保存上一次误差（函数内持久化）
    static float error_last1 = 0.0f;
    static float error_last2 = 0.0f;

    // 计算当前误差（目标转速 - 当前转速，单位必须统一：RPS）
    const float error_current1 = target_speed_of_motor1_RPS - speed_of_motor1;
    const float error_current2 = target_speed_of_motor2_RPS - speed_of_motor2;

    // PD控制算法
    pwm1_plusduty = error_current1 * P + (error_current1 - error_last1) * D;
    pwm2_plusduty = error_current2 * P + (error_current2 - error_last2) * D;

    printf("P1:%f  D1:%f\n",error_current1 * P ,(error_current1 - error_last1) * D);
   printf("P2:%f  D2:%f\n",error_current2 * P ,(error_current2 - error_last2) * D);
    // 更新上一次误差
    error_last1 = error_current1;
    error_last2 = error_current2;

    
}

/********************************************************************************
 * @brief   闭环控制主函数
 * @param   pwm1/pwm2: PWM驱动对象
 *          speed_of_motor1/2: 当前电机转速（输入）
 *          target_speed1/2: 目标转速（输入）
 ********************************************************************************/

void close_circle_control(
    float& speed_of_motor1,
    float& speed_of_motor2,
    int target_speed_of_motor1_RPS,
    int target_speed_of_motor2_RPS)
{
    // ===================== 核心修改 =====================
    // 静态变量：保存当前PWM占空比（闭环控制的核心！）
    static int current_pwm1 = 0;
    static int current_pwm2 = 0;
    //std::cout<<"caonima        "<<target_speed_of_motor1_RPS<<"    "<<target_speed_of_motor2_RPS<<std::endl;//test
    // PWM调整量（全变量初始化）
    int pwm1_plusduty = 0;
    int pwm2_plusduty = 0;

    // 计算PD调整量
    calculate_differential_for_motor(
        speed_of_motor1, speed_of_motor2,
        target_speed_of_motor1_RPS, target_speed_of_motor2_RPS,
        pwm1_plusduty, pwm2_plusduty);

    // PD调整量 叠加到 当前PWM占空比
    current_pwm1 += pwm1_plusduty;
    current_pwm2 += pwm2_plusduty;

    // ===================== 安全保护 =====================
    // PWM限幅
    const int MAX_PWM = 10000;
    const int MIN_PWM = 0;
    if (current_pwm1 > MAX_PWM) current_pwm1 = MAX_PWM;
    if (current_pwm1 < MIN_PWM) current_pwm1 = MIN_PWM;
    if (current_pwm2 > MAX_PWM) current_pwm2 = MAX_PWM;
    if (current_pwm2 < MIN_PWM) current_pwm2 = MIN_PWM;

    // ===================== 输出PWM =====================
    pwm1.atim_pwm_set_duty(current_pwm1);
    pwm2.atim_pwm_set_duty(current_pwm2);
}