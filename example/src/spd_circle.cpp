#include "lq_all_demo.hpp"

volatile float P = 80.0f;
volatile float I = 2.0f;
volatile float D = 0.0f;

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
    // 历史误差
    static float error_last1 = 0.0f;
    static float error_last2 = 0.0f;
    static float error_prev1 = 0.0f;
    static float error_prev2 = 0.0f;

    // 当前误差
    float error_current1 = target_speed_of_motor1_RPS - speed_of_motor1;
    float error_current2 = target_speed_of_motor2_RPS - speed_of_motor2;

    // 真正的增量式 PID
    float p_term1 = P * (error_current1 - error_last1);
    float p_term2 = P * (error_current2 - error_last2);

    float i_term1 = I * error_current1;
    float i_term2 = I * error_current2;

    float d_term1 = D * (error_current1 - 2.0f * error_last1 + error_prev1);
    float d_term2 = D * (error_current2 - 2.0f * error_last2 + error_prev2);

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
    error_prev1 = error_last1;
    error_prev2 = error_last2;
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
    float speed_of_motor1,
    float speed_of_motor2,
    int target_speed_of_motor1_RPS,
    int target_speed_of_motor2_RPS)
{
    // 内部控制状态
    static int current_pwm1 = 0;
    static int current_pwm2 = 0;

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

    const int MAX_PWM = 5000;

    // 内部状态双向限幅，防止 windup
    if (current_pwm1 > MAX_PWM) current_pwm1 = MAX_PWM;
    if (current_pwm1 < -MAX_PWM) current_pwm1 = -MAX_PWM;
    if (current_pwm2 > MAX_PWM) current_pwm2 = MAX_PWM;
    if (current_pwm2 < -MAX_PWM) current_pwm2 = -MAX_PWM;

    

    //pwm1.atim_pwm_set_duty(3000);

    //pwm2.atim_pwm_set_duty(3000);
    // motor1 输出
    if (current_pwm1 >= 0)
    {
        polar_pwm1.gpio_level_set(GPIO_LOW);   // 正转
        pwm1.atim_pwm_set_duty(current_pwm1);
    }
    else
    {
        polar_pwm1.gpio_level_set(GPIO_HIGH);    // 反转
        pwm1.atim_pwm_set_duty(-current_pwm1);
    }

    // motor2 输出
    if (current_pwm2 >= 0)
    {
        polar_pwm2.gpio_level_set(GPIO_LOW);    // 正转
        pwm2.atim_pwm_set_duty(current_pwm2);
    }
    else
    {
        polar_pwm2.gpio_level_set(GPIO_HIGH);   // 反转
        pwm2.atim_pwm_set_duty(-current_pwm2);
    }
}