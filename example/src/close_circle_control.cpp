#include "lq_all_demo.hpp"


volatile float P = 2.0f;
volatile float D = 0.0f; // PD_motor 参数

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
 
 

    // 静态变量：保存上一次误差（函数内持久化）
    static float error_last1 = 0.0f;
    static float error_last2 = 0.0f;
    static float error1_filt = 0.0f;
static float error2_filt = 0.0f;
static float error1_filt_last = 0.0f;
static float error2_filt_last = 0.0f;

    // 计算当前误差（目标转速 - 当前转速，单位必须统一：RPS）
    const float error_current1 = target_speed_of_motor1_RPS - speed_of_motor1;
    const float error_current2 = target_speed_of_motor2_RPS - speed_of_motor2;

    error1_filt = alpha_flit * error1_filt_last + (1.0f - alpha_flit) * error_current1;
error2_filt = alpha_flit * error2_filt_last + (1.0f - alpha_flit) *error_current2;

    // PD控制算法
    pwm1_plusduty = error_current1 * P + (error1_filt - error1_filt_last) * D;
    pwm2_plusduty = error_current2 * P + (error2_filt  - error2_filt_last) * D;
    P1_motor=error_current1 * P ;
    P2_motor=error_current2 * P ;

        D1_motor=(error1_filt - error1_filt_last) * D;
        D2_motor=(error2_filt  - error2_filt_last) * D;
        //printf("P1:%f  D1:%f\n",error_current1 * P ,(error_current1 - error_last1) * D);
    //printf("P2:%f  D2:%f\n",error_current2 * P ,(error_current2 - error_last2) * D);
        // 更新上一次误差
        error_last1 = error_current1;
        error_last2 = error_current2;
    error1_filt_last=error1_filt;
    error2_filt_last=error2_filt;

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
    // ===================== 核心修改 =====================
    // 静态变量：保存当前PWM占空比（闭环控制的核心！）
    static int current_pwm1 = 0;
    static int current_pwm2 = 0;
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
    //static int base_pwm_25 =  1950;
    //current_pwm1=base_pwm_25+ pwm1_plusduty;
    //current_pwm2=base_pwm_25+ pwm2_plusduty;

    // ===================== 安全保护 =====================
    // PWM限幅
    //const int MAX_PWM = 10000;
    //const int MIN_PWM = 0;
    //if (current_pwm1 > MAX_PWM) current_pwm1 = MAX_PWM;
    //if (current_pwm1 < MIN_PWM) current_pwm1 = MIN_PWM;
    //if (current_pwm2 > MAX_PWM) current_pwm2 = MAX_PWM;
    //if (current_pwm2 < MIN_PWM) current_pwm2 = MIN_PWM;
    const int MAX_PWM = 5000;

// motor1
if (current_pwm1 >= 0)
{
    // 正转
    polar_pwm1.gpio_level_set(GPIO_HIGH);
    if (current_pwm1 > MAX_PWM) current_pwm1 = MAX_PWM;
    pwm1.atim_pwm_set_duty(current_pwm1);
}
else
{
    // 反转
    polar_pwm1.gpio_level_set(GPIO_LOW);
    int duty1 = -current_pwm1;
    if (duty1 > MAX_PWM) duty1 = MAX_PWM;
    pwm1.atim_pwm_set_duty(duty1);
}

// motor2
if (current_pwm2 >= 0)
{
    // 正转
    polar_pwm2.gpio_level_set(GPIO_LOW);
    if (current_pwm2 > MAX_PWM) current_pwm2 = MAX_PWM;
    pwm2.atim_pwm_set_duty(current_pwm2);
}
else
{
    // 反转
    polar_pwm2.gpio_level_set(GPIO_HIGH);
    int duty2 = -current_pwm2;
    if (duty2 > MAX_PWM) duty2 = MAX_PWM;
    pwm2.atim_pwm_set_duty(duty2);
}

    // ===================== 输出PWM =====================
   // pwm1.atim_pwm_set_duty(current_pwm1);
   // pwm2.atim_pwm_set_duty(current_pwm2);
 
}