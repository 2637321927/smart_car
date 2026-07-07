#include "lq_all_demo.hpp"
#include "gyro_yaw_rate_control.hpp"

#include <cstdio>

volatile float dir_P = 0.128f;
volatile float dir_D = 1.55f;
volatile int spd_slow_ratio = 20;

namespace
{
void print_direction_control_status(const char *mode,
                                    const char *reason,
                                    float vision_error,
                                    float base_rps,
                                    float differential_rps)
{
    static int count = 0;
    if (++count < 50) {
        return;
    }
    count = 0;

    printf("[DIR] mode=%s reason=%s gyro_request=%d gyro_ready=%d err=%.2f base_rps=%.2f diff_rps=%.2f target1=%.2f target2=%.2f\n",
           mode,
           reason,
           gyro_yaw_rate_feedback_enabled ? 1 : 0,
           gyro_yaw_rate_control_is_ready() ? 1 : 0,
           vision_error,
           base_rps,
           differential_rps,
           pwm1_duty_rps,
           pwm2_duty_rps);
}
} // namespace
/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.

 ********************************************************************************/

float calculate_diffrential(float error,float expect_error)//给我误差值，给你差分输入值
        {
        float Diffrential=0;//diffrencial 差分输入，即输出轮胎的转速差
       static float error_current = 0.0f;
       static float error_last = 0.0f;// 当前误差和上一次误差
       error_current=error-expect_error;//当前误差

       Diffrential=error_current*dir_P+ (error_current-error_last)*dir_D;//PD控制算法
       error_last=error_current;//更新一下误差
     
      // printf("Df_P:%f Df_D %f\n",error_current*dir_P,(error_current-error_last)*dir_D);
         return Diffrential;//返回差分输入
        }
         




//以下是fuzzy PID的代码



//below we test the speed circle ,no error!!!!!
void PID_control_test(float error)
{
    const float max_error=100;
    const float dead_error=2;
    if(error>max_error) error=max_error;
    if(error<-max_error) error=-max_error;//restrct
    if((error<dead_error)&&(error>-dead_error)) error=0;
    // 方向环有三种可打印状态：
    // 1. VISUAL_PD：#gyro=0，请求使用原来的视觉 PD。
    // 2. GYRO_RATE：#gyro=1 且 MPU6050 ready，使用角速度反馈。
    // 3. VISUAL_PD_FALLBACK：#gyro=1 但 MPU6050 没 ready，自动回退旧视觉 PD。
    //
    // 旧视觉 PD：error 直接变成左右轮速度差。
    // 新角速度环：error 先变成目标角速度，
    //    再用 MPU6050 的 gz 闭环得到左右轮速度差。
    const bool gyro_feedback_requested = (gyro_yaw_rate_feedback_enabled != 0);
    const bool gyro_feedback_ready = gyro_yaw_rate_control_is_ready();
    const bool use_gyro_yaw_rate_feedback = gyro_feedback_requested && gyro_feedback_ready;

    const char *control_mode = "VISUAL_PD";
    const char *control_reason = "gyro_off";
    float diffrential = 0.0f;
    if (use_gyro_yaw_rate_feedback) {
        control_mode = "GYRO_RATE";
        control_reason = "mpu6050_feedback";
        diffrential = gyro_yaw_rate_control_update(error);
        gyro_yaw_rate_control_print_debug(100); // 8ms * 100 ~= 800ms 打印一次
    } else {
        if (gyro_feedback_requested && !gyro_feedback_ready) {
            control_mode = "VISUAL_PD_FALLBACK";
            control_reason = "mpu6050_not_ready";
        }
        diffrential = calculate_diffrential(error, 0);
    }

    const float max_dif=15.0f;
    if(diffrential>max_dif) diffrential=max_dif;
    if(diffrential<-max_dif) diffrential=-max_dif;
    float target_spd1 = set_speed_of_motor1_rps;
    set_speed_of_motor2_rps=target_spd1;//for test esay

    int slow_ratio_percent = spd_slow_ratio;
    if (slow_ratio_percent < 0) slow_ratio_percent = 0;
    if (slow_ratio_percent > 50) slow_ratio_percent = 50;

    // spd_slow_ratio 表示最大减速百分比，30 表示 error 达到 max_error 时最多降速 30%。
    // 这里仍保留 0.7 的下限保护，避免基准速度被降得太低。
    float abs_error = error;
    if (abs_error < 0) abs_error = -abs_error;
    float slow_ratio = 1.0f - (slow_ratio_percent / 100.0f) * abs_error / max_error;
    if (slow_ratio < 0.5f) slow_ratio = 0.5f;
    float set_spd1 = target_spd1 * slow_ratio;
  
    pwm1_duty_rps = set_spd1 + diffrential;
    pwm2_duty_rps = set_spd1 - diffrential;

    //pwm1_duty_rps = set_spd1 ;
    //pwm2_duty_rps = set_spd1 ;
    //to codex :this is for test ,rembmber to delete
    const float min_rps=-10.0f;
    if (pwm1_duty_rps < min_rps)
    {
        pwm1_duty_rps =min_rps;
    };
    if (pwm1_duty_rps > 200.0f)
    {
        pwm1_duty_rps = 200.0f;
    };
    if (pwm2_duty_rps < min_rps)
    {
        pwm2_duty_rps = min_rps;
    };
    if (pwm2_duty_rps > 200.0f)
    {
        pwm2_duty_rps = 200.0f;
    };

    print_direction_control_status(control_mode, control_reason, error, set_spd1, diffrential);
   
}
