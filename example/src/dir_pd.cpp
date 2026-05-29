#include "lq_all_demo.hpp"

volatile float dir_P = 0.096f;
volatile float dir_D = 0.91f;
volatile int spd_slow_ratio = 20;
/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.

 ********************************************************************************/

float calculate_diffrential(float error,float expect_error)//给我误差值，给你差分输入值
        {
        float Diffrential=0;//diffrencial 差分输入，即输出轮胎的转速差
       volatile static int error_current,error_last;// 当前误差和上一次误差
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
    const float dead_error=4;
    if(error>max_error) error=max_error;
    if(error<-max_error) error=-max_error;//restrct
    if((error<dead_error)&&(error>-dead_error)) error=0;
    int diffrential = calculate_diffrential(error, 0);

    const int max_dif=8;
    if(diffrential>max_dif) diffrential=max_dif;
    if(diffrential<-max_dif) diffrential=-max_dif;
    int target_spd1 = set_speed_of_motor1_rps;
    set_speed_of_motor2_rps=target_spd1;//for test esay

    int slow_ratio_percent = spd_slow_ratio;
    if (slow_ratio_percent < 0) slow_ratio_percent = 0;
    if (slow_ratio_percent > 50) slow_ratio_percent = 50;

    // spd_slow_ratio 表示最大减速百分比，30 表示 error 达到 max_error 时最多降速 30%。
    // 这里仍保留 0.7 的下限保护，避免基准速度被降得太低。
    int abs_error = error;
    if (abs_error < 0) abs_error = -abs_error;
    float slow_ratio = 1.0f - (slow_ratio_percent / 100.0f) * abs_error / max_error;
    if (slow_ratio < 0.5f) slow_ratio = 0.5f;
    int set_spd1 = static_cast<int>(target_spd1 * slow_ratio + 0.5f);

    pwm1_duty_rps = set_spd1 + diffrential;
    pwm2_duty_rps = set_spd1 - diffrential;
    const int min_rps=-10;
    if (pwm1_duty_rps < min_rps)
    {
        pwm1_duty_rps =min_rps;
    };
    if (pwm1_duty_rps > 200)
    {
        pwm1_duty_rps = 200;
    };
    if (pwm2_duty_rps < min_rps)
    {
        pwm2_duty_rps = min_rps;
    };
    if (pwm2_duty_rps > 200)
    {
        pwm2_duty_rps = 200;
    };

   
}
