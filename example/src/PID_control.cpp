#include "lq_all_demo.hpp"

volatile int error_on=1;
volatile float dir_P = 0.25f;
volatile float dir_D = 0.1f;
/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.

 ********************************************************************************/

int calculate_diffrential(int error,int expect_error)//给我误差值，给你差分输入值
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
void PID_control_test(int error)
{
    const int max_error=70;
    if(error>max_error) error=max_error;
    if(error<-max_error) error=-max_error;
     
    int diffrential = calculate_diffrential(error, 0);

    const int max_dif=15;
    if(diffrential>max_dif) diffrential=max_dif;
    if(diffrential<-max_dif) diffrential=-max_dif;
    //int diffrential = calculate_diffrential(0, 0);
    set_speed_of_motor2_rps=set_speed_of_motor1_rps;//for test esay
    pwm1_duty_rps = set_speed_of_motor1_rps + diffrential;
    pwm2_duty_rps = set_speed_of_motor2_rps - diffrential;
    if (pwm1_duty_rps < 0)
    {
        pwm1_duty_rps = 0;
    };
    if (pwm1_duty_rps > 200)
    {
        pwm1_duty_rps = 200;
    };
    if (pwm2_duty_rps < 0)
    {
        pwm2_duty_rps = 0;
    };
    if (pwm2_duty_rps > 200)
    {
        pwm2_duty_rps = 200;
    };

   
}
