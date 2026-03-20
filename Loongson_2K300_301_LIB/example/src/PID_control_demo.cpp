#include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   PD 控制demo
 * @param   none.
 * @return  none.
 * @note    纯PD驱动小车的Demo
            
 ********************************************************************************/
int calculate_diffrential(float error)//给我误差值，给你差分输入值
        {        
        float Diffrential=0;//diffrencial 差分输入，即输出轮胎的转速差
       float P = 10; // 比例系数
       float D= 0.1; // 差分系数
       volatile static float error_current,error_last;// 当前误差和上一次误差
       error_current=error;//当前误差
       
       Diffrential=error_current*P+ (error_current-error_last)*D;//PD控制算法
       error_last=error_current;//更新一下误差
         return Diffrential;//返回差分输入
        }
void PID_control_test(float error)
{
   
    ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, 0);
    ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, 0);//初始化PWM，初始占空比为0
   
     int diffrential = calculate_diffrential(error);
     int pwm1_duty1 = (1000 + diffrential < 5000) ? (1000 + diffrential) : 5000;//从1000开始，差分输入增加pwm1的占空比,并且做了限幅（不得大于5000）
      int pwm2_duty1 = (1000 - diffrential > 0) ? (1000 - diffrential) : 0;//从1000开始，差分输入减少pwm2的占空比，并且做了限幅（不得小于0）
    ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, pwm1_duty1);
    ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, pwm2_duty1);

pwm1.atim_pwm_set_duty(0);
pwm2.atim_pwm_set_duty(0);//全部失能
}
