include "lq_all_demo.hpp"

/********************************************************************************
 * @brief   test for colse circle control 
 * @param   none.
 * @return  none.
 * @note    input target speed (max 10000),adjust the speed of motor and control it with PI
            
 ********************************************************************************/
 int calculate_diffrential(float error)//给我误差值，给你差分输入值
        {        
        float Diffrential=0;//diffrencial 差分输入，即输出轮胎的转速差
       float P = 50; // 比例系数
       float D= 0.1; // 差分系数
       volatile static float error_current,error_last;// 当前误差和上一次误差
       error_current=error;//当前误差
       
       Diffrential=error_current*P+ (error_current-error_last)*D;//PD控制算法
       error_last=error_current;//更新一下误差
         return Diffrential;//返回差分输入
        }
void close_circle_control (ls_atim_pwm& pwm1,ls_atim_pwm& pwm2,&float speed_of_motor1,&float speed_of_motor2,int target_speed_of_motor1_RPS,int target_speed_of_motor2_RPS)
{
     int diffrential = calculate_diffrential(error);
     int pwm1_duty1 = (500 + diffrential < 1000) ? (500+ diffrential) : 1000;//从1000开始，差分输入增加pwm1的占空比,并且做了限幅（不得大于5000）
      int pwm2_duty1 = (500 - diffrential > 0) ? (500 - diffrential) : 0;//从1000开始，差分输入减少pwm2的占空比，并且做了限幅（不得小于0）
      pwm1.atim_pwm_set_duty(pwm1_duty1);
      pwm2.atim_pwm_set_duty(pwm2_duty1);
}
