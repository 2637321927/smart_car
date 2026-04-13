#include "lq_all_demo.hpp"
 
/********************************************************************************
 * @brief   PID 控制测试.
 * @param   none.
 * @return  none.
 * @note    GPIO 输出测试, 使用引脚 81/82 作为输出引脚, 交替输出高电平和低电平.
            
 ********************************************************************************/
/*
int calculate_diffrential(int error,int expect_error)//给我误差值，给你差分输入值
        {        
        float Diffrential=0;//diffrencial 差分输入，即输出轮胎的转速差
       float P = 0.25; // 比例系数
       float D= 0.1; // 差分系数
       volatile static int error_current,error_last;// 当前误差和上一次误差
       error_current=error-expect_error;//当前误差
       
       Diffrential=error_current*P+ (error_current-error_last)*D;//PD控制算法
       error_last=error_current;//更新一下误差
        
      // printf("Df_P:%f Df_D %f\n",error_current*P,(error_current-error_last)*D);
         return Diffrential;//返回差分输入
        }
         */

 // ------------- 7级量化 -------------
    int quantize(int x)
    {
        if (x <= -30) return 0; // NB
        if (x <= -18) return 1; // NM
        if (x <= -6)  return 2; // NS
        if (x <   6)  return 3; // ZO
        if (x <  18)  return 4; // PS
        if (x <  30)  return 5; // PM
        return 6;               // PB
    }
    
    
    int quantize_ec(int ec)
{
    if (ec <= -10) return 0;
    if (ec <= -6)  return 1;
    if (ec <= -2)  return 2;
    if (ec <   2)  return 3;
    if (ec <   6)  return 4;
    if (ec <  10)  return 5;
    return 6;
}




       int calculate_diffrential(int error_current, int error_expect)//new tape
{
     static int error_last = 0;

    // 当前误差
    int e = error_current - error_expect;

    // 误差变化量
    int ec = e - error_last;

    //小死区，避免直道微小误差来回修正
    if (e >= -4 && e <= 4)
    {
        e = 0;
    }

//useless for now
   
/*NB = Negative Big，负大
NM = Negative Medium，负中
NS = Negative Small，负小
ZO = Zero，零
PS = Positive Small，正小
PM = Positive Medium，正中
PB = Positive Big，正大*/
    int e_idx = quantize(e);
    int ec_idx = quantize_ec(ec); //index

    // ------------- Kp规则表 -------------
    // 行: e, 列: ec
static const int KP_RULE[7][7] =
{
    { 4, 4, 5, 6, 5, 4, 4 },
    { 3, 4, 4, 5, 4, 4, 3 },
    { 1, 1, 2, 2, 2, 1, 1 },
    { 0, 0, 1, 0, 1, 0, 0 },
    { 1, 1, 2, 2, 2, 1, 1 },
    { 3, 4, 4, 5, 4, 4, 3 },
    { 4, 4, 5, 6, 5, 4, 4 }
};
    // ------------- Kd规则表 -------------
static const int KD_RULE[7][7] =
{
    { 6, 5, 4, 3, 4, 5, 6 },
    { 5, 4, 3, 2, 3, 4, 5 },
    { 4, 3, 2, 2, 2, 3, 4 },
    { 6, 4, 2, 0, 2, 4, 6 },
    { 4, 3, 2, 2, 2, 3, 4 },
    { 5, 4, 3, 2, 3, 4, 5 },
    { 6, 5, 4, 3, 4, 5, 6 }
};

    // ------------- 等级映射到具体Kp/Kd -------------
    static const float KP_TABLE[7] = {
        0.08f, 0.12f, 0.16f, 0.22f, 0.30f, 0.42f, 0.58f
    };

    static const float KD_TABLE[7] = {
     0.00f, 0.01f, 0.02f, 0.035f, 0.055f, 0.075f, 0.10f
    };

    float P = KP_TABLE[KP_RULE[e_idx][ec_idx]];
    float D = KD_TABLE[KD_RULE[e_idx][ec_idx]];

    int Diffrential = (int)(e * P + ec * D);

    error_last = e;
    return Diffrential;
}  
void PID_control_test(int error)
{


        
     int diffrential = calculate_diffrential(error,40);
     pwm1_duty_rps=set_speed_of_motor1_rps + diffrential ;
     pwm2_duty_rps=set_speed_of_motor2_rps - diffrential ;
     if (pwm1_duty_rps<0){pwm1_duty_rps=0;};
     if (pwm1_duty_rps>200){pwm1_duty_rps=200;};
     if (pwm2_duty_rps<0){pwm2_duty_rps=0;};
     if (pwm2_duty_rps>200){pwm2_duty_rps=200;};

//test_enc_and_motor_rps();use no more

}