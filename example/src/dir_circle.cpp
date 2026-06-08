#include "lq_all_demo.hpp"



//fliter test
void input_speed_rps(){
    const float MAX_SPEED = 200.0f;
const float MIN_SPEED = 0.0f;
printf("请输入电机1、电机2目标转速(rps，空格分隔)：");
// 读取两个 float 型数据，避免小数 RPS 目标在进入速度环前被截断。
float speed1 = 0.0f;
float speed2 = 0.0f;
int res = scanf("%f %f", &speed1, &speed2);

// 合法性判断
if (res == 2) 
{
    if (speed1 >= MIN_SPEED && speed1 <= MAX_SPEED &&
        speed2 >= MIN_SPEED && speed2 <= MAX_SPEED)
    {
        set_speed_of_motor1_rps = speed1;
        set_speed_of_motor2_rps = speed2;
        printf("输入正确！电机1：%.2f，电机2：%.2f\n", set_speed_of_motor1_rps, set_speed_of_motor2_rps);
        // 这里可以直接调用你的闭环控制函数
    }
    else
    {
        printf("输入错误：转速超出范围！\n");
    }\
}
else
{
    printf("输入错误：请输入两个整数！\n");
    fflush(stdin); // 清空输入缓存
}}

void  test_enc_and_motor_rps()
{ 
    
    
     //std::lock_guard<std::mutex> lock(g_mutex);
//encoder_1=-enc1.encoder_get_count();// enc1 always gets a negative number 
//encoder_2=enc2.encoder_get_count();


//std::cout<<"fuck you"<<std::endl;
close_circle_control(
  encoder1_speed_avg,
  encoder2_speed_avg,
    pwm1_duty_rps,
    pwm2_duty_rps);
}
