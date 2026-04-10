#include "lq_all_demo.hpp"

float encoder_1=0;
float encoder_2=0;
int expected_speed_of_motor1_pwm=0;
int expected_speed_of_motor2_pwm=0;


void input_speed_rps(int&expected_speed_of_motor1_rps,int& expected_speed_of_motor2_rps){
    const int MAX_SPEED = 200;
const int MIN_SPEED = 0;
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
printf("请输入电机1、电机2目标转速(rps，空格分隔)：");
// 读取两个int型数据
int res = scanf("%d %d", &expected_speed_of_motor1_rps, &expected_speed_of_motor2_rps);

// 合法性判断
if (res == 2) 
{
    if (expected_speed_of_motor1_rps >= MIN_SPEED && expected_speed_of_motor1_rps <= MAX_SPEED &&
        expected_speed_of_motor2_rps >= MIN_SPEED && expected_speed_of_motor2_rps <= MAX_SPEED)
    {
        printf("输入正确！电机1：%d，电机2：%d\n", expected_speed_of_motor1_rps, expected_speed_of_motor2_rps);
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
    
    
encoder_1=std::fabs(enc1.encoder_get_count());//give it abs in case of negative num
encoder_2=std::fabs(enc2.encoder_get_count());

close_circle_control(
    encoder_1,
    encoder_2,
    pwm1_duty_rps,
    pwm2_duty_rps);

    
}

