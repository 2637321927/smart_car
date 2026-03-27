#include "lq_all_demo.hpp"

//ls_atim_pwm pwm1_test(ATIM_PWM0_PIN81, 50, 0);
//ls_atim_pwm pwm2_test(ATIM_PWM1_PIN82, 50, 0); 

//encoder
ls_encoder_pwm enc1_test(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc2_test(ENC_PWM1_PIN65, PIN_73);

int expected_speed_of_motor1_pwm=0;
int expected_speed_of_motor2_pwm=0;

void input_speed(int&expected_speed_of_motor1_rps,int& expected_speed_of_motor2_rps){
    const int MAX_SPEED = 9500;
const int MIN_SPEED = 0;
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
printf("请输入电机1、电机2目标转速(pwm，空格分隔)：");
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


void  test_enc_and_motor(int expected_speed_of_motor1_pwm,int expected_speed_of_motor2_pwm)
{ 
    while(1){
        usleep(1000);



pwm1.atim_pwm_set_duty(expected_speed_of_motor1_pwm);
pwm2.atim_pwm_set_duty(expected_speed_of_motor2_pwm);

float encoder_1=enc1_test.encoder_get_count();

float encoder_2=enc2_test.encoder_get_count();
// 正确写法：字符串单独闭合，变量写在外面，逗号分隔
printf("expected speed :%d:%d\n speed %f  %f\n", 
       expected_speed_of_motor1_pwm,
       expected_speed_of_motor2_pwm,
       encoder_1,
       encoder_2);
    }
}