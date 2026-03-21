#include "main.hpp"
bool need_exit = false;
//ls_atim_pwm pwm1(ATIM_PWM0_PIN81, 100, 0);
//ls_atim_pwm pwm2(ATIM_PWM1_PIN82, 100, 0); 
// Ctrl+C 触发的函数
void handle_exit(int sig)
{
    printf("\n⚠️  检测到 Ctrl + C，开始安全退出...\n");

    // ======================
    // 你想在退出前执行的代码
    // ======================
    printf("正在停止电机...\n");
    // 在这里写停车：pwm1.setDuty(0); pwm2.setDuty(0);
    //pwm1.atim_pwm_disable();
//pwm2.atim_pwm_disable();
    printf("正在关闭摄像头...\n");
    // 在这里写关闭摄像头
    
    printf("✅ 安全退出完成！\n");

    // 标记需要退出
    need_exit = true;
}
int main()
{
   img_test();
//lq_udp_img_trans_demo();
//PID_control_test();
//lq_atim_pwm_demo();
 //signal(SIGINT, handle_exit);  // 绑定 Ctrl+C
    uint16_t dis;

    lq_i2c_vl53l0x vl53l0x;

//img_test();
//lq_encoder_pwm_demo();


//below is the test demo of camera_PD control

float error_expected=40;
float error_value=0;
//while(!need_exit){
//error_value=img_return();
//img_test();
//PID_control_test(pwm1,pwm2,error_value- error_expected);
//}



    return 0;
}
