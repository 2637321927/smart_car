#include "main.hpp"
#include "lq_timer.hpp"

bool need_exit = false;

//begin to test timer
lq_timer speed_timer;
lq_timer dir_timer;

int latest_error = 0;

ls_atim_pwm pwm2(ATIM_PWM0_PIN81, 50, 0);
ls_atim_pwm pwm1(ATIM_PWM1_PIN82, 50, 0); 
ls_encoder_pwm enc2(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc1(ENC_PWM1_PIN65, PIN_73);
int set_speed_of_motor1_rps=0;
int set_speed_of_motor2_rps=0;
lq_udp_client udp_client;
int mid;
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
// 全局变量，保存原来的终端模式
int main()
{


input_speed_rps(set_speed_of_motor1_rps,set_speed_of_motor2_rps);

   speed_timer.set_seconds_ms(5, []() {
       test_enc_and_motor_rps;        // 直接调用你封装好的速度函数
    });

    dir_timer.set_seconds_ms(10, []() {
        PID_control_test(latest_error);   // 直接调用你封装好的方向函数
    });
start_camera();
while (1)
{
         if (has_input()) {
            char c = getchar();
            if (c == 'q') {
                std::cout<<"caonima"<<std::endl;
                cut();
                 while (getchar() != EOF); 
                break;
            } 
        }
 img_test();

}
     std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
    return 0;
}
