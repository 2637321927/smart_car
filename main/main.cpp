#include "main.hpp"
#include "lq_timer.hpp"

bool need_exit = false;
// 全局互斥锁（解决多线程冲突）
//std::mutex g_mutex;
//begin to test timer
lq_timer speed_timer;
lq_timer dir_timer;
volatile  int pwm1_duty_rps=0;
 volatile  int pwm2_duty_rps=0;
 volatile  int latest_error = 0;
 volatile  float encoder_1=0;
 volatile  float encoder_2=0;
ls_atim_pwm pwm2(ATIM_PWM0_PIN81, 50, 0);
ls_atim_pwm pwm1(ATIM_PWM1_PIN82, 50, 0); 
ls_encoder_pwm enc2(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc1(ENC_PWM1_PIN65, PIN_73);
 volatile int set_speed_of_motor1_rps=0;
 volatile int set_speed_of_motor2_rps=0;
lq_udp_client udp_client;
lq_udp_client udp_client_img;
volatile int test_count = 0;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 60;     // 帧率
static struct termios old_tio;
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
 volatile  int mid;
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
// 开启 非阻塞输入
void set_terminal_nonblock() {
    struct termios new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;

    // 关闭 行缓冲 + 关闭回显
    new_tio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

// 恢复终端（非常重要！）
void reset_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

// 判断：有没有按键输入？
bool has_input() {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    // 超时 0 → 不等待，直接返回
    struct timeval tv = {0, 0};
    select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);

    return FD_ISSET(STDIN_FILENO, &fds);
}
// 全局变量，保存原来的终端模式
int main()
{

   std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等线程就绪
input_speed_rps();
start_camera();
//set_terminal_nonblock();


   speed_timer.set_seconds_ms(3, []() {
     test_enc_and_motor_rps();   
     //   test_count++;   
     //  std::cout<<"fuck you"<<std::endl;  // 直接调用你封装好的速度函数
    });

    dir_timer.set_seconds_ms(6, []() {
        PID_control_test(latest_error);   // 直接调用你封装好的方向函数
    });
//std::cout<<"fuck you2"<<std::endl; 
  
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
              
// std::lock_guard<std::mutex> lock(g_mutex);
 cv::Mat frame = cam.get_raw_frame();
latest_error=img_test(frame);
//std::cout<<"fuck you"<<std::endl;
// 正确写法：字符串单独闭合，变量写在外面，逗号分隔

char encoder_str[128];
snprintf(encoder_str, sizeof(encoder_str),
         "{\"ex_rps1\":%d,\"ex_rps2\":%d,\"rps1\":%.2f,\"rps2\":%.2f,\"mid\":%d,\"road-wide\":%d}",
         pwm1_duty_rps, pwm2_duty_rps, encoder_1, encoder_2, mid,Road_Wide[25]);

// 发送函数
udp_client.udp_send_string(encoder_str);
    
     // std::this_thread::yield(); // 必须加！让定时器能跑
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 加这一句
}
     std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
    return 0;
}
