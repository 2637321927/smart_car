#include "main.hpp"
#include "lq_timer.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
bool need_exit = false;

//begin to test timer   
lq_timer base_timer;
volatile bool flag_5ms = false;
volatile bool flag_10ms = false;
int latest_error = 0;

ls_atim_pwm pwm2(ATIM_PWM0_PIN81, 200, 0);
ls_atim_pwm pwm1(ATIM_PWM1_PIN82, 200, 0); 
ls_encoder_pwm enc2(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc1(ENC_PWM1_PIN65, PIN_73);

int set_speed_of_motor1_rps=0;
int set_speed_of_motor2_rps=0;

lq_udp_client udp_client;
// 摄像头参数
const uint16_t    CAM_WIDTH    = 160;     // 宽
const uint16_t    CAM_HEIGHT   = 120;     // 高
const uint16_t    CAM_FPS      = 120;     // 帧率
static struct termios old_tio;
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
int mid;
std::atomic<bool> cam_thread_running{true};
cv::Mat global_frame;  // 全局图像
std::mutex frame_mutex; // 互斥锁，保护图像

void timer_tick()
{
    static int tick = 0;
    tick++;

    if (tick % 5 == 0)  flag_5ms = true;
    if (tick % 10 == 0) flag_10ms = true;

    if (tick >= 1000) tick = 0;//in case of sprinng
}//timer

// ===================== 【摄像头独立线程】 =====================
void camera_thread_func()
{
    while (cam_thread_running)
    {
        cv::Mat frame=cam.get_raw_frame();
        if (!frame.empty())
        {
            // 加锁 → 安全更新全局图像
            std::lock_guard<std::mutex> lock(frame_mutex);
            global_frame = frame.clone();
        }
        // 摄像头频率 30fps 足够
       // std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}


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

void image_thread_func()
{
    while (1)
    {
       

       

    }
}



int main()
{


input_speed_rps(set_speed_of_motor1_rps,set_speed_of_motor2_rps);
start_camera();
set_terminal_nonblock();

 base_timer.set_seconds_ms(1, timer_tick);//start kicker
std::thread cam_thread(camera_thread_func);
    cam_thread.detach();
/*
   speed_timer.set_seconds_ms(5, []() {
       test_enc_and_motor_rps();      
       std::cout<<"fuck you"<<std::endl;  // 直接调用你封装好的速度函数
    });

    dir_timer.set_seconds_ms(10, []() {
        PID_control_test(latest_error);   // 直接调用你封装好的方向函数
    });
*/


while (1)
{
    /*
         if (has_input()) {
            char c = getchar();
            if (c == 'q') {
                std::cout<<"caonima"<<std::endl;
                cut();q
                 while (getchar() != EOF); 
                break;
            } 
            std::cout<<"fuck you3"<<std::endl; 
        }
            */

 if (flag_10ms)
        {
            flag_10ms = false;
          PID_control_test(latest_error); // 直接调用你封装好的方向函数
        }

if (flag_5ms)
        {
            flag_5ms = false;
           test_enc_and_motor_rps(); // 直接调用你封装好的速度函数
        }
        
        //printf("\n expected speed: %d %d \n",pwm1_duty_rps,pwm2_duty_rps);

// 【安全读取图像】
        cv::Mat display_frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!global_frame.empty())
                display_frame = global_frame.clone();
        }

        if (!display_frame.empty())
        {
            // 在这里处理画面 + 发送UDP
            // img_test() 原来的逻辑，改用 display_frame
             img_test(display_frame);
        }

char encoder_str[64];
     /*
        snprintf(encoder_str, sizeof(encoder_str),
         "{\"ex_rps1\":%d,\"ex_rps2\":%d,\"rps1\":%f,\"rpd2\":%f,\"mid\":%d}\n",
         pwm1_duty_rps, pwm2_duty_rps, encoder_1, encoder_2, mid);
        // 发送编码器数据
        udp_client.udp_send_string(encoder_str);
*/

/*printf("expected speed :%d:%d\n speed %f  %f\n", 
       pwm1_duty_rps,
       pwm2_duty_rps,
       encoder_1,
       encoder_2);*/

}
     std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
    return 0;
}
