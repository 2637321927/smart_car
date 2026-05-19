#include "main.hpp"
#include "lq_timer.hpp"
#include <math.h>
bool need_exit = false;
// 全局互斥锁（解决多线程冲突）
static int vofa_recv_fd = -1;
float servo_kp = 1.2f;    // 给你PID用
float servo_ki = 0.05f;
float servo_kd = 0.3f;
// 单独初始化一个只用于接收的UDP socket
void vofa_recv_init()
{
    // 新建UDP socket
    vofa_recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(vofa_recv_fd < 0) return;

    // 端口复用
    int opt = 1;
    setsockopt(vofa_recv_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定本地 8080 端口，专门收VOFA
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(8080);
    local.sin_addr.s_addr = INADDR_ANY;
    bind(vofa_recv_fd, (struct sockaddr*)&local, sizeof(local));

    // 设置非阻塞
    int flags = fcntl(vofa_recv_fd, F_GETFL, 0);
    fcntl(vofa_recv_fd, F_SETFL, flags | O_NONBLOCK);
}

// 接收 + 解析 + 自动生效
void vofa_recv_cmd(void)
{
    if (vofa_recv_fd < 0) return;

    char buf[128] = {0};
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    int ret = recvfrom(vofa_recv_fd, buf, sizeof(buf)-1,
                       MSG_DONTWAIT,
                       (struct sockaddr*)&src_addr, &addr_len);

    if (ret <= 0)
    {
        // 无数据，直接返回
        return;
    }

    // ==================== 打印收到的指令 ====================
    printf("[VOFA] RX: %s\n", buf);

    // ==================== 解析指令 ====================
    float ftmp = 0;

    if (sscanf(buf, "#thres=%f;", &ftmp) == 1)
    {
        thres = ftmp;
        printf("[VOFA] thres = %.1f\n", thres);
    }
    if (sscanf(buf, "#aim=%f;", &ftmp) == 1)
    {
        aim_distance = ftmp;
        printf("[VOFA] aim = %.2f\n", aim_distance);
    }
    if (sscanf(buf, "#blur=%f;", &ftmp) == 1)
    {
        line_blur_kernel = ftmp;
        printf("[VOFA] blur = %.1f\n", line_blur_kernel);
    }
    if (sscanf(buf, "#Kp=%f;", &ftmp) == 1)
    {
        servo_kp = ftmp;
        printf("[VOFA] Kp = %.3f\n", servo_kp);
    }
    if (sscanf(buf, "#Ki=%f;", &ftmp) == 1)
    {
        servo_ki = ftmp;
        printf("[VOFA] Ki = %.3f\n", servo_ki);
    }
    if (sscanf(buf, "#Kd=%f;", &ftmp) == 1)
    {
        servo_kd = ftmp;
        printf("[VOFA] Kd = %.3f\n", servo_kd);
    }
        if (sscanf(buf, "#miny=%f;", &ftmp) == 1)
    {
        track_min_y= int(ftmp);
        printf("[VOFA] miny = %.3f\n", servo_kd);
    }
        if (sscanf(buf, "#maxy=%f;", &ftmp) == 1)
    {
        track_max_y = int(ftmp);
        printf("[VOFA] maxy = %.3f\n", servo_kd);
    }
}
//std::mutex g_mutex;
//begin to test timer
lq_timer speed_timer;
lq_timer dir_timer;
lq_timer encoder_ave_timer;
volatile  int pwm1_duty_rps=0;
 volatile  int pwm2_duty_rps=0;
 volatile  int latest_error = 0;
 volatile  float encoder_1=0;
 volatile  float encoder_2=0;
 volatile  float P1_motor=0;
 volatile  float P2_motor=0;
  volatile  float D1_motor=0;
 volatile  float D2_motor=0;
 volatile float alpha_flit = 0.0f;   // 可调，0.7~0.85都可以先试
 volatile float encoder1_speed_avg = 0.0f;
volatile float encoder2_speed_avg = 0.0f;//demo for encoder ave
ls_atim_pwm pwm2(ATIM_PWM0_PIN81, 17000, 0);
ls_atim_pwm pwm1(ATIM_PWM1_PIN82, 17000, 0); 
ls_gpio polar_pwm1(PIN_21, GPIO_MODE_OUT);
ls_gpio polar_pwm2(PIN_22, GPIO_MODE_OUT);
ls_encoder_pwm enc2(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc1(ENC_PWM1_PIN65, PIN_73);
 volatile int set_speed_of_motor1_rps=0;
 volatile int set_speed_of_motor2_rps=0;
lq_udp_client udp_client;
lq_udp_client udp_client_img;
lq_udp_client udp_client_img2;
lq_udp_client udp_receive;
volatile int test_count = 0;
// 摄像头参数
const std::string TARGET_IP    = "192.168.43.213";
const uint8_t     JPEG_QUALITY = 60;
const uint16_t    CAM_WIDTH    = 320;     // 宽vofa_receive(lq_udp_client &udp)
const uint16_t    CAM_HEIGHT   = 240;     // 高
const uint16_t    CAM_FPS      = 60;     // 帧率
static struct termios old_tio;
    lq_camera cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
 volatile  int mid;

 // 安全函数：把inf/nan变成0，不破坏JSON
float safe_float(float val) {
    return (isnan(val) || isinf(val)) ? 0.0f : val;
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
static int vofa_recv_fd = -1;
float servo_kp = 1.2f;    // 给你PID用
float servo_ki = 0.05f;
float servo_kd = 0.3f;
// 单独初始化一个只用于接收的UDP socket
void vofa_recv_init()
{
    // 新建UDP socket
    vofa_recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(vofa_recv_fd < 0) {
        printf("asddasdasdasd");
        return;
    }

    // 端口复用
    int opt = 1;
    setsockopt(vofa_recv_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定本地 8080 端口，专门收VOFA
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(8082);
    local.sin_addr.s_addr = INADDR_ANY;
    bind(vofa_recv_fd, (struct sockaddr*)&local, sizeof(local));

    // 设置非阻塞
    int flags = fcntl(vofa_recv_fd, F_GETFL, 0);
    fcntl(vofa_recv_fd, F_SETFL, flags | O_NONBLOCK);
}
// 接收 + 解析 + 自动生效
void vofa_recv_cmd(void)
{
    if (vofa_recv_fd < 0) return;
  
    char buf[128] = {0};
    struct sockaddr_in src_addr;
    socklen_t addr_len = sizeof(src_addr);

    int ret = recvfrom(vofa_recv_fd, buf, sizeof(buf)-1,
                       MSG_DONTWAIT,
                       (struct sockaddr*)&src_addr, &addr_len);

    if (ret <= 0)
    {
        // 无数据，直接返回
        return;
    }

    // ==================== 打印收到的指令 ====================
    printf("[VOFA] RX: %s\n", buf);

    // ==================== 解析指令 ====================
    float ftmp = 0;

   if (sscanf(buf, "#P=%f;", &ftmp) == 1)
{
    P = ftmp;
    printf("[VOFA] P = %.3f\n", P);
}

if (sscanf(buf, "#D=%f;", &ftmp) == 1)
{
    D = ftmp;
    printf("[VOFA] D = %.3f\n", D);
}

if (sscanf(buf, "#alpha=%f;", &ftmp) == 1)
{
    alpha_flit = ftmp;
    printf("[VOFA] alpha = %.3f\n", alpha_flit);
}

if (sscanf(buf, "#spd=%d;", &set_speed_of_motor1_rps) == 1)
{
    printf("[VOFA] spd = %d\n", set_speed_of_motor1_rps);
}

}
// 全局变量，保存原来的终端模式
void encoder_sample_1ms_thread()
{
    static float buf1[5] = {0};
    static float buf2[5] = {0};
    static int idx = 0;
    static float sum1 = 0.0f;
    static float sum2 = 0.0f;

    while (1)
    {
        float s1 = -enc1.encoder_get_count();
        float s2 = enc2.encoder_get_count();

        // 减掉即将被覆盖的旧值
        sum1 -= buf1[idx];
        sum2 -= buf2[idx];

        // 存入新采样
        buf1[idx] = s1;
        buf2[idx] = s2;

        // 加上新值
        sum1 += buf1[idx];
        sum2 += buf2[idx];

        // 移动索引
        idx = (idx + 1) % 5;

        // 计算平均
        encoder1_speed_avg = sum1 / 5.0f;
        encoder2_speed_avg = sum2 / 5.0f;

        usleep(1000);  // 1ms 采样周期
    }
}
int main()
{
   std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等线程就绪
input_speed_rps();
//test polor

start_camera();
save_per_map();

//set_terminal_nonblock();

vofa_recv_init();


   encoder_ave_timer.set_seconds_ms(1, []() {
     encoder_sample_1ms_thread();
    });

   speed_timer.set_seconds_ms(5, []() {
     test_enc_and_motor_rps();   
     //   test_count++;   
    });

    dir_timer.set_seconds_ms(10, []() {
      PID_control_test(latest_error);   // 直接调用你封装好的方向函数
    });
     img_line.width = IMG_W;
    img_line.height = IMG_H;
    img_line.data = new uint8_t[img_line.width * img_line.height];
    img_line.step = img_line.width;
  
while (1)
{
    
         if (has_input()) {
            char c = getchar();
            if (c == 'q') {
                std::cout<<"quit"<<std::endl;
                cut();
                 while (getchar() != EOF); 
                break;
            } 
        }
           vofa_recv_cmd()   ;
// std::lock_guard<std::mutex> lock(g_mutex);
 //cv::Mat frame = cam.get_raw_frame();
//latest_error=img_test(frame);
//std::cout<<"fuck you"<<std::endl;
// 正确写法：字符串单独闭合，变量写在外面，逗号分隔
encoder_1=-enc1.encoder_get_count();// enc1 always gets a negative number 
encoder_2=enc2.encoder_get_count();
char encoder_str[256];
snprintf(encoder_str, sizeof(encoder_str),
         "{\"encoder1_speed_avg\":%.2f,\"encoder2_speed_avg\":%.2f,\"latest_error\":%d,\"ex_rps1\":%d,\"ex_rps2\":%d,\"encoder_1\":%.2f,\"encoder_2\":%.2f,\"mid\":%d,\"road-wide\":%d,\"P1_motor\":%.2f,\"P2_motor\":%.2f,\"D1_motor\":%.2f,\"D2_motor\":%.2f}",
         safe_float(encoder1_speed_avg), safe_float(encoder2_speed_avg),latest_error, pwm1_duty_rps, pwm2_duty_rps, safe_float(encoder_1), safe_float(encoder_2), mid,Road_Wide[25], safe_float(P1_motor),    // 🔥 关键：修复这四个非法值
         safe_float(P2_motor),    
         safe_float(D1_motor),    
         safe_float(D2_motor));

    // 2. 一次性发送：数据 + 帧尾
    udp_client.udp_send((const void*)data, sizeof(data));          // 发N个浮点数
    udp_client.udp_send((const void*)VOFA_FRAME_TAIL, 4);         // 发帧尾
    */
//vofa_receive(udp_receive);
// 发送函数

udp_client.udp_send_string(encoder_str);
               clear_image(&img_line);
            // 绘制道路元素
            //draw_yroad();
            //draw_circle();
            draw_cross();


        // 绘制道路线            
            for (int i = 0; i < rpts0s_num; i++) {
                AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
                         clip(rpts0s[i][1], 0, img_line.height - 1)) = 255;
            }
            for (int i = 0; i < rpts1s_num; i++) {
                AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
                         clip(rpts1s[i][1], 0, img_line.height - 1)) = 255;
            }
            for (int i = 0; i < rptsn_num; i++) {
                AT_IMAGE(&img_line, clip(rptsn[i][0], 0, img_line.width - 1),
                         clip(rptsn[i][1], 0, img_line.height - 1)) = 255;
            }
            // 绘制锚点
            int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
            draw_x(&img_line, rptsn[aim_idx][0], rptsn[aim_idx][1], 3, 255);
            // 绘制角点
            if (Lpt0_found) {
                draw_x(&img_line, rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], 3, 255);
            }
            if (Lpt1_found) {
                draw_x(&img_line, rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], 3, 255);
            
        
        }///*
       
         cv::Mat draw_mat(img_line.height, img_line.width, CV_8UC1, img_line.data);
        ssize_t sent = udp_client_img.udp_send_image(draw_mat, JPEG_QUALITY);
        if (sent < 0) {
            printf("ERROR: Failed to send image\r\n");
        }
          //  */
            // 1. 把 OpenCV 灰度图转成彩色图，才能画彩色线
 
///*
    cv::Mat color_frame;
    cv::cvtColor(frame, color_frame, cv::COLOR_GRAY2BGR);

// 画左边线（原图）→ 蓝色
for (int i = 0; i < ipts0_num; i++) {
    int x = ipts0[i][0];  // 原图X
    int y = ipts0[i][1];  // 原图Y
    if (x >= 0 && x < 320 && y >=0 && y < 240) {
        color_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
    }
}

// 画右边线（原图）→ 绿色s
for (int i = 0; i < ipts1_num; i++) {
    int x = ipts1[i][0];
    int y = ipts1[i][1];
    if (x >= 0 && x < 320 && y >=0 && y < 240) {
        color_frame.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
    }
}
char angle_text[64];
// 把角度保留1位小数，拼成字符串
sprintf(angle_text, "Angle: %.1f deg", error);  

// 在图像左上角画出白色文字
cv::putText(
    color_frame,          // 要画的图（彩色原图）
    angle_text,           // 文字内容
    cv::Point(10, 30),    // 文字位置（x=10, y=30）
    cv::FONT_HERSHEY_SIMPLEX,  // 字体
    0.8,                  // 字体大小
    cv::Scalar(255,255,255),    // 颜色：白色
    1                    // 线条粗细
);
    // 5. 发送给上位机
    udp_client_img2.udp_send_image(color_frame, JPEG_QUALITY);
   
    
    //*/
     // std::this_thread::yield(); // 必须加！让定时器能跑
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 加这一句
}

     std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
    return 0;
}
