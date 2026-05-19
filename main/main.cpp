#include "main.hpp"
#include "lq_timer.hpp"
#include <math.h>
#include "img.hpp"
#include "circle.hpp"
bool need_exit = false;
// 全局互斥锁（解决多线程冲突）
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
  volatile  float I1_motor=0;
 volatile  float I2_motor=0;
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
volatile int test_count = 0;

 cv::Rect red_block_rect;   // 红色标记块外接矩形
 cv::Rect plate_rect;       // 目标板区域矩形
 bool have_target = false;

 image_t img_raw;
 image_t img0;
 image_t img_thres;
image_t img_line;
cv::Mat M = (cv::Mat_<float>(3, 3) <<
    -1.926069941510014, -3.371753034082217, 468.5182139603127,
    0.01304919847285211, -6.564635749345294, 571.4708741748802,
    0.0001131073803662276, -0.02128472622767542, 1);

cv::Mat M_Reverse = (cv::Mat_<float>(3, 3) <<
    -0.5213095398517877, 0.6145633088870834, -106.9620168336765,
    -0.004803300317385806, 0.1842669595441624, -103.0527667664332,
    -4.327297583230408e-05, 0.00385256014076622, -1.181351734105273);

// Auxiliary calibration values
// ground_width_m = 0.6
#define M2PIX 274.7333333333333 // 米转像素

 bool line_show_sample;
bool line_show_blur;
 bool track_left;

float angle;
 float mapx[IMG_H][IMG_W];
float mapy[IMG_H][IMG_W];

float thres = 95;            // 固定二值化阈值（判断黑线/背景）
float block_size = 7;         // 自适应阈值的窗口大小
float clip_value = 2;         // 自适应阈值减去的偏移量
float track_min_y = 70;   // 越小 → 巡得越远
float track_max_y = 238;  // 越大 → 巡到最底部
float begin_x = 40;           // 巡线起始点 水平偏移
float begin_y = 238;          // 巡线起始点 垂直位置（靠近车底）
float line_blur_kernel = 5;   // 边线滤波平滑程度7-5
float pixel_per_meter = M2PIX;  // 像素 → 实际距离换算比例
float sample_dist = 0.02;     // 点集等距采样步长（米）
float angle_dist = 0.2;       // 计算弯道角度的窗口长度
float far_rate = 0.5;         // 远处点权重（控制转向柔和度）
float aim_distance = 0.50;    // 目标点距离车身多远（米）
// 原图左右边线
int ipts0[POINTS_MAX_LEN][2];
 int ipts1[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num;
// 变换后左右边线
float rpts0[POINTS_MAX_LEN][2];
 float rpts1[POINTS_MAX_LEN][2];
 int rpts0_num, rpts1_num;
// 变换后左右边线+滤波
float rpts0b[POINTS_MAX_LEN][2];
 float rpts1b[POINTS_MAX_LEN][2];
 int rpts0b_num, rpts1b_num;
// 变换后左右边线+等距采样
float rpts0s[POINTS_MAX_LEN][2];
 float rpts1s[POINTS_MAX_LEN][2];
 int rpts0s_num, rpts1s_num;
// 左右边线局部角度变化率
 float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
 int rpts0a_num, rpts1a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
 int rpts0an_num, rpts1an_num;
// 左/右中线
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
 int rptsc0_num, rptsc1_num;
// 中线
float (*rpts)[2];
 int rpts_num;
// 归一化中线
 float rptsn[POINTS_MAX_LEN][2];
 int rptsn_num;

// Y角点
 int Ypt0_rpts0s_id, Ypt1_rpts1s_id;
bool Ypt0_found, Ypt1_found;

// L角点
 int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
bool Lpt0_found, Lpt1_found;



bool is_straight0, is_straight1;


 enum track_type_e track_type;
 // 保存映射
void save_per_map(void) {

    // 计算透视映射
    for (int y = 0; y < IMG_H; y++) {
        for (int x = 0; x < IMG_W; x++) {   
            float U = M.at<float>(0, 0) * x + M.at<float>(0, 1) * y + M.at<float>(0, 2);
            float V = M.at<float>(1, 0) * x + M.at<float>(1, 1) * y + M.at<float>(1, 2);
            float W = M.at<float>(2, 0) * x + M.at<float>(2, 1) * y + M.at<float>(2, 2);
            
            // 避免除以零
            if (W != 0.0f) {
                mapx[y][x] = U / W;
                mapy[y][x] = V / W;
            } 
            else {
                mapx[y][x] = 0.0f;
                mapy[y][x] = 0.0f;
            }
        }
    }
    
}
// 摄像头参数
const uint16_t    CAM_WIDTH    = 320;     // 宽
const uint16_t    CAM_HEIGHT   = 240;     // 高
const uint16_t    CAM_FPS      = 60;     // 帧率
const uint8_t     JPEG_QUALITY = 100;
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
 float error=0;
start_camera();
save_per_map();
     img_line.width = IMG_W;
    img_line.height = IMG_H;
    img_line.data = new uint8_t[img_line.width * img_line.height];

//set_terminal_nonblock();

vofa_recv_init();


   encoder_ave_timer.set_seconds_ms(1, []() {
     encoder_sample_1ms_thread();
    });

   speed_timer.set_seconds_ms(5, []() {
     test_enc_and_motor_rps();   
     //   test_count++;   
     //  std::cout<<"fuck you"<<std::endl;  // 直接调用你封装好的速度函数
    });

    dir_timer.set_seconds_ms(10, []() {
      PID_control_test(latest_error);   // 直接调用你封装好的方向函数
    });
//std::cout<<"fuck you2"s<<std::endl; 
  
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
           vofa_recv_cmd()   ;
// std::lock_guard<std::mutex> lock(g_mutex);
 //cv::Mat frame = cam.get_raw_frame();
//latest_error=img_test(frame);
 cv::Mat frame = cam.get_raw_frame();
       cv::flip(frame, frame, -1); //颠倒上下左右
 // 检测红色块和目标板
 //detectRedPlate(frame);

 // 如果找到了，就在原图上画框
 /*if (have_target)
 {
     cv::Mat aframe=frame;
     // 红色块：画红色框
     cv::rectangle(aframe, red_block_rect, cv::Scalar(0, 0, 255), 2);
     // 目标板区域：画青蓝色框
     cv::rectangle(aframe, plate_rect, cv::Scalar(255, 255, 0), 2);
     cv::imshow("AAA", aframe);
     cv::waitKey(1);
 }
 */
 cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        if (frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            continue;
        }
       // cv::flip(frame, frame, -1);  
        // 等待摄像头采集完毕
img_raw.data = frame.data;
img_raw.width = cam.get_width();
img_raw.height = cam.get_height();
img_raw.step=frame.step;
img0.data = frame.data;
img0.width = cam.get_width();
img0.height = cam.get_height();
img0.step=frame.step;
        // 开始处理摄像头图像
        process_image();    // 边线提取&处理
        find_corners();     // 角点提取&筛选

        // 预瞄距离,动态效果更佳
        aim_distance = 0.15;

        // 单侧线少，切换巡线方向  切外向圆
        if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60) {
            track_type = TRACK_RIGHT;
        } else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60) {
            track_type = TRACK_LEFT;
        } else if (rpts0s_num < 20 && rpts1s_num > rpts0s_num) {
            track_type = TRACK_RIGHT;
        } else if (rpts1s_num < 20 && rpts0s_num > rpts1s_num) {
            track_type = TRACK_LEFT;
        }


   
        // 分别检查十字 三叉 和圆环, 十字优先级最高
            check_cross();
        if (cross_type == CROSS_NONE)
            check_circle();
        if (cross_type != CROSS_NONE) {
            circle_type = CIRCLE_NONE;
        }

        //车库 ,十字清Aprltag标志
        //if (garage_type != GARAGE_NONE || cross_type != CROSS_NONE) apriltag_type = APRILTAG_NONE;

        //根据检查结果执行模式
        //if (yroad_type != YROAD_NONE) run_yroad();
        if (cross_type != CROSS_NONE) run_cross();
      if (circle_type != CIRCLE_NONE) run_circle();
       // if (garage_type != GARAGE_NONE) run_garage();

        // 中线跟踪
        ///*
        if (cross_type != CROSS_IN) {
            if (track_type == TRACK_LEFT) {
                rpts = rptsc0;
                rpts_num = rptsc0_num;
            } else {
                rpts = rptsc1;
                rpts_num = rptsc1_num;
            }
        } else {
            //十字根据远线控制
            if (track_type == TRACK_LEFT) {
                track_leftline(far_rpts0s + far_Lpt0_rpts0s_id, far_rpts0s_num - far_Lpt0_rpts0s_id, rpts,
                               (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
                rpts_num = far_rpts0s_num - far_Lpt0_rpts0s_id;
            } else {
                track_rightline(far_rpts1s + far_Lpt1_rpts1s_id, far_rpts1s_num - far_Lpt1_rpts1s_id, rpts,
                                (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
                rpts_num = far_rpts1s_num - far_Lpt1_rpts1s_id;
            }
        }
      //  */
        // 车轮对应点(纯跟踪起始点)
        float cx = mapx[(int) (IMG_H * 0.78f)][IMG_W / 2];
        float cy = mapy[(int) (IMG_H * 0.78f)][IMG_W / 2];

        // 找最近点(起始点中线归一化)
        float min_dist = 1e10;
        int begin_id = -1;
        for (int i = 0; i < rpts_num; i++) {
            float dx = rpts[i][0] - cx;
            float dy = rpts[i][1] - cy;
            float dist = sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                begin_id = i;
            }
        }

        // 特殊模式下，不找最近点(由于边线会绕一圈回来，导致最近点为边线最后一个点，从而中线无法正常生成)
      //  if (garage_type == GARAGE_IN_LEFT || garage_type == GARAGE_IN_RIGHT || cross_type == CROSS_IN) begin_id = 0;

        // 中线有点，同时最近点不是最后几个点
        if (begin_id >= 0 && rpts_num - begin_id >= 3) {
            // 归一化中线，如果是根据左右track寻仙则需要这么干
            rpts[begin_id][0] = cx;
            rpts[begin_id][1] = cy;
           rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
           resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);

            // 远预锚点位置
            int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
            // 近预锚点位置
            int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);

            // 计算远锚点偏差值
            float dx = rptsn[aim_idx][0] - cx;
            float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
            float dn = sqrt(dx * dx + dy * dy);
            error = -atan2f(dx, dy) * 180 / PI;
            assert(!isnan(error));

            // 若考虑近点远点,可近似构造Stanley算法,避免撞路肩
            // 计算近锚点偏差值
            float dx_near = rptsn[aim_idx_near][0] - cx;
            float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
            float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
            float error_near = -atan2f(dx_near, dy_near) * 180 / PI;
            assert(!isnan(error_near));


        }
latest_error=-error;
               clear_image(&img_line);
               cv::Mat birdview;

// 1. 直接用 OpenCV 官方 warpPerspective → 绝对正确！
cv::warpPerspective(frame, birdview, M, cv::Size(IMG_W, IMG_H));


// 2. 转彩色，用于画彩色线
cv::Mat bgr_bird;
cv::cvtColor(birdview, bgr_bird, cv::COLOR_GRAY2BGR);

// 3. 在鸟瞰图上画：左线(蓝)、右线(绿)、中线(白)
for (int i = 0; i < rpts0s_num; i++) {
    int x = cvRound(rpts0s[i][0]);
    int y = cvRound(rpts0s[i][1]);
    if (x >= 0 && x < IMG_W && y >= 0 && y < IMG_H)
        bgr_bird.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
}
for (int i = 0; i < rpts1s_num; i++) {
    int x = cvRound(rpts1s[i][0]);
    int y = cvRound(rpts1s[i][1]);
    if (x >= 0 && x < IMG_W && y >= 0 && y < IMG_H)
        bgr_bird.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
}
for (int i = 0; i < rptsn_num; i++) {
    int x = cvRound(rptsn[i][0]);
    int y = cvRound(rptsn[i][1]);
    if (x >= 0 && x < IMG_W && y >= 0 && y < IMG_H)
        bgr_bird.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
}

// 4. 画预瞄点（红色 X）
 int aim_idx = clip(cvRound(aim_distance / sample_dist), 0, rptsn_num - 1);
if (rptsn_num > 0) {
    int x = cvRound(rptsn[aim_idx][0]);
    int y = cvRound(rptsn[aim_idx][1]);
    cv::drawMarker(bgr_bird, cv::Point(x, y), cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 10, 2);
}

// 5. 画角点
if (Lpt0_found) {
    int x = cvRound(rpts0s[Lpt0_rpts0s_id][0]);
    int y = cvRound(rpts0s[Lpt0_rpts0s_id][1]);
    cv::drawMarker(bgr_bird, cv::Point(x, y), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 8, 2);
}
if (Lpt1_found) {
    int x = cvRound(rpts1s[Lpt1_rpts1s_id][0]);
    int y = cvRound(rpts1s[Lpt1_rpts1s_id][1]);
    cv::drawMarker(bgr_bird, cv::Point(x, y), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 8, 2);
}

// 6. 显示角度
char text[64];
sprintf(text, "Angle: %.1f", error);
cv::putText(bgr_bird, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

// 7. 显示最终鸟瞰图（就是你要的效果）
cv::resize(bgr_bird, bgr_bird, cv::Size(320, 240));
//std::cout<<"fuck you"<<std::endl;
// 正确写法：字符串单独闭合，变量写在外面，逗号分隔
encoder_1=-enc1.encoder_get_count();// enc1 always gets a negative number 
encoder_2=enc2.encoder_get_count();
char encoder_str[256];
snprintf(encoder_str, sizeof(encoder_str),
         "{\"encoder1_speed_avg\":%.2f,\"encoder2_speed_avg\":%.2f,\"latest_error\":%d,\"ex_rps1\":%d,\"ex_rps2\":%d,\"encoder_1\":%.2f,\"encoder_2\":%.2f,\"mid\":%d,\"road-wide\":%d,\"P1_motor\":%.2f,\"P2_motor\":%.2f,\"D1_motor\":%.2f,\"D2_motor\":%.2f}",
         safe_float(encoder1_speed_avg), safe_float(encoder2_speed_avg),latest_error, pwm1_duty_rps, pwm2_duty_rps, safe_float(encoder_1), safe_float(encoder_2), mid,Road_Wide[25], safe_float(P1_motor),    // 🔥 关键：修复这四个非法值
         safe_float(P2_motor),    
         safe_float(I1_motor),    
         safe_float(I2_motor));

// 发送函数
udp_client.udp_send_string(encoder_str);
 ssize_t sent =    udp_client_img.udp_send_image(bgr_bird, JPEG_QUALITY);
  if (sent < 0) {
          printf("ERROR: Failed to send image\r\n");
      }
     // std::this_thread::yield(); // 必须加！让定时器能跑
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 加这一句
}
     std::cout<<"caonissma"<<std::endl;
     reset_terminal(); // 必须恢复终端！
     std::cout<<"caonimssa"<<std::endl;
    return 0;
}