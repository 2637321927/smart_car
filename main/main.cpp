#include "main.hpp"
#include "lq_timer.hpp"
#include <math.h>
#include "img.hpp"
#include "circle.hpp"
#include "control_profile.hpp"
#include "front_ui.hpp"  // TFT18 屏幕 + 实体按键前端
#include "hardware_test.hpp"  // 停车态PWM1单通道硬件测试
#include "drive_by.hpp"  // 目标板触发后的固定动作脚本
#include "gyro_yaw_rate_control.hpp"  // MPU6050 角速度环 demo
#include "odometry.hpp"              // 编码器里程计（环岛出环距离）
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
using namespace std::chrono;
volatile float AIM =0.25;
int item_flag=1;
bool need_exit = false;
extern volatile float circle_exit_distance_m;  // 环岛出环距离阈值（定义在 circle.cpp）
// 全局互斥锁（解决多线程冲突）
//std::mutex g_mutex;
//begin to test timer
lq_timer speed_timer;
lq_timer dir_timer;
lq_timer encoder_ave_timer;
lq_timer udp_timer;
lq_timer gyro_watchdog_timer;
volatile  float pwm1_duty_rps=0.0f;
 volatile  float pwm2_duty_rps=0.0f;
 volatile  float latest_error = 0;
 volatile int vision_y_guard_active = 0;
 volatile int vision_y_guard_turn_sign = 0;
 volatile float vision_y_guard_target_dps = 500.0f;
 volatile float vision_y_guard_base_rps = 15.0f;
 volatile float vision_y_guard_turn_max_rps = 25.0f;
 volatile float vision_y_guard_aim_dy_px = 0.0f;
 volatile  float encoder_1=0;
 volatile  float encoder_2=0;
 volatile  float P1_motor=0;
 volatile  float P2_motor=0;
  volatile  float I1_motor=0;
 volatile  float I2_motor=0;
 volatile float alpha_flit = 0.0f;   // 可调，0.7~0.85都可以先试
 volatile float encoder1_speed_avg = 0.0f;
volatile float encoder2_speed_avg = 0.0f;//demo for encoder ave
int L_count=0;
int R_count=0;
// ====================== 误差滤波 全局变量 ======================
volatile float error_filtered = 0.0f;    // 滤波后误差
volatile float error_prev = 0.0f;        // 上一帧误差
const float filter_alpha = 0.8f;        // 滤波系数
const float max_delta = 80.0f;           // 每帧最大变化量
const float max_error = 120.0f;          // 最大误差限幅
// 误差滤波 + 突变限制
float filter_error(float new_error)
{
    // 1. 非法值保护
    if (isnan(new_error) || isinf(new_error)) {
        new_error = error_prev;
    }

    // 2. 限制突变：变化太大就只让它变一点点
    float delta = new_error - error_prev;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    new_error = error_prev + delta;

    // 3. 最大误差限幅
    if (new_error > max_error) new_error = max_error;
    if (new_error < -max_error) new_error = -max_error;

    // 4. 一阶低通滤波
    float filtered = filter_alpha * new_error + (1 - filter_alpha) * error_prev;

    // 保存上一帧
    error_prev = filtered;

    return filtered;
}

namespace {

constexpr float kVisionYGuardExitMarginPx = 20.0f;
constexpr int kZebraDetectionFrameInterval = 3;
void camera_performance_set_enabled(bool enabled);

void update_vision_y_guard(float raw_visual_error,
                           float aim_y,
                           float car_y,
                           bool line_lost,
                           bool center_line_valid,
                           int selected_aim_line)
{
    vision_y_guard_aim_dy_px = aim_y - car_y;

    // 救车只属于普通中线巡线。丢线、十字、环岛和绕行脚本都有自己的控制策略，
    // 若在这些状态叠加救车角速度，会让两个控制器争抢左右轮目标。
    const bool eligible = front_ui_is_running() &&
        selected_aim_line == 0 && center_line_valid && !line_lost &&
        cross_type == CROSS_NONE && circle_type == CIRCLE_NONE &&
        !drive_by_is_busy() && gyro_manual_target_enabled == 0 &&
        vision_y_guard_target_dps > 0.0f;
    if (!eligible) {
        vision_y_guard_active = 0;
        return;
    }

    if (vision_y_guard_active != 0) {
        // 进入后增加20px退出迟滞，避免瞄点在车体水平线附近抖动时反复开关。
        if (aim_y < car_y - kVisionYGuardExitMarginPx) {
            vision_y_guard_active = 0;
        }
        return;
    }

    if (aim_y < car_y) {
        // 瞄点仍在车前时持续记录可靠转向方向；一旦瞄点绕到车后，
        // 不能再用异常dx实时改符号，否则救车方向可能在相邻帧翻转。
        if (std::fabs(raw_visual_error) >= 1.0f) {
            vision_y_guard_turn_sign = raw_visual_error > 0.0f ? 1 : -1;
        }
        return;
    }

    int rescue_sign = vision_y_guard_turn_sign;
    if (rescue_sign == 0 && std::fabs((float)latest_error) >= 1.0f) {
        rescue_sign = latest_error > 0.0f ? 1 : -1;
    }
    if (rescue_sign == 0 && std::fabs(raw_visual_error) >= 1.0f) {
        rescue_sign = raw_visual_error > 0.0f ? 1 : -1;
    }
    if (rescue_sign != 0) {
        vision_y_guard_turn_sign = rescue_sign;
        vision_y_guard_active = 1;
    }
}

} // namespace
ls_atim_pwm pwm1(ATIM_PWM1_PIN82 ,17000, 0);
ls_atim_pwm pwm2(ATIM_PWM0_PIN81, 17000, 0); //2026/7/9teset
ls_gpio polar_pwm1(PIN_22, GPIO_MODE_OUT);
ls_gpio polar_pwm2(PIN_21, GPIO_MODE_OUT);//2026/7/9teset
ls_encoder_pwm enc2(ENC_PWM0_PIN64, PIN_72);
ls_encoder_pwm enc1(ENC_PWM1_PIN65, PIN_73);
 volatile float set_speed_of_motor1_rps=0.0f;
 volatile float set_speed_of_motor2_rps=0.0f;
lq_udp_client udp_client;
lq_udp_client udp_client_debugger;
lq_udp_client udp_client_img;
// Debugger默认发送参数波形和道路三线；传统VOFA波形回传默认关闭，避免重复占用带宽。
volatile int udp_debug_mode = 2;
volatile int vofa_telemetry_enabled = 0;
volatile int camera_performance_enabled = 0;
volatile unsigned long long udp_control_send_fail_total = 0;
volatile unsigned long long udp_road_send_fail_total = 0;
cv::Mat bgr_bird;
volatile int test_count = 0;
enum AvoidState
{
    AV_NORMAL,    // 正常巡线
    AV_GO_LEFT,   // 左绕(weapon)
    AV_GO_RIGHT   // 右绕(supplies)
};
int diu=0;
volatile AvoidState g_avoid_state = AV_NORMAL;
volatile int avoid_tick = 0;       // 计时计数器(基于主循环帧)
const int AVOID_FRAME_MAX = 160;    // 绕行总帧数(根据实际车速调)
 cv::Rect red_block_rect;   // 红色标记块外接矩形
 cv::Rect plate_rect;       // 目标板区域矩形
 bool have_target = false;
 int red_contour_area = 0;  // 当前严格红块检测得到的最大轮廓面积
 image_t img_raw;
 image_t img0;
 image_t img_thres;
image_t img_line;
cv::Mat M = (cv::Mat_<float>(3, 3) <<
    -1.320275258607639, -2.345812341903703, 369.2520228321467,
    0.02423637491927372, -4.119243269269744, 349.2685954256243,
    0.0001659173364317823, -0.01495167667279194, 1);

cv::Mat M_Reverse = (cv::Mat_<float>(3, 3) <<
    -0.7785225176134412, 2.241262449658709, -495.3315733233524,
    -0.02379763154276893, 0.9752042762116768, -331.8209042197335,
    -0.0002266441099311285, 0.01420907473194914, -3.879094777870544);

// Auxiliary calibration values
// ground_width_m = 0.6
#define M2PIX 206.5 // 米转像素
 bool line_show_sample;
bool line_show_blur;
 bool track_left;

float angle;
 float mapx[IMG_H][IMG_W];
float mapy[IMG_H][IMG_W];

int thres = 100;            // 固定二值化阈值（判断黑线/背景）
int block_size = 7;         // 自适应阈值的窗口大小
int clip_value = 2;         // 自适应阈值减去的偏移量
int track_min_y = 86;   // ԽС �� Ѳ��ԽԶ
int track_max_y = 180;  // Խ�� �� Ѳ����ײ�
int begin_x = 40;           // Ѳ����ʼ�� ˮƽƫ��
int begin_y = 180;          // Ѳ����ʼ�� ��ֱλ�ã��������ף�
int end_y=100;        // 巡线起始点 垂直位置（靠近车底）
int line_blur_kernel = 5;   // 边线滤波平滑程度7-5
float pixel_per_meter = M2PIX;  // 像素 → 实际距离换算比例
float sample_dist = 0.02;     // 点集等距采样步长（米）
float angle_dist = 0.2;       // 计算弯道角度的窗口长度
float far_rate = 0.5;         // 远处点权重（控制转向柔和度）
float aim_distance = 0.50;    // 目标点距离车身多远（米）
// 原图左右边线
int ipts0[POINTS_MAX_LEN][2];
 int ipts1[POINTS_MAX_LEN][2];
 int ipts2[POINTS_MAX_LEN][2];
int ipts0_num, ipts1_num,ipts2_num;
// 变换后左右边线
float rpts0[POINTS_MAX_LEN][2];
 float rpts1[POINTS_MAX_LEN][2];
 float rpts2[POINTS_MAX_LEN][2];
 int rpts0_num, rpts1_num,rpts2_num;
// 变换后左右边线+滤波
float rpts0b[POINTS_MAX_LEN][2];
 float rpts1b[POINTS_MAX_LEN][2];
 float rpts2b[POINTS_MAX_LEN][2];
 int rpts0b_num, rpts1b_num,rpts2b_num;
// 变换后左右边线+等距采样
float rpts0s[POINTS_MAX_LEN][2];
 float rpts1s[POINTS_MAX_LEN][2];
 float rpts2s[POINTS_MAX_LEN][2];
 int rpts0s_num, rpts1s_num,rpts2s_num;
// 左右边线局部角度变化率
 float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
float rpts2a[POINTS_MAX_LEN];
 int rpts0a_num, rpts1a_num,rpts2a_num;
// 左右边线局部角度变化率+非极大抑制
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
float rpts2an[POINTS_MAX_LEN];
 int rpts0an_num, rpts1an_num,rpts2an_num;
// 左/右中线
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
float rptsc2[POINTS_MAX_LEN][2];
int rptsc0_num, rptsc1_num,rptsc2_num;
// 中线
float (*rpts)[2];
 int rpts_num;
// 边线瞄准方案不能直接把rpts指向rpts0s/rpts1s，因为后面的起点归一化
// 会改写rpts[begin_id]。复制到临时数组可避免污染下一步仍要使用的原始边线。
float drive_by_aim_line_points[POINTS_MAX_LEN][2];
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
  enum lost_e lost;
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
const uint8_t     JPEG_QUALITY = 60;
static struct termios old_tio;
    lq_camera_ex cam(CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
 volatile  int mid;
 int is_udp_img=0;
 // 安全函数：把inf/nan变成0，不破坏JSON
float safe_float(float val) {
    return (isnan(val) || isinf(val)) ? 0.0f : val;
}

namespace {
constexpr const char *kGyroI2cAdapterDev = "/dev/i2c-1";
constexpr int kGyroI2cTimeout10msUnits = 2; // I2C_TIMEOUT uses 10 ms units.
constexpr int kGyroI2cRetries = 0;

void configure_gyro_i2c_adapter_timeout()
{
    int fd = open(kGyroI2cAdapterDev, O_RDWR);
    if (fd < 0) {
        printf("[I2C] skip adapter timeout tuning: open %s failed\n", kGyroI2cAdapterDev);
        return;
    }

    // This does not slow SCL down. It only asks the Linux I2C adapter to give
    // up a bad transfer sooner, so one MPU6050 fault is less likely to cost 2s.
    if (ioctl(fd, I2C_RETRIES, kGyroI2cRetries) < 0) {
        printf("[I2C] set retries failed, keep system default\n");
    }
    if (ioctl(fd, I2C_TIMEOUT, kGyroI2cTimeout10msUnits) < 0) {
        printf("[I2C] set timeout failed, keep system default\n");
    } else {
        printf("[I2C] %s timeout set to about %dms, retries=%d\n",
               kGyroI2cAdapterDev,
               kGyroI2cTimeout10msUnits * 10,
               kGyroI2cRetries);
    }

    close(fd);
}
} // namespace

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

// 判断：有没有按键输入
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
    int itmp = 0;

if (sscanf(buf, "#carProfile=%d;", &itmp) == 1)
{
    const ControlProfileMode requested = itmp == 1
        ? CONTROL_PROFILE_PRO
        : CONTROL_PROFILE_STABLE;
    const bool accepted = (itmp == 0 || itmp == 1) &&
        control_profile_switch(requested);
    printf("[VOFA] carProfile request=%d result=%s current=%d speed=%.2f RPS\n",
           requested == CONTROL_PROFILE_PRO ? 1 : 0,
           accepted ? "accepted" : "rejected: car must be stopped and tests idle",
           control_profile_mode() == CONTROL_PROFILE_PRO ? 1 : 0,
           control_profile_target_speed_rps());
}

if (sscanf(buf, "#profileSpd=%f;", &ftmp) == 1)
{
    const float applied = control_profile_set_target_speed_rps(ftmp);
    if (front_ui_is_running() && !drive_by_is_busy()) {
        set_speed_of_motor1_rps = applied;
        set_speed_of_motor2_rps = applied;
    }
    printf("[VOFA] profileSpd = %.2f RPS, profile=%d\n",
           applied, control_profile_mode() == CONTROL_PROFILE_PRO ? 1 : 0);
}

   if (sscanf(buf, "#P=%f;", &ftmp) == 1)
{
    P = ftmp;
    control_profile_capture_active();
    printf("[VOFA] P = %.3f\n", P);
}

if (sscanf(buf, "#I=%f;", &ftmp) == 1)
{
    I = ftmp;
    control_profile_capture_active();
    printf("[VOFA] I = %.3f\n", I);
}

if (sscanf(buf, "#D=%f;", &ftmp) == 1)
{
    D = ftmp;
    control_profile_capture_active();
    printf("[VOFA] D = %.3f\n", D);
}

if (sscanf(buf, "#dirP=%f;", &ftmp) == 1)
{
    dir_P = ftmp;
    control_profile_capture_active();
    printf("[VOFA] dirP = %.3f\n", dir_P);
}

if (sscanf(buf, "#dirD=%f;", &ftmp) == 1)
{
    dir_D = ftmp;
    control_profile_capture_active();
    printf("[VOFA] dirD = %.3f\n", dir_D);
}



if (sscanf(buf, "#spd=%f;", &ftmp) == 1)
{
    set_speed_of_motor1_rps = ftmp;
    set_speed_of_motor2_rps=set_speed_of_motor1_rps;
    printf("[VOFA] spd = %.2f\n", set_speed_of_motor1_rps);
}

if (sscanf(buf, "#udp=%d;", &itmp) == 1)
{
    if (itmp < 0) itmp = 0;
    if (itmp > 2) itmp = 2;
    udp_debug_mode = itmp;
    const char *mode_name = udp_debug_mode == 0
        ? "关闭"
        : (udp_debug_mode == 1 ? "仅波形" : "波形和道路三线");
    printf("[VOFA] UDP调试模式=%d（%s）\n", udp_debug_mode, mode_name);
}

if (sscanf(buf, "#camStats=%d;", &itmp) == 1)
{
    camera_performance_set_enabled(itmp != 0);
    printf("[VOFA] 相机状态统计=%s\n",
           camera_performance_enabled ? "开启" : "关闭");
}

if (sscanf(buf, "#vofa=%d;", &itmp) == 1)
{
    vofa_telemetry_enabled = itmp != 0 ? 1 : 0;
    printf("[VOFA] 波形回传=%s（192.168.43.146:8080）\n",
           vofa_telemetry_enabled ? "开启" : "关闭");
}

if (sscanf(buf, "#drive=%d;", &itmp) == 1)
{
    drive_by_set_enable(itmp != 0);
}

if (sscanf(buf, "#hwTest=%d;", &itmp) == 1)
{
    const bool requested_enable = itmp != 0;
    const bool accepted = hardware_test_set_enabled(requested_enable);
    printf("[硬件测试] 请求=%s 结果=%s 当前=%s PWM1=%d PWM2=0\n",
           requested_enable ? "开启" : "关闭",
           accepted ? "成功" : "已拒绝（需run=0且绕行/遥控/航向保持均关闭）",
           hardware_test_is_enabled() ? "开启" : "关闭",
           hardware_test_get_pwm());
}

if (sscanf(buf, "#hwPwm=%d;", &itmp) == 1)
{
    hardware_test_set_pwm(itmp);
    printf("[硬件测试] PWM1正向占空比=%d（范围0-7000，PWM2=0，当前%s）\n",
           hardware_test_get_pwm(),
           hardware_test_is_enabled() ? "已输出" : "仅保存未输出");
}

// ==================== 目标板高速绕行在线调参 ====================
// 识别阶段的 dbRecSpd 只改变基准速度，普通方向环仍会继续产生左右差速。
if (sscanf(buf, "#dbMode=%d;", &itmp) == 1)
{
    drive_by_mode = 0;
    printf("[VOFA] dbMode已固定为0（三阶段脚本），边线模式已停用\n");
}

if (sscanf(buf, "#dbSideMs=%d;", &itmp) == 1)
{
    printf("[VOFA] dbSideMs已停用，绕行固定使用三阶段脚本\n");
}

if (sscanf(buf, "#dbUseTangent=%d;", &itmp) == 1)
{
    drive_by_use_track_tangent = itmp != 0 ? 1 : 0;
    printf("[VOFA] dbUseTangent = %d（%s）\n",
           drive_by_use_track_tangent,
           drive_by_use_track_tangent ? "目标处赛道切线" : "脚本启动时车头");
}

if (sscanf(buf, "#dbNormalSpd=%f;", &ftmp) == 1)
{
    drive_by_normal_speed_rps = control_profile_set_target_speed_rps(ftmp);
    if (front_ui_is_running() && !drive_by_is_busy()) {
        set_speed_of_motor1_rps = drive_by_normal_speed_rps;
        set_speed_of_motor2_rps = drive_by_normal_speed_rps;
    }
    printf("[VOFA] dbNormalSpd = %.2f RPS\n", drive_by_normal_speed_rps);
}

if (sscanf(buf, "#dbRecSpd=%f;", &ftmp) == 1)
{
    drive_by_recognition_speed_rps = ftmp < 0.0f ? 0.0f : ftmp;
    printf("[VOFA] dbRecSpd = %.2f RPS\n", drive_by_recognition_speed_rps);
}

if (sscanf(buf, "#dbDetectEvery=%d;", &itmp) == 1)
{
    drive_by_detect_frame_interval = 2;
    printf("[VOFA] 红块预检测已固定为每2帧一次\n");
}

if (sscanf(buf, "#dbTurnAngle=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_turn_angle_deg = ftmp > 90.0f ? 90.0f : ftmp;
    printf("[VOFA] dbTurnAngle = %.2f deg\n", drive_by_turn_angle_deg);
}

if (sscanf(buf, "#dbReturnBias=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_return_bias_deg = ftmp > 91.0f ? 91.0f : ftmp;
    printf("[VOFA] dbReturnBias = %.2f deg\n", drive_by_return_bias_deg);
}

if (sscanf(buf, "#dbPassDist=%f;", &ftmp) == 1)
{
    drive_by_pass_distance_m = ftmp < 0.0f ? 0.0f : ftmp;
    printf("[VOFA] dbPassDist = %.3f m\n", drive_by_pass_distance_m);
}

if (sscanf(buf, "#dbExitDist=%f;", &ftmp) == 1)
{
    printf("[VOFA] dbExitDist is deprecated and ignored\n");
}

if (sscanf(buf, "#dbSafeDist=%f;", &ftmp) == 1)
{
    drive_by_target_after_margin_m = ftmp < 0.0f ? 0.0f : ftmp;
    printf("[VOFA] dbSafeDist = %.3f m\n", drive_by_target_after_margin_m);
}

if (sscanf(buf, "#dbRpsMps=%f;", &ftmp) == 1)
{
    drive_by_rps_to_mps = ftmp < 0.0f ? 0.0f : ftmp;
    printf("[VOFA] dbRpsMps = %.5f\n", drive_by_rps_to_mps);
}

if (sscanf(buf, "#dbViewMax=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_view_angle_max_deg = ftmp > 90.0f ? 90.0f : ftmp;
    printf("[VOFA] dbViewMax = %.2f deg\n", drive_by_view_angle_max_deg);
}

if (sscanf(buf, "#dbViewWait=%d;", &itmp) == 1)
{
    if (itmp < 0) itmp = 0;
    if (itmp > 2000) itmp = 2000;
    drive_by_view_wait_timeout_ms = itmp;
    printf("[VOFA] dbViewWait = %d ms\n", drive_by_view_wait_timeout_ms);
}

if (sscanf(buf, "#dbHKp=%f;", &ftmp) == 1)
{
    drive_by_heading_kp = ftmp;
    printf("[VOFA] dbHKp = %.3f\n", drive_by_heading_kp);
}

if (sscanf(buf, "#dbHKd=%f;", &ftmp) == 1)
{
    drive_by_heading_kd = ftmp;
    printf("[VOFA] dbHKd = %.3f\n", drive_by_heading_kd);
}

if (sscanf(buf, "#dbHMax=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_heading_max_dps = ftmp > 720.0f ? 720.0f : ftmp;
    printf("[VOFA] dbHMax = %.2f dps\n", drive_by_heading_max_dps);
}

if (sscanf(buf, "#dbHTol=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = 0.0f;
    if (ftmp > 10.0f) ftmp = 10.0f;
    drive_by_heading_tolerance_deg = ftmp;
    printf("[VOFA] dbHTol = %.2f deg（静默退出阈值=%.2f deg）\n",
           drive_by_heading_tolerance_deg,
           drive_by_heading_tolerance_deg + 1.0f);
}

if (sscanf(buf, "#dbRecoverDps=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_recovery_yaw_rate_dps = ftmp > 720.0f ? 720.0f : ftmp;
    printf("[VOFA] dbRecoverDps = %.2f dps\n",
           drive_by_recovery_yaw_rate_dps);
}

if (sscanf(buf, "#dbYawSign=%f;", &ftmp) == 1)
{
    drive_by_yaw_sign = ftmp >= 0.0f ? 1.0f : -1.0f;
    printf("[VOFA] dbYawSign = %.1f\n", drive_by_yaw_sign);
}

if (sscanf(buf, "#dbTurnRps=%f;", &ftmp) == 1)
{
    drive_by_turn_speed_rps = cvRound(ftmp < 0.0f ? 0.0f : ftmp);
    printf("[VOFA] dbTurnRps = %d\n", drive_by_turn_speed_rps);
}

if (sscanf(buf, "#dbForwardRps=%f;", &ftmp) == 1)
{
    drive_by_forward_speed_rps = cvRound(ftmp < 0.0f ? 0.0f : ftmp);
    printf("[VOFA] dbForwardRps = %d\n", drive_by_forward_speed_rps);
}

if (sscanf(buf, "#dbExitRps=%f;", &ftmp) == 1)
{
    drive_by_exit_speed_rps = cvRound(ftmp < 0.0f ? 0.0f : ftmp);
    printf("[VOFA] dbExitRps = %d\n", drive_by_exit_speed_rps);
}

if (sscanf(buf, "#dbBrakePwm=%d;", &itmp) == 1)
{
    if (itmp < 0) itmp = -itmp;
    if (itmp > 9000) itmp = 9000;
    drive_by_brake_pwm = itmp;
    printf("[VOFA] dbBrakePwm = %d\n", drive_by_brake_pwm);
}

if (sscanf(buf, "#dbBrakeRelease=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    if (ftmp > 200.0f) ftmp = 200.0f;
    drive_by_brake_release_rps = ftmp;
    printf("[VOFA] dbBrakeRelease = %.2f RPS\n", drive_by_brake_release_rps);
}

if (sscanf(buf, "#dbBrakeTimeout=%d;", &itmp) == 1)
{
    if (itmp < 1) itmp = 1;
    if (itmp > 2000) itmp = 2000;
    drive_by_brake_timeout_ms = itmp;
    printf("[VOFA] dbBrakeTimeout = %d ms\n", drive_by_brake_timeout_ms);
}

if (sscanf(buf, "#dbEarlyBrake=%d;", &itmp) == 1)
{
    drive_by_stable_early_brake_set_enable(itmp != 0);
    printf("[VOFA] 稳定组首次候选立即制动=%s（PRO模式始终开启）\n",
           drive_by_stable_early_brake_is_enabled() ? "开启" : "关闭");
}

if (sscanf(buf, "#dbTestDist=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = 0.0f;
    if (ftmp > 5.0f) ftmp = 5.0f;
    drive_by_test_target_distance_m = ftmp;
    printf("[VOFA] dbTestDist = %.3f m\n", drive_by_test_target_distance_m);
}

if (sscanf(buf, "#test_driveby=%d;", &itmp) == 1)
{
    const bool started = drive_by_start_test(
        itmp, drive_by_test_target_distance_m);
    printf("[VOFA] TEST绕行：方向=%s 距离=%.3fm 结果=%s\n",
           itmp == 0 ? "左绕" : (itmp == 2 ? "右绕" : "无效"),
           drive_by_test_target_distance_m,
           started ? "已启动" : "已拒绝（需先发车、方向为0/2且脚本空闲）");
}

if (sscanf(buf, "#test_brake=%d;", &itmp) == 1)
{
    const bool requested_enable = itmp != 0;
    const bool accepted = (!requested_enable || !hardware_test_is_enabled()) &&
        drive_by_brake_test_set_enable(requested_enable);
    printf("[VOFA] 刹车测试：请求=%s 结果=%s 当前=%s\n",
           requested_enable ? "开启" : "关闭",
           accepted ? "成功" : "已拒绝（绕行/航向保持/遥控/硬件测试正在占用）",
           drive_by_brake_test_is_enabled() ? "开启" : "关闭");
}

if (sscanf(buf, "#yawHold=%d;", &itmp) == 1)
{
    const bool enabled = !hardware_test_is_enabled() &&
        drive_by_heading_hold_set_enable(itmp != 0);
    printf("[VOFA] 航向保持测试：请求=%s 结果=%s\n",
           itmp != 0 ? "开启" : "关闭",
           enabled ? "成功" : "已拒绝（需run=0、脚本/遥控空闲且陀螺仪数据新鲜）");
}

if (sscanf(buf, "#yawHoldRMax=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    drive_by_heading_hold_max_turn_rps = ftmp > 40.0f ? 40.0f : ftmp;
    printf("[VOFA] yawHoldRMax = %.2f RPS\n",
           drive_by_heading_hold_max_turn_rps);
}

if (sscanf(buf, "#tangentDbg=%d;", &itmp) == 1)
{
    drive_by_tangent_debug_set_enable(itmp != 0);
    printf("[VOFA] 中线切线显示 = %s\n", itmp != 0 ? "开启" : "关闭");
}

// 环岛出环里程阈值，单位米。进入 RUNNING 态后累计行驶超过该距离则触发出环。
if (sscanf(buf, "#circle_exit=%f;", &ftmp) == 1)
{
    if (ftmp < 0.10f) ftmp = 0.10f;
    if (ftmp > 3.00f) ftmp = 3.00f;
    circle_exit_distance_m = ftmp;
    printf("[VOFA] circle_exit_distance_m = %.3f m\n", circle_exit_distance_m);
}

// VOFA command example: #AIM=0.30;
// AIM is the forward look-ahead distance in meters. The image-processing loop
// copies it to aim_distance before selecting the tracking target point.
if (sscanf(buf, "#AIM=%f;", &ftmp) == 1)
{
    AIM = ftmp;
    control_profile_capture_active();
    printf("[VOFA] AIM = %.3f m\n", AIM);
}

if (sscanf(buf, "#spd_slow_ratio=%f;", &ftmp) == 1)
{
    
    spd_slow_ratio = ftmp;
    control_profile_capture_active();
    printf("[VOFA] spd_slow_ratio = %d\n", spd_slow_ratio);
}

// ==================== 角速度环在线调参 ====================
// 推荐调参顺序：
// 1. 角速度反馈固定开启；旧#gyro命令只报告固定状态，不再切换视觉PD。
// 2. #gSign=-1;    如果手动右转时 gyro_z 正负反了，改它。
// 3. #tSign=-1;    如果一闭环就越修越偏，改它。
// 4. 先调 #gIP，再慢慢加 #gII。新手阶段 #gII 可以先设 0。
// 5. #gTMax=160;   限制外环最大目标角速度，等价 #gyro_target_max_dps=160;
// 6. #gRMax=15;    限制内环最大差速输出，等价 #gyro_turn_max_rps=15;
if (sscanf(buf, "#run=%f;", &ftmp) == 1)
{
    front_ui_set_running(ftmp != 0.0f);
    printf("[VOFA] run = %d\n", front_ui_is_running() ? 1 : 0);
}

if (sscanf(buf, "#remote=%d;", &itmp) == 1)
{
    // 0停、1前、2后、3左、4右。前端按住期间会周期续发；这里禁止逐包打印，
    // 避免100ms遥控心跳给行车线程制造额外终端I/O负担。
    front_ui_remote_set(itmp);
}

if (sscanf(buf, "#gyro=%f;", &ftmp) == 1)
{
    gyro_yaw_rate_feedback_enabled = 1;
    gyro_yaw_rate_control_reset();
    printf("[VOFA] 角速度反馈已固定开启，ready=%d\n",
           gyro_yaw_rate_control_is_ready() ? 1 : 0);
}

if (sscanf(buf, "#gDbg=%f;", &ftmp) == 1)
{
    gyro_manual_target_enabled = (ftmp != 0.0f) ? 1 : 0;
    if (gyro_manual_target_enabled) {
        gyro_yaw_rate_feedback_enabled = 1;
    }
    gyro_yaw_rate_control_reset();
    printf("[VOFA] gyro_manual_target_enabled = %d, target = %.2f dps, gyro = %d\n",
           gyro_manual_target_enabled,
           gyro_manual_target_dps,
           gyro_yaw_rate_feedback_enabled);
}

if (sscanf(buf, "#gTar=%f;", &ftmp) == 1)
{
    gyro_manual_target_dps = ftmp;
    gyro_yaw_rate_control_reset();
    printf("[VOFA] gyro_manual_target_dps = %.2f\n", gyro_manual_target_dps);
}

if (sscanf(buf, "#gOP=%f;", &ftmp) == 1)
{
    gyro_outer_kp = ftmp;
    control_profile_capture_active();
    printf("[VOFA] gyro_outer_kp = %.3f\n", gyro_outer_kp);
}

if (sscanf(buf, "#gOD=%f;", &ftmp) == 1)
{
    gyro_outer_kd = ftmp;
    control_profile_capture_active();
    printf("[VOFA] gyro_outer_kd = %.3f\n", gyro_outer_kd);
}

if (sscanf(buf, "#gIP=%f;", &ftmp) == 1)
{
    gyro_inner_kp = ftmp;
    control_profile_capture_active();
    printf("[VOFA] gyro_inner_kp = %.3f\n", gyro_inner_kp);
}

if (sscanf(buf, "#gII=%f;", &ftmp) == 1)
{
    gyro_inner_ki = ftmp;
    control_profile_capture_active();
    gyro_yaw_rate_control_reset();
    printf("[VOFA] gyro_inner_ki = %.3f\n", gyro_inner_ki);
}

if (sscanf(buf, "#gTMax=%f;", &ftmp) == 1 ||
    sscanf(buf, "#gyro_target_max_dps=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    gyro_target_max_dps = ftmp;
    control_profile_capture_active();
    printf("[VOFA] gyro_target_max_dps = %.2f\n", gyro_target_max_dps);
}

if (sscanf(buf, "#gRMax=%f;", &ftmp) == 1 ||
    sscanf(buf, "#gyro_turn_max_rps=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    gyro_turn_max_rps = ftmp;
    control_profile_capture_active();
    printf("[VOFA] gyro_turn_max_rps = %.2f\n", gyro_turn_max_rps);
}

if (sscanf(buf, "#yGuardDps=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    if (ftmp > 700.0f) ftmp = 700.0f;
    vision_y_guard_target_dps = ftmp;
    if (vision_y_guard_target_dps <= 0.0f) {
        vision_y_guard_active = 0;
    }
    control_profile_capture_active();
    printf("[VOFA] 瞄点越界救车角速度 = %.2f dps\n",
           vision_y_guard_target_dps);
}

if (sscanf(buf, "#yGuardBase=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = 0.0f;
    if (ftmp > 100.0f) ftmp = 100.0f;
    vision_y_guard_base_rps = ftmp;
    control_profile_capture_active();
    printf("[VOFA] yGuardBase = %.2f RPS\n", vision_y_guard_base_rps);
}

if (sscanf(buf, "#yGuardRMax=%f;", &ftmp) == 1)
{
    if (ftmp < 0.0f) ftmp = -ftmp;
    if (ftmp > 60.0f) ftmp = 60.0f;
    vision_y_guard_turn_max_rps = ftmp;
    control_profile_capture_active();
    printf("[VOFA] yGuardRMax = %.2f RPS\n", vision_y_guard_turn_max_rps);
}

if (sscanf(buf, "#gSign=%f;", &ftmp) == 1)
{
    gyro_z_sign = (ftmp >= 0.0f) ? 1.0f : -1.0f;
    gyro_yaw_rate_control_reset();
    printf("[VOFA] gyro_z_sign = %.1f\n", gyro_z_sign);
}

if (sscanf(buf, "#tSign=%f;", &ftmp) == 1)
{
    gyro_turn_sign = (ftmp >= 0.0f) ? 1.0f : -1.0f;
    gyro_yaw_rate_control_reset();
    printf("[VOFA] gyro_turn_sign = %.1f\n", gyro_turn_sign);
}

if (sscanf(buf, "#begin_x=%f;", &ftmp) == 1)
{
    
     begin_x = ftmp;
    printf("[VOFA]  begin_x = %d\n",  begin_x);
}
if (sscanf(buf, "#is_udp_img=%f;", &ftmp) == 1)
{
    
     is_udp_img = ftmp;
    printf("[VOFA]  is_udp_img = %d\n",  is_udp_img);
}

}
// 全局变量，保存原来的终端模式
void encoder_sample_1ms_thread()
{
    // 当前由 2ms 定时器调用。两点滑动平均能降低采样线程压力，
    // 同时避免 2ms 周期下继续用三点平均造成速度反馈过慢。
    static float buf1[2] = {0};
    static float buf2[2] = {0};
    static int idx = 0;
    static float sum1 = 0.0f;
    static float sum2 = 0.0f;

 //   while (1)
 //   {
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
        idx = (idx + 1) % 2;

        // 计算平均
        encoder1_speed_avg = sum1 / 2.0f;
        encoder2_speed_avg = sum2 / 2.0f;

        //usleep(2000);  // 2ms 采样周期
   // }
}

namespace {

constexpr int kRoadTelemetryMaxPoints = 64;
// RDL1 v1原头部为52字节。新增的4字节切线锚点位于尾部，服务端仍接受旧52字节包。
constexpr uint16_t kRoadTelemetryHeaderSize = 56;
constexpr int kRoadTelemetryMinIntervalMs = 20;
constexpr uint8_t kControlTelemetryVersion = 1;
constexpr uint8_t kControlTelemetryHeaderSize = 16;
constexpr int kControlTelemetryIntCount = 16;
constexpr int kControlTelemetryFloatCount = 33;
constexpr int kControlTelemetryPacketSize = kControlTelemetryHeaderSize +
    kControlTelemetryIntCount * 4 + kControlTelemetryFloatCount * 4;
// 可调参数变化远慢于控制波形。单独低频发送参数快照，既能让前端和录像
// 精确还原调参状态，也不会继续膨胀已经较大的 12ms 动态 JSON 数据包。
constexpr int kTuningTelemetryIntervalMs = 250;
constexpr float kCameraProcessOverrunMs = 25.0f;
constexpr int kCameraPerformanceWindowMs = 1000;

struct CameraPerformanceSnapshot
{
    volatile float last_ms;
    volatile float average_ms;
    volatile float max_ms;
    volatile float fps;
    volatile unsigned long long overrun_count;
};

CameraPerformanceSnapshot g_camera_performance = {};
steady_clock::time_point g_camera_performance_window_start = steady_clock::now();
double g_camera_performance_sum_ms = 0.0;
float g_camera_performance_window_max_ms = 0.0f;
int g_camera_performance_frame_count = 0;
bool g_camera_performance_started = false;

void camera_performance_set_enabled(bool enabled)
{
    camera_performance_enabled = enabled ? 1 : 0;
    g_camera_performance = CameraPerformanceSnapshot{};
    g_camera_performance_window_start = steady_clock::now();
    g_camera_performance_sum_ms = 0.0;
    g_camera_performance_window_max_ms = 0.0f;
    g_camera_performance_frame_count = 0;
    g_camera_performance_started = false;
}

void record_camera_frame_performance(steady_clock::time_point frame_start)
{
    if (camera_performance_enabled == 0) {
        return;
    }
    const steady_clock::time_point now = steady_clock::now();
    const float used_ms = static_cast<float>(
        duration<double, std::milli>(now - frame_start).count());
    g_camera_performance.last_ms = used_ms;
    g_camera_performance_sum_ms += used_ms;
    g_camera_performance_window_max_ms = std::max(
        g_camera_performance_window_max_ms, used_ms);
    ++g_camera_performance_frame_count;
    if (used_ms > kCameraProcessOverrunMs) {
        ++g_camera_performance.overrun_count;
    }

    if (!g_camera_performance_started) {
        g_camera_performance_window_start = frame_start;
        g_camera_performance_started = true;
    }
    const double window_ms = duration<double, std::milli>(
        now - g_camera_performance_window_start).count();
    if (window_ms < kCameraPerformanceWindowMs) {
        return;
    }

    if (g_camera_performance_frame_count > 0) {
        g_camera_performance.average_ms = static_cast<float>(
            g_camera_performance_sum_ms /
            static_cast<double>(g_camera_performance_frame_count));
        g_camera_performance.max_ms = g_camera_performance_window_max_ms;
        g_camera_performance.fps = static_cast<float>(
            static_cast<double>(g_camera_performance_frame_count) * 1000.0 /
            window_ms);
    }
    g_camera_performance_sum_ms = 0.0;
    g_camera_performance_window_max_ms = 0.0f;
    g_camera_performance_frame_count = 0;
    g_camera_performance_window_start = now;
}

void telemetry_write_u16(uint8_t *&cursor, uint16_t value)
{
    *cursor++ = static_cast<uint8_t>(value & 0xff);
    *cursor++ = static_cast<uint8_t>((value >> 8) & 0xff);
}

void telemetry_write_i16(uint8_t *&cursor, int value)
{
    if (value < -32768) value = -32768;
    if (value > 32767) value = 32767;
    telemetry_write_u16(cursor, static_cast<uint16_t>(static_cast<int16_t>(value)));
}

void telemetry_write_u32(uint8_t *&cursor, uint32_t value)
{
    *cursor++ = static_cast<uint8_t>(value & 0xff);
    *cursor++ = static_cast<uint8_t>((value >> 8) & 0xff);
    *cursor++ = static_cast<uint8_t>((value >> 16) & 0xff);
    *cursor++ = static_cast<uint8_t>((value >> 24) & 0xff);
}

void telemetry_write_i32(uint8_t *&cursor, int32_t value)
{
    telemetry_write_u32(cursor, static_cast<uint32_t>(value));
}

void telemetry_write_f32(uint8_t *&cursor, float value)
{
    uint32_t bits = 0;
    const float safe_value = safe_float(value);
    std::memcpy(&bits, &safe_value, sizeof(bits));
    telemetry_write_u32(cursor, bits);
}

int telemetry_point_count(int source_count)
{
    if (source_count <= 0) {
        return 0;
    }
    return source_count < kRoadTelemetryMaxPoints
        ? source_count
        : kRoadTelemetryMaxPoints;
}

void telemetry_write_line(uint8_t *&cursor,
                          float points[][2],
                          int source_count,
                          int output_count)
{
    for (int index = 0; index < output_count; ++index) {
        const int source_index = output_count <= 1
            ? 0
            : index * (source_count - 1) / (output_count - 1);
        telemetry_write_i16(cursor, cvRound(points[source_index][0]));
        telemetry_write_i16(cursor, cvRound(points[source_index][1]));
    }
}

void road_telemetry_send()
{
    if (udp_debug_mode < 2) {
        return;
    }

    static uint32_t sequence = 0;
    static const steady_clock::time_point started_at = steady_clock::now();
    static steady_clock::time_point last_sent =
        steady_clock::now() - milliseconds(kRoadTelemetryMinIntervalMs);

    const steady_clock::time_point now = steady_clock::now();
    if (duration_cast<milliseconds>(now - last_sent).count() <
        kRoadTelemetryMinIntervalMs) {
        return;
    }
    last_sent = now;

    const int left_count = telemetry_point_count(rpts0s_num);
    const int center_count = telemetry_point_count(rptsn_num);
    const int right_count = telemetry_point_count(rpts1s_num);
    const DriveByTangentDebug &tangent_debug = drive_by_tangent_debug_get();

    uint8_t flags = 0;
    if (have_target) flags |= 1u << 0;
    if (red_block_rect.width > 0 && red_block_rect.height > 0) flags |= 1u << 1;
    if (plate_rect.width > 0 && plate_rect.height > 0) flags |= 1u << 2;
    if (front_ui_is_running()) flags |= 1u << 3;
    if (drive_by_is_busy()) flags |= 1u << 4;
    if (tangent_debug.enabled && tangent_debug.valid) flags |= 1u << 5;

    int aim_x = -1;
    int aim_y = -1;
    if (rptsn_num > 0) {
        const int aim_index = clip(cvRound(aim_distance / sample_dist), 0, rptsn_num - 1);
        aim_x = cvRound(rptsn[aim_index][0]);
        aim_y = cvRound(rptsn[aim_index][1]);
    }

    uint8_t packet[kRoadTelemetryHeaderSize + kRoadTelemetryMaxPoints * 3 * 4] = {};
    uint8_t *cursor = packet;
    *cursor++ = 'R';
    *cursor++ = 'D';
    *cursor++ = 'L';
    *cursor++ = '1';
    *cursor++ = 1;
    *cursor++ = flags;
    telemetry_write_u16(cursor, kRoadTelemetryHeaderSize);
    telemetry_write_u32(cursor, ++sequence);
    telemetry_write_u32(cursor, static_cast<uint32_t>(
        duration_cast<milliseconds>(now - started_at).count()));
    telemetry_write_u16(cursor, IMG_W);
    telemetry_write_u16(cursor, IMG_H);
    *cursor++ = static_cast<uint8_t>(left_count);
    *cursor++ = static_cast<uint8_t>(center_count);
    *cursor++ = static_cast<uint8_t>(right_count);
    *cursor++ = static_cast<uint8_t>(item_flag & 0xff);
    telemetry_write_i16(cursor, red_block_rect.x);
    telemetry_write_i16(cursor, red_block_rect.y);
    telemetry_write_i16(cursor, red_block_rect.width);
    telemetry_write_i16(cursor, red_block_rect.height);
    telemetry_write_i16(cursor, plate_rect.x);
    telemetry_write_i16(cursor, plate_rect.y);
    telemetry_write_i16(cursor, plate_rect.width);
    telemetry_write_i16(cursor, plate_rect.height);
    telemetry_write_i16(cursor, aim_x);
    telemetry_write_i16(cursor, aim_y);
    telemetry_write_u16(cursor, static_cast<uint16_t>(rpts0s_num));
    telemetry_write_u16(cursor, static_cast<uint16_t>(rptsn_num));
    telemetry_write_u16(cursor, static_cast<uint16_t>(rpts1s_num));
    telemetry_write_i16(cursor, tangent_debug.valid
        ? cvRound(tangent_debug.angle_deg * 100.0f)
        : 0);
    telemetry_write_i16(cursor, tangent_debug.valid
        ? tangent_debug.anchor_x
        : -1);
    telemetry_write_i16(cursor, tangent_debug.valid
        ? tangent_debug.anchor_y
        : -1);

    telemetry_write_line(cursor, rpts0s, rpts0s_num, left_count);
    telemetry_write_line(cursor, rptsn, rptsn_num, center_count);
    telemetry_write_line(cursor, rpts1s, rpts1s_num, right_count);

    if (udp_client_debugger.udp_send(
            packet, static_cast<size_t>(cursor - packet)) < 0) {
        ++udp_road_send_fail_total;
    }
}

void tuning_telemetry_send()
{
    if (udp_debug_mode < 1) {
        return;
    }

    static steady_clock::time_point last_sent =
        steady_clock::now() - milliseconds(kTuningTelemetryIntervalMs);
    const steady_clock::time_point now = steady_clock::now();
    if (duration_cast<milliseconds>(now - last_sent).count() <
        kTuningTelemetryIntervalMs) {
        return;
    }
    last_sent = now;

    char tuning_str[2048];
    int tuning_length = snprintf(
        tuning_str,
        sizeof(tuning_str),
        "{"
        "\"packet_type\":\"tuning\","
        "\"carProfile\":%d,\"profileSpd\":%.2f,"
        "\"P\":%.3f,\"I\":%.3f,\"D\":%.3f,"
        "\"dirP\":%.4f,\"dirD\":%.4f,\"AIM\":%.3f,"
        "\"spd_slow_ratio\":%d,\"begin_x\":%d,"
        "\"gDbg\":%d,\"gTar\":%.2f,"
        "\"gOP\":%.4f,\"gOD\":%.4f,\"gIP\":%.4f,\"gII\":%.4f,"
        "\"gTMax\":%.2f,\"gRMax\":%.2f,\"yGuardDps\":%.2f,"
        "\"yGuardBase\":%.2f,\"yGuardRMax\":%.2f,"
        "\"gSign\":%.1f,\"tSign\":%.1f,"
        "\"dbUseTangent\":%d,"
        "\"dbNormalSpd\":%.2f,\"dbRecSpd\":%.2f,"
        "\"dbTurnAngle\":%.2f,\"dbReturnBias\":%.2f,\"dbPassDist\":%.3f,"
        "\"dbSafeDist\":%.3f,\"dbRpsMps\":%.5f,"
        "\"dbViewMax\":%.2f,\"dbViewWait\":%d,"
        "\"dbHKp\":%.3f,\"dbHKd\":%.3f,\"dbHMax\":%.2f,"
        "\"dbHTol\":%.2f,"
        "\"dbRecoverDps\":%.2f,\"dbYawSign\":%.1f,"
        "\"yawHoldRMax\":%.2f,"
        "\"dbTurnRps\":%d,\"dbForwardRps\":%d,\"dbExitRps\":%d,"
        "\"dbBrakePwm\":%d,\"dbBrakeRelease\":%.2f,"
        "\"dbBrakeTimeout\":%d,\"dbEarlyBrake\":%d,\"dbTestDist\":%.3f,"
        "\"circle_exit\":%.3f,\"udp\":%d,\"vofa\":%d,\"is_udp_img\":%d,"
        "\"hwTest\":%d,\"hwPwm\":%d,"
        "\"camStats\":%d,"
        "\"udp_control_send_fail_total\":%llu,"
        "\"udp_road_send_fail_total\":%llu",
        control_profile_mode() == CONTROL_PROFILE_PRO ? 1 : 0,
        safe_float(drive_by_normal_speed_rps),
        safe_float(P), safe_float(I), safe_float(D),
        safe_float(dir_P), safe_float(dir_D), safe_float(AIM),
        spd_slow_ratio, begin_x,
        gyro_manual_target_enabled,
        safe_float(gyro_manual_target_dps),
        safe_float(gyro_outer_kp), safe_float(gyro_outer_kd),
        safe_float(gyro_inner_kp), safe_float(gyro_inner_ki),
        safe_float(gyro_target_max_dps), safe_float(gyro_turn_max_rps),
        safe_float(vision_y_guard_target_dps),
        safe_float(vision_y_guard_base_rps),
        safe_float(vision_y_guard_turn_max_rps),
        safe_float(gyro_z_sign), safe_float(gyro_turn_sign),
        drive_by_use_track_tangent,
        safe_float(drive_by_normal_speed_rps),
        safe_float(drive_by_recognition_speed_rps),
        safe_float(drive_by_turn_angle_deg),
        safe_float(drive_by_return_bias_deg),
        safe_float(drive_by_pass_distance_m),
        safe_float(drive_by_target_after_margin_m),
        safe_float(drive_by_rps_to_mps),
        safe_float(drive_by_view_angle_max_deg),
        drive_by_view_wait_timeout_ms,
        safe_float(drive_by_heading_kp), safe_float(drive_by_heading_kd),
        safe_float(drive_by_heading_max_dps),
        safe_float(drive_by_heading_tolerance_deg),
        safe_float(drive_by_recovery_yaw_rate_dps),
        safe_float(drive_by_yaw_sign),
        safe_float(drive_by_heading_hold_max_turn_rps),
        drive_by_turn_speed_rps, drive_by_forward_speed_rps,
        drive_by_exit_speed_rps, drive_by_brake_pwm,
        safe_float(drive_by_brake_release_rps),
        drive_by_brake_timeout_ms,
        drive_by_stable_early_brake_is_enabled() ? 1 : 0,
        safe_float(drive_by_test_target_distance_m),
        safe_float(circle_exit_distance_m),
        udp_debug_mode, vofa_telemetry_enabled, is_udp_img,
        hardware_test_is_enabled() ? 1 : 0, hardware_test_get_pwm(),
        camera_performance_enabled,
        (unsigned long long)udp_control_send_fail_total,
        (unsigned long long)udp_road_send_fail_total);

    if (tuning_length > 0 &&
        tuning_length < static_cast<int>(sizeof(tuning_str))) {
        tuning_length += snprintf(
            tuning_str + tuning_length,
            sizeof(tuning_str) - static_cast<size_t>(tuning_length),
            "}");
    }

    if (tuning_length > 0 &&
        tuning_length < static_cast<int>(sizeof(tuning_str))) {
        if (udp_client_debugger.udp_send(
                tuning_str, static_cast<size_t>(tuning_length)) < 0) {
            ++udp_control_send_fail_total;
        }
    }

    // 相机统计默认关闭。开启时使用独立的小型低频包，避免把完整调参快照
    // 推过1000字节目标；PC端会把camera增量合并到最近一次调参快照。
    if (camera_performance_enabled != 0) {
        char camera_str[256];
        const int camera_length = snprintf(
            camera_str,
            sizeof(camera_str),
            "{\"packet_type\":\"camera\","
            "\"camera_fps\":%.2f,\"camera_process_last_ms\":%.2f,"
            "\"camera_process_avg_ms\":%.2f,\"camera_process_max_ms\":%.2f,"
            "\"camera_process_overrun_count\":%llu}",
            safe_float(g_camera_performance.fps),
            safe_float(g_camera_performance.last_ms),
            safe_float(g_camera_performance.average_ms),
            safe_float(g_camera_performance.max_ms),
            (unsigned long long)g_camera_performance.overrun_count);
        if (camera_length > 0 &&
            camera_length < static_cast<int>(sizeof(camera_str)) &&
            udp_client_debugger.udp_send(
                camera_str, static_cast<size_t>(camera_length)) < 0) {
            ++udp_control_send_fail_total;
        }
    }
}

void control_telemetry_send()
{
    if (udp_debug_mode < 1) {
        return;
    }

    static_assert(kControlTelemetryPacketSize < 256,
                  "CTL1 must stay below the planned 256-byte limit");
    static const steady_clock::time_point started_at = steady_clock::now();
    const steady_clock::time_point now = steady_clock::now();
    const GyroYawRateDebug &gyro_debug = gyro_yaw_rate_control_get_debug();
    const DriveByDebug &drive_debug = drive_by_get_debug();
    const DriveByHeadingHoldDebug &heading_hold_debug =
        drive_by_heading_hold_get_debug();
    const DriveByTangentDebug &tangent_debug = drive_by_tangent_debug_get();
    const bool heading_hold_enabled = drive_by_heading_hold_is_enabled();
    lq_timer_timeout_snapshot timeout_debug = {};
    lq_timer_timeout_get_snapshot(&timeout_debug);

    uint16_t flags0 = 0;
    if (vision_y_guard_active != 0) flags0 |= 1u << 0;
    if (front_ui_is_running()) flags0 |= 1u << 1;
    if (heading_hold_enabled) flags0 |= 1u << 2;
    if (drive_by_tangent_debug_is_enabled()) flags0 |= 1u << 3;
    if (tangent_debug.valid) flags0 |= 1u << 4;
    if (drive_by_is_enabled()) flags0 |= 1u << 5;
    if (drive_by_is_busy()) flags0 |= 1u << 6;
    if (drive_by_is_recognizing()) flags0 |= 1u << 7;
    if (drive_by_is_motion_phase()) flags0 |= 1u << 8;
    if (drive_debug.test_mode != 0) flags0 |= 1u << 9;
    if (drive_by_brake_test_is_enabled()) flags0 |= 1u << 10;
    if (drive_by_brake_test_is_holding()) flags0 |= 1u << 11;
    if (drive_debug.brake_active != 0) flags0 |= 1u << 12;
    if (drive_debug.target_geometry_valid != 0) flags0 |= 1u << 13;
    if (drive_debug.view_ready != 0) flags0 |= 1u << 14;
    if (drive_debug.red_candidate != 0) flags0 |= 1u << 15;

    uint16_t flags1 = 0;
    if (have_target) flags1 |= 1u << 0;
    if (hardware_test_is_enabled()) flags1 |= 1u << 1;
    if (camera_performance_enabled != 0) flags1 |= 1u << 2;
    if (front_ui_remote_is_active()) flags1 |= 1u << 3;

    uint8_t packet[kControlTelemetryPacketSize] = {};
    uint8_t *cursor = packet;
    *cursor++ = 'C';
    *cursor++ = 'T';
    *cursor++ = 'L';
    *cursor++ = '1';
    *cursor++ = kControlTelemetryVersion;
    *cursor++ = kControlTelemetryHeaderSize;
    telemetry_write_u16(cursor, kControlTelemetryPacketSize);
    telemetry_write_u32(cursor, static_cast<uint32_t>(
        duration_cast<milliseconds>(now - started_at).count()));
    telemetry_write_u16(cursor, flags0);
    telemetry_write_u16(cursor, flags1);

    telemetry_write_i32(cursor, gyro_debug.gyro_timeout_count);
    telemetry_write_i32(cursor, timeout_debug.id);
    telemetry_write_i32(cursor, timeout_debug.total > 0x7fffffffULL
        ? 0x7fffffff : static_cast<int32_t>(timeout_debug.total));
    telemetry_write_i32(cursor, front_ui_selected_speed());
    telemetry_write_i32(cursor, drive_debug.state_code);
    telemetry_write_i32(cursor, drive_debug.abort_reason);
    telemetry_write_i32(cursor, drive_debug.brake_pwm);
    telemetry_write_i32(cursor, drive_debug.brake_elapsed_ms);
    telemetry_write_i32(cursor, drive_debug.infer_valid_count);
    telemetry_write_i32(cursor, drive_debug.red_candidate_count);
    telemetry_write_i32(cursor, drive_debug.red_contour_area);
    telemetry_write_i32(cursor, drive_debug.detection_stage);
    telemetry_write_i32(cursor, static_cast<int32_t>(circle_type));
    telemetry_write_i32(cursor, static_cast<int32_t>(cross_type));
    telemetry_write_i32(cursor, static_cast<int32_t>(track_type));
    telemetry_write_i32(cursor, item_flag);

    telemetry_write_f32(cursor, encoder1_speed_avg);
    telemetry_write_f32(cursor, encoder2_speed_avg);
    telemetry_write_f32(cursor, latest_error);
    telemetry_write_f32(cursor, pwm1_duty_rps);
    telemetry_write_f32(cursor, pwm2_duty_rps);
    telemetry_write_f32(cursor, current_pwm1 / 100.0f);
    telemetry_write_f32(cursor, current_pwm2 / 100.0f);
    telemetry_write_f32(cursor, P1_motor);
    telemetry_write_f32(cursor, P2_motor);
    telemetry_write_f32(cursor, I1_motor);
    telemetry_write_f32(cursor, I2_motor);
    telemetry_write_f32(cursor, gyro_debug.target_yaw_rate_dps);
    telemetry_write_f32(cursor, gyro_debug.gyro_z_lpf);
    telemetry_write_f32(cursor, vision_y_guard_aim_dy_px);
    telemetry_write_f32(cursor, vision_y_guard_target_dps);
    telemetry_write_f32(cursor, timeout_debug.used_ms);
    telemetry_write_f32(cursor, timeout_debug.target_ms);
    telemetry_write_f32(cursor, tangent_debug.valid ? tangent_debug.angle_deg : 0.0f);
    telemetry_write_f32(cursor, heading_hold_enabled
        ? heading_hold_debug.yaw_deg : drive_debug.yaw_deg);
    telemetry_write_f32(cursor, heading_hold_enabled
        ? heading_hold_debug.target_yaw_deg : drive_debug.target_yaw_deg);
    telemetry_write_f32(cursor, heading_hold_enabled
        ? heading_hold_debug.heading_error_deg : drive_debug.heading_error_deg);
    telemetry_write_f32(cursor, drive_debug.track_heading_deg);
    telemetry_write_f32(cursor, drive_debug.target_track_heading_deg);
    telemetry_write_f32(cursor, drive_debug.view_angle_deg);
    telemetry_write_f32(cursor, drive_debug.target_distance_m);
    telemetry_write_f32(cursor, drive_debug.distance_since_trigger_m);
    telemetry_write_f32(cursor, drive_debug.phase_distance_m);
    telemetry_write_f32(cursor, heading_hold_enabled
        ? heading_hold_debug.target_yaw_rate_dps : drive_debug.target_yaw_rate_dps);
    telemetry_write_f32(cursor, heading_hold_enabled
        ? heading_hold_debug.turn_rps : drive_debug.turn_rps);
    telemetry_write_f32(cursor, circle_exit_distance_m);
    telemetry_write_f32(cursor, odometry_get_distance());
    telemetry_write_f32(cursor, AIM);
    telemetry_write_f32(cursor, drive_by_test_target_distance_m);

    if (cursor - packet != kControlTelemetryPacketSize) {
        return;
    }
    if (udp_client_debugger.udp_send(packet, sizeof(packet)) < 0) {
        ++udp_control_send_fail_total;
    }
}

} // namespace

void udp_send(void){
    control_telemetry_send();
    if (!vofa_telemetry_enabled) {
        tuning_telemetry_send();
        return;
    }

    char encoder_str[3072];
    static uint32_t udp_sequence = 0;
    static const steady_clock::time_point udp_started_at = steady_clock::now();
    const GyroYawRateDebug &gyro_debug = gyro_yaw_rate_control_get_debug();
    const DriveByDebug &drive_debug = drive_by_get_debug();
    const DriveByHeadingHoldDebug &heading_hold_debug =
        drive_by_heading_hold_get_debug();
    const DriveByTangentDebug &tangent_debug = drive_by_tangent_debug_get();
    const bool heading_hold_enabled =
        drive_by_heading_hold_is_enabled();
    lq_timer_timeout_snapshot timeout_debug = {};
    lq_timer_timeout_get_snapshot(&timeout_debug);
    const cv::Rect red_rect = red_block_rect;
    const cv::Rect target_rect = plate_rect;
    const uint32_t uptime_ms = static_cast<uint32_t>(duration_cast<milliseconds>(
        steady_clock::now() - udp_started_at).count());

    const int json_length = snprintf(encoder_str, sizeof(encoder_str),
             "{"
             "\"seq\":%u,"
             "\"uptime_ms\":%u,"
             "\"udp_mode\":%d,"
             "\"encoder1_speed_avg\":%.2f,"
             "\"encoder2_speed_avg\":%.2f,"
             "\"latest_error\":%.2f,"
             "\"ex_rps1\":%.2f,"
             "\"ex_rps2\":%.2f,"
             "\"current_pwm1\":%d,"
             "\"current_pwm2\":%d,"
             "\"P1_motor\":%.2f,"
             "\"P2_motor\":%.2f,"
             "\"I\":%.2f,"
             "\"D1_motor\":%.2f,"
             "\"D2_motor\":%.2f,"
             "\"spd_slow_ratio\":%d,"
             "\"gyro_target_dps\":%.2f,"
             "\"gyro_dps\":%.2f,"
             "\"y_guard_active\":%d,"
             "\"y_guard_aim_dy_px\":%.2f,"
             "\"y_guard_target_dps\":%.2f,"
             "\"gyro_timeout\":%d,"
             "\"gyro_read_ms\":%.2f,"
             "\"camera_process_last_ms\":%.2f,"
             "\"camera_process_avg_ms\":%.2f,"
             "\"camera_process_max_ms\":%.2f,"
             "\"camera_fps\":%.2f,"
             "\"camera_process_overrun_count\":%llu,"
             "\"to_id\":%d,"
             "\"to_used\":%.2f,"
             "\"to_target\":%.2f,"
             "\"to_total\":%llu,"
             "\"run\":%d,"
             "\"yaw_hold_enabled\":%d,"
             "\"tangent_debug_enabled\":%d,"
             "\"track_tangent_valid\":%d,"
             "\"track_tangent_deg\":%.2f,"
             "\"selected_speed\":%d,"
             "\"drive_enabled\":%d,"
             "\"drive_busy\":%d,"
             "\"drive_state\":\"%s\","
             "\"drive_abort_reason\":\"%s\","
             "\"drive_recognizing\":%d,"
             "\"drive_motion\":%d,"
             "\"drive_test_mode\":%d,"
             "\"drive_brake_test_enabled\":%d,"
             "\"drive_brake_test_holding\":%d,"
             "\"drive_brake_active\":%d,"
             "\"drive_brake_pwm\":%d,"
             "\"drive_brake_elapsed_ms\":%d,"
             "\"drive_test_target_distance_m\":%.3f,"
             "\"drive_yaw_deg\":%.2f,"
             "\"drive_target_yaw_deg\":%.2f,"
             "\"drive_heading_error_deg\":%.2f,"
             "\"drive_track_heading_deg\":%.2f,"
             "\"drive_target_track_heading_deg\":%.2f,"
             "\"drive_view_angle_deg\":%.2f,"
             "\"drive_target_distance_m\":%.3f,"
             "\"drive_distance_since_trigger_m\":%.3f,"
             "\"drive_phase_distance_m\":%.3f,"
             "\"drive_target_yaw_rate_dps\":%.2f,"
             "\"drive_turn_rps\":%.2f,"
             "\"drive_geometry_valid\":%d,"
             "\"drive_view_ready\":%d,"
             "\"drive_infer_valid_count\":%d,"
             "\"red_candidate\":%d,"
             "\"red_candidate_count\":%d,"
             "\"red_contour_area\":%d,"
             "\"drive_detection_stage\":%d,"
             "\"have_target\":%d,"
             "\"item_flag\":%d,"
             "\"red_x\":%d,\"red_y\":%d,\"red_w\":%d,\"red_h\":%d,"
             "\"plate_x\":%d,\"plate_y\":%d,\"plate_w\":%d,\"plate_h\":%d,"
             "\"left_n\":%d,\"mid_n\":%d,\"right_n\":%d,"
             "\"circle_type\":%d,\"cross_type\":%d,\"track_type\":%d,"
             "\"circle_exit_m\":%.3f,\"odom_m\":%.3f,"
             "\"AIM\":%.3f"
             "}",
             ++udp_sequence,
             uptime_ms,
             udp_debug_mode,
             safe_float(encoder1_speed_avg),
             safe_float(encoder2_speed_avg),
             safe_float(latest_error),
             safe_float(pwm1_duty_rps),
             safe_float(pwm2_duty_rps),
             current_pwm1/100,
             current_pwm2/100,
             safe_float(P1_motor),    // 修复非法浮点值，避免 JSON 被 nan/inf 破坏。
             safe_float(P2_motor),
             safe_float(I),
             safe_float(I1_motor),
             safe_float(I2_motor),
             spd_slow_ratio,
             safe_float(gyro_debug.target_yaw_rate_dps),
             safe_float(gyro_debug.gyro_z_lpf),
             vision_y_guard_active,
             safe_float(vision_y_guard_aim_dy_px),
             safe_float(vision_y_guard_target_dps),
             gyro_debug.gyro_timeout_count,
             safe_float(gyro_debug.gyro_read_last_ms),
             safe_float(g_camera_performance.last_ms),
             safe_float(g_camera_performance.average_ms),
             safe_float(g_camera_performance.max_ms),
             safe_float(g_camera_performance.fps),
             (unsigned long long)g_camera_performance.overrun_count,
             timeout_debug.id,
             safe_float(timeout_debug.used_ms),
             safe_float(timeout_debug.target_ms),
             (unsigned long long)timeout_debug.total,
             front_ui_is_running() ? 1 : 0,
             heading_hold_enabled ? 1 : 0,
             drive_by_tangent_debug_is_enabled() ? 1 : 0,
             tangent_debug.valid,
             safe_float(tangent_debug.angle_deg),
             front_ui_selected_speed(),
             drive_by_is_enabled() ? 1 : 0,
             drive_by_is_busy() ? 1 : 0,
             heading_hold_enabled ? "HEADING_HOLD" : drive_by_state_name(),
             drive_by_abort_reason(),
             drive_by_is_recognizing() ? 1 : 0,
             drive_by_is_motion_phase() ? 1 : 0,
             drive_debug.test_mode,
             drive_by_brake_test_is_enabled() ? 1 : 0,
             drive_by_brake_test_is_holding() ? 1 : 0,
             drive_debug.brake_active,
             drive_debug.brake_pwm,
             drive_debug.brake_elapsed_ms,
             safe_float(drive_by_test_target_distance_m),
             safe_float(heading_hold_enabled
                 ? heading_hold_debug.yaw_deg : drive_debug.yaw_deg),
             safe_float(heading_hold_enabled
                 ? heading_hold_debug.target_yaw_deg : drive_debug.target_yaw_deg),
             safe_float(heading_hold_enabled
                 ? heading_hold_debug.heading_error_deg : drive_debug.heading_error_deg),
             safe_float(drive_debug.track_heading_deg),
             safe_float(drive_debug.target_track_heading_deg),
             safe_float(drive_debug.view_angle_deg),
             safe_float(drive_debug.target_distance_m),
             safe_float(drive_debug.distance_since_trigger_m),
             safe_float(drive_debug.phase_distance_m),
             safe_float(heading_hold_enabled
                 ? heading_hold_debug.target_yaw_rate_dps
                 : drive_debug.target_yaw_rate_dps),
             safe_float(heading_hold_enabled
                 ? heading_hold_debug.turn_rps : drive_debug.turn_rps),
             drive_debug.target_geometry_valid,
             drive_debug.view_ready,
             drive_debug.infer_valid_count,
             drive_debug.red_candidate,
             drive_debug.red_candidate_count,
             drive_debug.red_contour_area,
             drive_debug.detection_stage,
             have_target ? 1 : 0,
             item_flag,
             red_rect.x, red_rect.y, red_rect.width, red_rect.height,
             target_rect.x, target_rect.y, target_rect.width, target_rect.height,
             rpts0s_num, rptsn_num, rpts1s_num,
             static_cast<int>(circle_type),
             static_cast<int>(cross_type),
             static_cast<int>(track_type),
             safe_float(circle_exit_distance_m),
             safe_float(odometry_get_distance()),
             safe_float(AIM));

// 截断的JSON没有调试价值，直接丢弃，避免PC端把它统计为协议错误。
if (json_length > 0 && json_length < static_cast<int>(sizeof(encoder_str))) {
    udp_client.udp_send(encoder_str, static_cast<size_t>(json_length));
}

// 调参快照采用独立的低频数据包，不占用12ms动态波形包的空间。
tuning_telemetry_send();
/*ssize_t sent =    udp_client_img.udp_send_image(bgr_bird, JPEG_QUALITY);
  if (sent < 0) {
          printf("ERROR: Failed to send image\r\n");
      }
*/
      }
#define RECOG_TOP      110  // 识别区域 距离顶部 125像素
#define RECOG_BOTTOM   40   // 识别区域 距离底部 100像素
#define RECOG_LEFT     60    // 识别区域 距离左边 30像素
#define RECOG_RIGHT    60  // 识别区域 距离右边 30像素

#define MIN_AREA       110   // 红色块最小面积
#define SAVE_SIZE      96  // 统一 96*96

// 功能：在图像帧中检测红色色块，并根据红色色块定位上方的车牌区域
// 参数：frame - 输入的图像帧（引用传递，避免拷贝）
void detectRedPlate(cv::Mat& frame)
{
    // 初始化标志位：默认无目标
    have_target = false;
    red_contour_area = 0;
    // 初始化红色色块矩形区域
    red_block_rect = cv::Rect(0, 0, 0, 0);
    // 初始化车牌矩形区域
    plate_rect = cv::Rect(0, 0, 0, 0);

    // ===================== 1. 定义有效识别区域并裁剪 =====================
    // 识别区域左上角X坐标
    int rx = RECOG_LEFT;
    // 识别区域左上角Y坐标
    int ry = RECOG_TOP;
    // 识别区域宽度 = 总宽度 - 左右边距
    int rw = frame.cols - RECOG_LEFT - RECOG_RIGHT;
    // 识别区域高度 = 总高度 - 上下边距
    int rh = frame.rows - RECOG_TOP - RECOG_BOTTOM;
    // 构建识别区域矩形
    cv::Rect recog_rect(rx, ry, rw, rh);
    // 先裁剪出 ROI，后续所有操作都在这个小区域上进行，大幅减少计算量
    cv::Mat roi = frame(recog_rect);

    // ===================== 2. 颜色空间转换与红色提取（仅在 ROI 内） =====================
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // 红色在HSV中分为两段：0~10 和 160~179
    cv::inRange(hsv, cv::Scalar(0, 120, 100), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 120, 100), cv::Scalar(179, 255, 255), mask2);
    mask = mask1 | mask2;

    // ===================== 3. 形态学操作去噪 =====================
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);

    // ===================== 4. 查找轮廓，筛选最大红色色块 =====================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int maxArea = 0;
    cv::Rect bestRect;  // 此时坐标相对于 ROI

    for (auto& cnt : contours)
    {
        int area = cv::contourArea(cnt);
        if (area < MIN_AREA) continue;

        cv::Rect r = cv::boundingRect(cnt);
        // 轮廓已在 ROI 内，不再需要 inside 检查

        if (area > maxArea)
        {
            maxArea = area;
            bestRect = r;
        }
    }

    // 未找到有效红色色块，直接返回
    if (maxArea <= 0) return;
    red_contour_area = maxArea;

    // ===================== 5. 坐标还原到原图坐标系 =====================
    // bestRect 的坐标是相对于 roi 的，加回 recog_rect 左上角偏移
    red_block_rect = bestRect;
    red_block_rect.x += rx;
    red_block_rect.y += ry;
    // 顶部被 ROI 切掉的情况：把 y 顶到 ROI 上边界
    if (red_block_rect.y <= recog_rect.y) {
        int hy = red_block_rect.y + red_block_rect.height - recog_rect.y;
        red_block_rect.y = recog_rect.y;
        red_block_rect.height = hy;
    }

    // ===================== 6. 根据红色色块定位上方车牌区域 =====================
    int side = red_block_rect.width;
    int px = red_block_rect.x;
    int py = red_block_rect.y - side - 2;
    int pw = side;
    int ph = side;

    if (py >= 0 && px >= 0 && px + pw <= frame.cols && py + ph <= frame.rows)
    {
        plate_rect = cv::Rect(px, py, pw, ph);
        have_target = true;
    }
}
int main()
{
control_profile_init();
   std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等线程就绪
drive_by_init();  // 初始化目标板脚本状态，默认 K0 目标板模式关闭
front_ui_init();  // 初始化前端，并默认保持停车，等待 K2 发车
//test polor
  float error=0;
 
start_camera();
save_per_map();
     img_line.width = IMG_W;
    img_line.height = IMG_H;
    img_line.data = new uint8_t[img_line.width * img_line.height];
 std::string model_param = "tiny_classifier_fp32.ncnn.param";
//set_terminal_nonblock();
  std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    int input_width    = 96;
    int input_height   = 96;
    
    // 类别标签（顺序必须与训练时一致）
    std::vector<std::string> labels = {"supplies", "vehicle", "weapon"};
    
    // 归一化参数（ImageNet标准）
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    // =================================================

    // 创建NCNN对象并配置
    LQ_NCNN ncnn;
    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);

    // 初始化模型
    printf( "正在加载模型...\n");
    if (!ncnn.Init()) {
        printf(" 模型加载失败!\n");
    }
    printf("模型加载成功!\n\n");
    
vofa_recv_init();
configure_gyro_i2c_adapter_timeout();
gyro_yaw_rate_control_init();

   gyro_watchdog_timer.set_debug_info(1, "陀螺仪服务");
   encoder_ave_timer.set_debug_info(2, "编码器");
   speed_timer.set_debug_info(3, "速度环");
   udp_timer.set_debug_info(4, "UDP");
   dir_timer.set_debug_info(5, "方向环");

   gyro_watchdog_timer.set_seconds_ms(5, []() {
    // lq_timer only runs lightweight scheduling here.
    // Blocking MPU6050 ioctl is isolated in gyro read workers.
    gyro_yaw_rate_control_service();
    });

   encoder_ave_timer.set_seconds_ms(2, []() {
         encoder_sample_1ms_thread();

        //encoder1_speed_avg = -enc1.encoder_get_count();
        //encoder2_speed_avg = enc2.encoder_get_count();
        


    });

   speed_timer.set_seconds_ms(3, []() {
     // 硬件测试是电机输出最高优先级。关闭时该函数只做一次快速判断；
     // 开启时本周期只允许PWM1正向输出，普通速度环和其它停车态控制均跳过。
     if (hardware_test_update()) {
       return;
     }
     // 正常发车优先级最高；run=0时航向保持和按住遥控按钮可单独使用速度环。
     if (front_ui_is_running()) {
       // 主动制动期间由drive_by直接输出受限反向PWM；返回false时，普通速度PID
       // 在同一周期从已复位状态接管识别阶段目标；刹车测试会继续独占零PWM。
       if (!drive_by_speed_control_update()) {
         test_enc_and_motor_rps();
       }
     } else if (drive_by_heading_hold_is_enabled()) {
       if (drive_by_heading_hold_is_quiet()) {
         // 静默死区内不再运行增量式速度PID，直接保持零PWM，避免历史PWM
         // 为了追踪极小轮速而反复克服电机静摩擦。
         motor_speed_force_pwm(0, 0);
       } else {
         // 航向保持用对称正负轮速原地回正，仍需3ms速度闭环跟踪差速目标。
         test_enc_and_motor_rps();
       }
     } else if (front_ui_remote_is_active()) {
       test_enc_and_motor_rps();
     } else {
       front_ui_hold_stop();
     }
    });

     udp_timer.set_seconds_ms(12, []() {
    if (udp_debug_mode >= 1 || vofa_telemetry_enabled) {
      udp_send();
    }

    });

    dir_timer.set_seconds_ms(8, []() {
      // 硬件测试期间禁止任何方向控制器改写左右轮目标或清空测试PWM。
      if (hardware_test_is_enabled()) {
        return;
      }
      if (front_ui_is_running()) {
        // 普通状态只经过一次快速状态判断；仅绕行会执行缓存积分和航向闭环。
        drive_by_control_update();
      }
      // K0只是目标板功能开关，不应该在没有目标时关闭正常方向环。
      // 识别阶段和边线方案复用普通方向环；两套航向闭环方案独占左右轮目标。
      if (front_ui_is_running() && !drive_by_brake_test_is_holding() &&
          (!drive_by_is_busy() || drive_by_is_recognizing() ||
           drive_by_uses_visual_direction_control())) {
        PID_control_test(latest_error);
      } else if (front_ui_is_running() && drive_by_is_motion_phase()) {
        // 三阶段会到这里，由drive_by航向闭环独占左右轮目标。
      } else if (drive_by_heading_hold_is_enabled()) {
        // 停车态航向保持只在这里运行，不执行视觉方向环。
        drive_by_heading_hold_control_update();
      } else if (front_ui_remote_is_active()) {
        // 停车遥控完全绕过视觉方向环：前后给固定RPS，左右给固定60dps。
        front_ui_remote_control_update();
      } else if (!front_ui_is_running()) {
        gyro_yaw_rate_control_reset();
        front_ui_hold_stop();
      }
    });
    
//std::cout<<"fuck you2"s<<std::endl; 
while (1)
{
     steady_clock::time_point camera_frame_start;
         if (has_input()) {
            char c = getchar();
            if (c == 'q') {
                std::cout << "quit requested\n";
                cut();
                 while (getchar() != EOF); 
                break;
            } 
        }
           front_ui_poll();  // 扫描 K0/K1/K2，并在 TFT18 上刷新状态
           vofa_recv_cmd()   ;
           // 默认关闭时不读取时钟；开启后只统计图像采集到本帧处理结束的耗时。
           if (camera_performance_enabled != 0) {
               camera_frame_start = steady_clock::now();
           }
// std::lock_guard<std::mutex> lock(g_mutex);
 //cv::Mat frame = cam.get_raw_frame();
//latest_error=img_test(frame);

 cv::Mat frame = cam.get_frame_raw();
       cv::flip(frame, frame, -1); //颠倒上下左右
 // 目标板逻辑统一交给 drive_by 状态机；没发车时不检测，避免停车待命也触发脚本。
 if (front_ui_is_running() || drive_by_is_busy()) {
    drive_by_update(frame, ncnn);
 }
 cv::cvtColor(frame, frame,cv::COLOR_BGR2GRAY);
        if (frame.empty()) {
            printf("ERROR: Failed to read frame\r\n");
            record_camera_frame_performance(camera_frame_start);
            continue;
        }
       // cv::flip(frame, frame, -1);  
        // 等待摄像头采集完毕
img_raw.data = frame.data;
img_raw.width = cam.get_camera_width();
img_raw.height = cam.get_camera_height();
img_raw.step=frame.step;
img0.data = frame.data;
img0.width = cam.get_camera_width();
img0.height = cam.get_camera_height();
img0.step=frame.step;
        // 开始处理摄像头图像
        static int zebra_detection_frame_counter = 0;
        bool zebra_detected = false;
        const bool zebra_detection_armed = car_running &&
            std::chrono::steady_clock::now() - last_start_time >=
                std::chrono::seconds(5);
        if (!zebra_detection_armed) {
            zebra_detection_frame_counter = 0;
        } else if (++zebra_detection_frame_counter >=
                   kZebraDetectionFrameInterval) {
            zebra_detection_frame_counter = 0;
            for (int y = 160; y <= 180 && !zebra_detected; ++y) {
                for (int x = 115; x <= 125; ++x) {
                    if (check_is_zebra(&img_raw, x, y, thres)) {
                        zebra_detected = true;
                        break;
                    }
                }
            }
        }
        if (zebra_detected) {
            printf("[斑马线] 检测到斑马线，停车\n");
            front_ui_stop();
        }
        process_image();    // 边线提取&处理
        find_corners();     // 角点提取&筛选
        // 预瞄距离,动态效果更佳
        aim_distance = AIM;

        // 目标板候选成立后暂停十字和环岛。脚本结束后再留300ms普通中线窗口，
        // 防止刚完成横移时立刻把残缺边线误判成新的赛道元素。
        static bool drive_by_track_features_were_suspended = false;
        static steady_clock::time_point drive_by_track_features_resume_at =
            steady_clock::now();
        const bool drive_by_suspends_track_features =
            drive_by_should_suspend_track_features();
        if (drive_by_suspends_track_features) {
            reset_special_track_state();
            drive_by_track_features_were_suspended = true;
        } else if (drive_by_track_features_were_suspended) {
            reset_special_track_state();
            drive_by_track_features_resume_at = steady_clock::now() + milliseconds(300);
            drive_by_track_features_were_suspended = false;
        }
        const bool track_features_blocked = drive_by_suspends_track_features ||
            steady_clock::now() < drive_by_track_features_resume_at;

        if (!track_features_blocked) {
            // 单侧线少，切换巡线方向，或为环岛选择外侧线。
            if(g_avoid_state==AV_NORMAL&&cross_type!=CROSS_IN){
                if (rpts0s_num < rpts1s_num / 2 && rpts0s_num < 60) {
                    track_type = TRACK_RIGHT;
                } else if (rpts1s_num < rpts0s_num / 2 && rpts1s_num < 60) {
                    track_type = TRACK_LEFT;
                } else if (rpts0s_num < 20 && rpts1s_num > rpts0s_num) {
                    track_type = TRACK_RIGHT;
                } else if (rpts1s_num < 20 && rpts0s_num > rpts1s_num) {
                    track_type = TRACK_LEFT;
                }
                else{
                    track_type=midd;
                }
            }
        // 分别检查十字 三叉 和圆环, 十字优先级最高
            check_cross();
            if (cross_type == CROSS_NONE){
                check_circle();
            }
            if (cross_type != CROSS_NONE) {
                circle_type = CIRCLE_NONE;
            }
            if(g_avoid_state!=AV_NORMAL){
                cross_type=CROSS_NONE;
                circle_type = CIRCLE_NONE;
            }
        //车库 ,十字清Aprltag标志
        //if (garage_type != GARAGE_NONE || cross_type != CROSS_NONE) apriltag_type = APRILTAG_NONE;

        //根据检查结果执行模式
        //if (yroad_type != YROAD_NONE) run_yroad();
            if (cross_type != CROSS_NONE) run_cross();
            if (circle_type != CIRCLE_NONE) run_circle();
        } else {
            reset_special_track_state();
        }
        if(cross_type != CROSS_BEGIN){
            L_count=0;
            R_count=0;
        }

       // if (garage_type != GARAGE_NONE) run_garage();

        // 中线跟踪
        ///*

        // 新视觉方案在前500ms直接把原始左/右边线当作瞄准线。必须先复制，
        // 否则后面的起点归一化会改写原始边线，影响下一帧判断和UDP显示。
        const int drive_by_aim_line = drive_by_visual_aim_line();
        if (drive_by_aim_line != 0) {
            const float (*source_line)[2] =
                drive_by_aim_line < 0 ? rpts0s : rpts1s;
            const int source_count =
                drive_by_aim_line < 0 ? rpts0s_num : rpts1s_num;
            rpts_num = clip(source_count, 0, POINTS_MAX_LEN);
            for (int i = 0; i < rpts_num; ++i) {
                drive_by_aim_line_points[i][0] = source_line[i][0];
                drive_by_aim_line_points[i][1] = source_line[i][1];
            }
            rpts = drive_by_aim_line_points;
        }
        else if (cross_type != CROSS_IN) {
            if (track_type == TRACK_LEFT) {
                rpts = rptsc0;
                rpts_num = rptsc0_num;
            } else if (track_type == TRACK_RIGHT){
                rpts = rptsc1;
                rpts_num = rptsc1_num;
            }
            else {
                rpts = rpts2s;
                rpts_num = rpts2s_num;
            }
        }
        else if( cross_type == CROSS_BEGIN) {
                            rpts = rpts2s;
                rpts_num = rpts2s_num;
        }
        else {
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

       if (cross_type == CROSS_BEGIN) {
        
        aim_distance=0.15;}
        else{
            aim_distance=AIM;
        }
        // 中线有点，同时最近点不是最后几个点
        const int aim_line_remaining_points =
            begin_id >= 0 ? rpts_num - begin_id : 0;
        // 边线阶段只需满足现有几何运算的3点下限；切回中线时要求至少15点，
        // 防止刚出现一小截噪声就误判为“中线恢复”并提前交还巡线。
        const bool drive_by_aim_line_valid = begin_id >= 0 &&
            aim_line_remaining_points >= (drive_by_aim_line == 0 ? 15 : 3);
        float current_aim_y = cy - kVisionYGuardExitMarginPx - 1.0f;
        bool current_aim_valid = false;
        if (begin_id >= 0 && rpts_num - begin_id >= 3) {
            // 归一化中线，如果是根据左右track寻仙则需要这么干
            rpts[begin_id][0] = cx;
            rpts[begin_id][1] = cy;
           rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
           resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num, sample_dist * pixel_per_meter);

            // 远预锚点位置
            int aim_idx = clip(round(aim_distance / sample_dist), 0, rptsn_num - 1);
            current_aim_y = rptsn[aim_idx][1];
            current_aim_valid = true;
            // 近预锚点位置
            int aim_idx_near = clip(round(0.25 / sample_dist), 0, rptsn_num - 1);

            // 计算远锚点偏差值
            float dx = rptsn[aim_idx][0] - cx;
            float dy = cy - rptsn[aim_idx][1] + 0.2 * pixel_per_meter;
            float dn = sqrt(dx * dx + dy * dy);
            //error = -atan2f(dx, dy) * 180 / PI;
             error=-dx;
            assert(!isnan(error));

            // 若考虑近点远点,可近似构造Stanley算法,避免撞路肩
            // 计算近锚点偏差值
            float dx_near = rptsn[aim_idx_near][0] - cx;
            float dy_near = cy - rptsn[aim_idx_near][1] + 0.2 * pixel_per_meter;
            float dn_near = sqrt(dx_near * dx_near + dy_near * dy_near);
            float error_near = -atan2f(dx_near, dy_near) * 180 / PI;
            assert(!isnan(error_near));


        }
// ====================== 误差计算与滤波处理 ======================
        const bool line_lost_now = check_line_lost();
        const float raw_visual_error = -error;
        update_vision_y_guard(
            raw_visual_error,
            current_aim_y,
            cy,
            line_lost_now,
            current_aim_valid && drive_by_aim_line_valid,
            drive_by_aim_line);
        if (cross_type == CROSS_BEGIN) {
            // 单侧丢线 → 保持上一帧误差，不走不可靠的中线
            if (!Lpt0_found || !Lpt1_found) {
                // 维持 latest_error 不变
            } else {
                float raw_err = -error;
                if (abs(latest_error) <= 15 && abs(raw_err - latest_error) >= 30) {
                    // 特殊十字边界保护，维持上一帧不突变
                } else {
                    // 使用你写好的 filter_error 保护函数进行滤波
                    latest_error = filter_error(raw_err);
                }
            }
        } 
        else if (line_lost_now && cross_type != CROSS_IN) {
            if (g_avoid_state == AV_GO_RIGHT) {
                //latest_error = filter_error(-100.0f); // 避障丢线，平滑过渡到最大打角
                diu++;
            } 
            else if (g_avoid_state == AV_GO_LEFT) {
                //latest_error = filter_error(100.0f);
                diu++;
            } 
            else if (cross_type == CROSS_NONE && circle_type == CIRCLE_NONE) {
                if (lost == RIGHT) {
                   // latest_error = filter_error(100.0f);  
                } else {   
                   // latest_error = filter_error(-100.0f);
                }
            } 
            else {
                // 其他情况保持原样或维持上一帧
            }
        }
        else {
            if (diu != 0) {
                diu = 0;
                g_avoid_state = AV_NORMAL;
                float pixel_per_meter = M2PIX;  // 像素 → 实际距离换算比例
            }
            // 正常巡线状态：对计算出的视觉误差进行低通与突变限幅滤波
           // if(cross_type==CROSS_IN)latest_error=0;

            else latest_error = filter_error(-error); 
        }

        // 新视觉绕行只在这里覆盖最终误差：边线阶段丢线时继续向绕行侧；
        // 回程丢中线时，陀螺仪可用则由脚本直接给dbRecoverDps，
        // 否则这里保留反方向±100视觉误差兜底。
        latest_error = drive_by_adjust_visual_error(
            latest_error, drive_by_aim_line, drive_by_aim_line_valid);
        
 // 道路处理完成后更新目标位置、赛道切线和弯道参考方向。
 // drive_by 在下一帧使用该缓存，避免为目标板逻辑复制整套寻线代码。
 drive_by_update_track_geometry();

 // 发送控制算法实际使用的鸟瞰左/中/右线。每个包小于MTU，不启用JPEG图传。
 road_telemetry_send();

 if(is_udp_img==1){
               clear_image(&img_line);
               cv::Mat birdview;

// 1. 直接用 OpenCV 官方 warpPerspective → 绝对正确！

cv::warpPerspective(frame, birdview, M, cv::Size(IMG_W, IMG_H));
auto t3 = high_resolution_clock::now();

// 2. 转彩色，用于画彩色线


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
sprintf(text, "Angle: %.1f", latest_error);
cv::putText(bgr_bird, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

// 7. 显示最终鸟瞰图（就是你要的效果）
cv::resize(bgr_bird, bgr_bird, cv::Size(320, 240));
//std::cout<<"fuck you"<<std::endl;

ssize_t sent =    udp_client_img.udp_send_image(bgr_bird, JPEG_QUALITY);
  if (sent < 0) {
          printf("ERROR: Failed to send image\r\n");
      }
}
else if(is_udp_img==2){
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
sprintf(angle_text, "Angle: %.1f deg", latest_error);  

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
ssize_t sent =    udp_client_img.udp_send_image( color_frame, JPEG_QUALITY);
}
// 正确写法：字符串单独闭合，变量写在外面，逗号分隔
encoder_1=-enc1.encoder_get_count();// enc1 always gets a negative number 
encoder_2=enc2.encoder_get_count();
/*
      auto end = high_resolution_clock::now();
auto p1 = duration_cast<milliseconds>(t1 - start).count();
auto p2 = duration_cast<milliseconds>(t2 - t1).count();
auto p3 = duration_cast<milliseconds>(t3 - t2).count();
auto p4 = duration_cast<milliseconds>(end - t3).count();
printf("process=%3d ms |corner=%3d ms| warp=%3d ms | udp=%3d ms | total=%3d ms\n", 
       p1, p2, p3, p4,p1+p2+p3+p4);
       */
        record_camera_frame_performance(camera_frame_start);
     // std::this_thread::yield(); // 必须加！让定时器能跑
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 加这一句
}

     std::cout << "restore terminal\n";
     reset_terminal(); // 必须恢复终端！
     std::cout << "exit done\n";
    return 0;
}
