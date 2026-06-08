#include "gyro_yaw_rate_control.hpp"
#include "lq_all_demo.hpp"

#include <cmath>
#include <cstdio>
#include <unistd.h>

// ========================= 可调参数区 =========================
//
// 这些变量故意不用 const，后面可以接 VOFA 或菜单在线调参。
// 初值给得比较保守，目的是先把闭环方向跑对，而不是一上来追求极限速度。

// 默认关闭，避免没验证 gyro_z_sign / gyro_turn_sign 前直接改变实车控制。
// 确认正负号后，用 VOFA 发送 #gyro=1; 请求启用角速度反馈。
volatile int gyro_yaw_rate_feedback_enabled = 0;

// 外环：视觉误差 -> 目标角速度(deg/s)。
// latest_error 大约被你限制在 -100~100，因此 kp=1.2 时，满误差目标角速度约 120 deg/s。
volatile float gyro_outer_kp = 1.2f;
volatile float gyro_outer_kd = 0.25f;

// 内环：角速度误差(deg/s) -> 差速修正(RPS)。
// 举例：实际角速度比目标少 100 deg/s，kp=0.045 会先给约 4.5 RPS 的差速修正。
volatile float gyro_inner_kp = 0.045f;
volatile float gyro_inner_ki = 0.015f;

// 符号修正：
// gyro_z_sign 修正 MPU6050 的正负方向；gyro_turn_sign 修正输出差速方向。
// 正确时应满足：目标右转 -> 左右轮差速产生右转 -> MPU6050 反馈逐渐接近目标。
volatile float gyro_z_sign = 1.0f;
volatile float gyro_turn_sign = 1.0f;

volatile float gyro_target_max_dps = 160.0f;
volatile float gyro_turn_max_rps = 15.0f;

namespace
{
// 你的 dir_timer 是 8ms 调一次 PID_control_test()，角速度环就在这个周期运行。
// 如果以后把方向环定时器改成 5ms，这里也要改成 0.005f。
constexpr float kControlDt = 0.008f;

// 你的 MPU6050 底层驱动初始化里设置的是 +/-2000dps：
// driver/i2c_mpu6050_driver/src/lq_mpu6050_drv.c 里 mpu6050_set_gyro_fsr(dev, 3)
// MPU6050 在 +/-2000dps 量程下的比例是 16.4 LSB/(deg/s)。
constexpr float kMpu6050GyroScale_2000Dps = 16.4f;

// 零偏标定采样次数。500 次、每次 1ms，大约半秒。
constexpr int kGyroOffsetSamples = 500;

bool g_gyro_ready = false;
float g_gyro_z_offset_raw = 0.0f;
float g_last_vision_error = 0.0f;
float g_rate_integral = 0.0f;
float g_gyro_z_lpf = 0.0f;
GyroYawRateDebug g_debug = {};

float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

lq_i2c_mpu6050 &mpu6050_device()
{
    // 函数内 static：第一次真正使用时才打开 /dev/lq_i2c_mpu6050。
    // 这样比全局对象更稳，避免程序启动很早时驱动节点还没准备好。
    static lq_i2c_mpu6050 mpu6050;
    return mpu6050;
}

bool read_gyro_z_raw(int16_t *gz_raw)
{
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    // get_mpu6050_ang 只读三轴角速度，比同时读加速度更直接。
    if (!mpu6050_device().get_mpu6050_ang(&gx, &gy, &gz)) {
        return false;
    }

    *gz_raw = gz;
    return true;
}

float calc_target_yaw_rate(float vision_error)
{
    // 1. 视觉误差限幅：避免丢线或误检时把外环目标角速度打爆。
    vision_error = clampf(vision_error, -100.0f, 100.0f);

    // 2. 小误差死区：直道附近不必让车一直微小摆动。
    if (std::fabs(vision_error) < 1.0f) {
        vision_error = 0.0f;
    }

    // 3. 外环 PD：
    //    P：偏得越多，目标角速度越大。
    //    D：误差变化越快，提前给一点角速度需求，减少滞后。
    const float outer_kp = gyro_outer_kp;
    const float outer_kd = gyro_outer_kd;
    const float d_error = vision_error - g_last_vision_error;
    float target_yaw_rate = outer_kp * vision_error + outer_kd * d_error;
    g_last_vision_error = vision_error;

    // 4. 目标角速度限幅：这里限制的是“想转多快”，不是电机输出。
    const float target_limit = std::fabs((float)gyro_target_max_dps);
    target_yaw_rate = clampf(target_yaw_rate, -target_limit, target_limit);

    g_debug.vision_error = vision_error;
    g_debug.target_yaw_rate_dps = target_yaw_rate;
    return target_yaw_rate;
}
} // namespace

void gyro_yaw_rate_control_init(void)
{
    gyro_yaw_rate_control_reset();
    g_gyro_ready = false;

    const uint8_t id = mpu6050_device().get_mpu6050_id();
    if (id != 0x68) {
        printf("[GYRO] MPU6050 init skipped: id=0x%02x, expect 0x68. Fallback to old visual PD.\n", id);
        return;
    }

    printf("[GYRO] Keep car still, calibrating MPU6050 gz offset...\n");

    double sum = 0.0;
    int valid_count = 0;

    for (int i = 0; i < kGyroOffsetSamples; ++i) {
        int16_t gz_raw = 0;
        if (read_gyro_z_raw(&gz_raw)) {
            sum += gz_raw;
            ++valid_count;
        }
        usleep(1000);
    }

    if (valid_count < kGyroOffsetSamples * 8 / 10) {
        printf("[GYRO] Calibration failed: only %d/%d samples valid. Fallback to old visual PD.\n",
               valid_count, kGyroOffsetSamples);
        return;
    }

    g_gyro_z_offset_raw = (float)(sum / valid_count);
    g_gyro_z_lpf = 0.0f;
    g_gyro_ready = true;

    printf("[GYRO] MPU6050 ready: gz_offset_raw=%.2f, scale=16.4 LSB/(deg/s).\n",
           g_gyro_z_offset_raw);
}

void gyro_yaw_rate_control_reset(void)
{
    // 积分和历史误差必须清。否则停车前残留的转向量，会在下一次发车瞬间继续输出。
    g_last_vision_error = 0.0f;
    g_rate_integral = 0.0f;
    g_gyro_z_lpf = 0.0f;
    g_debug.target_yaw_rate_dps = 0.0f;
    g_debug.yaw_rate_error = 0.0f;
    g_debug.turn_rps = 0.0f;
}

bool gyro_yaw_rate_control_is_ready(void)
{
    return g_gyro_ready;
}

float gyro_yaw_rate_control_get_gyro_z_dps(void)
{
    int16_t gz_raw = 0;
    if (!read_gyro_z_raw(&gz_raw)) {
        // 读取失败时返回上一次滤波值，避免控制输出突然跳变。
        return g_gyro_z_lpf;
    }

    // raw -> deg/s：先扣零偏，再除以量程比例。
    float gyro_z_dps = ((float)gz_raw - g_gyro_z_offset_raw) / kMpu6050GyroScale_2000Dps;

    // 符号修正。实车调试时，如果车头向右转但这里显示负数，就把 gyro_z_sign 改成 -1。
    gyro_z_dps *= gyro_z_sign;

    // 静止小死区：MPU6050 零偏不可能完全为 0，小于 0.5 deg/s 当作 0。
    if (std::fabs(gyro_z_dps) < 0.5f) {
        gyro_z_dps = 0.0f;
    }

    // 一阶低通：降低陀螺仪噪声。0.75 越大越稳但越慢，0.25 越大越灵敏但越抖。
    g_gyro_z_lpf = 0.75f * g_gyro_z_lpf + 0.25f * gyro_z_dps;

    g_debug.gyro_z_raw = (float)gz_raw;
    g_debug.gyro_z_dps = gyro_z_dps;
    g_debug.gyro_z_lpf = g_gyro_z_lpf;
    return g_gyro_z_lpf;
}

float gyro_yaw_rate_control_update(float vision_error)
{
    if (!g_gyro_ready) {
        return 0.0f;
    }

    // 外环：视觉误差 -> 目标角速度。
    const float target_yaw_rate = calc_target_yaw_rate(vision_error);

    // 反馈：MPU6050 实际 Z 轴角速度。
    const float actual_yaw_rate = gyro_yaw_rate_control_get_gyro_z_dps();

    // 内环误差：目标角速度 - 实际角速度。
    // 如果符号都对，误差为正时，输出应该让车“更正向地转”。
    const float rate_error = target_yaw_rate - actual_yaw_rate;

    // 积分项：用于处理“长期转不够”的情况。
    // 注意积分不是越大越好。新手调车时可以先把 gyro_inner_ki 设为 0，只调 P。
    g_rate_integral += rate_error * kControlDt;

    const float turn_limit = std::fabs((float)gyro_turn_max_rps);
    const float inner_ki = gyro_inner_ki;
    if (std::fabs(inner_ki) > 1e-6f) {
        const float integral_limit = turn_limit / std::fabs(inner_ki);
        g_rate_integral = clampf(g_rate_integral, -integral_limit, integral_limit);
    } else {
        g_rate_integral = 0.0f;
    }

    // 位置式 PI：输出单位直接设计成 RPS 差速修正量。
    // turn_rps > 0 时，方向环会让 motor1 目标速度增大、motor2 目标速度减小。
    float turn_rps = gyro_inner_kp * rate_error + inner_ki * g_rate_integral;

    // 输出符号修正。若角速度环一启用就越修越偏，先把 gyro_turn_sign 改成 -1。
    turn_rps *= gyro_turn_sign;

    // 输出限幅：保护速度环和电机，避免一次给太大的左右轮速度差。
    turn_rps = clampf(turn_rps, -turn_limit, turn_limit);

    g_debug.yaw_rate_error = rate_error;
    g_debug.turn_rps = turn_rps;
    return turn_rps;
}

const GyroYawRateDebug &gyro_yaw_rate_control_get_debug(void)
{
    return g_debug;
}

void gyro_yaw_rate_control_print_debug(int interval_count)
{
    static int count = 0;
    if (interval_count <= 0) {
        return;
    }
    if (++count < interval_count) {
        return;
    }
    count = 0;

    printf("[GYRO] target_loop err=%.1f target_dps=%.1f gyro_dps=%.1f rate_err=%.1f turn_rps=%.2f ready=%d\n",
           g_debug.vision_error,
           g_debug.target_yaw_rate_dps,
           g_debug.gyro_z_lpf,
           g_debug.yaw_rate_error,
           g_debug.turn_rps,
           g_gyro_ready ? 1 : 0);
}
