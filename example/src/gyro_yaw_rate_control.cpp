#include "gyro_yaw_rate_control.hpp"
#include "lq_all_demo.hpp"
#include "lq_timer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <unistd.h>

// ========================= 可调参数区 =========================
//
// 这些变量故意不用 const，后面可以接 VOFA 或菜单在线调参。
// 初值给得比较保守，目的是先把闭环方向跑对，而不是一上来追求极限速度。

// 默认开启：当前实车已验证 gyro_z_sign=-1、gyro_turn_sign=1 的方向组合可用。
// 如需临时回到原视觉 PD，用 VOFA 发送 #gyro=0;。
volatile int gyro_yaw_rate_feedback_enabled = 1;
volatile int gyro_manual_target_enabled = 0;
volatile float gyro_manual_target_dps = 0.0f;

// 外环：视觉误差 -> 目标角速度(deg/s)。
// latest_error 大约被你限制在 -100~100，因此 kp=1.4 时，满误差目标角速度约 140 deg/s。
volatile float gyro_outer_kp = 4.5f;
volatile float gyro_outer_kd = 3.3f;

// 内环：角速度误差(deg/s) -> 差速修正(RPS)。
// 举例：实际角速度比目标少 100 deg/s，kp=0.055 会先给约 5.5 RPS 的差速修正。
volatile float gyro_inner_kp = 0.40f;
volatile float gyro_inner_ki = 0.0f;

// 符号修正：
// gyro_z_sign 修正 MPU6050 的正负方向；gyro_turn_sign 修正输出差速方向。
// 正确时应满足：目标右转 -> 左右轮差速产生右转 -> MPU6050 反馈逐渐接近目标。
// 实车验证：MPU6050 安装方向下，手动左转时原始 gz 为正，因此默认取反。
volatile float gyro_z_sign = -1.0f;
volatile float gyro_turn_sign = 1.0f;

volatile float gyro_target_max_dps = 360.0f;
volatile float gyro_turn_max_rps = 20.0f;

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

// Async gyro read policy:
// - lq_timer calls gyro_yaw_rate_control_service() every 5 ms.
// - A single persistent reader thread samples MPU6050. Creating one detached
//   thread per sample caused large scheduling spikes when CPU usage was high.
// - Timeout is only a diagnostic mark; the reader thread keeps running.
// These constants are intentionally local: they are safety policy, not PID gains.
constexpr int kGyroReaderPeriodMs = 5;
constexpr int kGyroReadTimeoutMs = 100;
constexpr int kGyroStaleFreezeMs = 60;
constexpr int kGyroNoSampleAgeMs = 1000000;

bool g_gyro_ready = false;
float g_gyro_z_offset_raw = 0.0f;
float g_last_vision_error = 0.0f;
float g_rate_integral = 0.0f;
float g_gyro_z_lpf = 0.0f;
GyroYawRateDebug g_debug = {};

using Clock = std::chrono::steady_clock;

std::mutex g_gyro_async_mutex;
bool g_has_gyro_sample = false;
bool g_current_read_pending = false;
bool g_current_read_timeout_reported = false;
bool g_reader_thread_started = false;
int g_worker_count = 0;
int g_gyro_timeout_count = 0;
int g_gyro_read_sample_count = 0;
long long g_gyro_read_sum_us = 0;
int g_gyro_read_last_us = 0;
int g_gyro_read_min_us = 0;
int g_gyro_read_max_us = 0;
Clock::time_point g_current_read_start_time = Clock::now();
Clock::time_point g_last_gyro_update_time = Clock::now();

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

int elapsed_ms(Clock::time_point start, Clock::time_point end)
{
    return (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

int elapsed_us(Clock::time_point start, Clock::time_point end)
{
    return (int)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

int gyro_age_ms_locked(Clock::time_point now)
{
    if (!g_has_gyro_sample) {
        return kGyroNoSampleAgeMs;
    }
    return elapsed_ms(g_last_gyro_update_time, now);
}

void update_worker_debug_locked(Clock::time_point now)
{
    g_debug.gyro_age_ms = (float)gyro_age_ms_locked(now);
    g_debug.gyro_timeout_count = g_gyro_timeout_count;
    g_debug.gyro_worker_count = g_worker_count;
}

void update_read_time_debug_locked(int read_us)
{
    g_gyro_read_last_us = read_us;
    ++g_gyro_read_sample_count;
    g_gyro_read_sum_us += read_us;
    if (g_gyro_read_sample_count == 1 || read_us < g_gyro_read_min_us) {
        g_gyro_read_min_us = read_us;
    }
    if (read_us > g_gyro_read_max_us) {
        g_gyro_read_max_us = read_us;
    }

    g_debug.gyro_read_last_ms = read_us / 1000.0f;
    g_debug.gyro_read_min_ms = g_gyro_read_min_us / 1000.0f;
    g_debug.gyro_read_avg_ms = (g_gyro_read_sum_us / (float)g_gyro_read_sample_count) / 1000.0f;
    g_debug.gyro_read_max_ms = g_gyro_read_max_us / 1000.0f;
    g_debug.gyro_read_sample_count = g_gyro_read_sample_count;
}

void publish_gyro_sample(bool ok, int16_t gz_raw, int read_us)
{
    const Clock::time_point now = Clock::now();

    std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
    g_current_read_pending = false;
    g_worker_count = 0;

    if (ok) {
        update_read_time_debug_locked(read_us);

        float gyro_z_dps = ((float)gz_raw - g_gyro_z_offset_raw) / kMpu6050GyroScale_2000Dps;
        gyro_z_dps *= gyro_z_sign;

        if (std::fabs(gyro_z_dps) < 0.5f) {
            gyro_z_dps = 0.0f;
        }

        // The async worker owns the real hardware read. The control loop only
        // consumes this filtered cache, so a slow ioctl can no longer block dir_timer.
        g_gyro_z_lpf = 0.75f * g_gyro_z_lpf + 0.25f * gyro_z_dps;
        g_has_gyro_sample = true;
        g_last_gyro_update_time = now;

        g_debug.gyro_z_raw = (float)gz_raw;
        g_debug.gyro_z_dps = gyro_z_dps;
        g_debug.gyro_z_lpf = g_gyro_z_lpf;
    }

    update_worker_debug_locked(now);
}

void gyro_reader_thread()
{
    while (true) {
        if (!g_gyro_ready || gyro_yaw_rate_feedback_enabled == 0) {
            usleep(kGyroReaderPeriodMs * 1000);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
            g_current_read_pending = true;
            g_current_read_timeout_reported = false;
            g_current_read_start_time = Clock::now();
            g_worker_count = 1;
            update_worker_debug_locked(g_current_read_start_time);
        }

        int16_t gz_raw = 0;

        // Reuse the device handle opened during init. One persistent thread
        // avoids the huge scheduling jitter caused by creating a thread for
        // every 0.2ms MPU6050 read while the vision pipeline keeps CPU busy.
        const Clock::time_point read_start = Clock::now();
        const bool ok = read_gyro_z_raw(&gz_raw);
        const int read_us = elapsed_us(read_start, Clock::now());
        publish_gyro_sample(ok, gz_raw, read_us);

        usleep(kGyroReaderPeriodMs * 1000);
    }
}

void ensure_gyro_reader_thread_started_locked()
{
    if (g_reader_thread_started) {
        return;
    }
    g_reader_thread_started = true;
    std::thread(gyro_reader_thread).detach();
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
    const float target_limit = std::fabs((float)gyro_target_max_dps);
    if (gyro_manual_target_enabled != 0) {
        const float target_yaw_rate = clampf((float)gyro_manual_target_dps,
                                             -target_limit,
                                             target_limit);
        g_debug.vision_error = vision_error;
        g_debug.target_yaw_rate_dps = target_yaw_rate;
        return target_yaw_rate;
    }

    const float outer_kp = gyro_outer_kp;
    const float outer_kd = gyro_outer_kd;
    const float d_error = vision_error - g_last_vision_error;
    float target_yaw_rate = outer_kp * vision_error + outer_kd * d_error;
    g_last_vision_error = vision_error;

    // 4. 目标角速度限幅：这里限制的是“想转多快”，不是电机输出。
    target_yaw_rate = clampf(target_yaw_rate, -target_limit, target_limit);

    g_debug.vision_error = vision_error;
    g_debug.target_yaw_rate_dps = target_yaw_rate;
    return target_yaw_rate;
}

float update_rate_target_yaw_rate(float target_yaw_rate,
                                  float requested_target_limit,
                                  float requested_turn_limit)
{
    if (!g_gyro_ready) {
        return 0.0f;
    }

    const float target_limit = std::fabs(requested_target_limit);
    target_yaw_rate = clampf(target_yaw_rate, -target_limit, target_limit);

    // 绕行脚本和视觉外环最终都走这里，保证两种模式使用同一套角速度内环。
    // 这里读取的是异步缓存，不会再次触发 MPU6050 ioctl。
    const float cached_yaw_rate = gyro_yaw_rate_control_get_gyro_z_dps();
    const bool gyro_fresh = gyro_yaw_rate_control_gyro_is_fresh();
    const float actual_yaw_rate = gyro_fresh ? cached_yaw_rate : 0.0f;
    const float rate_error = target_yaw_rate - actual_yaw_rate;

    if (gyro_fresh) {
        g_rate_integral += rate_error * kControlDt;
        g_debug.integral_frozen = 0;
    } else {
        // 数据过期时不切回视觉 PD，但不允许旧数据继续积累 I。
        g_debug.integral_frozen = 1;
    }

    const float turn_limit = std::fabs(requested_turn_limit);
    const float inner_ki = gyro_inner_ki;
    if (std::fabs(inner_ki) > 1e-6f) {
        const float integral_limit = turn_limit / std::fabs(inner_ki);
        g_rate_integral = clampf(g_rate_integral, -integral_limit, integral_limit);
    } else {
        g_rate_integral = 0.0f;
    }

    float turn_rps = gyro_inner_kp * rate_error + inner_ki * g_rate_integral;
    turn_rps *= gyro_turn_sign;
    turn_rps = clampf(turn_rps, -turn_limit, turn_limit);

    g_debug.target_yaw_rate_dps = target_yaw_rate;
    g_debug.yaw_rate_error = rate_error;
    g_debug.turn_rps = turn_rps;
    g_debug.gyro_age_ms = (float)gyro_yaw_rate_control_gyro_age_ms();
    return turn_rps;
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

    {
        std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
        g_has_gyro_sample = false;
        g_current_read_pending = false;
        g_current_read_timeout_reported = false;
        g_worker_count = 0;
        g_gyro_timeout_count = 0;
        g_gyro_read_sample_count = 0;
        g_gyro_read_sum_us = 0;
        g_gyro_read_last_us = 0;
        g_gyro_read_min_us = 0;
        g_gyro_read_max_us = 0;
        g_debug.gyro_read_last_ms = 0.0f;
        g_debug.gyro_read_min_ms = 0.0f;
        g_debug.gyro_read_avg_ms = 0.0f;
        g_debug.gyro_read_max_ms = 0.0f;
        g_debug.gyro_read_sample_count = 0;
        update_worker_debug_locked(Clock::now());

        // The async reader is still started lazily by gyro_yaw_rate_control_service().
        // Keeping startup here lightweight avoids doing extra I2C work during init.
    }
}

void gyro_yaw_rate_control_reset(void)
{
    // 完整复位：除了控制器历史量，也清空缓存的新鲜度标记。
    g_last_vision_error = 0.0f;
    g_rate_integral = 0.0f;
    g_debug.target_yaw_rate_dps = 0.0f;
    g_debug.yaw_rate_error = 0.0f;
    g_debug.turn_rps = 0.0f;
    g_debug.integral_frozen = 0;

    {
        std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
        g_gyro_z_lpf = 0.0f;
        g_has_gyro_sample = false;
        update_worker_debug_locked(Clock::now());
    }
}

void gyro_yaw_rate_control_reset_controller(void)
{
    // 绕行进入航向控制阶段时只清除控制器历史量，保留最近一次有效陀螺仪数据。
    // 如果调用完整 reset，下一次异步采样到来前会短暂被判定为 stale。
    g_last_vision_error = 0.0f;
    g_rate_integral = 0.0f;
    g_debug.target_yaw_rate_dps = 0.0f;
    g_debug.yaw_rate_error = 0.0f;
    g_debug.turn_rps = 0.0f;
    g_debug.integral_frozen = 0;
}

bool gyro_yaw_rate_control_is_ready(void)
{
    return g_gyro_ready;
}

float gyro_yaw_rate_control_get_gyro_z_dps(void)
{
    // This is now a cache read. Hardware ioctl is isolated in async workers.
    std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
    update_worker_debug_locked(Clock::now());
    return g_gyro_z_lpf;
}

bool gyro_yaw_rate_control_gyro_is_fresh(void)
{
    std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
    return gyro_age_ms_locked(Clock::now()) <= kGyroStaleFreezeMs;
}

int gyro_yaw_rate_control_gyro_age_ms(void)
{
    std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
    return gyro_age_ms_locked(Clock::now());
}

void gyro_yaw_rate_control_service(void)
{
    if (!g_gyro_ready || gyro_yaw_rate_feedback_enabled == 0) {
        return;
    }

    const Clock::time_point now = Clock::now();
    std::lock_guard<std::mutex> lock(g_gyro_async_mutex);
    update_worker_debug_locked(now);
    ensure_gyro_reader_thread_started_locked();

    if (g_current_read_pending) {
        const int read_age_ms = elapsed_ms(g_current_read_start_time, now);
        if (read_age_ms > kGyroReadTimeoutMs && !g_current_read_timeout_reported) {
            ++g_gyro_timeout_count;
            g_current_read_timeout_reported = true;

            lq_timer_timeout_report(6,
                                    "陀螺仪读取",
                                    (uint64_t)read_age_ms * 1000000ULL,
                                    (uint64_t)kGyroReadTimeoutMs * 1000000ULL);

            return;
        }
    }
}

float gyro_yaw_rate_control_update(float vision_error)
{
    if (!g_gyro_ready) {
        return 0.0f;
    }

    // Outer loop: vision error -> target yaw rate.
    const float target_yaw_rate = calc_target_yaw_rate(vision_error);

    return update_rate_target_yaw_rate(
        target_yaw_rate,
        std::fabs((float)gyro_target_max_dps),
        std::fabs((float)gyro_turn_max_rps));
}

float gyro_yaw_rate_control_update_target_yaw_rate(float target_yaw_rate_dps)
{
    return update_rate_target_yaw_rate(
        target_yaw_rate_dps,
        std::fabs((float)gyro_target_max_dps),
        std::fabs((float)gyro_turn_max_rps));
}

float gyro_yaw_rate_control_update_target_yaw_rate_limited(
    float target_yaw_rate_dps,
    float max_turn_rps)
{
    return update_rate_target_yaw_rate(
        target_yaw_rate_dps,
        std::fabs((float)gyro_target_max_dps),
        std::min(std::fabs((float)gyro_turn_max_rps),
                 std::fabs(max_turn_rps)));
}

float gyro_yaw_rate_control_update_target_yaw_rate_with_limits(
    float target_yaw_rate_dps,
    float max_target_yaw_rate_dps,
    float max_turn_rps)
{
    return update_rate_target_yaw_rate(
        target_yaw_rate_dps,
        max_target_yaw_rate_dps,
        max_turn_rps);
}

const GyroYawRateDebug &gyro_yaw_rate_control_get_debug(void)
{
    return g_debug;
}

void gyro_yaw_rate_control_print_debug(int interval_count)
{
    (void)interval_count;
}
