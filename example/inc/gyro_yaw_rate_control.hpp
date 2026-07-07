#ifndef __GYRO_YAW_RATE_CONTROL_HPP
#define __GYRO_YAW_RATE_CONTROL_HPP

// 角速度环 demo：
//
// 你的原始方向控制是：
//   视觉 error -> 直接算左右轮速度差 -> 速度环 -> PWM
//
// 这里新增的角速度环是：
//   视觉 error -> 目标角速度 target_yaw_rate
//   MPU6050 gz -> 实际角速度 gyro_z
//   target_yaw_rate - gyro_z -> 角速度 PI -> 左右轮速度差
//
// 注意：这个文件只声明接口和可调参数，真正实现与详细注释在
// example/src/gyro_yaw_rate_control.cpp。

#include <stdint.h>

// 是否请求使用 MPU6050 角速度反馈。
// 1：请求 PID_control_test() 使用“视觉外环 + MPU6050角速度内环”。
// 0：请求 PID_control_test() 使用原来的“视觉 error 直接差速”。
//
// 注意：这个变量只是“用户请求”。实际是否进入角速度环还要看 MPU6050 是否 ready。
// 所以方向环里会打印：
//   mode=GYRO_RATE            请求打开，并且 MPU6050 ready
//   mode=VISUAL_PD            请求关闭，使用旧视觉 PD
//   mode=VISUAL_PD_FALLBACK   请求打开，但 MPU6050 没 ready，自动回退旧视觉 PD
extern volatile int gyro_yaw_rate_feedback_enabled;

// 外环参数：视觉误差 -> 目标角速度。
// 单位直觉：gyro_outer_kp 越大，同样的视觉 error 会要求车转得越快。
// gyro_outer_kd 用于抑制误差变化太快造成的滞后，但太大容易抖。
extern volatile float gyro_outer_kp;
extern volatile float gyro_outer_kd;

// 内环参数：目标角速度 - 实际角速度 -> 左右轮差速 RPS。
// gyro_inner_kp 越大，角速度跟随越硬；过大会左右抖。
// gyro_inner_ki 用于消除长期转不够的问题；新手阶段可以先设 0。
extern volatile float gyro_inner_kp;
extern volatile float gyro_inner_ki;

// 传感器符号修正。
// 如果手动把车头向右转，打印出来的 gyro_z_dps 正负和你期望相反，改成 -1。
extern volatile float gyro_z_sign;

// 输出符号修正。
// 如果角速度环一介入，车往误差更大的方向打，优先把这个值改成 -1。
extern volatile float gyro_turn_sign;

// 限幅。
// gyro_target_max_dps：外环允许给出的最大目标角速度，单位 deg/s。
// gyro_turn_max_rps：内环最多给左右轮叠加多少 RPS 差速。
extern volatile float gyro_target_max_dps;
extern volatile float gyro_turn_max_rps;

// 调试数据。刚开始调角速度环时，一定要看中间量，不要只看车跑没跑好。
typedef struct
{
    float vision_error;          // input latest_error
    float target_yaw_rate_dps;   // outer loop output, deg/s
    float gyro_z_raw;            // raw MPU6050 gz
    float gyro_z_dps;            // zero-offset corrected gz, deg/s
    float gyro_z_lpf;            // filtered gz used by inner loop
    float yaw_rate_error;        // target yaw rate - actual yaw rate
    float turn_rps;              // inner loop output, RPS differential correction
    float gyro_age_ms;           // age of latest valid async gyro sample, ms
    float gyro_read_last_ms;      // latest get_mpu6050_ang() cost
    float gyro_read_min_ms;       // min successful read cost since init
    float gyro_read_avg_ms;       // average successful read cost since init
    float gyro_read_max_ms;       // max successful read cost since init
    int gyro_read_sample_count;   // successful async read samples
    int gyro_timeout_count;      // number of timed-out async reads
    int gyro_worker_count;       // active or stuck read workers
    int integral_frozen;         // 1 means I update is frozen because gyro is stale
} GyroYawRateDebug;

// 初始化 MPU6050，并在车静止时标定 gz 零偏。
// 调用时车必须静止，否则零偏会被校歪，后面车会自己慢慢偏转。
void gyro_yaw_rate_control_init(void);

// 清空角速度环积分和历史误差。
// 发车、停车、目标板脚本接管时都应该调用，避免残留转向量。
void gyro_yaw_rate_control_reset(void);

// 返回 MPU6050 是否初始化并完成零偏标定。
bool gyro_yaw_rate_control_is_ready(void);

// Read filtered MPU6050 gz from async cache. This function does not touch hardware.
float gyro_yaw_rate_control_get_gyro_z_dps(void);

// Lightweight service called by lq_timer. It checks timeout and starts read workers.
void gyro_yaw_rate_control_service(void);

// True when async gyro cache is fresh enough for integral update.
bool gyro_yaw_rate_control_gyro_is_fresh(void);

// Return latest valid gyro sample age in ms. No sample returns a very large value.
int gyro_yaw_rate_control_gyro_age_ms(void);

// 核心控制函数：输入视觉误差，输出左右轮差速修正 turn_rps。
// 最后方向环会这样使用：
//   pwm1_duty_rps = base_speed + turn_rps;
//   pwm2_duty_rps = base_speed - turn_rps;
float gyro_yaw_rate_control_update(float vision_error);

const GyroYawRateDebug &gyro_yaw_rate_control_get_debug(void);
void gyro_yaw_rate_control_print_debug(int interval_count);

#endif
