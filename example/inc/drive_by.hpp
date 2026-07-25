#ifndef __DRIVE_BY_HPP
#define __DRIVE_BY_HPP

#include "lq_ncnn.hpp"

#include <opencv2/core.hpp>

// 普通巡线、识别阶段和绕行阶段的速度参数。
extern volatile float drive_by_normal_speed_rps;
extern volatile float drive_by_recognition_speed_rps;
extern volatile float drive_by_rps_to_mps;
// 0：现有三阶段角度闭环；1：实验性的边线瞄准绕行。
extern volatile int drive_by_mode;

// 目标板脚本的速度参数。整数参数保留，方便兼容之前的 VOFA/源码调参习惯。
extern int drive_by_turn_speed_rps;
extern int drive_by_turn_inner_speed_rps;
extern int drive_by_forward_speed_rps;
extern int drive_by_exit_speed_rps;
extern int drive_by_turn_out_ms;
extern int drive_by_forward_ms;
extern int drive_by_turn_back_ms;
extern int drive_by_exit_forward_ms;
extern int drive_by_stop_ms;
extern int drive_by_infer_timeout_ms;
extern int drive_by_cooldown_ms;

// 目标板几何和航向闭环参数。
extern volatile float drive_by_turn_angle_deg;
extern volatile float drive_by_return_bias_deg;
extern volatile float drive_by_pass_distance_m;
extern volatile float drive_by_target_after_margin_m;
extern volatile float drive_by_view_angle_max_deg;
extern volatile int drive_by_view_wait_timeout_ms;
extern volatile float drive_by_heading_kp;
extern volatile float drive_by_heading_kd;
extern volatile float drive_by_heading_max_dps;
extern volatile float drive_by_heading_tolerance_deg;
extern volatile float drive_by_rate_tolerance_dps;
extern volatile int drive_by_gyro_stale_ms;
extern volatile float drive_by_yaw_sign;

// 主动制动参数。PWM为正数幅值，实际制动时由速度线程输出负PWM。
extern volatile int drive_by_brake_pwm;
extern volatile float drive_by_brake_release_rps;
extern volatile int drive_by_brake_confirm_count;
extern volatile int drive_by_brake_timeout_ms;
extern volatile float drive_by_test_target_distance_m;

typedef struct
{
    float yaw_deg;
    float target_yaw_deg;
    float heading_error_deg;
    float track_heading_deg;
    float target_track_heading_deg;
    float view_angle_deg;
    float target_distance_m;
    float distance_since_trigger_m;
    float phase_distance_m;
    float target_yaw_rate_dps;
    float turn_rps;
    int target_geometry_valid;
    int view_ready;
    int infer_valid_count;
    int abort_reason;
    int red_candidate;
    int red_candidate_count;
    int red_contour_area;
    int detection_stage;
    int test_mode;
    int brake_active;
    int brake_pwm;
    int brake_elapsed_ms;
    float test_target_distance_m;
} DriveByDebug;

void drive_by_init();
void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn);
// 由8ms方向定时器调用：只读取缓存并更新绕行闭环，不做图像、推理、打印或硬件I/O。
void drive_by_control_update();
// 由3ms速度定时器调用。返回true表示本周期由绕行模块独占电机PWM。
bool drive_by_speed_control_update();
// 跳过红块与NCNN，直接模拟左绕(0)或右绕(2)，用于单独调试运动脚本。
bool drive_by_start_test(int simulated_item_flag,
                         float simulated_target_distance_m);
bool drive_by_is_busy();
bool drive_by_is_recognizing();
bool drive_by_is_motion_phase();
// 边线方案和两套方案共用的中线找回阶段由普通方向环执行；
// 旧角度方案的三个航向阶段仍由drive_by独占方向控制。
bool drive_by_uses_visual_direction_control();
// 返回当前相机帧应使用的瞄准线：-1左线、0正常中线、1右线。
int drive_by_visual_aim_line();
// 相机线程提交本帧实际使用的线和寻线结果；新方案丢线时返回强制误差。
// selected_aim_line与drive_by_visual_aim_line()使用相同约定。把它一并传入，
// 可以避免500ms切换恰好发生在一帧处理中间时，把旧边线误认为新中线。
float drive_by_adjust_visual_error(float computed_error,
                                   int selected_aim_line,
                                   bool aim_line_valid);
// 远距离候选出现后，目标板脚本需要暂时独占道路类型，避免环岛/十字改写中线。
bool drive_by_should_suspend_track_features();
bool drive_by_is_enabled();
void drive_by_set_enable(bool enable);
void drive_by_toggle_enable();
// 在 K2 或 #run=0 时取消当前会话并复位状态。
bool drive_by_cancel();
const char *drive_by_state_name();
const char *drive_by_abort_reason();
const DriveByDebug &drive_by_get_debug();

// 在道路处理完成后调用，刷新目标地面位置、赛道切线和观察夹角缓存。
void drive_by_update_track_geometry();

#endif
