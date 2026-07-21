#ifndef __DRIVE_BY_HPP
#define __DRIVE_BY_HPP

#include "lq_ncnn.hpp"

#include <opencv2/core.hpp>

// 普通巡线、识别阶段和绕行阶段的速度参数。
extern volatile float drive_by_normal_speed_rps;
extern volatile float drive_by_recognition_speed_rps;
extern volatile float drive_by_rps_to_mps;

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
extern volatile float drive_by_pass_distance_m;
extern volatile float drive_by_exit_distance_m;
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
extern volatile float drive_by_track_heading_alpha;

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
} DriveByDebug;

void drive_by_init();
void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn);
bool drive_by_is_busy();
bool drive_by_is_recognizing();
bool drive_by_is_motion_phase();
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
