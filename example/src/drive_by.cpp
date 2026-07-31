#include "drive_by.hpp"

#include "front_ui.hpp"
#include "control_profile.hpp"
#include "gyro_yaw_rate_control.hpp"
#include "img.hpp"
#include "lq_all_demo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

using DriveByClock = std::chrono::steady_clock;

// ===================== 高速识别和三阶段绕行调参区 =====================
// K0 开启但没有目标时按当前控制参数组速度正常巡线；红色触发后只把“基准速度”降到
// 0RPS，普通方向环仍会叠加左右差速，使车头可以原地修正姿态。
volatile float drive_by_normal_speed_rps = 35.0f;
volatile float drive_by_recognition_speed_rps = 0.0f;
volatile float drive_by_rps_to_mps = 0.047f;
volatile int drive_by_detect_frame_interval = 2;
volatile int drive_by_mode = 0;
volatile int drive_by_side_follow_ms = 500;
volatile int drive_by_use_track_tangent = 0;

// 三阶段分别保留独立基准速度：转出使用drive_by_turn_speed_rps，斜行使用
// drive_by_forward_speed_rps，转入使用drive_by_exit_speed_rps。它们都是左右
// 差速叠加前的基准速度，不表示某一侧车轮的最终目标速度。
int drive_by_turn_speed_rps = 0;
int drive_by_turn_inner_speed_rps = 0; // 兼容旧脚本，新的角度闭环不直接使用。
int drive_by_forward_speed_rps = 10;
int drive_by_exit_speed_rps = 10;
int drive_by_turn_out_ms = 600;        // 兼容旧参数，新的状态切换主要看角度。
int drive_by_forward_ms = 400;         // 兼容旧参数，新的状态切换主要看距离。
int drive_by_turn_back_ms = 800;       // 兼容旧参数，新的状态切换主要看角度。
int drive_by_exit_forward_ms = 150;    // 兼容旧参数，新的状态切换主要看角度。
int drive_by_stop_ms = 200;            // 兼容旧识别测试，不再用于识别后停车。
int drive_by_infer_timeout_ms = 500;
int drive_by_cooldown_ms = 1000;

volatile float drive_by_turn_angle_deg = 42.0f;
volatile float drive_by_return_bias_deg = 30.0f;
volatile float drive_by_pass_distance_m = 0.32f;
volatile float drive_by_target_after_margin_m = 0.0f;
volatile float drive_by_view_angle_max_deg = 46.0f;
volatile int drive_by_view_wait_timeout_ms = 120;
// 42度标准绕行转角下，KP=31会立即触发505dps外环限幅，保证起转阶段足够积极。
// KD只保留少量角速度阻尼，避免刚起转后过早压低目标角速度。
volatile float drive_by_heading_kp = 31.0f;
volatile float drive_by_heading_kd = 0.2f;
// 这是航向外环允许给出的目标角速度上限；接近目标角度时，KP/KD仍会主动降速。
volatile float drive_by_heading_max_dps = 505.0f;
// 航向保持测试只允许较小差速，避免调试KP时误用正常巡线的较大gRMax。
volatile float drive_by_heading_hold_max_turn_rps = 10.0f;
volatile float drive_by_recovery_yaw_rate_dps = 55.0f;
volatile float drive_by_heading_tolerance_deg = 4.5f;
volatile float drive_by_rate_tolerance_dps = 20.0f;
volatile int drive_by_gyro_stale_ms = 60;
volatile float drive_by_yaw_sign = -1.0f;
volatile int drive_by_brake_pwm = 9000;
volatile float drive_by_brake_release_rps = 0.0f;
volatile int drive_by_brake_confirm_count = 2;
volatile int drive_by_brake_timeout_ms = 511;
volatile float drive_by_test_target_distance_m = 0.50f;

namespace {

constexpr int kInferFrames = 3;
constexpr int kBrakePwmMax = 9000;
constexpr int kSaveSize = 96;
constexpr int kHeadingSettleCycles = 3;
constexpr float kHeadingDeadzoneHysteresisDeg = 1.0f;
constexpr float kHeadingQuietRateDps = 5.0f;
constexpr float kReturnHandoffYawDeg = 10.0f;
constexpr int kFarDetectTop = 60;
constexpr int kFarDetectBottom = 40;
constexpr int kFarDetectLeft = 60;
constexpr int kFarDetectRight = 60;
// 录像统计表明，真实目标在正式触发前经常出现55~59像素的临界轮廓。
// 这里仅放宽远距离预检测，正式红块确认仍保留110像素门槛以过滤误触发。
constexpr int kFarDetectMinArea = 55;
constexpr int kFarDetectMinWidth = 7;
constexpr float kFarDetectMinAspectRatio = 1.2f;
constexpr float kFarDetectMaxAspectRatio = 5.5f;
constexpr int kFarDetectWindowSize = 3;
constexpr int kFarDetectConfirmCount = 2;
constexpr int kApproachTimeoutMs = 300;
constexpr int kCandidateRetryCooldownMs = 300;
constexpr float kVisualRecoveryError = 100.0f;
constexpr int kTurnPhaseTimeoutMs = 1500;
constexpr int kPassPhaseTimeoutMs = 5000;
constexpr float kTurnPhaseMaxDistanceM = 0.80f;
constexpr float kPassPhaseMaxDistanceM = 3.00f;
constexpr int kVisualSideFollowTimeoutMarginMs = 500;
constexpr float kTrackTangentWindowM = 0.10f;
constexpr float kMaxIntegrationDtS = 0.10f;
constexpr float kMinWheelTargetRps = -10.0f;
constexpr float kMaxWheelTargetRps = 200.0f;

enum DriveByState {
    DB_IDLE,
    DB_APPROACH,
    DB_WAIT_VIEW,
    DB_INFER,
    DB_WAIT_BRAKE,
    DB_START_MOTION_PENDING,
    DB_TURN_OUT,
    DB_PASS_SHORT,
    DB_TURN_TO_TRACK,
    DB_FOLLOW_SIDE_LINE,
    DB_RECOVER_CENTER_LINE,
    DB_FINISH_PENDING,
};

enum DriveByAbortReason {
    DB_ABORT_NONE,
    DB_ABORT_VIEW_TIMEOUT,
    DB_ABORT_NO_TARGET_GEOMETRY,
    DB_ABORT_GYRO_NOT_READY,
    DB_ABORT_GYRO_STALE,
    DB_ABORT_PHASE_TIMEOUT,
    DB_ABORT_BRAKE_TIMEOUT,
};

enum RecognitionFrameStatus {
    RECOG_FRAME_NOT_PROCESSED,
    RECOG_FRAME_OK,
    RECOG_FRAME_NO_RED,
    RECOG_FRAME_INVALID_ROI,
    RECOG_FRAME_UNKNOWN_LABEL,
};

enum HeadingControlZone {
    HEADING_CONTROL_FULL,
    HEADING_CONTROL_BRAKE,
    HEADING_CONTROL_QUIET,
};

struct SavedControl {
    float set_speed1 = 0.0f;
    float set_speed2 = 0.0f;
    float pwm_target1 = 0.0f;
    float pwm_target2 = 0.0f;
    bool valid = false;
};

struct RecognitionFrameRecord {
    RecognitionFrameStatus status = RECOG_FRAME_NOT_PROCESSED;
    std::string label;
    int mapped_result = 1;
    double detect_ms = 0.0;
    double prepare_ms = 0.0;
    double infer_ms = 0.0;
    double frame_ms = 0.0;
    double since_trigger_ms = 0.0;
};

struct RecognitionReport {
    RecognitionFrameRecord frames[kInferFrames];
    int frame_count = 0;
    int valid_count = 0;
    int votes[3] = {0, 0, 0};
    bool early_decision = false;
    double trigger_detect_ms = 0.0;
    double infer_sum_ms = 0.0;
    double total_ms = 0.0;
    float trigger_left_rps = 0.0f;
    float trigger_right_rps = 0.0f;
    float finish_left_rps = 0.0f;
    float finish_right_rps = 0.0f;
};

struct BrakeTestReport {
    int brake_pwm = 0;
    int brake_elapsed_ms = 0;
    float start_left_rps = 0.0f;
    float start_right_rps = 0.0f;
    float release_left_rps = 0.0f;
    float release_right_rps = 0.0f;
    float brake_distance_m = 0.0f;
    int inference_result = 1;
    bool inference_complete = false;
    bool inference_succeeded = false;
    DriveByAbortReason reason = DB_ABORT_NONE;
};

struct TargetStopReport {
    bool result_valid = false;
    int result = 1;
    RecognitionFrameStatus last_frame_status = RECOG_FRAME_NOT_PROCESSED;
    int last_frame_result = 1;
};

struct FarRedCandidate {
    bool found = false;
    int max_contour_area = 0;
    cv::Rect rect;
};

volatile bool g_drive_by_enable = false;
volatile bool g_drive_by_busy = false;
bool g_seen_lock = false;
bool g_recognition_triggered = false;
bool g_target_geometry_captured = false;
bool g_track_reference_valid = false;
bool g_current_track_heading_valid = false;
volatile bool g_brake_active = false;
volatile bool g_brake_completed = false;
volatile bool g_candidate_brake_active = false;
volatile bool g_stable_early_brake_enabled = false;
volatile bool g_stop_at_next_target_armed = false;
bool g_stop_after_current_recognition = false;
volatile bool g_inference_complete = false;
volatile bool g_test_mode = false;
volatile bool g_brake_test_enabled = false;
volatile bool g_brake_test_active = false;
volatile bool g_brake_test_holding = false;
volatile bool g_stop_requested = false;
volatile bool g_report_pending = false;
volatile bool g_brake_test_report_pending = false;
volatile bool g_target_stop_report_pending = false;
volatile bool g_visual_aim_line_valid = false;
// 运行入口固定为三阶段脚本。保留该字段只为隔离尚未删除的历史边线实现，
// 任何在线命令都不能把当前绕行切换到边线状态机。
int g_active_mode = 0;
DriveByState g_state = DB_IDLE;
DriveByAbortReason g_abort_reason = DB_ABORT_NONE;
DriveByClock::time_point g_state_start = DriveByClock::now();
DriveByClock::time_point g_session_start = DriveByClock::now();
DriveByClock::time_point g_recognition_start = DriveByClock::now();
DriveByClock::time_point g_last_integration_time = DriveByClock::now();
DriveByClock::time_point g_cooldown_start = DriveByClock::now();
DriveByClock::time_point g_candidate_retry_after = DriveByClock::now();
DriveByClock::time_point g_brake_start = DriveByClock::now();
int g_detect_frame_counter = 0;
int g_far_candidate_count = 0;
bool g_far_candidate_window[kFarDetectWindowSize] = {};
int g_far_candidate_window_index = 0;
int g_far_candidate_window_count = 0;
int g_heading_settle_count = 0;
int g_brake_confirm_count = 0;
int g_pending_result = 1;
int g_turn_side = 0;
float g_yaw_deg = 0.0f;
float g_distance_since_trigger_m = 0.0f;
float g_phase_distance_m = 0.0f;
float g_current_track_heading_relative_deg = 0.0f;
float g_track_heading_reference_deg = 0.0f;
float g_target_track_heading_global_deg = 0.0f;
float g_target_distance_at_trigger_m = 0.0f;
SavedControl g_saved;
RecognitionReport g_report;
BrakeTestReport g_brake_test_report;
TargetStopReport g_target_stop_report;
float g_brake_test_start_left_rps = 0.0f;
float g_brake_test_start_right_rps = 0.0f;
float g_brake_test_release_left_rps = 0.0f;
float g_brake_test_release_right_rps = 0.0f;
int g_brake_test_elapsed_ms = 0;
DriveByDebug g_debug = {};
volatile bool g_heading_hold_enabled = false;
volatile bool g_tangent_debug_enabled = false;
volatile bool g_heading_hold_quiet = false;
bool g_motion_heading_quiet = false;
float g_heading_hold_yaw_deg = 0.0f;
DriveByClock::time_point g_heading_hold_last_update = DriveByClock::now();
DriveByHeadingHoldDebug g_heading_hold_debug = {};
DriveByTangentDebug g_tangent_debug = {};

float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

float normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

HeadingControlZone update_heading_control_zone(float heading_error_deg,
                                               float gyro_dps,
                                               volatile bool *quiet_latched)
{
    if (quiet_latched == nullptr) {
        return HEADING_CONTROL_FULL;
    }

    const float enter_deadzone_deg = std::max(
        0.0f, (float)drive_by_heading_tolerance_deg);
    const float leave_deadzone_deg =
        enter_deadzone_deg + kHeadingDeadzoneHysteresisDeg;
    const float abs_error_deg = std::fabs(heading_error_deg);

    if (*quiet_latched) {
        // 进入静默区后允许额外1度漂移。只有真正超过退出阈值才恢复完整KP，
        // 避免车头在允许误差边缘反复启停方向控制。
        if (abs_error_deg <= leave_deadzone_deg) {
            return HEADING_CONTROL_QUIET;
        }
        *quiet_latched = false;
        return HEADING_CONTROL_FULL;
    }

    if (abs_error_deg <= enter_deadzone_deg) {
        // 已经进入允许角度，但车身仍在旋转时不能立刻撤掉控制；先把目标角速度
        // 设为0，利用角速度内环刹住惯性，降到5dps以内后才彻底清差速。
        if (std::fabs(gyro_dps) <= kHeadingQuietRateDps) {
            *quiet_latched = true;
            return HEADING_CONTROL_QUIET;
        }
        return HEADING_CONTROL_BRAKE;
    }

    return HEADING_CONTROL_FULL;
}

long long elapsed_ms(DriveByClock::time_point start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               DriveByClock::now() - start)
        .count();
}

double duration_ms(DriveByClock::time_point start, DriveByClock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

const char *state_name(DriveByState state)
{
    switch (state) {
    case DB_IDLE: return "IDLE";
    case DB_APPROACH: return "APPROACH";
    case DB_WAIT_VIEW: return "WAIT_VIEW";
    case DB_INFER: return "INFER";
    case DB_WAIT_BRAKE: return "WAIT_BRAKE";
    case DB_START_MOTION_PENDING: return "START_MOTION_PENDING";
    case DB_TURN_OUT: return "TURN_OUT";
    case DB_PASS_SHORT: return "PASS_SHORT";
    case DB_TURN_TO_TRACK: return "TURN_TO_TRACK";
    case DB_FOLLOW_SIDE_LINE: return "FOLLOW_SIDE_LINE";
    case DB_RECOVER_CENTER_LINE: return "RECOVER_CENTER_LINE";
    case DB_FINISH_PENDING: return "FINISH_PENDING";
    default: return "UNKNOWN";
    }
}

int detection_stage(DriveByState state)
{
    switch (state) {
    case DB_APPROACH: return 1;
    case DB_WAIT_VIEW: return 2;
    case DB_INFER: return 3;
    case DB_WAIT_BRAKE: return 3;
    case DB_START_MOTION_PENDING:
    case DB_TURN_OUT:
    case DB_PASS_SHORT:
    case DB_TURN_TO_TRACK:
    case DB_FOLLOW_SIDE_LINE:
    case DB_RECOVER_CENTER_LINE:
    case DB_FINISH_PENDING:
        return 4;
    default:
        return 0;
    }
}

const char *abort_reason_name(DriveByAbortReason reason)
{
    switch (reason) {
    case DB_ABORT_NONE: return "none";
    case DB_ABORT_VIEW_TIMEOUT: return "view_timeout";
    case DB_ABORT_NO_TARGET_GEOMETRY: return "target_geometry_invalid";
    case DB_ABORT_GYRO_NOT_READY: return "gyro_not_ready";
    case DB_ABORT_GYRO_STALE: return "gyro_stale";
    case DB_ABORT_PHASE_TIMEOUT: return "phase_timeout";
    case DB_ABORT_BRAKE_TIMEOUT: return "brake_timeout";
    default: return "unknown";
    }
}

const char *abort_reason_chinese_name(DriveByAbortReason reason)
{
    switch (reason) {
    case DB_ABORT_NONE: return "用户主动停车";
    case DB_ABORT_VIEW_TIMEOUT: return "目标板观察或识别等待超时";
    case DB_ABORT_NO_TARGET_GEOMETRY: return "目标板位置或赛道几何无效";
    case DB_ABORT_GYRO_NOT_READY: return "陀螺仪尚未准备好";
    case DB_ABORT_GYRO_STALE: return "陀螺仪数据长时间未刷新";
    case DB_ABORT_PHASE_TIMEOUT: return "绕行运动阶段超时";
    case DB_ABORT_BRAKE_TIMEOUT: return "主动制动超时";
    default: return "未知停车原因";
    }
}

const char *frame_status_name(RecognitionFrameStatus status)
{
    switch (status) {
    case RECOG_FRAME_OK: return "成功";
    case RECOG_FRAME_NO_RED: return "未检测到红色";
    case RECOG_FRAME_INVALID_ROI: return "目标板区域无效";
    case RECOG_FRAME_UNKNOWN_LABEL: return "未知类别";
    default: return "未处理";
    }
}

const char *label_chinese_name(const std::string& label)
{
    if (label == "weapon") return "武器";
    if (label == "vehicle") return "车辆";
    if (label == "supplies") return "物资";
    return "未知类别";
}

const char *result_action_name(int result)
{
    if (result == 0) return "左绕";
    if (result == 2) return "右绕";
    return "直行";
}

const char *result_label_name(int result)
{
    if (result == 0) return "weapon";
    if (result == 2) return "supplies";
    return "vehicle";
}

const char *result_label_chinese_name(int result)
{
    if (result == 0) return "武器";
    if (result == 2) return "物资";
    return "车辆";
}

bool recognition_result_is_valid(const RecognitionReport& report, int result)
{
    if (result < 0 || result > 2 || report.valid_count <= 0 ||
        report.votes[result] < 2) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        if (index != result && report.votes[index] >= report.votes[result]) {
            return false;
        }
    }
    return true;
}

TargetStopReport make_target_stop_report(const RecognitionReport& report,
                                         int result)
{
    TargetStopReport target_report;
    target_report.result = result;
    target_report.result_valid = recognition_result_is_valid(report, result);
    if (report.frame_count > 0) {
        const RecognitionFrameRecord& last_frame =
            report.frames[report.frame_count - 1];
        target_report.last_frame_status = last_frame.status;
        target_report.last_frame_result = last_frame.mapped_result;
    }
    return target_report;
}

int last_successful_result(const RecognitionReport& report)
{
    for (int index = report.frame_count - 1; index >= 0; --index) {
        const RecognitionFrameRecord& frame = report.frames[index];
        if (frame.status == RECOG_FRAME_OK &&
            frame.mapped_result >= 0 && frame.mapped_result <= 2) {
            return frame.mapped_result;
        }
    }
    return -1;
}

void print_target_result_line(int result)
{
    if (result < 0 || result > 2) {
        printf("[目标板] 类型=未知\n");
        fflush(stdout);
        return;
    }
    printf("[目标板] 类型=%s（%s）\n",
           result_label_chinese_name(result),
           result_label_name(result));
    fflush(stdout);
}

void save_control_once()
{
    if (g_saved.valid) {
        return;
    }

    g_saved.set_speed1 = set_speed_of_motor1_rps;
    g_saved.set_speed2 = set_speed_of_motor2_rps;
    g_saved.pwm_target1 = pwm1_duty_rps;
    g_saved.pwm_target2 = pwm2_duty_rps;
    g_saved.valid = true;
}

void restore_control()
{
    if (!g_saved.valid) {
        return;
    }
    

    set_speed_of_motor1_rps = g_saved.set_speed1;
    set_speed_of_motor2_rps = g_saved.set_speed2;
    pwm1_duty_rps = g_saved.pwm_target1;
    pwm2_duty_rps = g_saved.pwm_target2;
    g_saved.valid = false;
}

void command_normal_base_speed()
{
    set_speed_of_motor1_rps = drive_by_normal_speed_rps;
    set_speed_of_motor2_rps = drive_by_normal_speed_rps;
}

void command_zero_targets()
{
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
}

void command_recognition_base_speed()
{
    if (g_brake_test_holding) {
        // 刹车测试达到释放阈值后只等待识别结束，绝不能让相机线程重新写入方向差速。
        command_zero_targets();
        return;
    }

    // 只改变基准速度，保留方向环刚刚给出的左右差速。
    // dir_timer 下一次运行时会继续用最新视觉误差更新该差速。
    float differential = (pwm1_duty_rps - pwm2_duty_rps) * 0.5f;
    if (!std::isfinite(differential)) {
        differential = 0.0f;
    }

    const float base_rps = drive_by_recognition_speed_rps;
    set_speed_of_motor1_rps = base_rps;
    set_speed_of_motor2_rps = base_rps;
    pwm1_duty_rps = base_rps + differential;
    pwm2_duty_rps = base_rps - differential;
}

void command_idle_candidate_speed()
{
    if (g_candidate_brake_active) {
        save_control_once();
        command_zero_targets();
        return;
    }

    if (g_far_candidate_count > 0) {
        // 第一票候选只要求普通速度闭环把基准降到识别速度，不允许提前输出反向PWM。
        // 原巡线参数只在第一次命中时保存，第二票确认后继续由完整识别流程使用。
        save_control_once();
        command_recognition_base_speed();
        return;
    }

    if (g_saved.valid) {
        // 最近3次检测里已经没有候选票，说明单次红色很可能是噪声，立即恢复触发前巡线。
        restore_control();
    }
    command_normal_base_speed();
}

void command_motion_wheels(float base_rps, float turn_rps)
{
    // 绕行运动阶段由航向闭环独占左右轮目标，普通方向环此时不会再写输出。
    const float left_rps = clampf(base_rps + turn_rps,
                                  kMinWheelTargetRps,
                                  kMaxWheelTargetRps);
    const float right_rps = clampf(base_rps - turn_rps,
                                   kMinWheelTargetRps,
                                   kMaxWheelTargetRps);
    set_speed_of_motor1_rps = base_rps;
    set_speed_of_motor2_rps = base_rps;
    pwm1_duty_rps = left_rps;
    pwm2_duty_rps = right_rps;
}

void start_active_brake()
{
    // 相机线程只提交制动状态，不直接碰电机。真正的反向PWM由3ms速度线程输出，
    // 因而不会和普通速度PID在同一个周期争抢硬件。
    g_brake_active = true;
    g_brake_completed = false;
    g_brake_confirm_count = 0;
    g_brake_start = DriveByClock::now();
    g_debug.brake_active = 1;
    g_debug.brake_pwm = -std::min(kBrakePwmMax,
                                  std::abs((int)drive_by_brake_pwm));
    g_debug.brake_elapsed_ms = 0;
    if (g_brake_test_active) {
        g_brake_test_start_left_rps = encoder1_speed_avg;
        g_brake_test_start_right_rps = encoder2_speed_avg;
        g_brake_test_release_left_rps = encoder1_speed_avg;
        g_brake_test_release_right_rps = encoder2_speed_avg;
        g_brake_test_elapsed_ms = 0;
    }
}

void clear_active_brake(bool clear_completed)
{
    g_brake_active = false;
    if (clear_completed) {
        g_brake_completed = false;
    }
    g_brake_confirm_count = 0;
    g_debug.brake_active = 0;
    g_debug.brake_pwm = 0;
    g_debug.brake_elapsed_ms = 0;
}

void start_candidate_brake()
{
    if (g_candidate_brake_active) {
        return;
    }

    save_control_once();
    g_candidate_brake_active = true;
    command_zero_targets();
    start_active_brake();
}

void stop_candidate_brake()
{
    if (!g_candidate_brake_active) {
        return;
    }

    g_candidate_brake_active = false;
    clear_active_brake(true);
    motor_speed_pid_reset();
}

bool should_detect_this_frame()
{
    // 红块预检测固定每2帧一次，避免逐帧颜色检测拖慢正常道路处理。
    const int frame_interval = 2;
    ++g_detect_frame_counter;
    if (g_detect_frame_counter < frame_interval) {
        return false;
    }
    g_detect_frame_counter = 0;
    return true;
}

void reset_far_candidate_window()
{
    std::fill(g_far_candidate_window,
              g_far_candidate_window + kFarDetectWindowSize,
              false);
    g_far_candidate_window_index = 0;
    g_far_candidate_window_count = 0;
    g_far_candidate_count = 0;
}

bool update_far_candidate_window(bool found)
{
    // 候选确认改为“最近3次检测中至少命中2次”。这样目标面积在阈值附近
    // 偶尔抖低一帧时不会把前一次有效结果完全丢掉，同时连续两次命中仍可立即触发。
    g_far_candidate_window[g_far_candidate_window_index] = found;
    g_far_candidate_window_index =
        (g_far_candidate_window_index + 1) % kFarDetectWindowSize;
    g_far_candidate_window_count = std::min(g_far_candidate_window_count + 1,
                                            kFarDetectWindowSize);

    g_far_candidate_count = 0;
    for (int index = 0; index < g_far_candidate_window_count; ++index) {
        if (g_far_candidate_window[index]) {
            ++g_far_candidate_count;
        }
    }

    return g_far_candidate_window_count >= kFarDetectConfirmCount &&
           g_far_candidate_count >= kFarDetectConfirmCount;
}

FarRedCandidate detect_far_red_candidate(const cv::Mat& frame)
{
    FarRedCandidate result;
    if (frame.empty()) {
        return result;
    }

    const int width = frame.cols - kFarDetectLeft - kFarDetectRight;
    const int height = frame.rows - kFarDetectTop - kFarDetectBottom;
    if (width <= 0 || height <= 0) {
        return result;
    }

    // 远距离阶段只做颜色与轮廓筛选，不运行NCNN。扩大到图像第60行后，
    // 远处小红块能更早进入ROI，同时仍比整帧检测节省计算量。
    const cv::Rect search_rect(kFarDetectLeft, kFarDetectTop, width, height);
    cv::Mat hsv;
    cv::Mat mask_low;
    cv::Mat mask_high;
    cv::Mat mask;
    cv::cvtColor(frame(search_rect), hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 120, 100), cv::Scalar(10, 255, 255), mask_low);
    cv::inRange(hsv, cv::Scalar(160, 120, 100), cv::Scalar(179, 255, 255), mask_high);
    mask = mask_low | mask_high;

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    int best_candidate_area = 0;
    for (const auto& contour : contours) {
        const int area = (int)cv::contourArea(contour);
        result.max_contour_area = std::max(result.max_contour_area, area);
        if (area < kFarDetectMinArea) {
            continue;
        }

        cv::Rect rect = cv::boundingRect(contour);
        if (rect.width < kFarDetectMinWidth || rect.height <= 0) {
            continue;
        }
        const float aspect_ratio = (float)rect.width / (float)rect.height;
        if (aspect_ratio < kFarDetectMinAspectRatio ||
            aspect_ratio > kFarDetectMaxAspectRatio ||
            area <= best_candidate_area) {
            continue;
        }

        best_candidate_area = area;
        rect.x += search_rect.x;
        rect.y += search_rect.y;
        result.rect = rect;
        result.found = true;
    }
    return result;
}

bool make_plate_roi(cv::Mat& frame, cv::Mat& roi)
{
    if (frame.empty() || plate_rect.width <= 0 || plate_rect.height <= 0) {
        return false;
    }

    const cv::Rect image_rect(0, 0, frame.cols, frame.rows);
    const cv::Rect safe_rect = plate_rect & image_rect;
    if (safe_rect.width != plate_rect.width || safe_rect.height != plate_rect.height) {
        return false;
    }

    roi = frame(safe_rect).clone();
    cv::resize(roi, roi, cv::Size(kSaveSize, kSaveSize), 0, 0, cv::INTER_AREA);
    return true;
}

int map_infer_result(const std::string& result)
{
    if (result == "weapon") return 0;
    if (result == "supplies") return 2;
    return 1;
}

bool is_known_infer_result(const std::string& result)
{
    return result == "weapon" || result == "vehicle" || result == "supplies";
}

int choose_vote_result()
{
    int best = 1;
    int best_votes = g_report.votes[1];
    bool tie = false;

    for (int index = 0; index < 3; ++index) {
        if (index == best) continue;
        if (g_report.votes[index] > best_votes) {
            best = index;
            best_votes = g_report.votes[index];
            tie = false;
        } else if (g_report.votes[index] == best_votes) {
            tie = true;
        }
    }

    if (best_votes == 0 || tie) {
        return 1;
    }
    return best;
}

bool recognition_should_finish()
{
    // 三票多数制中，任一结果拿到两票后，第三票已经不可能改变最终结果。
    // 只有前两帧都是有效结果且一致时才会提前结束；失败帧和未知类别不算票。
    for (int result = 0; result < 3; ++result) {
        if (g_report.votes[result] >= 2) {
            g_report.early_decision = g_report.frame_count < kInferFrames;
            return true;
        }
    }
    return g_report.frame_count >= kInferFrames;
}

void finish_frame_record(RecognitionFrameRecord& record,
                         DriveByClock::time_point frame_start)
{
    const DriveByClock::time_point frame_end = DriveByClock::now();
    record.frame_ms = duration_ms(frame_start, frame_end);
    record.since_trigger_ms = duration_ms(g_recognition_start, frame_end);
}

void process_inference_frame(cv::Mat& frame,
                             LQ_NCNN& ncnn,
                             bool reuse_red_detection,
                             double reused_detect_ms)
{
    if (g_report.frame_count >= kInferFrames) {
        return;
    }

    RecognitionFrameRecord& record = g_report.frames[g_report.frame_count++];
    const DriveByClock::time_point frame_start = DriveByClock::now();
    if (reuse_red_detection) {
        // 触发帧已经完成红色检测，直接复用矩形和耗时，避免同一帧重复做 HSV 与轮廓搜索。
        record.detect_ms = reused_detect_ms;
    } else {
        const DriveByClock::time_point detect_start = DriveByClock::now();
        detectRedPlate(frame);
        record.detect_ms = duration_ms(detect_start, DriveByClock::now());
    }
    g_debug.red_contour_area = red_contour_area;

    if (!have_target) {
        record.status = RECOG_FRAME_NO_RED;
        finish_frame_record(record, frame_start);
        return;
    }

    cv::Mat roi;
    const DriveByClock::time_point prepare_start = DriveByClock::now();
    if (!make_plate_roi(frame, roi)) {
        record.prepare_ms = duration_ms(prepare_start, DriveByClock::now());
        record.status = RECOG_FRAME_INVALID_ROI;
        finish_frame_record(record, frame_start);
        return;
    }
    record.prepare_ms = duration_ms(prepare_start, DriveByClock::now());

    const DriveByClock::time_point infer_start = DriveByClock::now();
    record.label = ncnn.Infer(roi);
    record.infer_ms = duration_ms(infer_start, DriveByClock::now());
    record.mapped_result = map_infer_result(record.label);

    if (is_known_infer_result(record.label)) {
        record.status = RECOG_FRAME_OK;
        ++g_report.votes[record.mapped_result];
        ++g_report.valid_count;
        g_report.infer_sum_ms += record.infer_ms;
    } else {
        record.status = RECOG_FRAME_UNKNOWN_LABEL;
    }

    g_debug.infer_valid_count = g_report.valid_count;
    finish_frame_record(record, frame_start);
}

void print_recognition_report(const RecognitionReport& report,
                              int final_result,
                              DriveByAbortReason abort_reason)
{
    (void)abort_reason;
    const int display_result = recognition_result_is_valid(report, final_result)
        ? final_result
        : last_successful_result(report);
    print_target_result_line(display_result);
}

void print_recognition_report(int final_result)
{
    print_recognition_report(g_report, final_result, g_abort_reason);
}

void print_brake_test_report()
{
    const BrakeTestReport report = g_brake_test_report;
    g_brake_test_report_pending = false;
    printf("[刹车测试] 制动PWM=%d，初始轮速=(%.2f, %.2f)RPS\n",
           report.brake_pwm,
           report.start_left_rps,
           report.start_right_rps);
    printf("[刹车测试] 释放轮速=(%.2f, %.2f)RPS，制动耗时=%d毫秒，制动距离=%.3f米\n",
           report.release_left_rps,
           report.release_right_rps,
           report.brake_elapsed_ms,
           report.brake_distance_m);
    if (!report.inference_complete) {
        printf("[刹车测试] 识别结果=未完成，结束原因=%s（%s）\n",
               abort_reason_chinese_name(report.reason),
               abort_reason_name(report.reason));
    } else if (!report.inference_succeeded ||
               report.reason == DB_ABORT_VIEW_TIMEOUT) {
        if (report.reason == DB_ABORT_NONE) {
            printf("[刹车测试] 识别结果=识别失败，结束原因=无有效分类结果（no_valid_result）\n");
        } else {
            printf("[刹车测试] 识别结果=识别失败，结束原因=%s（%s）\n",
                   abort_reason_chinese_name(report.reason),
                   abort_reason_name(report.reason));
        }
    } else {
        printf("[刹车测试] 识别结果=%d（%s），结束原因=%s（%s）\n",
               report.inference_result,
               result_action_name(report.inference_result),
               abort_reason_chinese_name(report.reason),
               abort_reason_name(report.reason));
    }
}

void print_target_stop_report(const TargetStopReport& report)
{
    const int display_result = report.result_valid
        ? report.result
        : (report.last_frame_status == RECOG_FRAME_OK
            ? report.last_frame_result : -1);
    print_target_result_line(display_result);
}

void print_target_stop_report()
{
    const TargetStopReport report = g_target_stop_report;
    g_target_stop_report_pending = false;
    print_target_stop_report(report);
}

bool compute_heading_from_line(int center_index, float *heading_deg)
{
    if (heading_deg == nullptr || rptsn_num < 3 || sample_dist <= 0.0f) {
        return false;
    }

    int half_window = (int)std::round(kTrackTangentWindowM / sample_dist);
    if (half_window < 1) half_window = 1;
    const int previous_index = std::max(0, center_index - half_window);
    const int next_index = std::min(rptsn_num - 1, center_index + half_window);
    if (next_index <= previous_index) {
        return false;
    }

    const float dx = rptsn[next_index][0] - rptsn[previous_index][0];
    const float forward = rptsn[previous_index][1] - rptsn[next_index][1];
    if (!std::isfinite(dx) || !std::isfinite(forward) ||
        std::fabs(dx) + std::fabs(forward) < 1e-3f) {
        return false;
    }

    *heading_deg = std::atan2(dx, forward) * 180.0f / PI;
    return std::isfinite(*heading_deg);
}

bool compute_current_track_heading(float *heading_deg)
{
    return compute_heading_from_line(0, heading_deg);
}

void update_tangent_debug_cache()
{
    g_tangent_debug.enabled = g_tangent_debug_enabled ? 1 : 0;
    if (!g_tangent_debug_enabled) {
        g_tangent_debug.valid = 0;
        return;
    }

    g_tangent_debug.sample_distance_m = std::max(0.0f, (float)AIM);
    g_tangent_debug.valid = 0;
    g_tangent_debug.anchor_x = -1;
    g_tangent_debug.anchor_y = -1;
    if (rptsn_num < 3 || sample_dist <= 0.0f) {
        return;
    }

    // 调试切线使用和方向控制相同的AIM前瞻位置。前后各取约0.10m中线点，
    // 因此画出的方向代表控制正在观察的位置，而不是车头附近的瞬时方向。
    const int center_index = clip(
        cvRound(g_tangent_debug.sample_distance_m / sample_dist),
        0,
        rptsn_num - 1);
    float heading_deg = 0.0f;
    if (!compute_heading_from_line(center_index, &heading_deg)) {
        return;
    }

    g_tangent_debug.valid = 1;
    g_tangent_debug.angle_deg = heading_deg;
    g_tangent_debug.anchor_x = cvRound(rptsn[center_index][0]);
    g_tangent_debug.anchor_y = cvRound(rptsn[center_index][1]);
}

bool compute_target_geometry(float *track_heading_deg,
                             float *view_angle_deg,
                             float *target_distance_m)
{
    if (red_block_rect.width <= 0 || red_block_rect.height <= 0 || rptsn_num < 3) {
        return false;
    }

    const int target_pixel_x = clip(red_block_rect.x + red_block_rect.width / 2,
                                    0,
                                    IMG_W - 1);
    const int target_pixel_y = clip(red_block_rect.y + red_block_rect.height,
                                    0,
                                    IMG_H - 1);
    const float target_x = mapx[target_pixel_y][target_pixel_x];
    const float target_y = mapy[target_pixel_y][target_pixel_x];
    const float vehicle_x = mapx[(int)(IMG_H * 0.78f)][IMG_W / 2];
    const float vehicle_y = mapy[(int)(IMG_H * 0.78f)][IMG_W / 2];
    if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(vehicle_x) || !std::isfinite(vehicle_y)) {
        return false;
    }

    int nearest_index = -1;
    float nearest_distance_sq = 1e30f;
    for (int index = 0; index < rptsn_num; ++index) {
        const float dx = rptsn[index][0] - target_x;
        const float dy = rptsn[index][1] - target_y;
        const float distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_distance_sq) {
            nearest_distance_sq = distance_sq;
            nearest_index = index;
        }
    }
    if (nearest_index < 0 || !compute_heading_from_line(nearest_index, track_heading_deg)) {
        return false;
    }

    const float view_heading_deg =
        std::atan2(target_x - vehicle_x, vehicle_y - target_y) * 180.0f / PI;
    float angle_difference = std::fabs(normalize_angle_deg(
        *track_heading_deg - view_heading_deg));
    if (angle_difference > 90.0f) {
        angle_difference = 180.0f - angle_difference;
    }

    *view_angle_deg = angle_difference;
    *target_distance_m = nearest_index * sample_dist;
    return std::isfinite(*view_angle_deg) && std::isfinite(*target_distance_m);
}

void integrate_session_motion()
{
    const DriveByClock::time_point now = DriveByClock::now();
    float dt_s = std::chrono::duration<float>(now - g_last_integration_time).count();
    g_last_integration_time = now;
    if (dt_s <= 0.0f) {
        return;
    }
    if (dt_s > kMaxIntegrationDtS) {
        dt_s = kMaxIntegrationDtS;
    }

    if (gyro_yaw_rate_control_is_ready() && gyro_yaw_rate_control_gyro_is_fresh()) {
        g_yaw_deg = normalize_angle_deg(
            g_yaw_deg + gyro_yaw_rate_control_get_gyro_z_dps() * dt_s);
    }

    float average_rps = (encoder1_speed_avg + encoder2_speed_avg) * 0.5f;
    if (!std::isfinite(average_rps) || average_rps < 0.0f) {
        average_rps = 0.0f;
    }
    const float distance_increment = average_rps * drive_by_rps_to_mps * dt_s;
    g_distance_since_trigger_m += distance_increment;
    if (drive_by_is_motion_phase()) {
        g_phase_distance_m += distance_increment;
    }

    g_debug.yaw_deg = g_yaw_deg;
    g_debug.distance_since_trigger_m = g_distance_since_trigger_m;
    g_debug.phase_distance_m = g_phase_distance_m;
}

void enter_state(DriveByState next)
{
    g_state = next;
    g_state_start = DriveByClock::now();
    g_phase_distance_m = 0.0f;
    g_heading_settle_count = 0;
    g_debug.phase_distance_m = 0.0f;
    g_debug.detection_stage = detection_stage(next);
    g_debug.state_code = (int)next;
}

void reset_runtime(bool restore_outputs)
{
    g_candidate_brake_active = false;
    g_stop_after_current_recognition = false;
    clear_active_brake(true);
    motor_speed_pid_reset();
    if (restore_outputs) {
        restore_control();
    }
    gyro_yaw_rate_control_reset_controller();
    g_drive_by_busy = false;
    g_seen_lock = false;
    g_recognition_triggered = false;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_current_track_heading_valid = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_brake_test_active = false;
    g_brake_test_holding = false;
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_motion_heading_quiet = false;
    g_active_mode = 0;
    g_state = DB_IDLE;
    g_abort_reason = DB_ABORT_NONE;
    g_candidate_retry_after = DriveByClock::now();
    g_detect_frame_counter = 0;
    reset_far_candidate_window();
    g_heading_settle_count = 0;
    g_pending_result = 1;
    g_turn_side = 0;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_report = RecognitionReport{};
    g_debug = DriveByDebug{};
    g_debug.brake_test_holding = 0;
    g_debug.test_target_distance_m = drive_by_test_target_distance_m;
    have_target = false;
    red_block_rect = cv::Rect();
    plate_rect = cv::Rect();
    red_contour_area = 0;
}

void finish_session(DriveByAbortReason reason)
{
    clear_active_brake(true);
    motor_speed_pid_reset();
    g_abort_reason = reason;
    g_debug.abort_reason = (int)reason;
    // 正常左右绕行可能在推理结束几秒后才完成，不能把整段绕行时间误写成
    // “三帧总时间”。只有尚未完成三帧（例如观察角度超时）时才在这里补值。
    if (g_report.total_ms <= 0.0) {
        g_report.total_ms = duration_ms(g_session_start, DriveByClock::now());
        g_report.finish_left_rps = encoder1_speed_avg;
        g_report.finish_right_rps = encoder2_speed_avg;
    }

    // 先恢复控制权并把 busy 清掉，再输出报告，避免 printf 延迟下一次方向控制。
    restore_control();
    gyro_yaw_rate_control_reset_controller();
    g_drive_by_busy = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_brake_test_active = false;
    g_brake_test_holding = false;
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_motion_heading_quiet = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
    g_state = DB_IDLE;
    g_debug.state_code = (int)DB_IDLE;
    reset_far_candidate_window();
    g_debug.red_candidate = 0;
    g_debug.red_candidate_count = 0;
    g_debug.detection_stage = 0;
    g_cooldown_start = DriveByClock::now();
    print_recognition_report(item_flag);
}

void complete_brake_test_recognition(DriveByAbortReason reason)
{
    if (g_report.total_ms <= 0.0) {
        g_report.total_ms = duration_ms(g_recognition_start, DriveByClock::now());
        g_report.finish_left_rps = encoder1_speed_avg;
        g_report.finish_right_rps = encoder2_speed_avg;
    }
    g_pending_result = item_flag;
    g_inference_complete = true;
    g_abort_reason = reason;
    g_debug.abort_reason = (int)reason;
    // 识别线程只提交完成状态。若制动尚未到阈值，3ms线程继续制动；
    // 若已经在零PWM保持，3ms线程会在下一个周期完成统一停车。
    enter_state(DB_WAIT_BRAKE);
}

void finish_brake_test_stop(DriveByAbortReason reason)
{
    BrakeTestReport report;
    report.brake_pwm = -std::min(kBrakePwmMax,
                                 std::abs((int)drive_by_brake_pwm));
    report.brake_elapsed_ms = g_brake_test_elapsed_ms;
    report.start_left_rps = g_brake_test_start_left_rps;
    report.start_right_rps = g_brake_test_start_right_rps;
    report.release_left_rps = g_brake_test_release_left_rps;
    report.release_right_rps = g_brake_test_release_right_rps;
    report.brake_distance_m = g_distance_since_trigger_m;
    report.inference_result = item_flag;
    report.inference_complete = g_inference_complete;
    report.inference_succeeded = g_report.valid_count > 0;
    report.reason = reason;

    g_brake_test_active = false;
    g_brake_test_holding = false;
    g_brake_active = false;
    g_drive_by_busy = false;
    command_zero_targets();
    motor_speed_force_pwm(0, 0);
    front_ui_stop();

    // front_ui_stop()会复位本次运行态。测试开关是持续配置，因此不在停车时清除；
    // 报告也在停车之后重新排队，由相机线程低优先级打印。
    g_brake_test_report = report;
    g_brake_test_report_pending = true;
    g_abort_reason = reason;
    g_debug.abort_reason = (int)reason;
    g_debug.brake_elapsed_ms = report.brake_elapsed_ms;
    g_debug.brake_test_holding = 0;
}

void start_approach_session(const FarRedCandidate& candidate)
{
    g_candidate_brake_active = false;
    g_stop_after_current_recognition = g_stop_at_next_target_armed;
    save_control_once();
    g_drive_by_busy = true;
    g_seen_lock = true;
    g_recognition_triggered = false;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_current_track_heading_valid = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_brake_test_active = g_brake_test_enabled;
    g_brake_test_holding = false;
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
    g_abort_reason = DB_ABORT_NONE;
    item_flag = 1;
    g_session_start = DriveByClock::now();
    g_recognition_start = g_session_start;
    g_last_integration_time = g_session_start;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_report = RecognitionReport{};
    g_report.trigger_left_rps = encoder1_speed_avg;
    g_report.trigger_right_rps = encoder2_speed_avg;
    g_debug = DriveByDebug{};
    g_debug.brake_test_holding = 0;
    g_debug.test_target_distance_m = drive_by_test_target_distance_m;
    g_debug.red_candidate = 1;
    g_debug.red_candidate_count = g_far_candidate_count;
    g_debug.red_contour_area = candidate.max_contour_area;
    have_target = false;
    red_block_rect = cv::Rect();
    plate_rect = cv::Rect();
    red_contour_area = 0;
    command_recognition_base_speed();
    start_active_brake();
    enter_state(DB_APPROACH);
}

void mark_recognition_trigger(double trigger_detect_ms)
{
    if (g_recognition_triggered) {
        return;
    }
    g_recognition_triggered = true;
    if (g_stop_after_current_recognition) {
        g_stop_at_next_target_armed = false;
    }
    g_recognition_start = DriveByClock::now();
    g_report.trigger_detect_ms = trigger_detect_ms;
    g_report.trigger_left_rps = encoder1_speed_avg;
    g_report.trigger_right_rps = encoder2_speed_avg;
}

void cancel_false_candidate()
{
    // 远距离检测只负责提前减速。300ms内始终达不到严格红块条件时，
    // 恢复原巡线输出，短暂冷却后允许同一目标在变大时再次触发。
    clear_active_brake(true);
    motor_speed_pid_reset();
    restore_control();
    gyro_yaw_rate_control_reset_controller();
    g_drive_by_busy = false;
    g_stop_after_current_recognition = false;
    g_state = DB_IDLE;
    g_debug.state_code = (int)DB_IDLE;
    g_cooldown_start = DriveByClock::now();
    g_candidate_retry_after = g_cooldown_start +
        std::chrono::milliseconds(kCandidateRetryCooldownMs);
    g_seen_lock = false;
    reset_far_candidate_window();
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_brake_test_active = false;
    g_brake_test_holding = false;
    g_stop_requested = false;
    g_visual_aim_line_valid = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
    g_debug.brake_test_holding = 0;
    g_debug.red_candidate = 0;
    g_debug.red_candidate_count = 0;
    g_debug.detection_stage = 0;
    have_target = false;
    red_block_rect = cv::Rect();
    plate_rect = cv::Rect();
    red_contour_area = 0;
}

void begin_motion_script(int result)
{
    // 识别结果0为左绕、2为右绕。g_turn_side只描述绕行侧，不直接等同于
    // 方向环误差符号；具体符号在视觉方案和角度方案中分别转换。
    g_turn_side = result == 0 ? 1 : -1;
    // 只保留0三阶段和1边线方案。即使旧配置残留值2，也安全回退到
    // 三阶段，不会被误映射成另一条仍在使用的路线。
    // 边线绕行已经停用。保留旧实现仅用于历史回退，运行入口固定走三阶段脚本。
    g_active_mode = 0;

    g_motion_heading_quiet = false;

    if (g_active_mode == 1) {
        // 边线方案只依赖相机方向误差，不要求目标切线或陀螺仪有效。
        // 普通方向环仍可按#gyro设置使用角速度反馈，但脚本自身不积分角度。
        clear_active_brake(false);
        g_visual_aim_line_valid = false;
        gyro_yaw_rate_control_reset_controller();
        command_motion_wheels((float)drive_by_forward_speed_rps, 0.0f);
        enter_state(DB_FOLLOW_SIDE_LINE);
        return;
    }

    if (!g_target_geometry_captured) {
        g_abort_reason = DB_ABORT_NO_TARGET_GEOMETRY;
        g_debug.abort_reason = (int)g_abort_reason;
        enter_state(DB_FINISH_PENDING);
        return;
    }
    if (!gyro_yaw_rate_control_is_ready()) {
        g_abort_reason = DB_ABORT_GYRO_NOT_READY;
        g_debug.abort_reason = (int)g_abort_reason;
        enter_state(DB_FINISH_PENDING);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        g_abort_reason = DB_ABORT_GYRO_STALE;
        g_debug.abort_reason = (int)g_abort_reason;
        enter_state(DB_FINISH_PENDING);
        return;
    }

    // 切线参考默认关闭：实车发现目标处切线容易受弯道拟合和短线段扰动。
    // 开启时恢复原来的目标处切线；关闭时以脚本真正开始这一刻的车头为0度。
    // 参考值在这里锁存，绕行中途修改VOFA开关不会造成目标航向突变。
    clear_active_brake(false);
    g_track_heading_reference_deg = drive_by_use_track_tangent != 0
        ? g_target_track_heading_global_deg
        : g_yaw_deg;
    g_track_reference_valid = true;
    gyro_yaw_rate_control_reset_controller();

    // 方案一中，左绕和右绕只改变相对赛道切线的偏角符号。
    enter_state(DB_TURN_OUT);
}

bool heading_is_settled(float heading_error_deg, float gyro_dps)
{
    if (std::fabs(heading_error_deg) <= drive_by_heading_tolerance_deg &&
        std::fabs(gyro_dps) <= drive_by_rate_tolerance_dps) {
        ++g_heading_settle_count;
    } else {
        g_heading_settle_count = 0;
    }
    return g_heading_settle_count >= kHeadingSettleCycles;
}

bool heading_target_has_been_crossed(float target_offset_deg,
                                     float turn_direction_sign)
{
    // g_track_heading_reference_deg是本次绕行锁定的0度参考。把当前积分航向和
    // 目标航向都投影到本阶段的转动方向后，只要当前进度达到目标进度，就说明
    // 车头已经到达或越过对应角度。该判断不要求角速度先降到接近0，碰撞、打滑
    // 或参数偏激时也不会因为无法“稳定三拍”而永远卡在同一转向阶段。
    const float direction_sign = turn_direction_sign >= 0.0f ? 1.0f : -1.0f;
    const float current_offset_deg = normalize_angle_deg(
        g_yaw_deg - g_track_heading_reference_deg);
    const float current_progress_deg = current_offset_deg * direction_sign;
    const float target_progress_deg = target_offset_deg * direction_sign;
    return current_progress_deg >= target_progress_deg;
}

bool return_yaw_has_crossed_zero()
{
    // g_yaw_deg以进入绕行时的车头为0度。左右绕的返回方向相反，因此先按
    // 返回方向归一化：左绕回正后为正，右绕回正后也转换为正。
    const float return_direction_sign =
        -g_turn_side * drive_by_yaw_sign;
    const float normalized_return_yaw_deg =
        g_yaw_deg * return_direction_sign;
    // 不在刚越过0度时立即交还，而是再多转10度，让车头微微朝向道路，
    // 降低视觉方向环接管后重新把车拉向赛道外侧的概率。
    return normalized_return_yaw_deg > kReturnHandoffYawDeg;
}

float phase_offset_deg()
{
    const float side_offset = g_turn_side * drive_by_yaw_sign * drive_by_turn_angle_deg;
    const float return_offset =
        -g_turn_side * drive_by_yaw_sign * std::fabs((float)drive_by_return_bias_deg);
    switch (g_state) {
    case DB_TURN_OUT:
    case DB_PASS_SHORT:
        return side_offset;
    case DB_TURN_TO_TRACK:
        return return_offset;
    default: return 0.0f;
    }
}

float phase_base_speed_rps()
{
    switch (g_state) {
    case DB_PASS_SHORT: return (float)drive_by_forward_speed_rps;
    case DB_TURN_TO_TRACK: return (float)drive_by_exit_speed_rps;
    default: return (float)drive_by_turn_speed_rps;
    }
}

bool motion_phase_guard_exceeded()
{
    if (g_state == DB_FOLLOW_SIDE_LINE) {
        // 边线方案允许在线调到2000ms，不能继续套用旧的1.5s/0.8m转向保护，
        // 否则合法参数会在计时结束前被误判为阶段超时。额外500ms只作为调度余量，
        // 这里不能再加固定距离上限，否则高dbForwardRps会让合法的2000ms提前结束。
        return elapsed_ms(g_state_start) >
            drive_by_side_follow_ms + kVisualSideFollowTimeoutMarginMs;
    }
    if (g_state == DB_PASS_SHORT) {
        return elapsed_ms(g_state_start) > kPassPhaseTimeoutMs ||
            g_phase_distance_m > kPassPhaseMaxDistanceM;
    }
    return elapsed_ms(g_state_start) > kTurnPhaseTimeoutMs ||
        g_phase_distance_m > kTurnPhaseMaxDistanceM;
}

float visual_forced_error(bool recovering_center)
{
    if (recovering_center) {
        // 左绕结束后向右找中线，右绕结束后向左找中线。
        return g_turn_side > 0 ? kVisualRecoveryError : -kVisualRecoveryError;
    }
    // 500ms边线阶段若所选边线短暂消失，继续向原绕行侧转，避免立刻回头。
    return g_turn_side > 0 ? -kVisualRecoveryError : kVisualRecoveryError;
}

bool recovery_uses_gyro_rate_control()
{
    return gyro_yaw_rate_feedback_enabled != 0 &&
        gyro_yaw_rate_control_is_ready();
}

void request_motion_stop(DriveByAbortReason reason)
{
    // 8ms方向线程不直接操作PWM。这里只清目标并提交停车请求，3ms速度线程
    // 会在下一个周期执行真正的停车，最坏额外等待约3ms。
    g_abort_reason = reason;
    g_debug.abort_reason = (int)reason;
    g_stop_requested = true;
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
}

void complete_motion_handoff()
{
    // 该函数运行在dir_timer中，只修改缓存控制状态，不能在这里打印。
    // busy清零后，main.cpp会在同一个8ms回调内使用最新视觉误差执行普通方向环。
    clear_active_brake(true);
    gyro_yaw_rate_control_reset_controller();

    // 不恢复识别前保存的旧差速，普通方向环会立刻计算一份新的差速。
    g_saved.valid = false;
    command_motion_wheels(drive_by_normal_speed_rps, 0.0f);

    g_abort_reason = DB_ABORT_NONE;
    g_debug.abort_reason = (int)DB_ABORT_NONE;
    g_drive_by_busy = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_stop_requested = false;
    g_visual_aim_line_valid = false;
    g_motion_heading_quiet = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
    g_state = DB_IDLE;
    g_debug.state_code = (int)DB_IDLE;
    reset_far_candidate_window();
    g_debug.red_candidate = 0;
    g_debug.red_candidate_count = 0;
    g_debug.detection_stage = 0;
    g_cooldown_start = DriveByClock::now();

    // 报告留给相机线程输出，避免控制权交接被printf阻塞。
    g_report_pending = true;
}

void update_motion_state()
{
    if (!gyro_yaw_rate_control_is_ready()) {
        request_motion_stop(DB_ABORT_GYRO_NOT_READY);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        request_motion_stop(DB_ABORT_GYRO_STALE);
        return;
    }
    if (motion_phase_guard_exceeded()) {
        request_motion_stop(DB_ABORT_PHASE_TIMEOUT);
        return;
    }

    const float gyro_dps = gyro_yaw_rate_control_get_gyro_z_dps();
    const float target_offset_deg = phase_offset_deg();
    const float target_yaw_deg = normalize_angle_deg(
        g_track_heading_reference_deg + target_offset_deg);
    const float heading_error_deg = normalize_angle_deg(target_yaw_deg - g_yaw_deg);
    const bool quiet_before_update = g_motion_heading_quiet;
    const HeadingControlZone control_zone = update_heading_control_zone(
        heading_error_deg, gyro_dps, &g_motion_heading_quiet);
    if (quiet_before_update != g_motion_heading_quiet) {
        // 切入或退出静默区时清角速度内环历史，防止上一次差速残留到新状态。
        // 绕行仍保留基准轮速，因此这里绝不能复位整套电机速度PID。
        gyro_yaw_rate_control_reset_controller();
    }

    float target_yaw_rate_dps = 0.0f;
    float turn_rps = 0.0f;
    if (control_zone != HEADING_CONTROL_QUIET) {
        if (control_zone == HEADING_CONTROL_FULL) {
            target_yaw_rate_dps =
                drive_by_heading_kp * heading_error_deg -
                drive_by_heading_kd * gyro_dps;
            const float rate_limit = std::fabs(
                (float)drive_by_heading_max_dps);
            target_yaw_rate_dps = clampf(
                target_yaw_rate_dps, -rate_limit, rate_limit);
        }
        // BRAKE区故意给0dps：内环会根据实际gyro产生反向差速，主动消除惯性。
        turn_rps = gyro_yaw_rate_control_update_target_yaw_rate(
            target_yaw_rate_dps);
    }
    command_motion_wheels(phase_base_speed_rps(), turn_rps);

    g_debug.yaw_deg = g_yaw_deg;
    g_debug.target_yaw_deg = target_yaw_deg;
    g_debug.heading_error_deg = heading_error_deg;
    g_debug.track_heading_deg = g_track_heading_reference_deg;
    g_debug.target_yaw_rate_dps = target_yaw_rate_dps;
    g_debug.turn_rps = turn_rps;

    switch (g_state) {
    case DB_TURN_OUT: {
        const float turn_out_direction_sign = g_turn_side * drive_by_yaw_sign;
        const bool target_angle_crossed = heading_target_has_been_crossed(
            target_offset_deg, turn_out_direction_sign);
        const bool settled_without_crossing = g_motion_heading_quiet &&
            heading_is_settled(heading_error_deg, gyro_dps);
        if (target_angle_crossed || settled_without_crossing) {
            // TURN_OUT和PASS_SHORT保持同一个目标航向，过线后立即切阶段不会
            // 产生目标角跳变，只是不再要求车头先停止旋转。
            enter_state(DB_PASS_SHORT);
        } else if (!g_motion_heading_quiet) {
            g_heading_settle_count = 0;
        }
        break;
    }

    case DB_PASS_SHORT: {
        const float pass_end_distance =
            g_target_distance_at_trigger_m + drive_by_target_after_margin_m;
        if (g_phase_distance_m >= drive_by_pass_distance_m &&
            g_distance_since_trigger_m >= pass_end_distance) {
            // 第二次转向期间开始观察中线。角度达到后只有中线可靠才直接交还，
            // 否则进入反方向满误差找线，避免车头朝向正确但车辆仍在赛道外侧。
            g_visual_aim_line_valid = false;
            enter_state(DB_TURN_TO_TRACK);
        }
        break;
    }

    case DB_TURN_TO_TRACK:
        // 相机可能先看到前方弯道或邻近道路，因此不能只凭check_line_lost()
        // 就提前交还。必须等陀螺仪积分沿回正方向越过本次绕行起点10度，确认
        // 车头已经微微朝向道路后，才允许恢复巡线并避免继续转向过度。
        if (return_yaw_has_crossed_zero() && g_visual_aim_line_valid) {
            complete_motion_handoff();
            break;
        }

        {
            const float return_direction_sign =
                -g_turn_side * drive_by_yaw_sign;
            const bool target_angle_crossed = heading_target_has_been_crossed(
                target_offset_deg, return_direction_sign);
            const bool settled_without_crossing = g_motion_heading_quiet &&
                heading_is_settled(heading_error_deg, gyro_dps);
            if (target_angle_crossed) {
                if (g_visual_aim_line_valid) {
                    complete_motion_handoff();
                } else {
                    enter_state(DB_RECOVER_CENTER_LINE);
                    latest_error = visual_forced_error(true);
                }
            } else if (settled_without_crossing) {
                // 尚未越线但已经稳定在容差内时仍保留旧兜底，避免控制器刚好
                // 停在目标角前一点而无法继续。看不到中线则进入原找线保护。
                enter_state(DB_RECOVER_CENTER_LINE);
                latest_error = visual_forced_error(true);
            } else if (!g_motion_heading_quiet) {
                g_heading_settle_count = 0;
            }
        }
        break;

    default:
        break;
    }
}

void update_visual_motion_state()
{
    if (motion_phase_guard_exceeded()) {
        request_motion_stop(DB_ABORT_PHASE_TIMEOUT);
        return;
    }

    // 新方案使用dbForwardRps；旧三阶段在额外找线时保持第二转的dbExitRps，
    // 避免刚结束角度闭环就突然提速。差速仍由随后执行的普通方向环计算。
    const float visual_base_rps = g_active_mode == 1
        ? (float)drive_by_forward_speed_rps
        : (float)drive_by_exit_speed_rps;
    set_speed_of_motor1_rps = visual_base_rps;
    set_speed_of_motor2_rps = visual_base_rps;

    if (g_state == DB_FOLLOW_SIDE_LINE &&
        elapsed_ms(g_state_start) >= drive_by_side_follow_ms) {
        // 先清掉边线帧的有效标志，再切回中线。切换当周期先标记为丢线，
        // 后续只有相机线程明确提交一帧有效中线后才允许交还普通巡线。
        g_visual_aim_line_valid = false;
        enter_state(DB_RECOVER_CENTER_LINE);
        latest_error = visual_forced_error(true);
        return;
    }

    if (g_state == DB_RECOVER_CENTER_LINE) {
        if (g_visual_aim_line_valid) {
            complete_motion_handoff();
            return;
        }

        if (recovery_uses_gyro_rate_control()) {
            // 丢线保护直接给目标角速度，不再把100误差交给视觉外环换算。
            // 左绕回程向右为正，右绕回程向左为负；实际轮速差仍由现有角速度内环生成。
            const float recovery_rate = std::fabs(
                (float)drive_by_recovery_yaw_rate_dps);
            const float target_yaw_rate_dps =
                g_turn_side > 0 ? recovery_rate : -recovery_rate;
            const float turn_rps =
                gyro_yaw_rate_control_update_target_yaw_rate(
                    target_yaw_rate_dps);
            command_motion_wheels(visual_base_rps, turn_rps);
            g_debug.target_yaw_rate_dps = target_yaw_rate_dps;
            g_debug.turn_rps = turn_rps;
        }
    }
}

void complete_inference()
{
    g_report.total_ms = duration_ms(g_recognition_start, DriveByClock::now());
    g_report.finish_left_rps = encoder1_speed_avg;
    g_report.finish_right_rps = encoder2_speed_avg;
    item_flag = choose_vote_result();

    g_pending_result = item_flag;
    g_inference_complete = true;
    if (g_brake_test_active) {
        complete_brake_test_recognition(DB_ABORT_NONE);
        return;
    }
    if (g_stop_after_current_recognition) {
        if (!g_brake_completed || g_brake_active) {
            enter_state(DB_WAIT_BRAKE);
        } else {
            command_zero_targets();
            enter_state(DB_FINISH_PENDING);
        }
        return;
    }
    if (!g_brake_completed || g_brake_active) {
        // 无论投票结果是否需要左右绕行，都先等主动制动完整结束。
        // 这样“直行”结果不会在仍高于释放速度时突然恢复35RPS。
        enter_state(DB_WAIT_BRAKE);
    } else if (g_pending_result == 0 || g_pending_result == 2) {
        enter_state(DB_START_MOTION_PENDING);
    } else {
        finish_session(DB_ABORT_NONE);
    }
}

void update_busy_state(cv::Mat& frame, LQ_NCNN& ncnn)
{
    switch (g_state) {
    case DB_APPROACH: {
        command_recognition_base_speed();
        const DriveByClock::time_point detect_start = DriveByClock::now();
        detectRedPlate(frame);
        const double detect_ms = duration_ms(detect_start, DriveByClock::now());
        g_debug.red_contour_area = red_contour_area;
        if (have_target) {
            mark_recognition_trigger(detect_ms);
            // APPROACH成立后主循环已经暂停十字/环岛，因此这里使用的是上一帧
            // 重新生成的普通中线，不会继承旧环岛的单侧贴线状态。
            drive_by_update_track_geometry();
            if (g_debug.view_ready != 0) {
                enter_state(DB_INFER);
                process_inference_frame(frame, ncnn, true, detect_ms);
                if (recognition_should_finish()) {
                    complete_inference();
                }
            } else {
                enter_state(DB_WAIT_VIEW);
            }
        } else if (elapsed_ms(g_state_start) >= kApproachTimeoutMs) {
            cancel_false_candidate();
        }
        break;
    }

    case DB_WAIT_VIEW:
        command_recognition_base_speed();
        {
            const DriveByClock::time_point detect_start = DriveByClock::now();
            detectRedPlate(frame);
            const double detect_ms = duration_ms(detect_start, DriveByClock::now());
            g_debug.red_contour_area = red_contour_area;
            drive_by_update_track_geometry();
            if (g_debug.view_ready != 0 && have_target) {
                enter_state(DB_INFER);
                process_inference_frame(frame, ncnn, true, detect_ms);
                if (recognition_should_finish()) {
                    complete_inference();
                }
            } else if (elapsed_ms(g_state_start) >= drive_by_view_wait_timeout_ms) {
                item_flag = 1;
                if (g_brake_test_active) {
                    complete_brake_test_recognition(DB_ABORT_VIEW_TIMEOUT);
                } else if (g_stop_after_current_recognition) {
                    g_pending_result = item_flag;
                    g_inference_complete = true;
                    g_abort_reason = DB_ABORT_VIEW_TIMEOUT;
                    g_debug.abort_reason = (int)DB_ABORT_VIEW_TIMEOUT;
                    enter_state(g_brake_completed && !g_brake_active
                        ? DB_FINISH_PENDING
                        : DB_WAIT_BRAKE);
                } else {
                    finish_session(DB_ABORT_VIEW_TIMEOUT);
                }
            }
        }
        break;

    case DB_INFER:
        command_recognition_base_speed();
        process_inference_frame(frame, ncnn, false, 0.0);
        if (recognition_should_finish()) {
            complete_inference();
        } else if (elapsed_ms(g_state_start) >= drive_by_infer_timeout_ms) {
            item_flag = 1;
            if (g_brake_test_active) {
                complete_brake_test_recognition(DB_ABORT_VIEW_TIMEOUT);
            } else if (g_stop_after_current_recognition) {
                g_pending_result = item_flag;
                g_inference_complete = true;
                g_abort_reason = DB_ABORT_VIEW_TIMEOUT;
                g_debug.abort_reason = (int)DB_ABORT_VIEW_TIMEOUT;
                enter_state(g_brake_completed && !g_brake_active
                    ? DB_FINISH_PENDING
                    : DB_WAIT_BRAKE);
            } else {
                finish_session(DB_ABORT_VIEW_TIMEOUT);
            }
        }
        break;

    case DB_WAIT_BRAKE:
        command_recognition_base_speed();
        break;

    case DB_START_MOTION_PENDING:
        // 由下一次8ms方向周期启动闭环，相机线程不写运动控制量。
        break;

    case DB_TURN_OUT:
    case DB_PASS_SHORT:
    case DB_TURN_TO_TRACK:
        // 三阶段航向闭环在8ms dir_timer中运行，相机线程只更新道路几何缓存。
        break;

    case DB_FOLLOW_SIDE_LINE:
    case DB_RECOVER_CENTER_LINE:
        // 新方案的相机工作仍在main.cpp中完成；这里不能重复运行寻线或方向环。
        break;

    case DB_FINISH_PENDING:
        if (g_stop_after_current_recognition) {
            const RecognitionReport report = g_report;
            const int final_result = item_flag;
            const TargetStopReport target_report =
                make_target_stop_report(report, final_result);

            g_drive_by_busy = false;
            g_stop_after_current_recognition = false;
            command_zero_targets();
            motor_speed_force_pwm(0, 0);
            front_ui_stop();

            print_target_stop_report(target_report);
        } else {
            finish_session(g_abort_reason);
        }
        break;

    default:
        finish_session(DB_ABORT_PHASE_TIMEOUT);
        break;
    }
}

void update_idle_detection(cv::Mat& frame, LQ_NCNN& ncnn)
{
    (void)ncnn;
    if (!should_detect_this_frame()) {
        return;
    }

    const FarRedCandidate candidate = detect_far_red_candidate(frame);
    g_debug.red_candidate = candidate.found ? 1 : 0;
    g_debug.red_contour_area = candidate.max_contour_area;
    g_debug.detection_stage = 0;
    have_target = false;
    red_block_rect = cv::Rect();
    plate_rect = cv::Rect();
    red_contour_area = 0;

    if (DriveByClock::now() < g_candidate_retry_after) {
        stop_candidate_brake();
        reset_far_candidate_window();
        g_debug.red_candidate_count = 0;
        command_idle_candidate_speed();
        return;
    }

    if (g_seen_lock) {
        stop_candidate_brake();
        reset_far_candidate_window();
        g_debug.red_candidate_count = 0;
        command_idle_candidate_speed();
        if (!candidate.found && elapsed_ms(g_cooldown_start) >= drive_by_cooldown_ms) {
            g_seen_lock = false;
        }
        return;
    }

    const bool candidate_confirmed =
        update_far_candidate_window(candidate.found);
    g_debug.red_candidate_count = g_far_candidate_count;
    if (candidate_confirmed) {
        start_approach_session(candidate);
        return;
    }
    const bool early_brake_enabled = control_profile_is_pro() ||
        g_stable_early_brake_enabled || g_stop_at_next_target_armed;
    if (early_brake_enabled && candidate.found) {
        start_candidate_brake();
    } else if (g_candidate_brake_active) {
        stop_candidate_brake();
    }
    command_idle_candidate_speed();
}

} // namespace

void drive_by_init()
{
    g_drive_by_enable = false;
    g_stable_early_brake_enabled = false;
    g_stop_at_next_target_armed = false;
    g_stop_after_current_recognition = false;
    g_brake_test_enabled = false;
    g_brake_test_report = BrakeTestReport{};
    g_brake_test_report_pending = false;
    g_target_stop_report = TargetStopReport{};
    g_target_stop_report_pending = false;
    g_heading_hold_enabled = false;
    g_heading_hold_quiet = false;
    g_tangent_debug_enabled = false;
    drive_by_detect_frame_interval = 2;
    g_detect_frame_counter = 0;
    g_heading_hold_debug = DriveByHeadingHoldDebug{};
    g_tangent_debug = DriveByTangentDebug{};
    reset_runtime(false);
}

void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn)
{
    if (g_target_stop_report_pending) {
        print_target_stop_report();
    }
    if (g_brake_test_report_pending) {
        print_brake_test_report();
    }
    if (g_report_pending) {
        // 8ms控制回调已经恢复普通巡线，这里只补做不会影响转向的报告输出。
        g_report_pending = false;
        print_recognition_report(item_flag);
    }
    if (!front_ui_is_running() && !g_drive_by_busy) {
        return;
    }

    // 刹车测试开关可以在停车时预先开启，但只有run=1才启动真实红块检测。
    // 已经触发的测试和TEST绕行不受K0开关变化影响，必须继续完成安全停车。
    const bool real_detection_enabled = g_drive_by_enable ||
        (g_brake_test_enabled && front_ui_is_running());
    if (!real_detection_enabled && !g_test_mode && !g_brake_test_active) {
        if (g_drive_by_busy || g_candidate_brake_active) {
            reset_runtime(true);
        }
        return;
    }
    if (frame.empty()) {
        return;
    }

    if (g_drive_by_busy) {
        update_busy_state(frame, ncnn);
    } else {
        // 默认每2帧才执行一次红块检测；未检测的中间帧也必须维持第一票候选的预减速。
        command_idle_candidate_speed();
        update_idle_detection(frame, ncnn);
    }
}

void drive_by_on_start()
{
    g_stop_at_next_target_armed = false;
    g_stop_after_current_recognition = false;
}

bool drive_by_on_zebra_detected()
{
    if (!g_drive_by_enable) {
        return false;
    }
    if (!g_stop_at_next_target_armed && !g_stop_after_current_recognition) {
        g_stop_at_next_target_armed = true;
    }
    return true;
}

bool drive_by_has_pending_report()
{
    return g_report_pending || g_brake_test_report_pending ||
        g_target_stop_report_pending;
}

void drive_by_control_update()
{
    if (!g_drive_by_busy) {
        return;
    }

    // 编码器和陀螺仪均读取现有缓存，dt使用真实调度间隔。该函数不访问相机、
    // NCNN、I2C或PWM硬件，因此可以稳定放在8ms方向定时器中。
    integrate_session_motion();
    if (g_state == DB_START_MOTION_PENDING && !g_stop_requested) {
        // 速度线程只提交该状态；航向闭环的初始化与首次更新统一留在方向线程。
        begin_motion_script(g_pending_result);
    }
    if (drive_by_is_motion_phase() && !g_stop_requested) {
        // RECOVER_CENTER_LINE是两套方案共用的视觉找线状态，不能继续执行
        // 旧方案的航向闭环，否则两个控制器会同时改写左右轮目标。
        if (g_state == DB_FOLLOW_SIDE_LINE ||
            g_state == DB_RECOVER_CENTER_LINE) {
            update_visual_motion_state();
        } else {
            update_motion_state();
        }
    }
}

bool drive_by_speed_control_update()
{
    if (g_stop_requested) {
        const DriveByAbortReason reason = g_abort_reason;
        const int stopped_brake_elapsed_ms = g_debug.brake_elapsed_ms;
        g_stop_requested = false;
        // 推理完成后因运动异常（陀螺仪/超时）停车：在清理数据前先输出识别报告。
        // 否则 front_ui_stop() → drive_by_cancel() → reset_runtime() 会清空 g_report，
        // 导致 printf 永远输出"未知"。
        if (g_inference_complete && g_report.valid_count > 0) {
            print_recognition_report(item_flag);
        }
        front_ui_stop();
        // front_ui_stop()会取消脚本并清空运行态；把原因写回调试快照，便于事后定位。
        g_abort_reason = reason;
        g_debug.abort_reason = (int)reason;
        g_debug.brake_elapsed_ms = stopped_brake_elapsed_ms;
        return true;
    }

    if (g_brake_test_active && g_brake_test_holding) {
        command_zero_targets();
        motor_speed_force_pwm(0, 0);
        g_debug.brake_test_holding = 1;
        if (g_inference_complete) {
            finish_brake_test_stop(g_abort_reason);
        }
        return true;
    }

    if (g_stop_after_current_recognition && g_brake_completed &&
        !g_brake_active) {
        command_zero_targets();
        motor_speed_force_pwm(0, 0);
        if (g_inference_complete && g_state == DB_WAIT_BRAKE) {
            enter_state(DB_FINISH_PENDING);
        }
        return true;
    }

    if (g_candidate_brake_active && !g_brake_active) {
        command_zero_targets();
        motor_speed_force_pwm(0, 0);
        return true;
    }

    if (!g_brake_active) {
        return false;
    }

    const int brake_elapsed_ms = (int)elapsed_ms(g_brake_start);
    g_debug.brake_elapsed_ms = brake_elapsed_ms;
    if (g_brake_test_active) {
        g_brake_test_elapsed_ms = brake_elapsed_ms;
    }

    const float release_rps = std::fabs((float)drive_by_brake_release_rps);
    // 主动制动只用于车辆原本向前行驶的场景。这里按“正向速度已经降到阈值”
    // 判断，不再取绝对值；若大制动力让轮速轻微穿过零点，负速度仍会满足释放条件，
    // 避免继续反向加速直到制动超时。连续确认次数仍保留，用于过滤单次编码器毛刺。
    const bool both_wheels_slow =
        encoder1_speed_avg <= release_rps &&
        encoder2_speed_avg <= release_rps;
    g_brake_confirm_count = both_wheels_slow ? g_brake_confirm_count + 1 : 0;

    const int required_count = std::max(1, (int)drive_by_brake_confirm_count);
    if (g_brake_confirm_count >= required_count) {
        // 先撤销PWM独占并完整复位增量式PID，再让main.cpp在同一个3ms周期
        // 调用普通速度环，从零状态接管当前识别基准速度目标。
        clear_active_brake(false);
        g_brake_completed = true;
        motor_speed_pid_reset();
        if (g_candidate_brake_active) {
            command_zero_targets();
            motor_speed_force_pwm(0, 0);
            return true;
        }
        if (g_brake_test_active) {
            g_brake_test_holding = true;
            g_brake_test_release_left_rps = encoder1_speed_avg;
            g_brake_test_release_right_rps = encoder2_speed_avg;
            g_debug.brake_test_holding = 1;
            command_zero_targets();
            motor_speed_force_pwm(0, 0);
            if (g_inference_complete) {
                finish_brake_test_stop(g_abort_reason);
            }
            return true;
        }
        command_recognition_base_speed();
        if (g_inference_complete && g_state == DB_WAIT_BRAKE) {
            if (g_stop_after_current_recognition) {
                command_zero_targets();
                motor_speed_force_pwm(0, 0);
                enter_state(DB_FINISH_PENDING);
                return true;
            } else if (g_pending_result == 0 || g_pending_result == 2) {
                enter_state(DB_START_MOTION_PENDING);
            } else {
                // 报告打印与参数恢复交给相机线程，3ms速度线程只提交结束状态。
                enter_state(DB_FINISH_PENDING);
            }
        }
        return false;
    }

    if (brake_elapsed_ms >= std::max(1, (int)drive_by_brake_timeout_ms)) {
        const DriveByAbortReason reason = DB_ABORT_BRAKE_TIMEOUT;
        if (g_brake_test_active) {
            g_brake_test_release_left_rps = encoder1_speed_avg;
            g_brake_test_release_right_rps = encoder2_speed_avg;
            g_brake_test_elapsed_ms = brake_elapsed_ms;
            finish_brake_test_stop(reason);
            return true;
        }
        if (g_stop_after_current_recognition) {
            g_target_stop_report = make_target_stop_report(g_report, item_flag);
            g_target_stop_report_pending = true;
            g_drive_by_busy = false;
            g_stop_after_current_recognition = false;
        }
        clear_active_brake(true);
        motor_speed_pid_reset();
        g_abort_reason = reason;
        g_debug.abort_reason = (int)reason;
        front_ui_stop();
        // 停车会复位脚本状态，重新写回原因供UDP事后查看。
        g_abort_reason = reason;
        g_debug.abort_reason = (int)reason;
        g_debug.brake_elapsed_ms = brake_elapsed_ms;
        return true;
    }

    const int brake_pwm = std::min(kBrakePwmMax,
                                   std::abs((int)drive_by_brake_pwm));
    g_debug.brake_active = 1;
    g_debug.brake_pwm = -brake_pwm;
    motor_speed_force_brake_pwm(-brake_pwm, -brake_pwm);
    return true;
}

bool drive_by_start_test(int simulated_item_flag,
                         float simulated_target_distance_m)
{
    if (!front_ui_is_running() || g_drive_by_busy || g_brake_test_enabled ||
        (simulated_item_flag != 0 && simulated_item_flag != 2)) {
        return false;
    }

    save_control_once();
    g_drive_by_busy = true;
    g_test_mode = true;
    g_seen_lock = false;
    g_recognition_triggered = true;
    g_target_geometry_captured = true;
    g_track_reference_valid = false;
    g_abort_reason = DB_ABORT_NONE;
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_active_mode = 0;
    g_inference_complete = true;
    g_pending_result = simulated_item_flag;
    item_flag = simulated_item_flag;

    const DriveByClock::time_point now = DriveByClock::now();
    g_session_start = now;
    g_recognition_start = now;
    g_last_integration_time = now;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_report = RecognitionReport{};
    g_report.trigger_left_rps = encoder1_speed_avg;
    g_report.trigger_right_rps = encoder2_speed_avg;

    float track_heading_relative_deg = 0.0f;
    g_current_track_heading_valid =
        compute_current_track_heading(&track_heading_relative_deg);
    g_current_track_heading_relative_deg = g_current_track_heading_valid
        ? track_heading_relative_deg
        : 0.0f;
    // TEST只验证左右绕行脚本本身，因此把按键瞬间的车头方向固定为0度基准。
    // 若把当前中线切线叠加进来，切线角可能抵消某一侧的25度目标，造成
    // 左绕明显而右绕几乎不转的假性不对称。真实识别仍使用目标位置处切线。
    g_target_track_heading_global_deg = g_yaw_deg;
    g_target_distance_at_trigger_m = std::max(0.0f, simulated_target_distance_m);

    g_debug = DriveByDebug{};
    g_debug.test_mode = 1;
    g_debug.target_geometry_valid = 1;
    g_debug.view_ready = 1;
    g_debug.target_track_heading_deg = g_target_track_heading_global_deg;
    g_debug.target_distance_m = g_target_distance_at_trigger_m;
    g_debug.test_target_distance_m = g_target_distance_at_trigger_m;

    command_recognition_base_speed();
    start_active_brake();
    enter_state(DB_WAIT_BRAKE);
    return true;
}

void drive_by_update_track_geometry()
{
    // 中线切线调试与绕行状态完全独立。关闭时这里只多一次布尔判断；开启后
    // 只复用本帧已经生成的rptsn，不执行额外图像处理，也不写任何控制量。
    update_tangent_debug_cache();

    if (!g_drive_by_busy) {
        return;
    }

    float current_heading_relative_deg = 0.0f;
    g_current_track_heading_valid =
        compute_current_track_heading(&current_heading_relative_deg);
    if (g_current_track_heading_valid) {
        g_current_track_heading_relative_deg = current_heading_relative_deg;
    }

    // 目标位置和观察夹角只在识别阶段需要。进入三阶段运动后锁定目标位置处切线，
    // 不再用横移时的视觉中线更新目标，否则目标航向会跟着车身一起漂移。
    float target_track_heading_relative_deg = 0.0f;
    float view_angle_deg = g_debug.view_angle_deg;
    float target_distance_now_m = 0.0f;
    const bool target_geometry_valid = !drive_by_is_motion_phase() &&
        compute_target_geometry(&target_track_heading_relative_deg,
                                &view_angle_deg,
                                &target_distance_now_m);

    if (target_geometry_valid) {
        g_debug.view_angle_deg = view_angle_deg;
        if (!g_target_geometry_captured) {
            // 如果几何信息在触发后的下一帧才可用，需要把已经走过的距离加回来，
            // 才能得到“红色触发瞬间到目标”的沿线距离。
            g_target_distance_at_trigger_m =
                target_distance_now_m + g_distance_since_trigger_m;
            g_target_track_heading_global_deg = normalize_angle_deg(
                g_yaw_deg + target_track_heading_relative_deg);
            g_target_geometry_captured = true;
        }
    }

    g_debug.target_geometry_valid = g_target_geometry_captured ? 1 : 0;
    g_debug.target_track_heading_deg = g_target_track_heading_global_deg;
    g_debug.target_distance_m = g_target_geometry_captured
        ? g_target_distance_at_trigger_m
        : drive_by_pass_distance_m;
    g_debug.track_heading_deg = g_track_heading_reference_deg;
    g_debug.view_ready = !drive_by_is_motion_phase() &&
        target_geometry_valid &&
        plate_rect.width > 0 && plate_rect.height > 0 &&
        view_angle_deg <= drive_by_view_angle_max_deg
        ? 1
        : 0;
}

bool drive_by_is_busy()
{
    return g_drive_by_busy;
}

bool drive_by_is_recognizing()
{
    return g_drive_by_busy &&
        (g_state == DB_APPROACH || g_state == DB_WAIT_VIEW ||
         g_state == DB_INFER || g_state == DB_WAIT_BRAKE);
}

bool drive_by_is_motion_phase()
{
    return g_drive_by_busy &&
        (g_state == DB_TURN_OUT ||
         g_state == DB_PASS_SHORT ||
         g_state == DB_TURN_TO_TRACK ||
         g_state == DB_FOLLOW_SIDE_LINE ||
         g_state == DB_RECOVER_CENTER_LINE);
}

bool drive_by_uses_visual_direction_control()
{
    if (!g_drive_by_busy) {
        return false;
    }
    if (g_state == DB_FOLLOW_SIDE_LINE) {
        return true;
    }
    // MPU6050可用时，回程丢线由drive_by直接给固定目标角速度；
    // 未开启或未就绪时才保留原来的±100视觉误差兜底。
    return g_state == DB_RECOVER_CENTER_LINE &&
        !recovery_uses_gyro_rate_control();
}

int drive_by_visual_aim_line()
{
    if (!g_drive_by_busy) {
        return 0;
    }
    if (g_state == DB_FOLLOW_SIDE_LINE) {
        return g_turn_side > 0 ? -1 : 1;
    }
    return 0;
}

float drive_by_adjust_visual_error(float computed_error,
                                   int selected_aim_line,
                                   bool aim_line_valid)
{
    if (!g_drive_by_busy) {
        return computed_error;
    }

    if (g_state == DB_FOLLOW_SIDE_LINE) {
        const int expected_line = g_turn_side > 0 ? -1 : 1;
        // selected_aim_line来自本帧开始时的状态快照。只有本帧确实使用了
        // 期望边线，才允许它更新有效标志，避免状态并发切换造成误判。
        const bool valid = selected_aim_line == expected_line && aim_line_valid;
        g_visual_aim_line_valid = valid;
        return valid ? computed_error : visual_forced_error(false);
    }

    if (g_state == DB_TURN_TO_TRACK) {
        // check_line_lost()读取左右边线点数并更新lost枚举，必须留在相机线程调用。
        // 返回阶段只要它判断道路未丢失，就允许8ms方向线程立即交还普通巡线。
        g_visual_aim_line_valid =
            selected_aim_line == 0 && !check_line_lost();
        return computed_error;
    }

    if (g_state == DB_RECOVER_CENTER_LINE) {
        const bool valid = selected_aim_line == 0 && !check_line_lost();
        g_visual_aim_line_valid = valid;
        return valid ? computed_error : visual_forced_error(true);
    }

    return computed_error;
}

bool drive_by_should_suspend_track_features()
{
    // 从远距离候选成立开始就暂停十字/环岛。这样严格识别使用的上一帧中线
    // 已经回到普通中线，不会被旧的环岛阶段或单侧贴线继续污染。
    return g_drive_by_busy;
}

bool drive_by_is_enabled()
{
    return g_drive_by_enable;
}

void drive_by_set_enable(bool enable)
{
    if (g_drive_by_enable == enable) {
        return;
    }
    g_drive_by_enable = enable;
    g_stop_at_next_target_armed = false;
    g_stop_after_current_recognition = false;
    if (!enable && !g_brake_test_enabled) {
        reset_runtime(true);
    }
    printf("[目标板识别] 状态=%s\n", enable ? "开启" : "关闭");
}

void drive_by_toggle_enable()
{
    drive_by_set_enable(!g_drive_by_enable);
}

bool drive_by_brake_test_set_enable(bool enable)
{
    if (enable) {
        if (g_brake_test_enabled) {
            return true;
        }
        if (g_drive_by_busy || g_heading_hold_enabled ||
            front_ui_remote_is_active()) {
            return false;
        }
        g_brake_test_enabled = true;
        if (front_ui_is_running()) {
            // 测试可在车辆已经运行时开启，此时立即使用当前参数组目标速度；
            // 停车时开启则由下一次front_ui_start()应用当前组的相同速度。
            const float target_speed_rps = control_profile_target_speed_rps();
            set_speed_of_motor1_rps = target_speed_rps;
            set_speed_of_motor2_rps = target_speed_rps;
        }
        return true;
    }

    g_brake_test_enabled = false;
    if (g_brake_test_active) {
        // 运行中关闭测试绝不能恢复触发前速度，直接走统一停车清零路径。
        g_drive_by_busy = false;
        front_ui_stop();
    }
    return true;
}

bool drive_by_brake_test_is_enabled()
{
    return g_brake_test_enabled;
}

bool drive_by_brake_test_is_holding()
{
    return g_brake_test_holding;
}

void drive_by_stable_early_brake_set_enable(bool enable)
{
    g_stable_early_brake_enabled = enable;
    if (!enable && !control_profile_is_pro()) {
        stop_candidate_brake();
    }
}

bool drive_by_stable_early_brake_is_enabled()
{
    return g_stable_early_brake_enabled;
}

bool drive_by_cancel()
{
    const bool was_busy = g_drive_by_busy;
    reset_runtime(true);
    return was_busy;
}

const char *drive_by_state_name()
{
    return state_name(g_state);
}

const char *drive_by_abort_reason()
{
    return abort_reason_name(g_abort_reason);
}

const char *drive_by_abort_reason_chinese()
{
    return abort_reason_chinese_name(g_abort_reason);
}

const DriveByDebug &drive_by_get_debug()
{
    return g_debug;
}

bool drive_by_heading_hold_set_enable(bool enable)
{
    if (!enable) {
        if (!g_heading_hold_enabled) {
            return true;
        }

        g_heading_hold_enabled = false;
        g_heading_hold_quiet = false;
        g_heading_hold_debug.enabled = 0;
        g_heading_hold_debug.target_yaw_rate_dps = 0.0f;
        g_heading_hold_debug.turn_rps = 0.0f;
        set_speed_of_motor1_rps = 0.0f;
        set_speed_of_motor2_rps = 0.0f;
        pwm1_duty_rps = 0.0f;
        pwm2_duty_rps = 0.0f;
        gyro_yaw_rate_control_reset_controller();
        motor_speed_pid_reset();
        pwm1.atim_pwm_set_duty(0);
        pwm2.atim_pwm_set_duty(0);
        return true;
    }

    if (g_heading_hold_enabled) {
        return true;
    }
    if (front_ui_is_running() || g_drive_by_busy || g_brake_test_enabled ||
        front_ui_remote_is_active() ||
        !gyro_yaw_rate_control_is_ready() ||
        !gyro_yaw_rate_control_gyro_is_fresh()) {
        return false;
    }

    // 开启瞬间把当前车头定义为0度。之后只积分相对转角，不依赖长期绝对航向，
    // 所以适合原地拨动车头观察KP/KD，而不会把绕行前后的积分误差带进来。
    g_heading_hold_yaw_deg = 0.0f;
    g_heading_hold_quiet = false;
    g_heading_hold_last_update = DriveByClock::now();
    g_heading_hold_debug = DriveByHeadingHoldDebug{};
    g_heading_hold_debug.enabled = 1;
    g_heading_hold_debug.target_yaw_deg = 0.0f;
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
    gyro_yaw_rate_control_reset_controller();
    motor_speed_pid_reset();
    g_heading_hold_enabled = true;
    return true;
}

bool drive_by_heading_hold_is_enabled()
{
    return g_heading_hold_enabled;
}

bool drive_by_heading_hold_is_quiet()
{
    return g_heading_hold_enabled && g_heading_hold_quiet;
}

void drive_by_heading_hold_control_update()
{
    if (!g_heading_hold_enabled) {
        return;
    }

    // 发车、绕行、遥控或陀螺仪失效都必须立即退出，防止停车态调试控制
    // 混入正常巡线，也避免使用旧陀螺仪数据继续驱动车轮。
    if (front_ui_is_running() || g_drive_by_busy ||
        front_ui_remote_is_active() ||
        !gyro_yaw_rate_control_is_ready() ||
        !gyro_yaw_rate_control_gyro_is_fresh()) {
        drive_by_heading_hold_set_enable(false);
        return;
    }

    const DriveByClock::time_point now = DriveByClock::now();
    float dt_s = std::chrono::duration<float>(
        now - g_heading_hold_last_update).count();
    g_heading_hold_last_update = now;
    if (dt_s <= 0.0f) {
        return;
    }
    if (dt_s > kMaxIntegrationDtS) {
        dt_s = kMaxIntegrationDtS;
    }

    const float gyro_dps = gyro_yaw_rate_control_get_gyro_z_dps();
    g_heading_hold_yaw_deg = normalize_angle_deg(
        g_heading_hold_yaw_deg + gyro_dps * dt_s);
    const float heading_error_deg = normalize_angle_deg(
        -g_heading_hold_yaw_deg);
    const bool quiet_before_update = g_heading_hold_quiet;
    const HeadingControlZone control_zone = update_heading_control_zone(
        heading_error_deg, gyro_dps, &g_heading_hold_quiet);
    if (quiet_before_update != g_heading_hold_quiet) {
        gyro_yaw_rate_control_reset_controller();
        motor_speed_pid_reset();
    }

    float target_yaw_rate_dps = 0.0f;
    float turn_rps = 0.0f;
    if (control_zone != HEADING_CONTROL_QUIET) {
        if (control_zone == HEADING_CONTROL_FULL) {
            target_yaw_rate_dps =
                drive_by_heading_kp * heading_error_deg -
                drive_by_heading_kd * gyro_dps;
            const float yaw_rate_limit = std::fabs(
                (float)drive_by_heading_max_dps);
            target_yaw_rate_dps = clampf(
                target_yaw_rate_dps, -yaw_rate_limit, yaw_rate_limit);
        }

        // 角速度内环仍使用现有gIP/gII，但额外套用yawHoldRMax，确保本测试
        // 不会把正常巡线所需的较大差速上限带到原地调参场景。
        turn_rps = gyro_yaw_rate_control_update_target_yaw_rate_limited(
            target_yaw_rate_dps,
            drive_by_heading_hold_max_turn_rps);
    }
    // 绕行轮速函数带有“倒车最多-10RPS”的前进安全限幅，不适合原地对称转向。
    // 航向保持直接写+turn/-turn，实际幅度仍受yawHoldRMax和gRMax双重限制。
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = turn_rps;
    pwm2_duty_rps = -turn_rps;

    g_heading_hold_debug.enabled = 1;
    g_heading_hold_debug.yaw_deg = g_heading_hold_yaw_deg;
    g_heading_hold_debug.target_yaw_deg = 0.0f;
    g_heading_hold_debug.heading_error_deg = heading_error_deg;
    g_heading_hold_debug.target_yaw_rate_dps = target_yaw_rate_dps;
    g_heading_hold_debug.turn_rps = turn_rps;
}

const DriveByHeadingHoldDebug &drive_by_heading_hold_get_debug()
{
    return g_heading_hold_debug;
}

void drive_by_tangent_debug_set_enable(bool enable)
{
    g_tangent_debug_enabled = enable;
    g_tangent_debug.enabled = enable ? 1 : 0;
    if (!enable) {
        g_tangent_debug.valid = 0;
        g_tangent_debug.anchor_x = -1;
        g_tangent_debug.anchor_y = -1;
    }
}

bool drive_by_tangent_debug_is_enabled()
{
    return g_tangent_debug_enabled;
}

const DriveByTangentDebug &drive_by_tangent_debug_get()
{
    return g_tangent_debug;
}
