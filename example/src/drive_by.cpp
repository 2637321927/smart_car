#include "drive_by.hpp"

#include "front_ui.hpp"
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
// K0 开启但没有目标时按 35RPS 正常巡线；红色触发后只把“基准速度”降到
// 12.5RPS，普通方向环仍会在该基准上叠加左右差速。
volatile float drive_by_normal_speed_rps = 35.0f;
volatile float drive_by_recognition_speed_rps = 12.5f;
volatile float drive_by_rps_to_mps = 0.047f;
volatile int drive_by_mode = 0;
volatile int drive_by_use_track_tangent = 0;

// 这些整数变量保留原名称，避免之前的调参经验失效。闭环版本中
// drive_by_turn_speed_rps 表示转向阶段的前进基准速度，不再表示外侧轮速度。
int drive_by_turn_speed_rps = 15;
int drive_by_turn_inner_speed_rps = 0; // 兼容旧脚本，新的角度闭环不直接使用。
int drive_by_forward_speed_rps = 20;
int drive_by_exit_speed_rps = 15;
volatile float drive_by_learned_speed_rps = 15.0f;
volatile float drive_by_learned_distance_m = 0.96f;
volatile float drive_by_learned_yaw_scale = 1.0f;
int drive_by_turn_out_ms = 600;        // 兼容旧参数，新的状态切换主要看角度。
int drive_by_forward_ms = 400;         // 兼容旧参数，新的状态切换主要看距离。
int drive_by_turn_back_ms = 800;       // 兼容旧参数，新的状态切换主要看角度。
int drive_by_exit_forward_ms = 150;    // 兼容旧参数，新的状态切换主要看角度。
int drive_by_stop_ms = 200;            // 兼容旧识别测试，不再用于识别后停车。
int drive_by_infer_timeout_ms = 500;
int drive_by_cooldown_ms = 1000;

volatile float drive_by_turn_angle_deg = 25.0f;
volatile float drive_by_return_bias_deg = 53.0f;
volatile float drive_by_pass_distance_m = 0.0f;
volatile float drive_by_target_after_margin_m = 0.30f;
volatile float drive_by_view_angle_max_deg = 45.0f;
volatile int drive_by_view_wait_timeout_ms = 120;
// 25度标准绕行转角下，KP=8可在起步时直接请求约200dps。
// KD只保留少量角速度阻尼，避免原来的0.8在刚起转后过早压低目标角速度。
volatile float drive_by_heading_kp = 12.0f;
volatile float drive_by_heading_kd = 0.2f;
// 这是航向外环允许给出的目标角速度上限；接近目标角度时，KP/KD仍会主动降速。
volatile float drive_by_heading_max_dps = 200.0f;
// 航向保持测试只允许较小差速，避免调试KP时误用正常巡线的较大gRMax。
volatile float drive_by_heading_hold_max_turn_rps = 10.0f;
volatile float drive_by_recovery_yaw_rate_dps = 55.0f;
volatile float drive_by_heading_tolerance_deg = 4.5f;
volatile float drive_by_rate_tolerance_dps = 20.0f;
volatile int drive_by_gyro_stale_ms = 60;
volatile float drive_by_yaw_sign = -1.0f;
volatile int drive_by_brake_pwm = 5000;
volatile float drive_by_brake_release_rps = 15.0f;
volatile int drive_by_brake_confirm_count = 2;
volatile int drive_by_brake_timeout_ms = 300;
volatile float drive_by_test_target_distance_m = 0.50f;

namespace {

constexpr int kInferFrames = 3;
constexpr int kBrakePwmMax = 5000;
constexpr int kSaveSize = 96;
constexpr int kDetectFrameInterval = 2;
constexpr int kHeadingSettleCycles = 3;
constexpr float kHeadingDeadzoneHysteresisDeg = 1.0f;
constexpr float kHeadingQuietRateDps = 5.0f;
constexpr float kReturnHandoffYawDeg = 10.0f;
constexpr int kFarDetectTop = 60;
constexpr int kFarDetectBottom = 40;
constexpr int kFarDetectLeft = 60;
constexpr int kFarDetectRight = 60;
// 远距离候选只用于提前减速。面积门槛过低时，赛道上的小块红色噪声也可能连续触发；
// 从40提高到60，在保留远距离预警作用的同时，降低误触发概率。
constexpr int kFarDetectMinArea = 60;
constexpr int kFarDetectMinWidth = 8;
constexpr int kFarDetectConfirmCount = 2;
constexpr int kApproachTimeoutMs = 300;
constexpr int kCandidateRetryCooldownMs = 300;
constexpr int kVisualSideFollowMs = 500;
constexpr float kVisualRecoveryError = 100.0f;
constexpr int kTurnPhaseTimeoutMs = 1500;
constexpr int kPassPhaseTimeoutMs = 5000;
constexpr int kLearnedPathTimeoutMs = 4000;
constexpr float kTurnPhaseMaxDistanceM = 0.80f;
constexpr float kPassPhaseMaxDistanceM = 3.00f;
constexpr float kLearnedPathMaxExtraDistanceM = 0.50f;
constexpr float kTrackTangentWindowM = 0.10f;
constexpr float kMaxIntegrationDtS = 0.10f;
constexpr float kMinWheelTargetRps = -10.0f;
constexpr float kMaxWheelTargetRps = 200.0f;

struct LearnedPathPoint {
    float progress;
    float yaw_deg;
};

// 最近六次手推录像按行驶距离归一化后的镜像平均值。正值表示先向绕行侧
// 偏出，负值表示越过赛道切线并把车头轻微指向赛道内侧。
constexpr LearnedPathPoint kLearnedPath[] = {
    {0.0f, 0.00f},
    {0.1f, 7.26f},
    {0.2f, 9.38f},
    {0.3f, 8.14f},
    {0.4f, 5.18f},
    {0.5f, 1.85f},
    {0.6f, -1.11f},
    {0.7f, -4.98f},
    {0.8f, -6.41f},
    {0.9f, -5.34f},
    {1.0f, -3.55f},
};
constexpr int kLearnedPathPointCount =
    static_cast<int>(sizeof(kLearnedPath) / sizeof(kLearnedPath[0]));

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
    DB_LEARNED_PATH,
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
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float dir_p = 0.0f;
    float dir_d = 0.0f;
    float aim = 0.0f;
    int slow_ratio = 0;
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
volatile bool g_inference_complete = false;
volatile bool g_test_mode = false;
volatile bool g_stop_requested = false;
volatile bool g_report_pending = false;
volatile bool g_visual_aim_line_valid = false;
// 每次开始运动时锁存drive_by_mode。在线调参即使恰好发生在绕行中途，
// 当前脚本也不会从一套状态机跳到另一套，新的模式从下一次绕行生效。
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
float g_learned_path_total_m = 0.96f;
SavedControl g_saved;
RecognitionReport g_report;
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

float learned_path_template_yaw_deg(float progress)
{
    const float clamped_progress = clampf(progress, 0.0f, 1.0f);
    for (int upper_index = 1;
         upper_index < kLearnedPathPointCount;
         ++upper_index) {
        const LearnedPathPoint& upper = kLearnedPath[upper_index];
        if (clamped_progress > upper.progress) {
            continue;
        }

        const LearnedPathPoint& lower = kLearnedPath[upper_index - 1];
        const float segment_length = upper.progress - lower.progress;
        if (segment_length <= 0.0f) {
            return upper.yaw_deg;
        }
        const float local_ratio =
            (clamped_progress - lower.progress) / segment_length;
        return lower.yaw_deg + (upper.yaw_deg - lower.yaw_deg) * local_ratio;
    }

    return kLearnedPath[kLearnedPathPointCount - 1].yaw_deg;
}

float learned_path_offset_deg()
{
    const float total_m = std::max(0.10f, g_learned_path_total_m);
    const float progress = g_phase_distance_m / total_m;
    const float outward_sign = g_turn_side * drive_by_yaw_sign;
    return outward_sign * learned_path_template_yaw_deg(progress) *
        std::fabs((float)drive_by_learned_yaw_scale);
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
    case DB_LEARNED_PATH: return "LEARNED_PATH";
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
    case DB_LEARNED_PATH:
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

void save_control_once()
{
    if (g_saved.valid) {
        return;
    }

    g_saved.set_speed1 = set_speed_of_motor1_rps;
    g_saved.set_speed2 = set_speed_of_motor2_rps;
    g_saved.pwm_target1 = pwm1_duty_rps;
    g_saved.pwm_target2 = pwm2_duty_rps;
    g_saved.p = P;
    g_saved.i = I;
    g_saved.d = D;
    g_saved.dir_p = dir_P;
    g_saved.dir_d = dir_D;
    g_saved.aim = AIM;
    g_saved.slow_ratio = spd_slow_ratio;
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
    P = g_saved.p;
    I = g_saved.i;
    D = g_saved.d;
    dir_P = g_saved.dir_p;
    dir_D = g_saved.dir_d;
    AIM = g_saved.aim;
    spd_slow_ratio = g_saved.slow_ratio;
    g_saved.valid = false;
}

void command_normal_base_speed()
{
    set_speed_of_motor1_rps = drive_by_normal_speed_rps;
    set_speed_of_motor2_rps = drive_by_normal_speed_rps;
}

void command_recognition_base_speed()
{
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

bool should_detect_this_frame()
{
    ++g_detect_frame_counter;
    if (g_detect_frame_counter < kDetectFrameInterval) {
        return false;
    }
    g_detect_frame_counter = 0;
    return true;
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
        if (aspect_ratio < 1.3f || aspect_ratio > 5.0f || area <= best_candidate_area) {
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

void print_recognition_report(int final_result)
{
    printf("[识别测试] 检测到红色：检测耗时=%.2f毫秒，左轮=%.2fRPS，右轮=%.2fRPS\n",
           g_report.trigger_detect_ms,
           g_report.trigger_left_rps,
           g_report.trigger_right_rps);

    for (int index = 0; index < g_report.frame_count; ++index) {
        const RecognitionFrameRecord& record = g_report.frames[index];
        printf("[识别测试] 第%d/%d帧：%s\n",
               index + 1,
               kInferFrames,
               frame_status_name(record.status));
        if (!record.label.empty()) {
            printf("  模型标签=%s（%s），识别结果=%d（%s）\n",
                   record.label.c_str(),
                   label_chinese_name(record.label),
                   record.mapped_result,
                   result_action_name(record.mapped_result));
        }
        printf("  红色检测=%.2f毫秒，图像准备=%.2f毫秒，模型推理=%.2f毫秒，累计=%.2f毫秒\n",
               record.detect_ms,
               record.prepare_ms,
               record.infer_ms,
               record.since_trigger_ms);
    }

    const char *decision_mode = g_report.early_decision
        ? "前两帧一致，提前结束"
        : (g_report.frame_count >= kInferFrames ? "完成三帧投票" : "流程提前退出");
    printf("[识别测试] 最终结果=%d（%s），投票：左绕=%d，直行=%d，右绕=%d，有效帧=%d/%d，判定=%s\n",
           final_result,
           result_action_name(final_result),
           g_report.votes[0],
           g_report.votes[1],
           g_report.votes[2],
           g_report.valid_count,
           g_report.frame_count,
           decision_mode);
    printf("[识别测试] 识别总时间=%.2f毫秒，推理合计=%.2f毫秒，结束轮速=(%.2f, %.2f)RPS\n",
           g_report.total_ms,
           g_report.infer_sum_ms,
           g_report.finish_left_rps,
           g_report.finish_right_rps);
    if (g_abort_reason != DB_ABORT_NONE) {
        printf("[绕行脚本] 已退出：原因=%s\n", abort_reason_name(g_abort_reason));
    }
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
}

void reset_runtime(bool restore_outputs)
{
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
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_motion_heading_quiet = false;
    g_active_mode = 0;
    g_state = DB_IDLE;
    g_abort_reason = DB_ABORT_NONE;
    g_candidate_retry_after = DriveByClock::now();
    g_detect_frame_counter = 0;
    g_far_candidate_count = 0;
    g_heading_settle_count = 0;
    g_pending_result = 1;
    g_turn_side = 0;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_learned_path_total_m = drive_by_learned_distance_m;
    g_report = RecognitionReport{};
    g_debug = DriveByDebug{};
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
    g_stop_requested = false;
    g_report_pending = false;
    g_visual_aim_line_valid = false;
    g_motion_heading_quiet = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
    g_state = DB_IDLE;
    g_far_candidate_count = 0;
    g_debug.red_candidate = 0;
    g_debug.red_candidate_count = 0;
    g_debug.detection_stage = 0;
    g_cooldown_start = DriveByClock::now();
    print_recognition_report(item_flag);
}

void start_approach_session(const FarRedCandidate& candidate)
{
    save_control_once();
    g_drive_by_busy = true;
    g_seen_lock = true;
    g_recognition_triggered = false;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_current_track_heading_valid = false;
    g_inference_complete = false;
    g_test_mode = false;
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
    g_state = DB_IDLE;
    g_cooldown_start = DriveByClock::now();
    g_candidate_retry_after = g_cooldown_start +
        std::chrono::milliseconds(kCandidateRetryCooldownMs);
    g_seen_lock = false;
    g_far_candidate_count = 0;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_inference_complete = false;
    g_test_mode = false;
    g_stop_requested = false;
    g_visual_aim_line_valid = false;
    g_active_mode = 0;
    g_debug.test_mode = 0;
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
    g_active_mode = std::max(0, std::min(2, (int)drive_by_mode));

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
        item_flag = 1;
        g_abort_reason = DB_ABORT_NO_TARGET_GEOMETRY;
        g_debug.abort_reason = (int)g_abort_reason;
        enter_state(DB_FINISH_PENDING);
        return;
    }
    if (!gyro_yaw_rate_control_is_ready()) {
        item_flag = 1;
        g_abort_reason = DB_ABORT_GYRO_NOT_READY;
        g_debug.abort_reason = (int)g_abort_reason;
        enter_state(DB_FINISH_PENDING);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        item_flag = 1;
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

    if (g_active_mode == 2) {
        // 示教曲线默认按录像中位路程0.96m复现。如果目标板实际更远，则只拉伸
        // 距离轴，确保轨迹末端不会早于“目标距离+安全余量”。
        const float remaining_safe_distance_m = std::max(
            0.0f,
            g_target_distance_at_trigger_m + drive_by_target_after_margin_m -
                g_distance_since_trigger_m);
        g_learned_path_total_m = std::max(
            std::max(0.10f, (float)drive_by_learned_distance_m),
            remaining_safe_distance_m);
        enter_state(DB_LEARNED_PATH);
        return;
    }

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
    case DB_LEARNED_PATH: return drive_by_learned_speed_rps;
    case DB_PASS_SHORT: return (float)drive_by_forward_speed_rps;
    case DB_TURN_TO_TRACK: return (float)drive_by_exit_speed_rps;
    default: return (float)drive_by_turn_speed_rps;
    }
}

bool motion_phase_guard_exceeded()
{
    if (g_state == DB_LEARNED_PATH) {
        return elapsed_ms(g_state_start) > kLearnedPathTimeoutMs ||
            g_phase_distance_m >
                std::max(0.10f, g_learned_path_total_m) +
                    kLearnedPathMaxExtraDistanceM;
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
    g_far_candidate_count = 0;
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
        item_flag = 1;
        request_motion_stop(DB_ABORT_GYRO_NOT_READY);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        item_flag = 1;
        request_motion_stop(DB_ABORT_GYRO_STALE);
        return;
    }
    if (motion_phase_guard_exceeded()) {
        item_flag = 1;
        request_motion_stop(DB_ABORT_PHASE_TIMEOUT);
        return;
    }

    const float gyro_dps = gyro_yaw_rate_control_get_gyro_z_dps();
    const float target_offset_deg = g_state == DB_LEARNED_PATH
        ? learned_path_offset_deg()
        : phase_offset_deg();
    const float target_yaw_deg = normalize_angle_deg(
        g_track_heading_reference_deg + target_offset_deg);
    const float heading_error_deg = normalize_angle_deg(target_yaw_deg - g_yaw_deg);
    const bool use_heading_deadzone = g_state != DB_LEARNED_PATH;
    const bool quiet_before_update = g_motion_heading_quiet;
    const HeadingControlZone control_zone = use_heading_deadzone
        ? update_heading_control_zone(
            heading_error_deg, gyro_dps, &g_motion_heading_quiet)
        : HEADING_CONTROL_FULL;
    if (!use_heading_deadzone) {
        g_motion_heading_quiet = false;
    }
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
    case DB_LEARNED_PATH:
        if (g_phase_distance_m >= g_learned_path_total_m &&
            heading_is_settled(heading_error_deg, gyro_dps)) {
            if (g_visual_aim_line_valid) {
                complete_motion_handoff();
            } else {
                enter_state(DB_RECOVER_CENTER_LINE);
                latest_error = visual_forced_error(true);
            }
        }
        break;

    case DB_TURN_OUT:
        if (g_motion_heading_quiet &&
            heading_is_settled(heading_error_deg, gyro_dps)) {
            enter_state(DB_PASS_SHORT);
        } else if (!g_motion_heading_quiet) {
            g_heading_settle_count = 0;
        }
        break;

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

        if (g_motion_heading_quiet &&
            heading_is_settled(heading_error_deg, gyro_dps)) {
            // 固定回转角已经完成但仍看不到中线时，才进入反方向找线保护。
            // 左绕回程向右找中线，右绕则向左找。
            enter_state(DB_RECOVER_CENTER_LINE);
            latest_error = visual_forced_error(true);
        } else if (!g_motion_heading_quiet) {
            g_heading_settle_count = 0;
        }
        break;

    default:
        break;
    }
}

void update_visual_motion_state()
{
    if (motion_phase_guard_exceeded()) {
        item_flag = 1;
        request_motion_stop(DB_ABORT_PHASE_TIMEOUT);
        return;
    }

    // 新方案使用dbForwardRps；旧三阶段在额外找线时保持第二转的dbExitRps，
    // 避免刚结束角度闭环就突然提速。差速仍由随后执行的普通方向环计算。
    const float visual_base_rps = g_active_mode == 1
        ? (float)drive_by_forward_speed_rps
        : (g_active_mode == 2
            ? (float)drive_by_learned_speed_rps
            : (float)drive_by_exit_speed_rps);
    set_speed_of_motor1_rps = visual_base_rps;
    set_speed_of_motor2_rps = visual_base_rps;

    if (g_state == DB_FOLLOW_SIDE_LINE &&
        elapsed_ms(g_state_start) >= kVisualSideFollowMs) {
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
                finish_session(DB_ABORT_VIEW_TIMEOUT);
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
            finish_session(DB_ABORT_VIEW_TIMEOUT);
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
    case DB_LEARNED_PATH:
        // 航向闭环方案在8ms dir_timer中运行，相机线程只更新道路几何缓存。
        break;

    case DB_FOLLOW_SIDE_LINE:
    case DB_RECOVER_CENTER_LINE:
        // 新方案的相机工作仍在main.cpp中完成；这里不能重复运行寻线或方向环。
        break;

    case DB_FINISH_PENDING:
        finish_session(g_abort_reason);
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
        g_far_candidate_count = 0;
        g_debug.red_candidate_count = 0;
        return;
    }

    if (g_seen_lock) {
        g_far_candidate_count = 0;
        g_debug.red_candidate_count = 0;
        if (!candidate.found && elapsed_ms(g_cooldown_start) >= drive_by_cooldown_ms) {
            g_seen_lock = false;
        }
        return;
    }

    g_far_candidate_count = candidate.found
        ? std::min(g_far_candidate_count + 1, kFarDetectConfirmCount)
        : 0;
    g_debug.red_candidate_count = g_far_candidate_count;
    if (g_far_candidate_count >= kFarDetectConfirmCount) {
        start_approach_session(candidate);
    }
}

} // namespace

void drive_by_init()
{
    g_drive_by_enable = false;
    g_heading_hold_enabled = false;
    g_heading_hold_quiet = false;
    g_tangent_debug_enabled = false;
    g_heading_hold_debug = DriveByHeadingHoldDebug{};
    g_tangent_debug = DriveByTangentDebug{};
    reset_runtime(false);
}

void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn)
{
    if (g_report_pending) {
        // 8ms控制回调已经恢复普通巡线，这里只补做不会影响转向的报告输出。
        g_report_pending = false;
        print_recognition_report(item_flag);
    }

    // TEST模式不改变K0开关，因此即使目标板模式关闭，也必须允许已经启动的
    // 测试脚本继续运行；只有普通识别流程受K0开关约束。
    if (!g_drive_by_enable && !g_test_mode) {
        if (g_drive_by_busy) {
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
        command_normal_base_speed();
        update_idle_detection(frame, ncnn);
    }
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
        front_ui_stop();
        // front_ui_stop()会取消脚本并清空运行态；把原因写回调试快照，便于事后定位。
        g_abort_reason = reason;
        g_debug.abort_reason = (int)reason;
        g_debug.brake_elapsed_ms = stopped_brake_elapsed_ms;
        return true;
    }

    if (!g_brake_active) {
        return false;
    }

    const int brake_elapsed_ms = (int)elapsed_ms(g_brake_start);
    g_debug.brake_elapsed_ms = brake_elapsed_ms;

    const float release_rps = std::fabs((float)drive_by_brake_release_rps);
    // 主动制动只用于车辆原本向前行驶的场景。这里按“正向速度已经降到阈值”
    // 判断，不再取绝对值；若大制动力让轮速轻微穿过零点，负速度仍会满足释放条件，
    // 避免继续反向加速直到300ms超时。连续确认次数仍保留，用于过滤单次编码器毛刺。
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
        command_recognition_base_speed();
        if (g_inference_complete && g_state == DB_WAIT_BRAKE) {
            if (g_pending_result == 0 || g_pending_result == 2) {
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
    if (!front_ui_is_running() || g_drive_by_busy ||
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
         g_state == DB_LEARNED_PATH ||
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

    if (g_state == DB_LEARNED_PATH) {
        // 航向闭环仍由drive_by控制，这里只旁路记录中线是否可靠，
        // 不覆盖computed_error，也不让普通方向环提前参与。
        g_visual_aim_line_valid = selected_aim_line == 0 && aim_line_valid;
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
    if (!enable) {
        reset_runtime(true);
    }
    printf("[目标板识别] 状态=%s\n", enable ? "开启" : "关闭");
}

void drive_by_toggle_enable()
{
    drive_by_set_enable(!g_drive_by_enable);
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
    if (front_ui_is_running() || g_drive_by_busy ||
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
