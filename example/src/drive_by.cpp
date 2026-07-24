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

#include <opencv2/imgproc.hpp>

using DriveByClock = std::chrono::steady_clock;

// ===================== 高速识别和 S 形绕行调参区 =====================
// K0 开启但没有目标时按 35RPS 正常巡线；红色触发后只把“基准速度”降到
// 10RPS，普通方向环仍会在该基准上叠加左右差速。
volatile float drive_by_normal_speed_rps = 35.0f;
volatile float drive_by_recognition_speed_rps = 10.0f;
volatile float drive_by_rps_to_mps = 0.047f;

// 这些整数变量保留原名称，避免之前的调参经验失效。闭环版本中
// drive_by_turn_speed_rps 表示转向阶段的前进基准速度，不再表示外侧轮速度。
int drive_by_turn_speed_rps = 10;
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

volatile float drive_by_turn_angle_deg = 25.0f;
volatile float drive_by_pass_distance_m = 0.80f;
volatile float drive_by_exit_distance_m = 0.15f;
volatile float drive_by_target_after_margin_m = 0.30f;
volatile float drive_by_view_angle_max_deg = 45.0f;
volatile int drive_by_view_wait_timeout_ms = 120;
// 25度标准绕行转角下，KP=8可在起步时直接请求约200dps。
// KD只保留少量角速度阻尼，避免原来的0.8在刚起转后过早压低目标角速度。
volatile float drive_by_heading_kp = 8.0f;
volatile float drive_by_heading_kd = 0.2f;
// 这是航向外环允许给出的目标角速度上限；接近目标角度时，KP/KD仍会主动降速。
volatile float drive_by_heading_max_dps = 200.0f;
volatile float drive_by_heading_tolerance_deg = 2.0f;
volatile float drive_by_rate_tolerance_dps = 20.0f;
volatile int drive_by_gyro_stale_ms = 60;
volatile float drive_by_yaw_sign = -1.0f;
volatile float drive_by_track_heading_alpha = 0.80f;

namespace {

constexpr int kInferFrames = 3;
constexpr int kSaveSize = 96;
constexpr int kDetectFrameInterval = 2;
constexpr int kHeadingSettleCycles = 3;
// 默认绕行速度为10RPS（约0.47m/s），走完约1.1m通常需要2秒以上。
// 8秒只是防止状态机永久卡住，不参与正常阶段切换。
constexpr int kMotionPhaseTimeoutMs = 8000;
constexpr float kTrackTangentWindowM = 0.10f;
constexpr float kMaxIntegrationDtS = 0.10f;
constexpr float kMinWheelTargetRps = -10.0f;
constexpr float kMaxWheelTargetRps = 200.0f;

enum DriveByState {
    DB_IDLE,
    DB_WAIT_VIEW,
    DB_INFER,
    DB_SHIFT_OUT_A,
    DB_SHIFT_OUT_B,
    DB_PASS_TARGET,
    DB_SHIFT_IN_A,
    DB_SHIFT_IN_B,
};

enum DriveByAbortReason {
    DB_ABORT_NONE,
    DB_ABORT_VIEW_TIMEOUT,
    DB_ABORT_NO_TARGET_GEOMETRY,
    DB_ABORT_GYRO_NOT_READY,
    DB_ABORT_GYRO_STALE,
    DB_ABORT_PHASE_TIMEOUT,
};

enum RecognitionFrameStatus {
    RECOG_FRAME_NOT_PROCESSED,
    RECOG_FRAME_OK,
    RECOG_FRAME_NO_RED,
    RECOG_FRAME_INVALID_ROI,
    RECOG_FRAME_UNKNOWN_LABEL,
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
    double trigger_detect_ms = 0.0;
    double infer_sum_ms = 0.0;
    double total_ms = 0.0;
    float trigger_left_rps = 0.0f;
    float trigger_right_rps = 0.0f;
    float finish_left_rps = 0.0f;
    float finish_right_rps = 0.0f;
};

volatile bool g_drive_by_enable = false;
volatile bool g_drive_by_busy = false;
bool g_seen_lock = false;
bool g_target_geometry_captured = false;
bool g_track_reference_valid = false;
bool g_current_track_heading_valid = false;
DriveByState g_state = DB_IDLE;
DriveByAbortReason g_abort_reason = DB_ABORT_NONE;
DriveByClock::time_point g_state_start = DriveByClock::now();
DriveByClock::time_point g_session_start = DriveByClock::now();
DriveByClock::time_point g_last_integration_time = DriveByClock::now();
DriveByClock::time_point g_cooldown_start = DriveByClock::now();
int g_detect_frame_counter = 0;
int g_heading_settle_count = 0;
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
DriveByDebug g_debug = {};

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

float blend_angle_deg(float old_angle, float new_angle, float old_weight)
{
    const float delta = normalize_angle_deg(new_angle - old_angle);
    return normalize_angle_deg(old_angle + (1.0f - old_weight) * delta);
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
    case DB_WAIT_VIEW: return "WAIT_VIEW";
    case DB_INFER: return "INFER";
    case DB_SHIFT_OUT_A: return "SHIFT_OUT_A";
    case DB_SHIFT_OUT_B: return "SHIFT_OUT_B";
    case DB_PASS_TARGET: return "PASS_TARGET";
    case DB_SHIFT_IN_A: return "SHIFT_IN_A";
    case DB_SHIFT_IN_B: return "SHIFT_IN_B";
    default: return "UNKNOWN";
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
    default: return "unknown";
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

bool should_detect_this_frame()
{
    ++g_detect_frame_counter;
    if (g_detect_frame_counter < kDetectFrameInterval) {
        return false;
    }
    g_detect_frame_counter = 0;
    return true;
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

void finish_frame_record(RecognitionFrameRecord& record,
                         DriveByClock::time_point frame_start)
{
    const DriveByClock::time_point frame_end = DriveByClock::now();
    record.frame_ms = duration_ms(frame_start, frame_end);
    record.since_trigger_ms = duration_ms(g_session_start, frame_end);
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

    printf("[识别测试] 最终结果=%d（%s），投票：左绕=%d，直行=%d，右绕=%d，有效帧=%d/%d\n",
           final_result,
           result_action_name(final_result),
           g_report.votes[0],
           g_report.votes[1],
           g_report.votes[2],
           g_report.valid_count,
           kInferFrames);
    printf("[识别测试] 三帧总时间=%.2f毫秒，推理合计=%.2f毫秒，结束轮速=(%.2f, %.2f)RPS\n",
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
}

void reset_runtime(bool restore_outputs)
{
    if (restore_outputs) {
        restore_control();
    }
    gyro_yaw_rate_control_reset_controller();
    g_drive_by_busy = false;
    g_seen_lock = false;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_current_track_heading_valid = false;
    g_state = DB_IDLE;
    g_abort_reason = DB_ABORT_NONE;
    g_detect_frame_counter = 0;
    g_heading_settle_count = 0;
    g_turn_side = 0;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_report = RecognitionReport{};
    g_debug = DriveByDebug{};
    have_target = false;
}

void finish_session(DriveByAbortReason reason)
{
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
    g_state = DB_IDLE;
    g_cooldown_start = DriveByClock::now();
    print_recognition_report(item_flag);
}

void start_target_session(double trigger_detect_ms)
{
    save_control_once();
    g_drive_by_busy = true;
    g_seen_lock = true;
    g_target_geometry_captured = false;
    g_track_reference_valid = false;
    g_current_track_heading_valid = false;
    g_abort_reason = DB_ABORT_NONE;
    item_flag = 1;
    g_session_start = DriveByClock::now();
    g_last_integration_time = g_session_start;
    g_yaw_deg = 0.0f;
    g_distance_since_trigger_m = 0.0f;
    g_phase_distance_m = 0.0f;
    g_report = RecognitionReport{};
    g_report.trigger_detect_ms = trigger_detect_ms;
    g_report.trigger_left_rps = encoder1_speed_avg;
    g_report.trigger_right_rps = encoder2_speed_avg;
    g_debug = DriveByDebug{};
    command_recognition_base_speed();
    enter_state(DB_WAIT_VIEW);
}

void begin_motion_script(int result)
{
    if (!g_target_geometry_captured) {
        item_flag = 1;
        finish_session(DB_ABORT_NO_TARGET_GEOMETRY);
        return;
    }
    if (!gyro_yaw_rate_control_is_ready()) {
        item_flag = 1;
        finish_session(DB_ABORT_GYRO_NOT_READY);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        item_flag = 1;
        finish_session(DB_ABORT_GYRO_STALE);
        return;
    }

    // 左绕和右绕只改变相对赛道切线的偏角符号。
    g_turn_side = result == 0 ? 1 : -1;
    g_track_heading_reference_deg = g_target_track_heading_global_deg;
    g_track_reference_valid = true;
    gyro_yaw_rate_control_reset_controller();
    enter_state(DB_SHIFT_OUT_A);
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

float phase_offset_deg()
{
    const float side_offset = g_turn_side * drive_by_yaw_sign * drive_by_turn_angle_deg;
    switch (g_state) {
    case DB_SHIFT_OUT_A: return side_offset;
    case DB_SHIFT_IN_A: return -side_offset;
    default: return 0.0f;
    }
}

float phase_base_speed_rps()
{
    switch (g_state) {
    case DB_PASS_TARGET: return (float)drive_by_forward_speed_rps;
    case DB_SHIFT_IN_B: return (float)drive_by_exit_speed_rps;
    default: return (float)drive_by_turn_speed_rps;
    }
}

void update_motion_state()
{
    if (!gyro_yaw_rate_control_is_ready()) {
        item_flag = 1;
        finish_session(DB_ABORT_GYRO_NOT_READY);
        return;
    }
    if (gyro_yaw_rate_control_gyro_age_ms() > drive_by_gyro_stale_ms) {
        item_flag = 1;
        finish_session(DB_ABORT_GYRO_STALE);
        return;
    }
    if (elapsed_ms(g_state_start) > kMotionPhaseTimeoutMs) {
        item_flag = 1;
        finish_session(DB_ABORT_PHASE_TIMEOUT);
        return;
    }

    const float gyro_dps = gyro_yaw_rate_control_get_gyro_z_dps();
    const float target_yaw_deg = normalize_angle_deg(
        g_track_heading_reference_deg + phase_offset_deg());
    const float heading_error_deg = normalize_angle_deg(target_yaw_deg - g_yaw_deg);
    float target_yaw_rate_dps =
        drive_by_heading_kp * heading_error_deg - drive_by_heading_kd * gyro_dps;
    const float rate_limit = std::fabs((float)drive_by_heading_max_dps);
    target_yaw_rate_dps = clampf(target_yaw_rate_dps, -rate_limit, rate_limit);
    const float turn_rps =
        gyro_yaw_rate_control_update_target_yaw_rate(target_yaw_rate_dps);
    command_motion_wheels(phase_base_speed_rps(), turn_rps);

    g_debug.yaw_deg = g_yaw_deg;
    g_debug.target_yaw_deg = target_yaw_deg;
    g_debug.heading_error_deg = heading_error_deg;
    g_debug.track_heading_deg = g_track_heading_reference_deg;
    g_debug.target_yaw_rate_dps = target_yaw_rate_dps;
    g_debug.turn_rps = turn_rps;

    switch (g_state) {
    case DB_SHIFT_OUT_A:
        if (heading_is_settled(heading_error_deg, gyro_dps)) {
            enter_state(DB_SHIFT_OUT_B);
        }
        break;

    case DB_SHIFT_OUT_B:
        if (heading_is_settled(heading_error_deg, gyro_dps)) {
            enter_state(DB_PASS_TARGET);
        }
        break;

    case DB_PASS_TARGET: {
        const float pass_end_distance =
            g_target_distance_at_trigger_m + drive_by_target_after_margin_m;
        if (g_distance_since_trigger_m >= pass_end_distance) {
            enter_state(DB_SHIFT_IN_A);
        }
        break;
    }

    case DB_SHIFT_IN_A:
        if (heading_is_settled(heading_error_deg, gyro_dps)) {
            enter_state(DB_SHIFT_IN_B);
        }
        break;

    case DB_SHIFT_IN_B:
        if (heading_is_settled(heading_error_deg, gyro_dps) &&
            g_phase_distance_m >= drive_by_exit_distance_m) {
            finish_session(DB_ABORT_NONE);
        }
        break;

    default:
        break;
    }
}

void complete_inference()
{
    g_report.total_ms = duration_ms(g_session_start, DriveByClock::now());
    g_report.finish_left_rps = encoder1_speed_avg;
    g_report.finish_right_rps = encoder2_speed_avg;
    item_flag = choose_vote_result();

    if (item_flag == 1) {
        finish_session(DB_ABORT_NONE);
        return;
    }
    begin_motion_script(item_flag);
}

void update_busy_state(cv::Mat& frame, LQ_NCNN& ncnn)
{
    integrate_session_motion();

    switch (g_state) {
    case DB_WAIT_VIEW:
        command_recognition_base_speed();
        {
            bool detected_this_frame = false;
            double detect_ms = 0.0;
            if (should_detect_this_frame()) {
                const DriveByClock::time_point detect_start = DriveByClock::now();
                detectRedPlate(frame);
                detect_ms = duration_ms(detect_start, DriveByClock::now());
                detected_this_frame = true;

                // 主循环的道路结果来自上一张相机帧，时间上只差一个帧周期。
                // 这里立即更新观察夹角，使合格的当前帧可以直接成为第1帧推理。
                drive_by_update_track_geometry();
            }
            if (g_debug.view_ready != 0 && have_target) {
                enter_state(DB_INFER);
                process_inference_frame(frame, ncnn, detected_this_frame, detect_ms);
                if (g_report.frame_count >= kInferFrames) {
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
        if (g_report.frame_count >= kInferFrames) {
            complete_inference();
        } else if (elapsed_ms(g_state_start) >= drive_by_infer_timeout_ms) {
            item_flag = 1;
            finish_session(DB_ABORT_VIEW_TIMEOUT);
        }
        break;

    case DB_SHIFT_OUT_A:
    case DB_SHIFT_OUT_B:
    case DB_PASS_TARGET:
    case DB_SHIFT_IN_A:
    case DB_SHIFT_IN_B:
        update_motion_state();
        break;

    default:
        finish_session(DB_ABORT_PHASE_TIMEOUT);
        break;
    }
}

void update_idle_detection(cv::Mat& frame, LQ_NCNN& ncnn)
{
    if (!should_detect_this_frame()) {
        return;
    }

    const DriveByClock::time_point detect_start = DriveByClock::now();
    detectRedPlate(frame);
    const double detect_ms = duration_ms(detect_start, DriveByClock::now());

    if (g_seen_lock) {
        if (!have_target && elapsed_ms(g_cooldown_start) >= drive_by_cooldown_ms) {
            g_seen_lock = false;
        }
        return;
    }

    if (have_target) {
        start_target_session(detect_ms);

        // 复用上一帧已经提取好的道路中线，立即计算本触发帧的目标几何。
        // 若观察角已经合格，本帧直接作为第1帧，不再额外等待一个相机周期。
        drive_by_update_track_geometry();
        if (g_debug.view_ready != 0) {
            enter_state(DB_INFER);
            process_inference_frame(frame, ncnn, true, detect_ms);
        }
    }
}

} // namespace

void drive_by_init()
{
    g_drive_by_enable = false;
    reset_runtime(false);
}

void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn)
{
    if (!g_drive_by_enable) {
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

void drive_by_update_track_geometry()
{
    if (!g_drive_by_busy) {
        return;
    }

    float current_heading_relative_deg = 0.0f;
    g_current_track_heading_valid =
        compute_current_track_heading(&current_heading_relative_deg);
    if (g_current_track_heading_valid) {
        g_current_track_heading_relative_deg = current_heading_relative_deg;
        const float measured_global_heading = normalize_angle_deg(
            g_yaw_deg + current_heading_relative_deg);
        if (drive_by_is_motion_phase()) {
            if (!g_track_reference_valid) {
                g_track_heading_reference_deg = measured_global_heading;
                g_track_reference_valid = true;
            } else {
                const float alpha = clampf(drive_by_track_heading_alpha, 0.0f, 1.0f);
                g_track_heading_reference_deg = blend_angle_deg(
                    g_track_heading_reference_deg,
                    measured_global_heading,
                    alpha);
            }
        }
    }

    // 目标位置和观察夹角只在识别阶段需要。进入S形后目标几何已经锁定，
    // 每帧只更新当前赛道切线，避免重复遍历整条中线。
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
    return g_drive_by_busy && (g_state == DB_WAIT_VIEW || g_state == DB_INFER);
}

bool drive_by_is_motion_phase()
{
    return g_drive_by_busy &&
        (g_state == DB_SHIFT_OUT_A ||
         g_state == DB_SHIFT_OUT_B ||
         g_state == DB_PASS_TARGET ||
         g_state == DB_SHIFT_IN_A ||
         g_state == DB_SHIFT_IN_B);
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
    printf("[drive_by] enable=%d\n", enable ? 1 : 0);
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

const DriveByDebug &drive_by_get_debug()
{
    return g_debug;
}
