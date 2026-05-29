#include "drive_by.hpp"
#include "lq_all_demo.hpp"

#include <chrono>
#include <cstdio>
#include <string>
#include <opencv2/imgproc.hpp>

using DriveByClock = std::chrono::steady_clock;

// 这些默认值来自你推车绕行时的编码器曲线：
// 先单侧轮约 3rps 转出，再双轮约 6rps 前进，最后换另一侧轮约 3rps 转回。
int drive_by_turn_speed_rps = 3;
int drive_by_turn_inner_speed_rps = 0;
int drive_by_forward_speed_rps = 6;
int drive_by_exit_speed_rps = 0;
int drive_by_turn_out_ms = 1300;
int drive_by_forward_ms = 550;
int drive_by_turn_back_ms = 450;
int drive_by_exit_forward_ms = 150;
int drive_by_stop_ms = 300;
int drive_by_infer_timeout_ms = 1000;
int drive_by_cooldown_ms = 3000;

namespace {

enum DriveByState {
    DB_IDLE,
    DB_STOPPING,
    DB_INFER,
    DB_LEFT_TURN_OUT,
    DB_LEFT_FORWARD,
    DB_LEFT_TURN_BACK,
    DB_LEFT_EXIT_FORWARD,
    DB_RIGHT_TURN_OUT,
    DB_RIGHT_FORWARD,
    DB_RIGHT_TURN_BACK,
    DB_RIGHT_EXIT_FORWARD,
};

struct SavedControl {
    int set_speed1 = 0;
    int set_speed2 = 0;
    int pwm_target1 = 0;
    int pwm_target2 = 0;
    float p = 0.0f;
    float i = 0.0f;
    float d = 0.0f;
    float dir_p = 0.0f;
    float dir_d = 0.0f;
    float aim = 0.0f;
    int slow_ratio = 0;
    bool valid = false;
};

constexpr int kInferFrames = 5;
constexpr int kSaveSize = 96;
constexpr int kDetectFrameInterval = 2;

volatile bool g_drive_by_enable = false;
volatile bool g_drive_by_busy = false;
bool g_seen_lock = false;
DriveByState g_state = DB_IDLE;
DriveByClock::time_point g_state_start = DriveByClock::now();
DriveByClock::time_point g_cooldown_start = DriveByClock::now();
int g_frame_counter = 0;
int g_votes[3] = {0, 0, 0};
int g_infer_count = 0;
SavedControl g_saved;

long long elapsed_ms(DriveByClock::time_point start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               DriveByClock::now() - start)
        .count();
}

const char *state_name(DriveByState state)
{
    switch (state) {
    case DB_IDLE: return "IDLE";
    case DB_STOPPING: return "STOPPING";
    case DB_INFER: return "INFER";
    case DB_LEFT_TURN_OUT: return "LEFT_TURN_OUT";
    case DB_LEFT_FORWARD: return "LEFT_FORWARD";
    case DB_LEFT_TURN_BACK: return "LEFT_TURN_BACK";
    case DB_LEFT_EXIT_FORWARD: return "LEFT_EXIT_FORWARD";
    case DB_RIGHT_TURN_OUT: return "RIGHT_TURN_OUT";
    case DB_RIGHT_FORWARD: return "RIGHT_FORWARD";
    case DB_RIGHT_TURN_BACK: return "RIGHT_TURN_BACK";
    case DB_RIGHT_EXIT_FORWARD: return "RIGHT_EXIT_FORWARD";
    default: return "UNKNOWN";
    }
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

    // 不恢复旧 error，避免脚本结束后一瞬间按过期误差猛打方向。
    latest_error = 0;
    g_saved.valid = false;
}

void command_wheel_speed(int left_rps, int right_rps)
{
    // 这里只写目标 RPS；真正的闭环跟踪由 3ms 速度环 test_enc_and_motor_rps() 完成。
    // set_speed* 同步写，方便 TFT/VOFA 调试时看到脚本正在给什么目标速度。
    set_speed_of_motor1_rps = left_rps;
    set_speed_of_motor2_rps = right_rps;
    pwm1_duty_rps = left_rps;
    pwm2_duty_rps = right_rps;
    latest_error = 0;
}

void command_stop()
{
    command_wheel_speed(0, 0);
    PID_control_test(0);
}

void enter_state(DriveByState next)
{
    g_state = next;
    g_state_start = DriveByClock::now();

    switch (next) {
    case DB_STOPPING:
        command_stop();
        break;
    case DB_INFER:
        g_votes[0] = 0;
        g_votes[1] = 0;
        g_votes[2] = 0;
        g_infer_count = 0;
        command_stop();
        break;
    case DB_LEFT_TURN_OUT:
        command_wheel_speed(drive_by_turn_inner_speed_rps, drive_by_turn_speed_rps);
        break;
    case DB_LEFT_FORWARD:
        command_wheel_speed(drive_by_forward_speed_rps, drive_by_forward_speed_rps);
        break;
    case DB_LEFT_TURN_BACK:
        command_wheel_speed(drive_by_turn_speed_rps, drive_by_turn_inner_speed_rps);
        break;
    case DB_LEFT_EXIT_FORWARD:
        command_wheel_speed(drive_by_exit_speed_rps, drive_by_exit_speed_rps);
        break;
    case DB_RIGHT_TURN_OUT:
        command_wheel_speed(drive_by_turn_speed_rps, drive_by_turn_inner_speed_rps);
        break;
    case DB_RIGHT_FORWARD:
        command_wheel_speed(drive_by_forward_speed_rps, drive_by_forward_speed_rps);
        break;
    case DB_RIGHT_TURN_BACK:
        command_wheel_speed(drive_by_turn_inner_speed_rps, drive_by_turn_speed_rps);
        break;
    case DB_RIGHT_EXIT_FORWARD:
        command_wheel_speed(drive_by_exit_speed_rps, drive_by_exit_speed_rps);
        break;
    default:
        break;
    }

    printf("[drive_by] state=%s\n", state_name(g_state));
}

void finish_script()
{
    restore_control();
    g_drive_by_busy = false;
    g_state = DB_IDLE;
    g_cooldown_start = DriveByClock::now();
    printf("[drive_by] finish, item_flag=%d\n", item_flag);
}

void start_script()
{
    save_control_once();
    g_drive_by_busy = true;
    g_seen_lock = true;
    item_flag = 1;
    enter_state(DB_STOPPING);
}

void reset_runtime(bool restore_outputs)
{
    if (restore_outputs) {
        restore_control();
    }
    g_drive_by_busy = false;
    g_seen_lock = false;
    g_state = DB_IDLE;
    g_frame_counter = 0;
    g_votes[0] = 0;
    g_votes[1] = 0;
    g_votes[2] = 0;
    g_infer_count = 0;
    have_target = false;
}

bool should_check_this_frame()
{
    g_frame_counter++;
    if (g_frame_counter >= kDetectFrameInterval) {
        g_frame_counter = 0;
        return true;
    }
    return false;
}

bool make_plate_roi(cv::Mat& frame, cv::Mat& roi)
{
    if (frame.empty() || plate_rect.width <= 0 || plate_rect.height <= 0) {
        return false;
    }

    cv::Rect image_rect(0, 0, frame.cols, frame.rows);
    cv::Rect safe_rect = plate_rect & image_rect;
    if (safe_rect.width <= 0 || safe_rect.height <= 0) {
        return false;
    }

    roi = frame(safe_rect).clone();
    cv::resize(roi, roi, cv::Size(kSaveSize, kSaveSize), 0, 0, cv::INTER_AREA);
    return true;
}

int map_infer_result(const std::string& result)
{
    if (result == "weapon") {
        return 0;
    }
    if (result == "supplies") {
        return 2;
    }
    return 1;
}

int choose_vote_result()
{
    int best = 1;
    int best_votes = g_votes[1];
    bool tie = false;

    for (int i = 0; i < 3; ++i) {
        if (i == best) {
            continue;
        }
        if (g_votes[i] > best_votes) {
            best = i;
            best_votes = g_votes[i];
            tie = false;
        } else if (g_votes[i] == best_votes) {
            tie = true;
        }
    }

    if (best_votes == 0 || tie) {
        return 1;
    }
    return best;
}

void infer_one_valid_frame(cv::Mat& frame, LQ_NCNN& ncnn)
{
    detectRedPlate(frame);
    if (!have_target) {
        return;
    }

    cv::Mat roi;
    if (!make_plate_roi(frame, roi)) {
        return;
    }

    const std::string result = ncnn.Infer(roi);
    const int mapped = map_infer_result(result);
    g_votes[mapped]++;
    g_infer_count++;
    printf("[drive_by] infer %d/%d: %s -> %d\n",
           g_infer_count, kInferFrames, result.c_str(), mapped);
}

void update_idle_detection(cv::Mat& frame)
{
    if (!should_check_this_frame()) {
        return;
    }

    detectRedPlate(frame);

    if (g_seen_lock) {
        if (!have_target && elapsed_ms(g_cooldown_start) >= drive_by_cooldown_ms) {
            g_seen_lock = false;
            printf("[drive_by] target lock cleared\n");
        }
        return;
    }

    if (have_target) {
        printf("[drive_by] target found\n");
        start_script();
    }
}

void update_busy_state(cv::Mat& frame, LQ_NCNN& ncnn)
{
    switch (g_state) {
    case DB_STOPPING:
        command_stop();
        if (elapsed_ms(g_state_start) >= drive_by_stop_ms) {
            enter_state(DB_INFER);
        }
        break;

    case DB_INFER:
        command_stop();
        infer_one_valid_frame(frame, ncnn);
        if (g_infer_count >= kInferFrames ||
            elapsed_ms(g_state_start) >= drive_by_infer_timeout_ms) {
            item_flag = choose_vote_result();
            printf("[drive_by] vote: L=%d S=%d R=%d => %d\n",
                   g_votes[0], g_votes[1], g_votes[2], item_flag);
            if (item_flag == 0) {
                enter_state(DB_LEFT_TURN_OUT);
            } else if (item_flag == 2) {
                enter_state(DB_RIGHT_TURN_OUT);
            } else {
                finish_script();
            }
        }
        break;

    case DB_LEFT_TURN_OUT:
        command_wheel_speed(drive_by_turn_inner_speed_rps, drive_by_turn_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_turn_out_ms) {
            enter_state(DB_LEFT_FORWARD);
        }
        break;

    case DB_LEFT_FORWARD:
        command_wheel_speed(drive_by_forward_speed_rps, drive_by_forward_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_forward_ms) {
            enter_state(DB_LEFT_TURN_BACK);
        }
        break;

    case DB_LEFT_TURN_BACK:
        command_wheel_speed(drive_by_turn_speed_rps, drive_by_turn_inner_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_turn_back_ms) {
            enter_state(DB_LEFT_EXIT_FORWARD);
        }
        break;

    case DB_LEFT_EXIT_FORWARD:
        command_wheel_speed(drive_by_exit_speed_rps, drive_by_exit_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_exit_forward_ms) {
            finish_script();
        }
        break;

    case DB_RIGHT_TURN_OUT:
        command_wheel_speed(drive_by_turn_speed_rps, drive_by_turn_inner_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_turn_out_ms) {
            enter_state(DB_RIGHT_FORWARD);
        }
        break;

    case DB_RIGHT_FORWARD:
        command_wheel_speed(drive_by_forward_speed_rps, drive_by_forward_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_forward_ms) {
            enter_state(DB_RIGHT_TURN_BACK);
        }
        break;

    case DB_RIGHT_TURN_BACK:
        command_wheel_speed(drive_by_turn_inner_speed_rps, drive_by_turn_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_turn_back_ms) {
            enter_state(DB_RIGHT_EXIT_FORWARD);
        }
        break;

    case DB_RIGHT_EXIT_FORWARD:
        command_wheel_speed(drive_by_exit_speed_rps, drive_by_exit_speed_rps);
        if (elapsed_ms(g_state_start) >= drive_by_exit_forward_ms) {
            finish_script();
        }
        break;

    default:
        finish_script();
        break;
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
        update_idle_detection(frame);
    }
}

bool drive_by_is_busy()
{
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
    printf("[drive_by] enable=%d\n", enable ? 1 : 0);
}

void drive_by_toggle_enable()
{
    drive_by_set_enable(!g_drive_by_enable);
}

void drive_by_cancel()
{
    reset_runtime(true);
}

const char *drive_by_state_name()
{
    return state_name(g_state);
}
