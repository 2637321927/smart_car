#include "drive_by.hpp"
#include "lq_all_demo.hpp"
#include "front_ui.hpp"
#include <chrono>
#include <cstdio>
#include <string>
#include <opencv2/imgproc.hpp>

using DriveByClock = std::chrono::steady_clock;


// ===================== drive_by 绕行动作调参区 =====================
// 这些默认值来自你推车绕行时的编码器曲线。
// 脚本通过“速度环闭环目标 RPS + 固定持续时间”复现轨迹，不直接打 PWM。
//
// 姿态调节优先改时间，再小幅改速度：
// 1. 转出不够/太多：先调 drive_by_turn_out_ms，再调 drive_by_turn_speed_rps。
// 2. 横向绕行距离不够/太远：调 drive_by_forward_ms。
// 3. 回正不够/过头：先调 drive_by_turn_back_ms，再调 drive_by_turn_speed_rps。
// 4. 结束时还想向前补一点：调 drive_by_exit_speed_rps 和 drive_by_exit_forward_ms。
//
// 左绕会镜像右绕：
// 左绕转出 = 左轮 inner、右轮 outer；右绕转出 = 左轮 outer、右轮 inner。
int drive_by_turn_speed_rps = 3;          // 外侧轮转向速度，越大转得越猛，单位 rps
int drive_by_turn_inner_speed_rps = 0;    // 内侧轮转向速度，0 表示近似单轮转向
int drive_by_forward_speed_rps = 6;       // 绕开目标板时双轮前进速度，单位 rps
int drive_by_exit_speed_rps = 0;          // 脚本结束前补偿前进速度，0 表示不补前进
int drive_by_turn_out_ms = 600;          // 第一次向外转的持续时间，影响绕行起始姿态
int drive_by_forward_ms = 400;            // 绕过目标板时直行持续时间，影响横向/前向绕行距离
int drive_by_turn_back_ms = 800;          // 往回转的持续时间，影响回正姿态
int drive_by_exit_forward_ms = 150;       // 回正后补偿前进时间
int drive_by_stop_ms = 200;               // 旧绕行脚本停车参数，当前动态三帧测试不使用
int drive_by_infer_timeout_ms = 1000;     // 旧绕行脚本推理超时，当前动态三帧测试不使用
int drive_by_cooldown_ms =1000;             // 脚本完成后的再次触发冷却，0 表示目标离开画面即可解锁

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

constexpr int kInferFrames = 3;
constexpr int kSaveSize = 96;
constexpr int kDetectFrameInterval = 2;
constexpr int kRecognitionTestSpeedRps = 35;

enum RecognitionFrameStatus {
    TEST_FRAME_NOT_PROCESSED,
    TEST_FRAME_OK,
    TEST_FRAME_NO_RED,
    TEST_FRAME_INVALID_ROI,
    TEST_FRAME_UNKNOWN_LABEL,
};

struct RecognitionFrameRecord {
    RecognitionFrameStatus status = TEST_FRAME_NOT_PROCESSED;
    std::string label;
    int mapped_result = 1;
    double detect_ms = 0.0;
    double prepare_ms = 0.0;
    double infer_ms = 0.0;
    double frame_ms = 0.0;
    double since_trigger_ms = 0.0;
};

struct RecognitionTestReport {
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
DriveByState g_state = DB_IDLE;
DriveByClock::time_point g_state_start = DriveByClock::now();
DriveByClock::time_point g_cooldown_start = DriveByClock::now();
int g_frame_counter = 0;
int g_votes[3] = {0, 0, 0};
SavedControl g_saved;
RecognitionTestReport g_test_report;
DriveByClock::time_point g_test_trigger_time = DriveByClock::now();

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

const char *frame_status_name(RecognitionFrameStatus status)
{
    switch (status) {
    case TEST_FRAME_OK: return "成功";
    case TEST_FRAME_NO_RED: return "未检测到红色";
    case TEST_FRAME_INVALID_ROI: return "目标板区域无效";
    case TEST_FRAME_UNKNOWN_LABEL: return "未知类别";
    default: return "未处理";
    }
}

const char *label_chinese_name(const std::string& label)
{
    if (label == "weapon") {
        return "武器";
    }
    if (label == "vehicle") {
        return "车辆";
    }
    if (label == "supplies") {
        return "物资";
    }
    return "未知类别";
}

const char *result_action_name(int result)
{
    switch (result) {
    case 0: return "左绕";
    case 2: return "右绕";
    default: return "直行";
    }
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

void command_recognition_test_cruise()
{
    // 测速模式只允许直行：左右轮目标严格相等，不调用方向环产生差速。
    // 两个独立速度环仍会分别修正各自 PWM，因此实际编码器值允许存在小误差。
    set_speed_of_motor1_rps = kRecognitionTestSpeedRps;
    set_speed_of_motor2_rps = kRecognitionTestSpeedRps;
    pwm1_duty_rps = kRecognitionTestSpeedRps;
    pwm2_duty_rps = kRecognitionTestSpeedRps;
    latest_error = 0;
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

void start_recognition_test(double trigger_detect_ms)
{
    save_control_once();
    g_drive_by_busy = true;
    g_seen_lock = true;
    item_flag = 1;
    g_state = DB_INFER;
    g_state_start = DriveByClock::now();
    g_test_trigger_time = g_state_start;
    g_test_report = RecognitionTestReport{};
    g_test_report.trigger_detect_ms = trigger_detect_ms;
    g_test_report.trigger_left_rps = encoder1_speed_avg;
    g_test_report.trigger_right_rps = encoder2_speed_avg;
    g_votes[0] = 0;
    g_votes[1] = 0;
    g_votes[2] = 0;
    command_recognition_test_cruise();
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
    g_test_report = RecognitionTestReport{};
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

bool is_known_infer_result(const std::string& result)
{
    return result == "weapon" || result == "vehicle" || result == "supplies";
}

void finish_frame_record(RecognitionFrameRecord& record,
                         DriveByClock::time_point frame_start)
{
    const DriveByClock::time_point frame_end = DriveByClock::now();
    record.frame_ms = duration_ms(frame_start, frame_end);
    record.since_trigger_ms = duration_ms(g_test_trigger_time, frame_end);
}

void process_recognition_test_frame(cv::Mat& frame,
                                    LQ_NCNN& ncnn,
                                    bool reuse_trigger_detection)
{
    if (g_test_report.frame_count >= kInferFrames) {
        return;
    }

    RecognitionFrameRecord& record =
        g_test_report.frames[g_test_report.frame_count];
    g_test_report.frame_count++;
    const DriveByClock::time_point frame_start = DriveByClock::now();

    // 第1帧复用刚刚触发测试时得到的 plate_rect，避免同一帧重复做红色检测。
    // 第2、3帧必须重新检测，才能真实观察运动中红色和目标板 ROI 是否稳定。
    if (!reuse_trigger_detection) {
        const DriveByClock::time_point detect_start = DriveByClock::now();
        detectRedPlate(frame);
        const DriveByClock::time_point detect_end = DriveByClock::now();
        record.detect_ms = duration_ms(detect_start, detect_end);

        if (!have_target) {
            record.status = TEST_FRAME_NO_RED;
            finish_frame_record(record, frame_start);
            return;
        }
    }

    cv::Mat roi;
    const DriveByClock::time_point prepare_start = DriveByClock::now();
    if (!make_plate_roi(frame, roi)) {
        record.prepare_ms = duration_ms(prepare_start, DriveByClock::now());
        record.status = TEST_FRAME_INVALID_ROI;
        finish_frame_record(record, frame_start);
        return;
    }
    const DriveByClock::time_point prepare_end = DriveByClock::now();
    record.prepare_ms = duration_ms(prepare_start, prepare_end);

    const DriveByClock::time_point infer_start = DriveByClock::now();
    record.label = ncnn.Infer(roi);
    const DriveByClock::time_point infer_end = DriveByClock::now();
    record.infer_ms = duration_ms(infer_start, infer_end);
    record.mapped_result = map_infer_result(record.label);

    if (is_known_infer_result(record.label)) {
        record.status = TEST_FRAME_OK;
        g_votes[record.mapped_result]++;
        g_test_report.votes[record.mapped_result]++;
        g_test_report.valid_count++;
        g_test_report.infer_sum_ms += record.infer_ms;
    } else {
        record.status = TEST_FRAME_UNKNOWN_LABEL;
    }

    finish_frame_record(record, frame_start);
}

void print_recognition_test_report(const RecognitionTestReport& report,
                                   int final_result)
{
    printf("[识别测试] 检测到红色：检测耗时=%.2f毫秒，左轮=%.2fRPS，右轮=%.2fRPS\n",
           report.trigger_detect_ms,
           report.trigger_left_rps,
           report.trigger_right_rps);

    for (int index = 0; index < report.frame_count; ++index) {
        const RecognitionFrameRecord& record = report.frames[index];
        printf("[识别测试] 第%d/%d帧：%s\n",
               index + 1,
               kInferFrames,
               frame_status_name(record.status));

        if (!record.label.empty()) {
            printf("  模型标签=%s（%s）\n",
                   record.label.c_str(),
                   label_chinese_name(record.label));
            printf("  识别结果=%d（%s）\n",
                   record.mapped_result,
                   result_action_name(record.mapped_result));
        }

        printf("  红色检测=%.2f毫秒，图像准备=%.2f毫秒，模型推理=%.2f毫秒\n",
               record.detect_ms,
               record.prepare_ms,
               record.infer_ms);
        printf("  单帧处理=%.2f毫秒，触发后累计=%.2f毫秒\n",
               record.frame_ms,
               record.since_trigger_ms);
    }

    const char *stability = "无有效结果";
    if (report.valid_count == kInferFrames) {
        stability = "成功";
    } else if (report.valid_count > 0) {
        stability = "部分成功";
    }

    printf("[识别测试] 三帧汇总\n");
    printf("  最终结果=%d（%s）\n", final_result, result_action_name(final_result));
    printf("  投票结果：左绕=%d，直行=%d，右绕=%d\n",
           report.votes[0], report.votes[1], report.votes[2]);
    printf("  有效帧数=%d/%d\n", report.valid_count, kInferFrames);
    printf("  推理耗时合计=%.2f毫秒，三帧总时间=%.2f毫秒\n",
           report.infer_sum_ms,
           report.total_ms);
    printf("  稳定性评价=%s\n", stability);
    printf("  第三帧结束时：左轮=%.2fRPS，右轮=%.2fRPS\n",
           report.finish_left_rps,
           report.finish_right_rps);
    printf("[识别测试] 已停车：左轮目标=%.2fRPS，右轮目标=%.2fRPS，PWM目标=(%.2f, %.2f)\n",
           static_cast<double>(set_speed_of_motor1_rps),
           static_cast<double>(set_speed_of_motor2_rps),
           static_cast<double>(pwm1_duty_rps),
           static_cast<double>(pwm2_duty_rps));
}

void finish_recognition_test()
{
    g_test_report.total_ms = duration_ms(g_test_trigger_time, DriveByClock::now());
    g_test_report.finish_left_rps = encoder1_speed_avg;
    g_test_report.finish_right_rps = encoder2_speed_avg;
    item_flag = choose_vote_result();

    // front_ui_stop() 会取消 drive_by 并清空运行状态，因此先复制报告。
    // 停车必须早于 printf，避免控制台输出延迟停车命令。
    const RecognitionTestReport report = g_test_report;
    const int final_result = item_flag;
    // 本测试结束后必须保持停车，不需要在 cancel 中短暂恢复触发前的35RPS目标。
    g_saved.valid = false;
    // 正常完成不是“中止”，先清 busy，避免 front_ui_stop() 输出中止提示。
    g_drive_by_busy = false;
    front_ui_stop();
    print_recognition_test_report(report, final_result);
}

bool update_idle_detection(cv::Mat& frame)
{
    if (!should_check_this_frame()) {
        return false;
    }

    const DriveByClock::time_point detect_start = DriveByClock::now();
    detectRedPlate(frame);
    const DriveByClock::time_point detect_end = DriveByClock::now();

    if (g_seen_lock) {
        if (!have_target && elapsed_ms(g_cooldown_start) >= drive_by_cooldown_ms) {
            g_seen_lock = false;
            printf("[drive_by] target lock cleared\n");
        }
        return false;
    }

    if (have_target) {
        start_recognition_test(duration_ms(detect_start, detect_end));
        return true;
    }

    return false;
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
        // 三帧完成前保持35RPS直行；不等待三次成功，失败帧同样计数。
        command_recognition_test_cruise();
        process_recognition_test_frame(frame, ncnn, false);
        if (g_test_report.frame_count >= kInferFrames) {
            finish_recognition_test();
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
        // K0测速模式启用后持续锁定35RPS直行，防止K1或VOFA残留速度影响实验。
        command_recognition_test_cruise();
        if (update_idle_detection(frame)) {
            // 触发帧就是第1帧，直接复用当前 plate_rect，不等下一张图。
            process_recognition_test_frame(frame, ncnn, true);
        }
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
