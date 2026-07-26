#include "front_ui.hpp"
#include "circle.hpp"
#include "drive_by.hpp"
#include "lq_all_demo.hpp"
#include "gyro_yaw_rate_control.hpp"
#include "lq_display_tft18.hpp"

#include <chrono>
#include <cstdio>
int selected_strategy = 1;
bool car_running = false;
bool ui_ready = false;
volatile int tft_UI_switch = 0;
std::chrono::steady_clock::time_point last_start_time;
namespace {

// 一个速度策略由“屏幕显示名”和“目标速度 rps”组成。
// 后续想改速度档位，只需要改下面 kStrategies 这张表。
struct SpeedStrategy {
    const char *name;
    int speed_rps;
};

// TFT18 前端提供的速度档位。默认 selected_strategy = 1，即 35 RPS。
constexpr SpeedStrategy kStrategies[] = {
    {"20", 20},
    {"35", 35},
    {"0", 0},
    {"15", 15},
};

constexpr int kStrategyCount = sizeof(kStrategies) / sizeof(kStrategies[0]);

// 301 母板按键一般是按下接地，所以这里按低电平为“按下”处理。
// 如果实测按键逻辑相反，把 GPIO_LOW 改成 GPIO_HIGH 即可。
constexpr gpio_level_t kKeyPressedLevel = GPIO_LOW;

// 去抖时间：避免一次按键因为机械抖动被识别成多次。
constexpr auto kDebounceTime = std::chrono::milliseconds(180);

// 没有按键动作时，也定时刷新一次屏幕，方便显示当前输出值。
constexpr auto kRefreshTime = std::chrono::milliseconds(500);

// 遥控只用于run=0时把车低速开回起点。前进/后退固定15RPS，
// 左右使用陀螺仪角速度内环原地转向，固定目标角速度60dps。
constexpr float kRemoteLinearSpeedRps = 15.0f;
constexpr float kRemoteYawRateDps = 60.0f;
constexpr auto kRemoteCommandTimeout = std::chrono::milliseconds(250);

// 引脚来自 Doc/301母版引脚资源分配.png：
// GPIO44 -> 按键0，GPIO45 -> 按键1，GPIO80 -> 按键2。
ls_gpio key_prev(PIN_44, GPIO_MODE_IN);
ls_gpio key_next(PIN_45, GPIO_MODE_IN);
ls_gpio key_start_stop(PIN_80, GPIO_MODE_IN);

// selected_strategy 保存当前档位下标；car_running 是一键发车/停车状态。

// 每个按键都保存上一次状态和上一次有效触发时间，用于边沿检测和去抖。
struct ButtonState {
    bool last_pressed = false;
    std::chrono::steady_clock::time_point last_event =
        std::chrono::steady_clock::now() - kDebounceTime;
};

ButtonState prev_state;
ButtonState next_state;
ButtonState start_stop_state;
std::chrono::steady_clock::time_point last_refresh =
    std::chrono::steady_clock::now() - kRefreshTime;
bool tft_ready = false;
volatile int remote_command = FRONT_UI_REMOTE_STOP;
std::chrono::steady_clock::time_point remote_last_refresh =
    std::chrono::steady_clock::now() - kRemoteCommandTimeout;

void clear_remote_outputs()
{
    remote_command = FRONT_UI_REMOTE_STOP;
    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    pwm1_duty_rps = 0.0f;
    pwm2_duty_rps = 0.0f;
    gyro_yaw_rate_control_reset_controller();
    motor_speed_pid_reset();
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
}

bool is_pressed(ls_gpio &key)
{
    return key.gpio_level_get() == kKeyPressedLevel;
}

// 只在“松开 -> 按下”的瞬间返回 true。
// 这样长按不会一直连发，同时用 kDebounceTime 过滤按键抖动。
bool pressed_edge(ls_gpio &key, ButtonState &state)
{
    const bool pressed = is_pressed(key);
    const auto now = std::chrono::steady_clock::now();
    const bool trigger = pressed && !state.last_pressed && now - state.last_event >= kDebounceTime;

    state.last_pressed = pressed;
    if (trigger) {
        state.last_event = now;
    }

    return trigger;
}

void apply_speed_strategy()
{
    // 发车状态下，把当前档位速度同步给左右两个电机目标速度。
    set_speed_of_motor1_rps = kStrategies[selected_strategy].speed_rps;
    set_speed_of_motor2_rps = kStrategies[selected_strategy].speed_rps;
}

void select_speed_strategy(int speed_rps)
{
    for (int index = 0; index < kStrategyCount; ++index) {
        if (kStrategies[index].speed_rps != speed_rps) {
            continue;
        }

        selected_strategy = index;
        if (car_running) {
            apply_speed_strategy();
        }
        return;
    }
}

void stop_car()
{
    // K2 停车是最高优先级：如果目标板脚本正在接管，先取消脚本，再清零电机。
    drive_by_heading_hold_set_enable(false);
    const bool recognition_test_aborted = drive_by_cancel();
    reset_special_track_state();
    // 停车不仅清目标速度，也清方向环输出和当前 PWM，避免定时器残留输出。
    car_running = false;
    remote_command = FRONT_UI_REMOTE_STOP;
    gyro_yaw_rate_control_reset();
    set_speed_of_motor1_rps = 0;
    set_speed_of_motor2_rps = 0;
    pwm1_duty_rps = 0;
    pwm2_duty_rps = 0;
    motor_speed_pid_reset();
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
    if (recognition_test_aborted) {
        printf("[识别测试] 测试已中止，车辆已停车\n");
    }
}

void start_car()
{
    // 发车时不直接写死速度，而是使用当前屏幕上选中的速度策略。
    // 遥控松开包即使稍晚到达，也不能覆盖已经开始的正常行驶。
    // 航向保持属于停车态专用测试，发车前必须先清掉它的对称轮速目标。
    drive_by_heading_hold_set_enable(false);
    remote_command = FRONT_UI_REMOTE_STOP;
    car_running = true;
    gyro_yaw_rate_control_reset();
    motor_speed_pid_reset();
    pwm1.atim_pwm_set_duty(0);
    pwm2.atim_pwm_set_duty(0);
    apply_speed_strategy();
    std::cout << "run\n";

    last_start_time = std::chrono::steady_clock::now();


}

void draw_line(uint8_t row, const char *text, lq_display_color_t color)
{
    if (!tft_ready) {
        return;
    }

    // TFT18 的 p8x16 字符坐标按字符格计算，row=0/1/2... 每行 16 像素高。
    lq_tft18_drv_p8x16_str(0, row, text, color, U16BLACK);
}

void draw_ui()
{
    if (tft_UI_switch == 0 || !tft_ready) {
        return;
    }

    char line[24] = {0};
    const lq_display_color_t state_color = car_running ? U16GREEN : U16RED;

    // 第一行用色条显示 RUNNING/STOPPED，方便一眼判断车是否允许运动。
    lq_tft18_drv_cls(U16BLACK);
    lq_tft18_drv_fill_area(0, 0, 127, 15, state_color);
    lq_tft18_drv_p8x16_str(0, 0, car_running ? "RUNNING" : "STOPPED", U16BLACK, state_color);

    // MODE 是当前策略名，SPD 是策略目标速度，OUT 是实际写给左右电机的目标速度。
    snprintf(line, sizeof(line), "MODE : %s", kStrategies[selected_strategy].name);
    draw_line(2, line, U16CYAN);

    snprintf(line, sizeof(line), "SPD  : %02d RPS", kStrategies[selected_strategy].speed_rps);
    draw_line(3, line, U16YELLOW);

    snprintf(line, sizeof(line), "OUT  : %04.1f/%04.1f",
             set_speed_of_motor1_rps, set_speed_of_motor2_rps);
    draw_line(4, line, U16WHITE);

    snprintf(line, sizeof(line), "TARGET:%s", drive_by_is_enabled() ? "ON" : "OFF");
    draw_line(5, line, drive_by_is_enabled() ? U16GREEN : U16RED);

    draw_line(6, "K0 TARGET K1 SPD", U16WHITE);
    draw_line(7, "K2 START/STOP", U16WHITE);
}

void select_prev_strategy()
{
    // 加 kStrategyCount 再取模，可以让 LOW 往前切时回到 MAX。
    selected_strategy = (selected_strategy + kStrategyCount - 1) % kStrategyCount;
    if (car_running) {
        apply_speed_strategy();
    }
    printf("target speed=%d RPS\n", kStrategies[selected_strategy].speed_rps);
}

void select_next_strategy()
{
    // 往后切档位，到最后一个档位后再回到第一个档位。
    selected_strategy = (selected_strategy + 1) % kStrategyCount;
    if (car_running) {
        apply_speed_strategy();
    }
    printf("target speed=%d RPS\n", kStrategies[selected_strategy].speed_rps);
}

} // namespace

void front_ui_init()
{
    stop_car();
    ui_ready = true;

    // 按键功能必须保留；这里只根据开关决定是否启用 TFT 显示。
    if (tft_UI_switch != 0) {
        // 0 表示 TFT18 横屏初始化；启动后先停车，等 K2 发车。
        lq_tft18_drv_init(0);
        tft_ready = true;
        draw_ui();
    } else {
        tft_ready = false;
    }
}

void front_ui_poll()
{
    if (!ui_ready) {
        return;
    }

    bool dirty = false;

    // K0：目标板识别与绕行开关。关闭时完全不检测红色，不影响普通巡线。
    if (pressed_edge(key_prev, prev_state)) {
        drive_by_toggle_enable();
        if (drive_by_is_enabled()) {
            // 目标板模式正常阶段固定35RPS，红色触发后由drive_by主动制动并降到10RPS。
            select_speed_strategy(35);
        }
        dirty = true;
        std::cout << "drive_by_enable:" << drive_by_is_enabled() << '\n';
    }

    // K1：下一个速度策略。
    if (pressed_edge(key_next, next_state)) {
        // 目标板模式的正常速度固定35RPS，因此K0开启期间忽略K1切档。
        if (!drive_by_is_enabled()) {
            select_next_strategy();
        }
        dirty = true;
    }

    // K2：一键发车/停车。
    if (pressed_edge(key_start_stop, start_stop_state)) {
        front_ui_set_running(!car_running);
        dirty = true;
    }

    // 有按键动作立即刷新；没动作时也按固定周期刷新，显示最新输出值。
    const auto now = std::chrono::steady_clock::now();
    if (dirty || now - last_refresh >= kRefreshTime) {
        draw_ui();
        last_refresh = now;
    }
}

bool front_ui_is_running()
{
    return car_running;
}

void front_ui_start()
{
    start_car();
}

void front_ui_stop()
{
    stop_car();
}

void front_ui_set_running(bool running)
{
    if (running && car_running) {
        return;
    }

    if (running) {
        front_ui_start();
    } else {
        front_ui_stop();
    }
}

bool front_ui_remote_set(int command)
{
    if (command < FRONT_UI_REMOTE_STOP || command > FRONT_UI_REMOTE_RIGHT) {
        return false;
    }

    if (command == FRONT_UI_REMOTE_STOP) {
        // 如果车辆已经正常发车，只清遥控标志，绝不能把正常巡线目标清零。
        remote_command = FRONT_UI_REMOTE_STOP;
        if (!car_running) {
            clear_remote_outputs();
        }
        return true;
    }

    // 遥控只属于停车模式。run=1时无条件拒绝，确保不会插入正常方向环。
    if (car_running) {
        remote_command = FRONT_UI_REMOTE_STOP;
        return false;
    }

    // 航向保持和停车遥控都会在run=0时驱动车轮，二者不能同时写控制目标。
    // 用户开始按遥控方向键时，优先退出航向保持再接管。
    drive_by_heading_hold_set_enable(false);

    const auto now = std::chrono::steady_clock::now();
    if (remote_command != command) {
        // 切换方向时清除上一个速度/角速度控制器的历史量，避免反向瞬间残留。
        gyro_yaw_rate_control_reset_controller();
        motor_speed_pid_reset();
        set_speed_of_motor1_rps = 0.0f;
        set_speed_of_motor2_rps = 0.0f;
        pwm1_duty_rps = 0.0f;
        pwm2_duty_rps = 0.0f;
        pwm1.atim_pwm_set_duty(0);
        pwm2.atim_pwm_set_duty(0);
    }
    remote_last_refresh = now;
    remote_command = command;
    return true;
}

bool front_ui_remote_is_active()
{
    if (car_running || remote_command == FRONT_UI_REMOTE_STOP) {
        return false;
    }

    if (std::chrono::steady_clock::now() - remote_last_refresh >
        kRemoteCommandTimeout) {
        clear_remote_outputs();
        return false;
    }
    return true;
}

void front_ui_remote_control_update()
{
    if (!front_ui_remote_is_active()) {
        return;
    }

    const int command = remote_command;
    if (command == FRONT_UI_REMOTE_FORWARD ||
        command == FRONT_UI_REMOTE_BACKWARD) {
        const float target_rps = command == FRONT_UI_REMOTE_FORWARD
            ? kRemoteLinearSpeedRps
            : -kRemoteLinearSpeedRps;
        set_speed_of_motor1_rps = target_rps;
        set_speed_of_motor2_rps = target_rps;
        pwm1_duty_rps = target_rps;
        pwm2_duty_rps = target_rps;
        return;
    }

    set_speed_of_motor1_rps = 0.0f;
    set_speed_of_motor2_rps = 0.0f;
    if (!gyro_yaw_rate_control_is_ready()) {
        // 没有角速度反馈时无法保证60dps，宁可保持停止，也不盲目给固定差速。
        pwm1_duty_rps = 0.0f;
        pwm2_duty_rps = 0.0f;
        return;
    }

    // 当前符号约定：负目标角速度左转，正目标角速度右转。
    const float target_yaw_rate_dps = command == FRONT_UI_REMOTE_LEFT
        ? -kRemoteYawRateDps
        : kRemoteYawRateDps;
    const float turn_rps =
        gyro_yaw_rate_control_update_target_yaw_rate(target_yaw_rate_dps);
    pwm1_duty_rps = turn_rps;
    pwm2_duty_rps = -turn_rps;
}

void front_ui_hold_stop()
{
    // 停车状态下由速度/方向定时器持续调用，防止其他控制环写入非零输出。
    stop_car();
}

int front_ui_selected_speed()
{
    return kStrategies[selected_strategy].speed_rps;
}
