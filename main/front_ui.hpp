#ifndef __FRONT_UI_HPP
#define __FRONT_UI_HPP
#include <chrono>
// 初始化车载前端：三颗实体按键始终保留，TFT18 屏幕由 tft_UI_switch 控制。
void front_ui_init();

// 在主循环中反复调用，用来扫描按键并刷新屏幕。
void front_ui_poll();

void front_ui_start();
void front_ui_stop();
void front_ui_set_running(bool running);

// 停车遥控命令。仅run=0时接受；前端按住按钮期间需要周期续发，
// 超过看门狗时间未收到续发会自动停止，防止UDP断开后持续运动。
enum FrontUiRemoteCommand {
    FRONT_UI_REMOTE_STOP = 0,
    FRONT_UI_REMOTE_FORWARD = 1,
    FRONT_UI_REMOTE_BACKWARD = 2,
    FRONT_UI_REMOTE_LEFT = 3,
    FRONT_UI_REMOTE_RIGHT = 4,
};
bool front_ui_remote_set(int command);
bool front_ui_remote_is_active();
void front_ui_remote_control_update();

// 停车保持：停车且未启用遥控时调用，持续把目标速度和PWM清零。
void front_ui_hold_stop();

// 给主控制循环判断：当前是否允许速度环/方向环工作。
bool front_ui_is_running();
extern int selected_strategy;
extern bool car_running;
extern bool ui_ready;
extern volatile int tft_UI_switch;
// 当前选中的速度策略目标值，单位 rps。
int front_ui_selected_speed();
// 选择已有速度策略；车辆正在运行时会立即应用新的基准速度。
bool front_ui_select_speed(int speed_rps);
extern volatile float AIM;
extern std::chrono::steady_clock::time_point last_start_time;
#endif
