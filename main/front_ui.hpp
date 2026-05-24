#ifndef __FRONT_UI_HPP
#define __FRONT_UI_HPP

// 初始化车载前端：TFT18 屏幕、三颗实体按键、默认停车状态。
void front_ui_init();

// 在主循环中反复调用，用来扫描按键并刷新屏幕。
void front_ui_poll();

// 停车保持：定时器在停车状态下调用，持续把目标速度和 PWM 清零。
void front_ui_hold_stop();

// 给主控制循环判断：当前是否允许速度环/方向环工作。
bool front_ui_is_running();

// 当前选中的速度策略目标值，单位 rps。
int front_ui_selected_speed();

#endif
