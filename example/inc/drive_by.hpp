#ifndef __DRIVE_BY_HPP
#define __DRIVE_BY_HPP

#include "lq_ncnn.hpp"

#include <opencv2/core.hpp>

// 目标板脚本的占位参数，后续可以按实车/路线图继续调。
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

void drive_by_init();
void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn);
bool drive_by_is_busy();
bool drive_by_is_enabled();
void drive_by_set_enable(bool enable);
void drive_by_toggle_enable();
// 取消当前测试并复位状态；返回 true 表示取消前正在执行三帧测试。
bool drive_by_cancel();
const char *drive_by_state_name();

#endif
