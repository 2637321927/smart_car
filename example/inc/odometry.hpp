#ifndef __ODOMETRY_HPP
#define __ODOMETRY_HPP

// ====================== 编码器里程计 ======================
// 用途：在环岛场景中，用编码器累计行驶距离，到达阈值后出环。
//
// 用法：
//   1. 进入环岛 RUNNING 态时调用 odometry_reset()
//   2. 每个主循环调用 odometry_update(avg_rps, dt_sec)
//   3. 当 odometry_get_distance() > 阈值时，触发 CIRCLE_*_OUT
//
// 注意：encoder1_speed_avg 值为负数（方向定义），取绝对值即可。

// ========== 需要实测的参数（车不同，值不同）==========

// 轮子直径（米），用软尺量轮胎外径。默认 64mm 供参考。
#define WHEEL_DIAMETER_M         (0.064f)

// 轮子周长（米）= π × 直径
#define WHEEL_CIRCUMFERENCE_M    (3.1415926f * WHEEL_DIAMETER_M)

// 环岛出环距离阈值（米），即进入环岛后跑多远就出环。
// 实际值需要根据赛道环岛弧长实测调整。
#define CIRCLE_EXIT_DISTANCE_M   (0.50f)

// ========== API ==========

#ifdef __cplusplus
extern "C" {
#endif

// 重置里程累计（进入环岛 RUNNING 态时调用）
void odometry_reset();

// 更新里程（每个主循环调用一次）
// avg_rps: 左右轮平均转速（RPS，取绝对值）
// dt_sec:  距离上次调用的时间间隔（秒）
void odometry_update(float avg_rps, float dt_sec);

// 获取累计里程（米）
float odometry_get_distance();

#ifdef __cplusplus
}
#endif

#endif // __ODOMETRY_HPP
