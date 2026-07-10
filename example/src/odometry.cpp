#include "odometry.hpp"

// 累计行驶距离（米）
static float g_odom_distance_m = 0.0f;

void odometry_reset()
{
    g_odom_distance_m = 0.0f;
}

void odometry_update(float avg_rps, float dt_sec)
{
    // 安全检查：防止异常大的 dt（比如刚启动、暂停恢复）
    if (dt_sec <= 0.0f || dt_sec > 0.5f) return;
    if (avg_rps < 0.0f) avg_rps = -avg_rps;  // 取绝对值

    // distance += speed(m/s) × time(s)
    // speed = avg_rps × wheel_circumference (m)
    g_odom_distance_m += avg_rps * WHEEL_CIRCUMFERENCE_M * dt_sec;
}

float odometry_get_distance()
{
    return g_odom_distance_m;
}
