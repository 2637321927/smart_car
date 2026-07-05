# 角度环可行性方案

## 1. 结论

| 项目 | 结论 |
|---|---|
| 是否可行 | 可行，建议先用于目标板绕行脚本的“短时定角转向” |
| 推荐定位 | 角度环只控制短时间相对角度，不做长期绝对航向导航 |
| 推荐控制层级 | 角度环输出左右轮目标 RPS，继续复用现有速度环闭环到 PWM |
| 首版目标 | 用陀螺仪积分控制左/右转出角度、回正角度，替代纯时间转向 |
| 最大风险 | 陀螺仪量程比例和安装轴向未知，需要实车标定 |

## 2. 当前代码基础

| 模块 | 当前情况 | 对角度环的意义 |
|---|---|---|
| IMU 例程 | `MPU6050`、`ICM42688`、`LSM6DSR` 都有例程 | 可以直接读取三轴角速度和加速度 |
| IMU 输出 | 驱动返回 `int16_t ax/ay/az/gx/gy/gz` 原始值 | 需要自行做零偏、比例换算、积分 |
| 方向环 | `PID_control_test(latest_error)` 每 `8ms` 左右执行 | 正常巡线时继续使用，角度脚本 busy 时暂停 |
| 速度环 | `test_enc_and_motor_rps()` 每 `3ms` 执行 | 角度环只给目标 RPS，速度环负责闭环跟踪 |
| 目标板脚本 | `drive_by.cpp` 已经能写左右轮目标 RPS | 最适合先接入角度环 |
| 定时器 | `lq_timer` 超时只报警，不会打断回调 | 角度环回调必须很轻，不打印、不做复杂逻辑 |

## 3. 为什么先做“短时相对角度环”

| 方案 | 可行性 | 原因 |
|---|---|---|
| 短时相对角度 | 高 | 目标板绕行只需要几十度到一百多度，积分漂移影响小 |
| 长期绝对航向 | 中低 | 只有陀螺仪积分会漂移，长时间跑会累积误差 |
| 用加速度修正 yaw | 不推荐 | 加速度只能辅助俯仰/横滚，不能直接修正平面 yaw |
| 和图像方向环融合 | 可做二期 | 需要避免和 `latest_error` 抢控制，第一版风险较高 |

## 4. 推荐控制架构

| 层级 | 输入 | 输出 | 周期 |
|---|---|---|---|
| IMU 采样/积分层 | `gx/gy/gz` 原始角速度 | `gyro_rate_dps`、`yaw_deg` | `5ms` 建议值 |
| 角度控制层 | `target_yaw_deg - yaw_deg` | `angle_diff_rps` | `8ms` 或随脚本主循环 |
| 差速目标层 | `base_rps`、`angle_diff_rps` | `pwm1_duty_rps/pwm2_duty_rps` | 与角度控制同周期 |
| 速度闭环层 | 编码器速度、目标 RPS | 实际 PWM | 现有 `3ms` |

控制链建议如下：

```text
陀螺仪原始值 -> 零偏修正 -> 角速度 dps -> yaw 积分
target_angle - yaw -> 角度 PD -> 左右轮目标 RPS
左右轮目标 RPS -> 现有速度环 -> PWM
```

## 5. 拟新增接口

| 文件 | 内容 |
|---|---|
| `example/inc/angle_loop.hpp` | 对外接口、参数声明 |
| `example/src/angle_loop.cpp` | IMU 选择、零偏标定、yaw 积分、角度 PD |

建议接口：

```cpp
bool angle_loop_init();
void angle_loop_update_imu();
void angle_loop_reset_zero();
void angle_loop_start(float target_deg, int base_rps, int max_diff_rps, int timeout_ms);
void angle_loop_update_control();
bool angle_loop_is_busy();
bool angle_loop_is_ready();
void angle_loop_cancel();
```

## 6. 拟新增可调参数

| 参数 | 初值建议 | 含义 |
|---|---:|---|
| `angle_gyro_axis` | `2` | 使用哪个轴做 yaw，`0=gx`、`1=gy`、`2=gz` |
| `angle_gyro_sign` | `1` | 角速度方向，实测反了就改成 `-1` |
| `angle_gyro_scale` | 待标定 | 原始角速度转 `deg/s` 的比例 |
| `angle_gyro_bias` | 自动标定 | 静止时角速度零偏 |
| `angle_P` | `0.18` | 角度误差到差速 RPS 的比例 |
| `angle_D` | `0.02` | 角速度阻尼，防止转过头 |
| `angle_max_diff_rps` | `6` | 角度环最大左右轮差速 |
| `angle_tolerance_deg` | `3` | 认为到位的角度误差 |
| `angle_stable_ms` | `80` | 误差进入容差后保持多久才结束 |
| `angle_timeout_ms` | `1200` | 单次角度动作最长时间 |

说明：`angle_gyro_scale` 不建议拍脑袋写死，第一版应通过“手转 90 度”实测标定。

## 7. 角度环控制公式

| 变量 | 含义 |
|---|---|
| `yaw_deg` | 从动作开始积分得到的相对角度 |
| `target_deg` | 本次动作目标角度，比如左转 `+45`，右转 `-45` |
| `rate_dps` | 当前 yaw 角速度，单位 `deg/s` |
| `angle_error` | `target_deg - yaw_deg` |
| `diff_rps` | 输出给左右轮的差速目标 |

建议公式：

```text
rate_dps = (raw_axis - gyro_bias) * angle_gyro_scale * angle_gyro_sign
yaw_deg += rate_dps * dt

angle_error = target_deg - yaw_deg
diff_rps = angle_P * angle_error - angle_D * rate_dps
diff_rps = limit(diff_rps, -angle_max_diff_rps, angle_max_diff_rps)

left_target  = base_rps - diff_rps
right_target = base_rps + diff_rps
```

如果不希望倒车，可额外限制：

```text
left_target  >= angle_min_rps
right_target >= angle_min_rps
```

目标板绕行第一版建议 `base_rps = 0` 或 `1~2`，主要靠差速完成转向；如果现场发现原地/低速转不稳，再改成小前进弧线转。

## 8. 与 `drive_by.cpp` 的集成方案

| 当前阶段 | 现状 | 接入角度环后 |
|---|---|---|
| `DB_LEFT_TURN_OUT` | 固定左右轮 RPS + 固定时间 | `angle_loop_start(+turn_out_deg, base_rps, max_diff, timeout)` |
| `DB_LEFT_FORWARD` | 固定直行时间 | 保持不变 |
| `DB_LEFT_TURN_BACK` | 固定左右轮 RPS + 固定时间 | `angle_loop_start(-turn_back_deg, base_rps, max_diff, timeout)` |
| `DB_RIGHT_TURN_OUT` | 固定左右轮 RPS + 固定时间 | `angle_loop_start(-turn_out_deg, base_rps, max_diff, timeout)` |
| `DB_RIGHT_FORWARD` | 固定直行时间 | 保持不变 |
| `DB_RIGHT_TURN_BACK` | 固定左右轮 RPS + 固定时间 | `angle_loop_start(+turn_back_deg, base_rps, max_diff, timeout)` |

新增脚本参数建议：

| 参数 | 初值建议 | 说明 |
|---|---:|---|
| `drive_by_turn_out_deg` | `35` | 第一次向外转角度 |
| `drive_by_turn_back_deg` | `35` | 回正角度 |
| `drive_by_angle_base_rps` | `0` | 角度动作基础前进速度 |
| `drive_by_angle_max_diff_rps` | `4` | 角度动作最大差速 |
| `drive_by_angle_timeout_ms` | `1000` | 单段角度动作超时 |

保留原来的时间脚本作为降级方案：

| 条件 | 动作 |
|---|---|
| IMU 初始化失败 | 自动使用当前固定时间脚本 |
| 角度动作超时 | 停止角度环，进入下一阶段或恢复巡线 |
| K2 停车 | `angle_loop_cancel()`，清零目标速度 |

## 9. 与正常巡线的关系

| 场景 | 是否启用角度环 | 原因 |
|---|---|---|
| 普通巡线 | 暂不启用 | 避免和图像方向环抢控制 |
| 目标板脚本转向 | 启用 | 图像方向环本来已经暂停，角度环可以接管左右轮目标 |
| 目标板脚本直行 | 不启用 | 直行仍用固定目标 RPS 即可 |
| 斑马线停车 | 不启用 | 停车逻辑优先 |

后续二期可以考虑“直道 yaw-rate 阻尼”，用于抑制直道左右摇头，但这会改变正常巡线手感，建议等目标板脚本稳定后再做。

## 10. 初始化与标定流程

| 步骤 | 内容 | 通过标准 |
|---|---|---|
| 1 | 启动后尝试打开 IMU 设备 | 能读到 ID 或读取数据成功 |
| 2 | 小车静止采样 `300~500` 次 | 算出 `gyro_bias` |
| 3 | 选择 yaw 轴 | 原地左/右转时某一轴变化最大 |
| 4 | 手动旋转 90 度标定比例 | `yaw_deg` 接近 `90` |
| 5 | 低速角度动作测试 | 目标角误差小于 `3~5` 度 |

可选 IMU 优先级建议：

| 优先级 | 设备 | 原因 |
|---:|---|---|
| 1 | `ICM42688` | 例程完整，温度也可读，通常性能更好 |
| 2 | `LSM6DSR` | 六轴数据接口简单 |
| 3 | `MPU6050` | 老牌传感器，备用 |

如果你确认硬件只装了一种 IMU，就不要自动探测，直接写死对应设备，减少启动不确定性。

## 11. 测试计划

| 阶段 | 测试方式 | 目标 |
|---|---|---|
| A | 只读 IMU，VOFA/终端看 `gx/gy/gz` | 找出 yaw 轴和符号 |
| B | 静止 10 秒看 `yaw_deg` 漂移 | 判断零偏是否稳定 |
| C | 手转小车 90 度 | 标定 `angle_gyro_scale` |
| D | 架空车轮执行 `angle_loop_start(30)` | 看左右轮方向是否正确 |
| E | 地面低速执行 `30/45/60` 度 | 调 `angle_P/D/max_diff` |
| F | 接入目标板脚本 | 调 `turn_out_deg/turn_back_deg/forward_ms` |

## 12. 风险与规避

| 风险 | 表现 | 规避 |
|---|---|---|
| 角速度比例未知 | 设 45 度实际只转 20 或 90 度 | 用手转 90 度标定 `angle_gyro_scale` |
| 轴向/符号反了 | 左转命令变右转 | 暴露 `angle_gyro_axis/sign` |
| 陀螺仪漂移 | 静止 yaw 慢慢变 | 每次角度动作开始前 `reset_zero()`，只做短动作 |
| 定时器超时 | 终端出现 Timeout | IMU 回调不打印，采样周期不要低于 `5ms` |
| 和方向环抢输出 | 左右轮目标被覆盖 | `angle_loop_is_busy()` 时暂停 `PID_control_test()` |
| K2 停车不彻底 | 停车后角度环继续写目标速度 | K2 停车时调用 `angle_loop_cancel()` |

## 13. 审批点

| 决策 | 推荐选择 | 备注 |
|---|---|---|
| 首版用途 | 只用于目标板脚本转向 | 风险最小 |
| IMU 类型 | 如果不确定，先自动探测；如果确定，写死实际型号 | 写死更稳定 |
| 控制输出 | 输出左右轮目标 RPS | 复用现有速度环 |
| 正常巡线融合 | 暂不做 | 等脚本稳定后再考虑 |
| 标定方式 | 启动静止零偏 + 手转 90 度调比例 | 最可靠 |

## 14. 建议实施顺序

| 顺序 | 内容 |
|---:|---|
| 1 | 新增 `angle_loop.hpp/cpp`，只实现 IMU 读取、零偏、yaw 积分 |
| 2 | 把 `yaw_deg/rate_dps/bias` 加入 VOFA 或终端低频打印 |
| 3 | 完成轴向、符号、比例标定 |
| 4 | 实现 `angle_loop_start/update/cancel`，先架空测试 |
| 5 | 接入 `drive_by` 的转出/回正阶段，保留时间脚本降级 |
| 6 | 实车调 `turn_out_deg/turn_back_deg/forward_ms` |

