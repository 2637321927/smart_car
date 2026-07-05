# 角速度环异步读取说明

## 1. 为什么要改

原来的角速度环在 `dir_timer` 里直接读取 MPU6050：

```text
8ms 方向环 -> gyro_yaw_rate_control_update()
           -> gyro_yaw_rate_control_get_gyro_z_dps()
           -> ioctl 读取 MPU6050
```

问题是 `ioctl` 属于同步硬件访问。如果某一次 I2C/驱动读取超过 8ms，`dir_timer` 只能等它结束，方向环就会超时。`lq_timer` 当前只会在回调结束后打印 `Timeout`，不会打断正在执行的回调，所以同步读取会直接影响小车响应。

新的设计把“硬件读取”和“方向控制”拆开：

```text
lq_timer(5ms) -> gyro_yaw_rate_control_service()
              -> 启动/检查 MPU6050 read worker
              -> worker 更新 gyro cache

8ms 方向环 -> gyro_yaw_rate_control_update()
           -> 只读取 gyro cache
```

这样即使某一次 MPU6050 读取卡住，方向环也不会等待它。

## 2. 新架构

| 模块 | 作用 |
|---|---|
| `gyro_watchdog_timer` | 在 `main.cpp` 中新增，每 5ms 调用一次 `gyro_yaw_rate_control_service()` |
| `gyro_yaw_rate_control_service()` | 轻量服务函数，只检查超时和启动 worker，不直接读硬件 |
| read worker | 独立线程，负责一次 MPU6050 读取，完成后更新缓存 |
| gyro cache | 保存最近一次有效的 `gyro_z_lpf`、更新时间、超时计数和 worker 数量 |
| 角速度环 | 继续使用 `gyro_yaw_rate_control_update()`，但实际角速度来自缓存 |

关键原则：

```text
方向环永远不直接读 MPU6050。
方向环只读最近一次有效缓存。
超时 worker 的结果即使晚回来，也不能覆盖缓存。
```

## 3. 超时废弃策略

每个 worker 启动时都会带一个 `generation_id`。

| 情况 | 处理 |
|---|---|
| 读取在 20ms 内完成 | 如果 `generation_id` 仍是当前代，就更新缓存 |
| 读取超过 20ms | `service()` 递增 `generation_id`，旧 worker 被逻辑作废 |
| 旧 worker 后来返回 | 因为 `generation_id` 不匹配，直接丢弃结果 |
| worker 数量达到 3 | 暂停新建 worker 50ms，避免线程无限增长 |

这里没有强杀线程。原因是线程可能正卡在内核 `ioctl` 里，C++ 层强杀不安全，可能留下锁或设备状态问题。现在采用的是“逻辑废弃”：旧线程可以自己晚点结束，但它的结果不再被使用。

## 4. 控制策略

运行中不切回视觉 PD。

当陀螺仪缓存新鲜时：

```text
正常角速度 PI
正常更新 I 积分
```

当陀螺仪缓存超过 30ms 没更新时：

```text
继续使用最后一次有效 gyro_z_lpf
不切回视觉 PD
冻结 I 积分
P 项仍然根据当前目标角速度和最后 gyro_z 计算
```

冻结积分是为了防止这种情况：

```text
gyro_z 已经过旧
角速度误差不可信
如果继续积分，I 项可能越积越偏
```

## 5. 默认参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `gyro_watchdog_period_ms` | 5ms | `lq_timer` 调用 service 的周期 |
| `gyro_read_timeout_ms` | 20ms | 单次读取超过这个时间就作废 |
| `gyro_stale_freeze_ms` | 30ms | 缓存超过这个年龄后冻结 I 积分 |
| `gyro_max_worker_count` | 3 | 最多允许 3 个未退出 worker |
| `gyro_restart_pause_ms` | 50ms | worker 太多时暂停新建的时间 |

这些参数目前写在 `example/src/gyro_yaw_rate_control.cpp` 的常量区，属于安全策略，不是 PID 增益。

## 6. 调试方法

角速度环打印中新增了这些字段：

| 字段 | 意义 |
|---|---|
| `age` | 最近一次有效陀螺仪数据的年龄，单位 ms |
| `timeout` | MPU6050 读取超过 20ms 后被废弃的次数 |
| `workers` | 当前还没有退出的读取 worker 数量 |
| `I_freeze` | 1 表示陀螺仪数据不新鲜，I 积分被冻结 |

正常情况建议观察：

```text
age 通常应小于 30ms
timeout 偶尔增加可以接受
workers 大多数时候应为 0 或 1
I_freeze 大多数时候应为 0
```

如果出现：

```text
workers 长时间等于 3
timeout 持续快速增加
age 长时间远大于 30ms
```

说明 I2C 或 MPU6050 驱动可能整体卡住了。此时控制环不会被直接阻塞，但反馈已经不可靠，需要检查硬件连接、驱动、供电或 I2C 总线。

## 7. 风险边界

这个方案解决的是：

```text
某一次 MPU6050 读取超时，不拖死方向环。
```

它不能完全解决：

```text
I2C 总线永久卡死。
MPU6050 驱动永久阻塞。
内核层设备异常导致所有 worker 都无法返回。
```

如果遇到永久卡死，当前保护会限制 worker 数量，防止线程爆炸；角速度环会继续使用最后一次有效值并冻结积分。更彻底的隔离方案是把陀螺仪读取放到独立进程中，进程卡死后可以由主程序杀掉并重启，但那会比当前版本改动更大。
