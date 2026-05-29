# drive_by.cpp 目标板脚本实现方案

## 1. 目标

当目标板模式开启后，小车在识别到 `have_target == 1` 时，不再沿用当前 `main.cpp` 里直接切换 `g_avoid_state` 的临时逻辑，而是进入独立脚本 `drive_by.cpp`。

脚本目标如下：

| 序号 | 需求 | 实现目标 |
|---|---|---|
| 1 | K0 作为目标板模式开关 | K0 开启后才检测 `have_target`，关闭时完全正常巡线 |
| 2 | `have_target == 1` 后立即停车 | 脚本接管后立刻令 `spd=0`、`error=0`，通过闭环控制减速到 0 |对
| 3 | 连续 5 帧目标板推理 | 每帧使用新的摄像头图像和 `plate_rect` ROI 推理一次，共 5 次 |
| 4 | 投票得到 `item_flag` | 5 次结果中最多的类别作为结果，平票默认直行 |
| 5 | 分支执行 | `item_flag==1` 恢复巡线，`0` 执行左转脚本，`2` 执行右转脚本 |
| 6 | 脚本完成后恢复原状态 | 恢复原来的巡线控制、速度策略、方向环参数 |

## 2. 当前代码现状

| 位置 | 当前作用 | 新方案处理 |
|---|---|---|
| `main/main.cpp` | 定义 `item_flag`、`have_target`、`de_flag`、目标板识别流程 | 保留全局变量，目标板处理逻辑迁移/封装到 `drive_by.cpp` |
| `main/front_ui.cpp` | K0 当前切换 `de_flag` | K0 改成目标板模式开关改名为 `drive_by_enable` |默认关闭
| `main/main.cpp` 主循环 | 当前 `have_target` 后推理并直接切 `g_avoid_state` | 改为调用 `drive_by_update(frame, ncnn)` |
| `example/src/dir_pd.cpp` | `PID_control_test(latest_error)` 方向环 | 脚本执行期间暂停方向环或强制 `error=0` |
| `example/src/dir_circle.cpp` | `test_enc_and_motor_rps()` 速度环闭环 | 脚本执行期间仍可用速度环执行目标速度 |对
| `example/inc/lq_all_demo.hpp` | 暴露控制函数和全局变量 | 新增 `drive_by` 相关函数声明 |

## 3. 建议新增文件

| 文件 | 作用 |
|---|---|
| `example/inc/drive_by.hpp` | 声明脚本接口、状态枚举、参数结构 |
| `example/src/drive_by.cpp` | 实现目标板模式状态机、五帧推理、左/右脚本 |

也可以放到 `main/drive_by.cpp`，但当前项目里控制算法大多在 `example/src`，所以更建议放在 `example/src`，头文件放 `example/inc`。

## 4. 目标状态机

建议使用一个独立状态机，不要在 `main` 里用一堆 `if` 拼脚本。

| 状态名 | 含义 | 进入条件 | 下一状态 |
|---|---|---|---|
| `DB_IDLE` | 空闲，不接管小车 | 默认状态 | `have_target==1` 且 K0 开启时进入 `DB_STOPPING` |
| `DB_STOPPING` | 立即停车，`spd=0`、`error=0` | 目标板触发 | 速度接近 0 或等待固定停车时间后进入 `DB_INFER` |
| `DB_INFER` | 连续 5 帧推理 | 停车完成 | 5 帧完成后投票，进入分支 |
| `DB_GO_STRAIGHT` | 直行结果，恢复巡线 | `item_flag==1` | 直接 `DB_FINISH` |
| `DB_LEFT_TURN_OUT` | 左转固定时间 | `item_flag==0` | `DB_LEFT_FORWARD` |
| `DB_LEFT_FORWARD` | 左绕行固定前进时间 | 左转完成 | `DB_LEFT_TURN_BACK` |
| `DB_LEFT_TURN_BACK` | 往回转固定时间 | 前进完成 | `DB_LEFT_EXIT_FORWARD` |
| `DB_LEFT_EXIT_FORWARD` | 结束前固定前进一段 | 回正完成 | `DB_FINISH` |
| `DB_RIGHT_TURN_OUT` | 右转固定时间 | `item_flag==2` | `DB_RIGHT_FORWARD` |
| `DB_RIGHT_FORWARD` | 右绕行固定前进时间 | 右转完成 | `DB_RIGHT_TURN_BACK` |
| `DB_RIGHT_TURN_BACK` | 往回转固定时间 | 前进完成 | `DB_RIGHT_EXIT_FORWARD` |
| `DB_RIGHT_EXIT_FORWARD` | 结束前固定前进一段 | 回正完成 | `DB_FINISH` |
| `DB_FINISH` | 恢复巡线状态 | 脚本完成 | `DB_IDLE` |

## 5. K0 开关方案

当前 K0 在 `front_ui.cpp` 里切换 `de_flag`。

建议如下：

| 变量 | 建议 |
|---|---|
| `de_flag` | 可以继续复用，但建议后续改名为 `drive_by_enable`，语义更清楚 |可以
| K0 短按 | 切换目标板脚本模式开/关 |
| TFT18 显示 | 增加一行 `TARGET: ON/OFF`，方便赛前确认模式 |
| 模式关闭时 | 不检测 `have_target`，不调用 `drive_by_update()` |

## 6. 脚本接管原则

| 控制源 | 普通巡线 | 脚本执行中 |
|---|---|---|
| 速度环 `test_enc_and_motor_rps()` | 正常运行 | 继续运行，用来闭环跟踪脚本给出的左右轮目标速度 |
| 方向环 `PID_control_test(latest_error)` | 正常运行 | 暂停，或强制 `latest_error=0`，避免和脚本抢左右轮速度 |
| 图像巡线 `process_image()` 等 | 正常运行 | 可以继续跑，但输出不接管车轮 |对
| 目标板检测 | 仅 K0 开启时运行 | 脚本 busy 时不重复触发 |

建议增加：

| 变量 | 类型 | 含义 |
|---|---|---|
| `bool drive_by_enable` | 全局 | K0 开关 |
| `bool drive_by_busy` | 全局 | 脚本是否正在接管 |
| `bool drive_by_seen_lock` | 全局 | 防止同一个目标板重复触发 |
| `DriveByState drive_by_state` | 枚举 | 当前脚本状态 |

## 7. 停车阶段实现

进入 `DB_STOPPING` 后，立即执行：

| 变量/函数 | 操作 |
|---|---|
| `set_speed_of_motor1_rps` | 置 0 |
| `set_speed_of_motor2_rps` | 置 0 |
| `pwm1_duty_rps` | 置 0 |
| `pwm2_duty_rps` | 置 0 |
| `latest_error` | 置 0 |
| `PID_control_test(0)` | 可调用一次，确保方向环目标归零 |
| `close_circle_control(...)` 或速度环自然执行 | 让电机闭环减速到 0 |
对，要闭环减速，速度会很快的

停车完成判断建议二选一：

| 方案 | 判断条件 | 优缺点 |
|---|---|---|
| 固定时间 | 等待 `drive_by_stop_ms`，例如 300ms | 简单稳定，不依赖编码器瞬时值 |要这个



## 8. 连续 5 帧推理方案

不能在同一帧连续推理 5 次，而是每次主循环进来拿到新 `frame` 后推理一次。

| 步骤 | 操作 |
|---|---|
| 1 | 进入 `DB_INFER` 时清空计数：`vote[3]={0,0,0}`，`infer_count=0` |
| 2 | 每一帧调用 `detectRedPlate(frame)` 更新 `have_target` 和 `plate_rect` |
| 3 | 如果 `have_target==1`，裁剪 `plate_rect` 得到 ROI |
| 4 | `resize(roi, roi, cv::Size(SAVE_SIZE, SAVE_SIZE))` |
| 5 | 调用当前 main 里已加载好的 `ncnn.Infer(roi)` |
| 6 | 将结果映射到 `item_flag` 并计入 `vote[]` |
| 7 | `infer_count == 5` 后投票 |

建议映射关系：

| 推理字符串 | `item_flag` | 动作 |
|---|---:|---|
| `"weapon"` | 0 | 左转脚本 |
| `"vehicle"` | 1 | 恢复巡线直行 |
| `"supplies"` | 2 | 右转脚本 |

注意：当前旧代码里 `supplies -> 右`、`weapon -> 左`、其他 -> 直行，所以这里沿用这个语义。

投票规则：

| 情况 | 结果 |
|---|---|
| 单一最大票 | 最大票类别 |
| 平票 | `item_flag = 1`，默认直行 |
| 5 帧中有某帧没检测到目标板 | 建议跳过该帧，不计入 5 次有效推理；如果超时太久则默认直行 |

## 9. 左/右脚本参数

你后续会补具体速度和时间，所以先设计成可配置变量。

| 参数名 | 默认占位 | 含义 |
|---|---:|---|
| `drive_by_turn_speed_rps` | 8 | 原地/差速转向速度 |
| `drive_by_forward_speed_rps` | 10 | 固定前进速度 |
| `drive_by_turn_out_ms` | 500 | 第一次向外转的时间 |
| `drive_by_forward_ms` | 700 | 绕行前进时间 |
| `drive_by_turn_back_ms` | 500 | 往回转的时间 |
| `drive_by_exit_forward_ms` | 500 | 回正后前进时间 |
| `drive_by_stop_ms` | 300 | 进入脚本后的停车等待时间 |
| `drive_by_infer_timeout_ms` | 1000 | 5 帧推理最长等待时间 |

左转脚本建议输出：

| 阶段 | 左轮目标 | 右轮目标 | 说明 |
|---|---:|---:|---|
| `LEFT_TURN_OUT` | `-turn_speed` | `turn_speed` | 向左转固定时间 |
| `LEFT_FORWARD` | `forward_speed` | `forward_speed` | 前进固定时间 |
| `LEFT_TURN_BACK` | `turn_speed` | `-turn_speed` | 往回转固定时间 |
| `LEFT_EXIT_FORWARD` | `forward_speed` | `forward_speed` | 前进后恢复巡线 |

右转脚本建议输出：

| 阶段 | 左轮目标 | 右轮目标 | 说明 |
|---|---:|---:|---|
| `RIGHT_TURN_OUT` | `turn_speed` | `-turn_speed` | 向右转固定时间 |
| `RIGHT_FORWARD` | `forward_speed` | `forward_speed` | 前进固定时间 |
| `RIGHT_TURN_BACK` | `-turn_speed` | `turn_speed` | 往回转固定时间 |
| `RIGHT_EXIT_FORWARD` | `forward_speed` | `forward_speed` | 前进后恢复巡线 |

这个后面我会传图给你，你自己分析给值

## 10. 与主循环的集成点

主循环中当前逻辑大致是：

| 当前流程 | 新流程 |
|---|---|
| 拿到摄像头 `frame` | 保持 |
| K0 开启时 `detectRedPlate(frame)` | 移到 `drive_by_update()` 内部或由它统一管理 |
| `have_target` 后直接推理并切 `g_avoid_state` | 改为 `drive_by_update(frame, ncnn)` |
| 后续正常巡线处理 | 如果 `drive_by_busy==false` 才允许方向环/巡线输出接管 |

建议主循环伪代码：

```cpp
front_ui_poll();
vofa_recv_cmd();

cv::Mat frame = cam.get_frame_raw();
cv::flip(frame, frame, -1);

if (drive_by_enable) {
    drive_by_update(frame, ncnn);
}

if (!drive_by_busy) {
    // 原来的巡线、环岛、普通避障逻辑
    process_image();
    find_corners();
    latest_error = ...
}
```

## 11. 与定时器的集成点

速度环建议一直运行，因为脚本也需要闭环控制左右轮目标速度。

方向环需要在脚本执行中暂停。

```cpp
speed_timer.set_seconds_ms(3, []() {
    if (front_ui_is_running()) {
        test_enc_and_motor_rps();
    } else {
        front_ui_hold_stop();
    }
});

dir_timer.set_seconds_ms(8, []() {
    if (front_ui_is_running() && !drive_by_busy) {
        PID_control_test(latest_error);
    } else if (drive_by_busy) {
        PID_control_test(0);
    } else {
        front_ui_hold_stop();
    }
});
```对

## 12. 防重复触发

问题：目标板进入视野后，`have_target` 可能连续很多帧为 1。如果脚本完成后立刻恢复巡线，可能又马上二次进入脚本。

建议规则：

| 变量 | 规则 |
|---|---|
| `drive_by_seen_lock` | 进入脚本时置 1 |
| 解锁条件 | `have_target == 0` 连续若干帧，或脚本完成后等待固定冷却时间 |
| 推荐初值 | `false` |
| 推荐冷却 | `3000ms` |
可以
触发条件建议：

```cpp
if (drive_by_enable && !drive_by_busy && !drive_by_seen_lock && have_target) {
    drive_by_start();
}
```

## 13. 需要从旧逻辑移除或替换的点

当前 `main.cpp` 中这段旧目标板逻辑需要替换成新脚本入口：

| 旧逻辑 | 新逻辑 |
|---|---|
| `target_count==2` 控制检测频率 | `drive_by_update()` 内部管理检测和 5 帧推理 |
不对！，我们要每两帧检查一次防止占用太多cpu，检查到了有目标停车慢慢推理
| `tar_count==TAR`，其中 `TAR=2` | 改成 `DRIVE_BY_INFER_FRAMES=5` |
| `select_max_target(tar)` 平票优先 2 | 新投票函数平票返回 1 |
| 直接设置 `g_avoid_state=AV_GO_LEFT/RIGHT` | 进入 `drive_by` 左/右固定速度时间脚本 |
| `pixel_per_meter/=2` | 先不在脚本中使用，除非后续你确认还需要 |

## 14. 建议对外接口

`drive_by.hpp` 建议接口：

```cpp
void drive_by_init();
void drive_by_update(cv::Mat& frame, LQ_NCNN& ncnn);
bool drive_by_is_busy();
void drive_by_set_enable(bool enable);
bool drive_by_is_enabled();
```

如果不想暴露 `LQ_NCNN` 类型，也可以写成：

```cpp
typedef std::string (*DriveByInferFn)(cv::Mat& roi);
void drive_by_update(cv::Mat& frame, DriveByInferFn infer_fn);
```

但当前 main 里已经有 `LQ_NCNN ncnn` 对象，直接传引用最简单。

## 15. 实施步骤

| 步骤 | 文件 | 内容 |
|---|---|---|
| 1 | `example/inc/drive_by.hpp` | 新增状态机接口和参数声明 |
| 2 | `example/src/drive_by.cpp` | 实现 `drive_by_update()`、五帧投票、左右脚本 |
| 3 | `example/inc/lq_all_demo.hpp` | include 或声明 drive_by 接口 |
| 4 | `main/front_ui.cpp` | K0 改为切换 `drive_by_enable`，屏幕显示 ON/OFF |
| 5 | `main/main.cpp` | 删除旧 `have_target` 分支，改为调用 `drive_by_update(frame, ncnn)` |
| 6 | `main/main.cpp` 定时器 | 脚本 busy 时暂停方向环，速度环继续闭环 |
| 7 | 实车调参 | 调 `turn_speed/forward_speed/turn_ms/forward_ms` |

## 16. 待你补充的参数

请后续在这里填：

| 参数 | 你的备注 |
|---|---|
| 左转第一次转向速度 |  |
| 左转第一次转向时间 |  |
| 左转前进速度 |  |
| 左转前进时间 |  |
| 左转回正速度 |  |
| 左转回正时间 |  |
| 左转结束前进时间 |  |
| 右转第一次转向速度 |  |
| 右转第一次转向时间 |  |
| 右转前进速度 |  |
| 右转前进时间 |  |
| 右转回正速度 |  |
| 右转回正时间 |  |
| 右转结束前进时间 |  |
| 停车等待时间 |  |
| 推理超时时间 |  |
| 脚本完成冷却时间 |  |
这个我后面给你图形你自己分析
## 17. 风险点

| 风险 | 说明 | 建议 |
|---|---|---|
| 脚本和方向环抢输出 | 方向环会改左右轮目标速度 | `drive_by_busy` 时暂停方向环 |同意
| `have_target` 连续为 1 | 会重复触发脚本 | 增加 `drive_by_seen_lock` 和冷却 |同意
| 目标板 ROI 越界 | `plate_rect` 可能超出图像范围 | 推理前必须检查 ROI 合法 |同意
| 5 帧推理期间目标丢失 | 可能卡住等待 | 加 `infer_timeout_ms`，超时默认直行 |同意
| 固定速度时间受地面影响 | 转角和距离会变 | 后续实车调参，必要时加入编码器辅助 |后面给你图像你自己分析一般要多少
| 脚本恢复后参数不一致 | 目标速度可能被脚本改过 | 进入脚本前保存原速度，结束时恢复 |同意，参数PD这些也要保留

## 18. 当前建议结论

第一版先实现最小稳定版本：

| 决策 | 采用 |
|---|---|
| K0 专用目标板模式开关 | 是 |
| `have_target` 触发脚本 | 是 |
| 进入脚本先停车 | 是 |
| 连续 5 帧推理 | 是 |
| 平票默认直行 | 是 |
| 固定速度 x 时间脚本 | 是 |
| 脚本执行时暂停方向环 | 是 |
| 脚本完成后恢复原巡线 | 是 |

ps：多打点注释！
