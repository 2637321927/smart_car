# MPU6050 角速度环设计说明

本文档对应当前工程 `D:\Smart_Car\smart_car` 里新增的 MPU6050 角速度环 demo。

它的目的不是先追求最快，而是先把你心里的控制链条讲清楚，并把代码实现、调试方法、审查重点写明白，方便你确认方案是否符合你们的车。

## 1. 先用一句话抓住核心

你前面理解的那句话是对的：

```text
视觉给出 error
    -> 根据 error 大小给出“希望车身旋转多快”
    -> MPU6050 测出“车身实际旋转多快”
    -> 闭环调整左右轮差速，让实际旋转速度跟上目标旋转速度
    -> 车头逐渐摆正，视觉 error 逐渐消除
```

这套东西本质上是一个串级控制：

```text
视觉外环：error -> 目标角速度 target_yaw_rate
角速度内环：target_yaw_rate - gyro_z -> 左右轮差速 turn_rps
电机速度环：左右轮目标 RPS -> PWM
```

所以现在不是让视觉 error 直接决定轮子差多少，而是让视觉 error 先变成一个更物理、更可反馈的量：车身绕 Z 轴的角速度。

## 2. 旧方案和新方案的区别

原来的方向控制大致是：

```text
latest_error
    -> calculate_diffrential(error, 0)
    -> pwm1_duty_rps = base_speed + diffrential
    -> pwm2_duty_rps = base_speed - diffrential
    -> 速度环
    -> 电机 PWM
```

也就是说，原方案只根据视觉误差决定左右轮速度差。它没有直接确认车身到底有没有按预期旋转。

新增角速度环后变成：

```text
latest_error
    -> 外环 PD
    -> target_yaw_rate_dps
    -> 和 MPU6050 实际 gyro_z_lpf 比较
    -> 内环 PI
    -> turn_rps
    -> pwm1_duty_rps = base_speed + turn_rps
    -> pwm2_duty_rps = base_speed - turn_rps
    -> 原有速度环
    -> 电机 PWM
```

新方案多了一层反馈：MPU6050 告诉我们车身实际有没有转起来、转得够不够、转得过不过。

这就是角速度环最大的价值。

## 3. 角速度环和角度环不是一回事

很多人容易把这两个词混在一起。

角速度环控制的是：

```text
我希望车现在以 80 deg/s 右转
实际 MPU6050 测到 50 deg/s
差了 30 deg/s
那就继续加大左右轮差速
```

角度环控制的是：

```text
我希望车头角度是 20 deg
当前车头角度是 12 deg
差了 8 deg
那就让车继续转
```

MPU6050 的陀螺仪原始输出不是角度，而是角速度。也就是 `deg/s`。

如果你想得到角度，需要积分：

```text
yaw_angle += gyro_z_dps * dt
```

但积分会累积零偏误差。MPU6050 只要静止时还有一点点零偏，长时间积分出来的角度就会慢慢漂。

对循迹车来说，视觉本身已经在持续告诉你“线在左边还是右边”，因此更适合这样分工：

```text
视觉负责判断位置误差和方向趋势
陀螺仪负责让车身旋转速度更稳定、更可控
```

所以当前 demo 先做角速度环，不做完整 yaw 角度环。这更稳，也更贴近你现在的循迹需求。

## 4. MPU6050 在这里到底提供了什么

当前代码只用 MPU6050 的 Z 轴陀螺仪 `gz`。

小车在地面上转向时，主要是绕竖直方向转，也就是 yaw 方向。正常安装时，这个方向对应 MPU6050 的 Z 轴。

底层读取接口在：

```text
libraries/app/gyro/lq_i2c_mpu6050.*
```

新增控制模块里通过下面的函数读三轴角速度：

```cpp
mpu6050_device().get_mpu6050_ang(&gx, &gy, &gz)
```

然后只使用 `gz`。

### 4.1 原始值为什么要换算

底层驱动把 MPU6050 陀螺仪量程设置成了 `+/-2000 dps`。

在这个量程下，MPU6050 的比例是：

```text
16.4 LSB/(deg/s)
```

所以原始值换成角速度的公式是：

```text
gyro_z_dps = (gz_raw - gz_offset_raw) / 16.4
```

其中：

```text
gz_raw        MPU6050 当前读到的原始 Z 轴角速度
gz_offset_raw 小车静止时标定出来的零偏
gyro_z_dps    换算后的实际角速度，单位 deg/s
```

### 4.2 为什么必须静止标定零偏

理想情况下，小车静止时 `gz_raw` 应该是 0。

实际 MPU6050 会有零偏，比如静止时读出来可能是 30、-80、120 等等。这个值不是车真的在转，而是传感器偏差。

所以初始化时要让车保持静止，连续采样 500 次，求平均值作为零偏：

```text
gz_offset_raw = 静止时 gz_raw 的平均值
```

之后每次读取都扣掉这个平均值。

如果初始化时车在动，程序会把“真实转动”也当成零偏扣掉，后面控制就会明显歪。

### 4.3 为什么要低通滤波

陀螺仪会有噪声。直接拿噪声进 PI 控制，电机会抖。

当前 demo 用一阶低通：

```text
gyro_z_lpf = 0.75 * 上一次滤波值 + 0.25 * 当前测量值
```

它的含义是：

```text
0.75 越大：越稳，但反应越慢
0.25 越大：越灵敏，但更容易抖
```

送进角速度内环的是 `gyro_z_lpf`，不是完全未滤波的 `gyro_z_dps`。

## 5. 当前代码里的控制结构

新增文件：

```text
example/inc/gyro_yaw_rate_control.hpp
example/src/gyro_yaw_rate_control.cpp
```

主要接入点：

```text
main/main.cpp
example/src/dir_pd.cpp
main/front_ui.cpp
```

### 5.1 初始化位置

在 `main/main.cpp` 中，VOFA 初始化后调用：

```cpp
gyro_yaw_rate_control_init();
```

它会做三件事：

```text
1. 重置历史误差和积分
2. 读取 MPU6050 ID，期望值是 0x68
3. 车静止时采样 gz，标定零偏
```

如果 ID 不对，或者采样失败太多，程序不会强行使用角速度环，而是回退到原来的视觉 PD。

### 5.2 方向环接入位置

在 `example/src/dir_pd.cpp` 的 `PID_control_test(float error)` 中，原来直接算差速：

```cpp
diffrential = calculate_diffrential(error, 0);
```

现在改成了模式切换：

```cpp
if (gyro_yaw_rate_feedback_enabled && gyro_yaw_rate_control_is_ready()) {
    diffrential = gyro_yaw_rate_control_update(error);
    gyro_yaw_rate_control_print_debug(50);
} else {
    diffrential = calculate_diffrential(error, 0);
}
```

也就是说：

```text
#gyro=0; 或 MPU6050 不可用 -> 原来的视觉 PD
#gyro=1; 且 MPU6050 ready -> 新的角速度环
```

这个设计是为了安全。默认不改变原车行为。

### 5.3 发车和停车时为什么要 reset

在 `main/front_ui.cpp` 的 `start_car()` 和 `stop_car()` 中调用：

```cpp
gyro_yaw_rate_control_reset();
```

原因是角速度环里面有：

```text
上一次视觉误差
内环积分量
滤波历史值
```

如果停车前车正在修方向，积分项里可能还残留一个转向趋势。下一次发车如果不清空，就可能一启动先莫名其妙打一把方向。

所以发车、停车、目标板脚本接管时都清空，这是合理的。

## 6. 外环：视觉 error 如何变成目标角速度

外环在 `calc_target_yaw_rate(float vision_error)` 中。

当前公式是：

```text
target_yaw_rate = gyro_outer_kp * vision_error
                + gyro_outer_kd * (vision_error - last_vision_error)
```

含义：

```text
P 项：偏得越多，希望转得越快
D 项：误差变化越快，提前给出一点旋转趋势
```

比如当前参数：

```text
gyro_outer_kp = 1.2
gyro_outer_kd = 0.25
```

如果视觉误差是 50，且上一帧误差也是 50，那么：

```text
target_yaw_rate = 1.2 * 50 + 0.25 * 0 = 60 deg/s
```

也就是说，视觉认为偏差为 50 时，我们希望车身大约以 `60 deg/s` 的角速度往修正方向转。

如果视觉误差突然从 20 变成 50：

```text
d_error = 50 - 20 = 30
target_yaw_rate = 1.2 * 50 + 0.25 * 30 = 67.5 deg/s
```

D 项会让目标角速度稍微更积极。

注意：这里的 D 项没有除以 `dt`。这不是数学上最标准的微分写法，但在嵌入式调车里常见，因为控制周期固定，`gyro_outer_kd` 已经把周期影响吸收进去了。只要你按当前 8ms 周期调参，就可以用。

### 6.1 外环限幅

视觉误差先限到：

```text
-100 到 100
```

目标角速度再限到：

```text
-gyro_target_max_dps 到 gyro_target_max_dps
```

当前默认：

```text
gyro_target_max_dps = 160.0
```

这表示无论视觉 error 多大，外环最多只要求车以 `160 deg/s` 左右的角速度转。

限幅的意义是防止丢线、误检时给出过激目标。

## 7. 内环：目标角速度如何变成左右轮差速

内环在 `gyro_yaw_rate_control_update(float vision_error)` 中。

先得到：

```text
target_yaw_rate = 外环输出
actual_yaw_rate = MPU6050 测到的 gyro_z_lpf
```

然后算角速度误差：

```text
rate_error = target_yaw_rate - actual_yaw_rate
```

再用 PI 算差速：

```text
rate_integral += rate_error * 0.008
turn_rps = gyro_inner_kp * rate_error
         + gyro_inner_ki * rate_integral
```

当前默认：

```text
gyro_inner_kp = 0.045
gyro_inner_ki = 0.015
```

举例：

```text
目标角速度 target_yaw_rate = 100 deg/s
实际角速度 actual_yaw_rate = 40 deg/s
角速度误差 rate_error = 60 deg/s
```

只看 P 项时：

```text
turn_rps = 0.045 * 60 = 2.7 RPS
```

这个 `turn_rps` 最后会叠加到左右轮目标速度上：

```text
pwm1_duty_rps = base_speed + turn_rps
pwm2_duty_rps = base_speed - turn_rps
```

所以 `turn_rps` 不是 PWM，而是“左右轮目标速度差的一半”，单位是 RPS。

### 7.1 内环 P 项负责什么

内环 P 项负责快速响应。

```text
目标角速度比实际角速度大很多 -> 加大差速
目标角速度和实际角速度接近 -> 减小差速
实际角速度超过目标角速度 -> 反向修正
```

P 太小：

```text
车转得慢，视觉 error 消得慢，弯道跟不上
```

P 太大：

```text
左右来回抖，甚至一会儿过转一会儿反打
```

### 7.2 内环 I 项负责什么

I 项负责消除长期误差。

比如你一直要求 `80 deg/s`，但因为电池电压、地面摩擦、电机差异，实际只能到 `65 deg/s`。这时 P 项会给一个固定差速，但可能还不够。

积分项会把这 15 deg/s 的长期误差慢慢累积起来，让输出继续加一点，直到实际角速度接近目标。

但 I 项很容易带来副作用：

```text
积分太大 -> 反应慢半拍、过弯后还继续打方向、停车再发车有残留
```

所以新手调车建议先：

```text
#gII=0;
```

先只把 P 调到能跟随，再一点点加 I。

### 7.3 为什么要做积分限幅

代码里有积分限幅：

```text
integral_limit = gyro_turn_max_rps / abs(gyro_inner_ki)
```

它的目的是防止积分一直累积，把输出顶死。

这种问题叫积分饱和。表现通常是车已经过了弯，但控制器还残留很大的转向输出，导致来回甩。

## 8. 符号问题是实车调试第一关

闭环系统最怕符号错。

符号错时，控制器以为自己在修正，实际却在把误差越推越大。

当前 demo 留了两个符号修正参数：

```text
gyro_z_sign    修正 MPU6050 读数正负
gyro_turn_sign 修正左右轮输出差速方向
```

### 8.1 先确认 gyro_z_sign

让车静止启动，MPU6050 ready 后，手动把车头向右转一点，看调试输出里的 `gyro` 正负。

如果你定义“右转应该是正”，但输出是负数，就发送：

```text
#gSign=-1;
```

如果输出方向已经符合你的定义，就保持：

```text
#gSign=1;
```

注意，这里的“右转为正”不是世界标准，只是我们调车时要在整个控制链里保持一致。

### 8.2 再确认 gyro_turn_sign

把车轮架空，打开角速度环：

```text
#gyro=1;
```

给一个视觉误差，让它产生转向需求，观察左右轮差速方向。

如果角速度环一启用，车就往误差更大的方向修，说明输出方向反了，发送：

```text
#tSign=-1;
```

如果方向正确，就保持：

```text
#tSign=1;
```

调试顺序一定是先传感器符号，再输出符号。

## 9. VOFA 在线调参命令

当前在 `main/main.cpp` 中加入了这些命令：

| 命令 | 作用 | 建议 |
| --- | --- | --- |
| `#gyro=0;` | 请求使用原视觉 PD | 默认安全模式 |
| `#gyro=1;` | 请求使用角速度反馈 | 只有 MPU6050 ready 时才会进入 `GYRO_RATE` |
| `#gOP=1.2;` | 设置视觉外环 P | error 越大，目标角速度越大 |
| `#gOD=0.25;` | 设置视觉外环 D | 抑制滞后，但太大会抖 |
| `#gIP=0.045;` | 设置角速度内环 P | 先调它 |
| `#gII=0.015;` | 设置角速度内环 I | 新手阶段可先设 0 |
| `#gSign=-1;` | 反转 MPU6050 Z 轴符号 | 手动转车验证 |
| `#tSign=-1;` | 反转差速输出符号 | 闭环越修越偏时改 |

当前 `gyro_target_max_dps` 和 `gyro_turn_max_rps` 是代码里的可调全局变量，但暂时没有加 VOFA 命令。如果你希望实车调试更方便，后续可以加：

```text
#gMax=160;
#tMax=15;
```

## 10. 调试输出怎么看

角速度环启用后，约每 400ms 打印一次：

```text
[DIR] mode=... reason=... gyro_request=... gyro_ready=... err=... base_rps=... diff_rps=... target1=... target2=...
[GYRO] target_loop err=... target_dps=... gyro_dps=... rate_err=... turn_rps=... ready=...
```

`[DIR]` 先看，它告诉你方向环当前到底走哪条路：

| mode | 含义 |
| --- | --- |
| `VISUAL_PD` | `#gyro=0;`，请求使用原来的视觉 PD |
| `GYRO_RATE` | `#gyro=1;` 且 MPU6050 ready，正在使用角速度反馈 |
| `VISUAL_PD_FALLBACK` | `#gyro=1;` 但 MPU6050 没 ready，自动回退旧视觉 PD |

`reason` 用来说明为什么是这个状态：

| reason | 含义 |
| --- | --- |
| `gyro_off` | 用户没有请求角速度反馈 |
| `mpu6050_feedback` | 用户请求角速度反馈，且 MPU6050 可用 |
| `mpu6050_not_ready` | 用户请求角速度反馈，但 MPU6050 初始化或标定没成功 |

`[GYRO]` 再看，它只在角速度环真正运行时打印内部量。

每个 `[GYRO]` 字段含义：

| 字段 | 含义 | 怎么判断 |
| --- | --- | --- |
| `err` | 当前视觉误差 | 应该和视觉检测方向一致 |
| `target` | 外环给出的目标角速度，单位 deg/s | error 大时绝对值变大 |
| `gyro` | MPU6050 测到的实际角速度，单位 deg/s | 手动转车时应该有明显变化 |
| `rate_err` | 目标角速度减实际角速度 | 内环真正控制的误差 |
| `turn` | 输出给左右轮的差速修正，单位 RPS | 正负决定左右轮谁快 |
| `ready` | MPU6050 是否初始化成功 | 必须是 1 才能闭环 |

调车时不要只看车跑得好不好。刚开始一定要盯这些中间量，因为它们能直接暴露问题在哪一层。

## 11. 推荐调试流程

### 11.1 第一步：确认旧控制没被破坏

先关闭角速度环：

```text
#gyro=0;
```

然后按原来的方式跑车。

如果这一步不正常，问题不在 MPU6050 角速度环，而应该先回头看原方向环、速度环、视觉 error。

### 11.2 第二步：确认 MPU6050 初始化

启动程序时车必须静止。

正常应看到类似：

```text
[GYRO] Keep car still, calibrating MPU6050 gz offset...
[GYRO] MPU6050 ready: gz_offset_raw=..., scale=16.4 LSB/(deg/s).
```

如果看到：

```text
Fallback to old visual PD
```

说明 MPU6050 没 ready。优先检查：

```text
1. /dev/lq_i2c_mpu6050 设备节点是否存在
2. 驱动是否加载
3. MPU6050 接线和地址是否正确
4. get_mpu6050_id() 是否能读到 0x68
```

### 11.3 第三步：确认陀螺仪符号

手动把车头向你定义的正方向转，观察 `gyro`。

如果相反：

```text
#gSign=-1;
```

再试一次。

### 11.4 第四步：架空车轮确认输出方向

先用比较保守的参数：

```text
#gyro=1;
#gII=0;
#gIP=0.03;
#gOP=1.0;
#gOD=0;
```

让车轮离地，观察差速方向。

如果闭环一介入就越修越偏：

```text
#tSign=-1;
```

### 11.5 第五步：调内环 P

保持：

```text
#gII=0;
```

逐渐增大：

```text
#gIP=0.03;
#gIP=0.04;
#gIP=0.05;
```

观察：

```text
target 和 gyro 是否能靠近
turn 是否经常顶到最大
车身是否开始左右抖
```

如果 `gyro` 永远跟不上 `target`，可以略增大 `gIP`。

如果开始明显左右抖，就减小 `gIP`。

### 11.6 第六步：少量加入内环 I

当 P 基本能用后，再加一点 I：

```text
#gII=0.005;
#gII=0.010;
#gII=0.015;
```

I 的判断标准：

```text
弯道长期转不够 -> 可以加一点 I
出弯拖泥带水、反应慢、过头 -> I 太大
```

### 11.7 第七步：再调外环

内环像“执行器”，外环像“大脑给目标”。

内环没调顺之前，不建议猛调外环。

外环 P：

```text
太小 -> error 消得慢，入弯不积极
太大 -> 目标角速度太激进，车容易甩
```

外环 D：

```text
适量 -> 减少视觉滞后，提前修正
太大 -> 视觉噪声被放大，车会抖
```

建议顺序：

```text
先 gOD=0，只调 gOP
再一点点加 gOD
```

## 12. 审查时重点看什么

你审查这份实现时，建议按下面顺序看。

### 12.1 控制链条是否合理

当前链条是：

```text
latest_error
-> target_yaw_rate_dps
-> rate_error
-> turn_rps
-> pwm1_duty_rps / pwm2_duty_rps
-> 原有速度环
```

这符合你前面说的“视觉给 error，再闭环调旋转速度消除 error”。

### 12.2 单位是否清楚

| 变量 | 单位 | 说明 |
| --- | --- | --- |
| `latest_error` | 视觉误差单位 | 由视觉算法定义，当前按 -100 到 100 使用 |
| `target_yaw_rate_dps` | deg/s | 外环希望车身达到的角速度 |
| `gyro_z_raw` | LSB | MPU6050 原始读数 |
| `gyro_z_dps` | deg/s | 扣零偏后的角速度 |
| `gyro_z_lpf` | deg/s | 低通后的角速度反馈 |
| `rate_error` | deg/s | 目标角速度和实际角速度的差 |
| `turn_rps` | RPS | 左右轮差速修正 |
| `pwm1_duty_rps` / `pwm2_duty_rps` | RPS | 速度环目标 |

只要单位链条不乱，调参就有方向。

### 12.3 失败时是否安全

当前设计里：

```text
gyro_yaw_rate_feedback_enabled 默认是 0
MPU6050 不 ready 会回退到旧视觉 PD
发车、停车、脚本接管会 reset
输出有 target 角速度限幅和 turn_rps 限幅
```

这几个点是为了让 demo 不会一上来破坏原车。

### 12.4 build 是否能自动包含新源文件

工程 `main/CMakeLists.txt` 里有：

```cmake
aux_source_directory(${PROJECT_SOURCE_DIR}/../example/src SRC)
```

所以新增的：

```text
example/src/gyro_yaw_rate_control.cpp
```

应该会被自动加入编译，不需要手动改 CMakeLists。

## 13. 当前实现里我认为需要你实车确认的问题

下面这些不是代码一定错，而是必须靠你的车来定。

### 13.1 `latest_error` 的正方向

你需要确认：

```text
latest_error > 0 时，你希望车向左修还是向右修？
```

这个方向必须和 `target_yaw_rate`、`gyro_z_sign`、`gyro_turn_sign` 统一。

### 13.2 motor1 / motor2 分别是哪一侧

当前输出是：

```cpp
pwm1_duty_rps = set_spd1 + diffrential;
pwm2_duty_rps = set_spd1 - diffrential;
```

所以 `diffrential > 0` 时：

```text
motor1 目标速度更大
motor2 目标速度更小
```

但这到底对应左转还是右转，取决于你车上 motor1/motor2 的物理安装。

### 13.3 MPU6050 的安装方向

默认认为车身 yaw 对应 MPU6050 的 `gz`。

如果你的 MPU6050 板子竖着装、侧着装，或者 Z 轴不是车身竖直方向，那就不能直接用 `gz`，需要改成 `gx` 或 `gy`，或者做轴映射。

这个必须实车手动转动验证。

### 13.4 方向环周期是否始终是 8ms

当前内环积分用的是：

```text
kControlDt = 0.008
```

它对应 `dir_timer` 每 8ms 调用一次方向控制。

如果以后你把方向环改成 5ms 或 10ms，这里也要改。

## 14. 浮点 RPS 链路说明

`gyro_yaw_rate_control_update()` 输出的是 `float turn_rps`，现在 `dir_pd.cpp` 中的方向差速、减速后的基准速度、`pwm1_duty_rps`、`pwm2_duty_rps`、速度环目标参数都保持 `float`。

这样做的原因是：

```text
turn_rps = 0.8  仍然能作为 0.8 RPS 的小差速送进速度环
turn_rps = 1.2  仍然能作为 1.2 RPS 的小差速送进速度环
```

最终只有 PWM 占空比仍然是 `int`，因为硬件 PWM 的比较值本身就是整数计数。这一层截断是合理的；方向环和速度环之间不应该提前截断。

另外，原视觉 PD 里的 `error_current/error_last` 也改成了 `float`，避免视觉 error 的小数部分在方向环第一步就丢失。

## 15. 我建议你审查后的下一步

先不要急着把角速度环当最终版。

建议下一步按这个顺序推进：

```text
1. 编译确认新增文件没有 C++ 语法问题
2. 上车静止启动，确认 MPU6050 ready
3. #gyro=0; 确认旧方向环仍正常
4. 手动转车确认 gyro 正负
5. 架空轮子 #gyro=1; 确认输出方向
6. #gII=0; 只调内环 P
7. 能稳定跟随 target 后，再加一点 I
8. 最后再调外环 P、D
```

只要这条链路调顺，你脑子里就可以这样理解这辆车：

```text
视觉 error 不是直接命令电机
视觉 error 是在告诉车“你现在应该以多快的角速度把车头转回来”
MPU6050 负责告诉控制器“你实际转得够不够”
左右轮差速只是实现这个角速度目标的手段
```
