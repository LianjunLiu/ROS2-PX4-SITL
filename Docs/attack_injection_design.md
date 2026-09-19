# 攻击注入系统设计（总览）

> 本文给出攻击注入系统的总览——架构、消息协议、注入点、控制操作、日志分析与设计决策、顶层限制；关联文档：[原语分册](attack_primitive_reference.md)、[管理器分册](attack_manager_reference.md)； 版本 v5 · 状态 已实现 · 代码 `PX4-Autopilot/src/modules/simulation/gz_bridge/`。

## 1. 目标

在 PX4 SITL 中注入可配置攻击，研究攻击对飞控的影响。系统须满足三项核心能力：

1. 运行时随时开启 / 关闭任意攻击；
2. 运行时随时修改任意攻击参数；
3. 攻击全过程记入 ulog，可复现、可对齐分析。

| 攻击面 | 对象 | 注入点 | 通道 |
|---|---|---|---|
| 传感器攻击 | GPS（位置 / 速度） | `GZBridge::navSatCallback` → `sensor_gps` | 6 |
| 执行器攻击 | 4 个电机（桨转速） | `GZMixingInterfaceESC::updateOutputs` → `gz.msgs.Actuators` | 4 |

## 2. 术语

三个文档共用同一套记号。

| 术语 | 记号 | 定义 |
|---|---|---|
| **通道** | `Channel` | 注入点词汇表，编号 0–9（GPS 6 + 电机 4），语义只存在于注入点 |
| **原语** | `PrimitiveType` | 13 种攻击形状之一（原语分册 §4） |
| **真值** | `truth` | 注入点传入的干净测量 / 指令值 |
| **篡改值** | `v` | 原语作用后的输出 |
| **丢弃** | `pass = false` | `DROP` 原语要求本拍不发布；如何「丢」由注入点决定（§5.3） |
| **配置** | `AttackSpec` | 每通道可序列化配置（`type` / `a` / `b` / `c` / `d` / `start_us` / `end_us` / `seed`） |
| **状态** | `ChannelState` | 每通道运行时状态（`last_value` / `rng_state` / 历史 / 快照） |
| **时间窗** | `[start_us, end_us)` | 攻击生效区间，半开；`end_us = 0` 表示无终点 |
| **时间戳** | `hrt_absolute_time()` | PX4 上电起算的单调微秒时钟，全文统一 |
| **种子** | `seed` | 随机原语的可复现密钥；默认每通道确定（§7.2） |

## 3. 架构

### 3.1 分层

```text
Layer A（AttackLibrary）  「怎么篡改一个标量」       纯数学，零 PX4/gz/uORB 依赖
        ▲ 调用
Layer C（AttackManager）  「哪个通道挂了什么攻击」    路由 + 指令解析 + 状态发布
        ▲ 调用
Layer B（注入点）          「GPS 纬度 / 电机 0」     语义 ↔ Channel 映射，极薄
```

**收益**：加新攻击种类 → 只改 A；加新攻击对象 → 只改 B；C 对两者透明。

### 3.2 数据流

```text
ROS2 节点 ──ros2 topic pub──▶ /fmu/in/attack_command ──(DDS)──▶ uORB attack_command
                                                                      │
                                                        AttackManager::update() (~100 Hz)
                                                                      │  on_command → set
                                                                      ▼
                                                          _specs[ch] = AttackSpec
                                                                      │
      GPS 注入点（navSatCallback）           电机注入点（updateOutputs）
                └──────┐                                   └──────┐
                       ▼    AttackManager::apply(ch, value)       ▼
                                  │ 查表 → AttackLibrary::apply
                                  ▼
                 pass=true → 篡改值；pass=false → 注入点丢弃
                                                                      │
                                              publish_status() (10 Hz) → attack_status
                                                                      ▼
                                                     logger → ulog（可复现、可对齐）
```

> **注**：命令投递链已打通——`dds_topics.yaml` 已注册 `/fmu/in/attack_command`（订阅）与 `/fmu/out/attack_status`（发布），`uxrce_dds_client` 据此在运行期动态桥接 uORB 话题（§6）。

## 4. 消息

### 4.1 命令

```text
uint64 timestamp        # 系统启动以来时间（微秒）
uint8  channel          # Channel 枚举值（0..9）
uint8  type             # PrimitiveType 枚举值；0 = NONE（清除攻击）
float64[4] param        # a, b, c, d
uint64 t0_us            # 相对指令到达的启动延迟（µs）；0 = 立即
uint64 t1_us            # 相对启动的持续时间（µs）；0 = 无限
uint32 seed             # 随机原语种子；0 = 每通道默认
```

> **注**：一条消息 = 一个通道；`type = NONE` 即清除该通道攻击；`t0_us` / `t1_us` 是相对偏移，由管理器在收到指令的当下解析成绝对时间（管理器分册 §5.1）。

### 4.2 状态

```text
uint64 timestamp        # 系统启动以来时间（微秒）
uint8[10]  active       # 每通道此刻是否正在生效（0/1）
uint8[10]  type         # 每通道当前原语类型（active 时有效）
float64[10] param_a     # 每通道参数 a
float64[10] param_b     # 每通道参数 b
float64[10] param_c     # 每通道参数 c
float64[10] param_d     # 每通道参数 d
```

> **注**：`active` 显式区分「挂了规格」（`type != NONE`）与「此刻正在生效」（`now ∈ [start_us, end_us)`），能正确反映「命令已到但还没到 start」的中间态。参数按 `param_a` / `param_b` / `param_c` / `param_d` 四个独立数组存（非二维数组），与 uORB 生成器对齐。

## 5. 注入点

### 5.1 卫星

GPS 注入点位于 `GZBridge.cpp` 的 `navSatCallback`，在现有 `addGpsNoise()` 之后、`_sensor_gps_pub.publish()` 之前插入：

```cpp
double lat_att = latitude, lon_att = longitude, alt_att = altitude;
double vn_att = static_cast<double>(vel_north);
double ve_att = static_cast<double>(vel_east);
double vd_att = static_cast<double>(vel_down);

// 六个分量逐个过攻击层。用 &=（不短路）保证 6 次调用一个不落，再「与」成最终裁决。
bool gps_publish = true;
gps_publish &= _attack.apply(attack::Channel::GPS_LAT,  lat_att, timestamp);
gps_publish &= _attack.apply(attack::Channel::GPS_LON,  lon_att, timestamp);
gps_publish &= _attack.apply(attack::Channel::GPS_ALT,  alt_att, timestamp);
gps_publish &= _attack.apply(attack::Channel::GPS_VEL_N, vn_att, timestamp);
gps_publish &= _attack.apply(attack::Channel::GPS_VEL_E, ve_att, timestamp);
gps_publish &= _attack.apply(attack::Channel::GPS_VEL_D, vd_att, timestamp);

if (!gps_publish) { return; }   // 任一维丢包 → 整条 fix 不发布（§5.3）

latitude = lat_att;  longitude = lon_att;  altitude = alt_att;
vel_north = static_cast<float>(vn_att);
vel_east  = static_cast<float>(ve_att);
vel_down  = static_cast<float>(vd_att);
```

`DROP` 时 GPS 整条跳过：任一维触发 `pass = false` 即 `return`，本拍不发布 `sensor_gps`，飞控这一拍读不到任何 GPS 测量（§5.3）。

### 5.2 电机

`GZMixingInterfaceESC.cpp` 的 `updateOutputs`，逐电机独立处理：

```cpp
for (unsigned i = 0; i < active_output_count; i++) {
    uint16_t value = outputs[i];

    if (_attack != nullptr) {
        const attack::Channel ch = attack::motor_channel(i);
        double v = static_cast<double>(outputs[i]);

        if (_attack->apply(ch, v, hrt_absolute_time())) {
            value = static_cast<uint16_t>(math::constrain(v, 0.0, 65535.0));  // 类型转换 + 限幅
        } else {
            value = _last_motor[i];   // DROP：仅该路保持上一拍转速（§5.3）
        }

        _last_motor[i] = value;       // 无论通过与否都记录本拍实际发送值
    }

    rotor_velocity_message.set_velocity(i, value);
}
```

`DROP` 时电机保持上一拍：仅该路 `hold last`（`_last_motor[i]`），其余电机照常输出（§5.3）。

> **注**：`GZBridge` 按值持有 `AttackManager _attack`；GPS 回调在 `GZBridge` 内部直接 `_attack.apply(...)`；ESC 类是独立类，靠 `_mixing_interface_esc.setAttack(&_attack)` 注入指针。

### 5.3 丢弃

同样 `DROP`，两个注入点的「丢弃」粒度不同，由数据的物理不可分性决定：

| 对比项 | GPS | 电机 |
|---|---|---|
| 数据形态 | 1 条 `sensor_gps` 消息，6 字段不可分割 | 4 路彼此独立的 PWM/DShot 输出 |
| 丢弃粒度 | 整条消息 | 单路 |
| `pass = false` 时 | `return`，整条 fix 不发布 | 仅该路 hold last，其余照常 |
| 模拟的物理现象 | 接收机整周期失锁 | 单个执行器丢帧 |

为什么 GPS 不能只丢一个分量：1）uORB 的发布单位是整条消息，lat/lon/alt/vel 是同一个 `sensor_gps_s` 里的字段，做不到「只发纬度、不发经度」。2）物理上真实 GPS 接收机要么解算出一组完整 fix，要么失锁整组一起没。因此「只坏一个 GPS 分量」应使用完整性原语（`BIAS` / `SPOOF` / `NOISE` 等，整条消息照发）；`DROP` 在 GPS 侧严格对应「整周期失锁」。

## 6. 控制

链路已配置完成：`msg/CMakeLists.txt` 注册两个 `.msg`（生成 `px4_msgs/msg/AttackCommand`、`px4_msgs/msg/AttackStatus`）；`dds_topics.yaml` 加两条桥接；ROS2 侧直接用 `px4_msgs`，uXRCE-DDS Agent 自动桥接，无需自研消息。

```bash
# 开启：GPS 纬度欺骗到 31.23°（channel 0 = GPS_LAT, type 2 = SPOOF）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 2, param: [31.23, 0.0, 0.0, 0.0]}" --once

# 延迟 1 秒开始、持续 5 秒的 GPS 纬度偏置（BIAS +0.001°）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 1, param: [0.001, 0.0, 0.0, 0.0], t0_us: 1000000, t1_us: 5000000}" --once

# GPS 噪声（σ=0.0001°）；seed: 42 覆盖每通道默认种子
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 3, param: [0.0001, 0.0, 0.0, 0.0], seed: 42}" --once

# 关闭
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 0}" --once
```

「随时」语义：开 = 发 `type=X`；改参数 = 再发一条同 channel 新参数；关 = 发 `type=NONE`。从该通道下一条消息起生效（GPS ~5–10 Hz、电机高频，几乎即时）。

> **注**：可选在 `x500_plus` 加 `attack_controller` 节点（照 `wind_injector` 模式）做友好封装，非必需。

## 7. 日志

### 7.1 登记

uORB 不自动记 ulog：PX4 logger 只记录显式登记的话题（`logged_topics.cpp`）。两个攻击话题已登记：

```cpp
add_optional_topic("attack_command");   // 事件型，有就记、无也不阻塞
add_topic("attack_status", 100);        // 10 Hz 快照
```

| 话题 | 登记方式 | 频率 | 用途 |
|---|---|---|---|
| `attack_command` | `add_optional_topic` | 事件（来一条记一条） | 还原「何时下了什么指令」的精确时间线 |
| `attack_status` | `add_topic` | 10 Hz | 任意时刻激活状态，对齐 EKF 偏差；兼作生效心跳 |

> **注**：`attack_command` 用 `add_optional_topic`，因为固件内无静态 advertiser（命令经 DDS 动态进来），`add_topic` 会让 logger 等 publisher 而阻塞；`attack_status` 用 `add_topic`，因为 `AttackManager::init()` 即 advertise，必然存在。

### 7.2 复现

- **时间戳统一**：uORB 消息体 `timestamp`、ulog 数据记录头、攻击的 `start_us` / `end_us`，都是 `hrt_absolute_time()` 微秒单调时钟（上电起算）。事后对齐攻击窗口与飞控数据，就是直接比同一个 µs 数值。
- **种子语义**：`seed = 0`（默认）→ 每通道确定性默认种子，可复现且通道间不相关；`seed ≠ 0` → 直接用该值。默认种子公式（管理器分册 §5.2）；种子语义（原语分册 §5）。
- **配置零 param**：攻击配置全走 uORB 消息，不注册任何 param（param 会数量爆炸且 float32 丢 GPS 精度）。

### 7.3 分析

三条时间线对齐：

| 时间线 | 内容 | 话题 | 是否默认记录 |
|---|---|---|---|
| 攻击侧 | 何时、哪通道、挂了什么攻击 | `attack_command` / `attack_status` | 已登记（§7.1） |
| 反应侧 | 飞控在攻击下怎么飞 | `vehicle_local_position` / `vehicle_attitude` / `actuator_outputs` / `sensor_gps` | 默认记录 |
| 真值侧 | 无人机「真正」在哪 | `vehicle_*_groundtruth` | SITL 默认记录 |

分析 = 从 `attack_status` 提取攻击窗口，叠在飞控状态上对齐。工具：`pyulog`（批处理）、PlotJuggler（交互）、PX4 Flight Review。

> **注**：`sensor_gps` 默认只记 1 Hz（logger 里 `add_topic_multi("sensor_gps", 1000, 2)`），而 SITL GPS ~5 Hz。要看细粒度 GPS 篡改（如 `DELAY` 几百毫秒），需把记录间隔调到 ~100 ms。

**真值 ≠ 反事实基线**：`groundtruth` 是 gazebo 的物理真值（不受攻击影响），但不等于「无攻击时 EKF 的估计值」。攻击发生在 EKF 之前，EKF 一旦被污染，无法在同一次运行里得到「无攻击 EKF 估计」这个反事实基线——仅靠真值话题测出的是「固有估计误差 + 攻击效应」的混合。

- **推荐**：用配对运行隔离攻击效应——同一场景 + 同一种子/风/指令，跑一次无攻击基线 + 一次有攻击，比较两次 EKF 估计之差。辅助信号：`sensor_gps`（被篡改测量，即 EKF 实际所见）、GPS innovation 突变（EKF 自带检测信号）、`actuator_outputs`（被篡改指令）。
- **备选**（更稳但改动大）：同一次运行跑两个 EKF（一个喂干净 GPS、一个喂被攻击 GPS）直接取差。PX4 默认单 EKF，暂列扩展。

## 8. 决策

| # | 决策 | 理由 |
|---|---|---|
| 1 | 库零 PX4 依赖 | 可独立单测原语 |
| 2 | `Channel` 无语义编号 | 语义只存在于注入点 |
| 3 | 注入点极薄（一行调用） | 加新对象成本最低 |
| 4 | 数值统一 `double` | 保 GPS 精度；类型转换/限幅留在注入点 |
| 5 | `AttackSpec` 纯值可序列化 | 可随消息走、可进 ulog、可复现 |
| 6 | 攻击信息零 param，全走 uORB 消息 | param 会数量爆炸且 float32 丢精度 |
| 7 | 配置与状态分离 | `AttackSpec`（可序列化）与 `ChannelState`（运行时）分开 |
| 8 | 时间窗消息用相对偏移、管理器转绝对 | ROS2 侧友好、内部精确 |
| 9 | `seed` 默认每通道确定、显式非零覆盖 | 默认即可复现，且多通道不相关 |
| 10 | 电机 `DROP` = hold last、不建模 ESC 失步 | 简化但语义保守（§9） |

## 9. 限制

本文列顶层限制；原语级边界（原语分册 §8），管理器级边界（管理器分册 §8）。

| # | 事项 | 说明 |
|---|---|---|
| 1 | 时间单位两套 | `now_us` / `start_us` / `end_us` / `t0_us` / `t1_us` 是微秒；`DELAY` / `REPLAY` 的 `a` 是秒（内部 ×1e6）。 |
| 2 | 电机 DROP 退化 | `DROP` 的 `a` 逼近 1 时电机退化为 `FREEZE`，且当前不建模 ESC 失步——「接近全丢」时仿真比真实更温柔（原语分册 §8）。 |
| 3 | GPS 篡改分辨率受 logger 限制 | `sensor_gps` 默认 1 Hz 记录，细粒度篡改需调日志间隔（§7.3）。 |
| 4 | 随机原语需 seed 重放 | 逐拍随机事件（丢哪条、噪声值）不靠日志全量记录，靠同 `seed` 重放复现。 |

## 10. 扩展

| 想扩展 | 改哪层 | 动什么 |
|---|---|---|
| 攻击 IMU / 磁力计 | B | `Channel` 加项 + 对应回调加一行调用 |
| 加更多原语（饱和、突刺等） | A | 枚举末尾追加 + `transform` 加分支 |
| ESC 失步建模 | B（`GZMixingInterfaceESC.cpp`） | 注入点加连续丢帧计时 + 超时归零 |
| 攻击舵机 / 轮子 | B | `GZMixingInterfaceServo` / `GZMixingInterfaceWheel` 加调用 |
| 更友好 ROS2 封装 | ROS2 侧 | `x500_plus` 加 `attack_controller` 节点 |
| 双 EKF 反事实 | 飞控侧 | 一个喂干净 GPS、一个喂攻击 GPS 直接取差 |

## 11. 文件清单

```text
PX4-Autopilot/src/modules/simulation/gz_bridge/
├── attack/
│   ├── AttackPrimitive.hpp/.cpp   # Layer A：原语库（零依赖）
│   └── AttackManager.hpp/.cpp     # Layer C：路由 + 指令解析 + 状态发布
├── GZBridge.cpp/.hpp              # 改：navSatCallback 注入 GPS；持有 _attack；update()
└── GZMixingInterfaceESC.cpp/.hpp  # 改：updateOutputs 注入电机；setAttack 注入

PX4-Autopilot/msg/
├── AttackCommand.msg              # uORB 消息（输入）
└── AttackStatus.msg               # uORB 消息（输出）

PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml   # 已加 2 条（DDS 桥）
PX4-Autopilot/src/modules/logger/logged_topics.cpp           # 已加 2 条（ulog 登记）
PX4-Autopilot/msg/CMakeLists.txt                             # 已注册两个 .msg
```

## 12. 结论

本系统在 `gz_bridge` 进程内以三层解耦（原语库 / 注入点 / 管理器）实现 GPS 与电机攻击的可配置注入：攻击指令经 ROS2 → DDS → uORB 下达，`AttackManager` 解析相对时间窗并按通道路由，注入点在 EKF 之前篡改测量或指令，全过程记入 ulog，时间戳统一于 `hrt_absolute_time()` 微秒单调时钟，配合确定性种子与配对运行可实现攻击效应的可复现与可对齐分析。原语细节（原语分册 §4），管理器细节（管理器分册 §5）。
