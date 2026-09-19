# 状态攻击设计（post-EKF）

> 本文给出「状态攻击」的设计——目标、架构、注入点、消息协议、控制、日志分析与设计决策、限制；关联文档：[攻击注入总览](attack_injection_design.md)、[原语分册](attack_primitive_reference.md)、[管理器分册](attack_manager_reference.md)； 版本 v1 · 状态 点攻击已实现（SITL 编译通过）；DELAY / REPLAY 未实施（§10）、DROP 不可实施（§9 #2） · 代码`PX4-Autopilot/src/lib/state_attack/`、`src/modules/mc_pos_control/`、`src/modules/mc_att_control/`。

## 1. 目标

在 PX4 SITL 中新增一类与现有攻击隔离的攻击：作用于 EKF 融合完成之后的导航状态与姿态，即「跳过 EKF、攻击 EKF 之后的结果」。它复用现有攻击的运行时能力——随时开 / 关、随时改参数、全程不改代码（只发命令消息），但使用独立的命令通道与独立代码，不与 gz_bridge 的传感器 / 执行器攻击耦合。

三类攻击面并列：

| 攻击面 | 对象 | 注入点 | 通道 |
|---|---|---|---|
| 传感器攻击（现有） | GPS（位置 / 速度） | `GZBridge::navSatCallback` → `sensor_gps` | 6 |
| 执行器攻击（现有） | 4 个电机（桨转速） | `GZMixingInterfaceESC::updateOutputs` | 4 |
| **状态攻击（新增）** | EKF 输出：导航状态 + 姿态 | `mc_pos_control`（7）+ `mc_att_control`（3） | **10** |

## 2. 术语

沿用总览 §2 的记号（`truth` / 篡改值 / `AttackSpec` / `ChannelState` / 时间窗 / 种子）。新增：

| 术语 | 定义 |
|---|---|
| **EKF 输入攻击** | 在 EKF 上游篡改原始传感器 / 执行器指令（现有 gz_bridge 攻击）。EKF 会做一致性校验，可部分自愈 |
| **EKF 输出攻击** | 在 EKF 下游篡改融合后的状态（本文）。EKF 无机会拒掉，控制器直接信任被污染状态 |
| **导航状态** | `vehicle_local_position` 中供位置控制器使用的 7 个标量：位置 x/y/z、速度 vx/vy/vz、航向 heading |
| **姿态** | `vehicle_attitude.q` 分解出的 3 个欧拉角：横滚 roll、俯仰 pitch、偏航 yaw |
| **控制器所见** | 攻击后控制器实际拿到的（被污染）状态，与 uORB 里的真值话题（`vehicle_local_position` / `vehicle_attitude`）不同 |

## 3. 架构

### 3.1 攻击隔离

现有攻击在宿主机 `gz_bridge` 进程（仿真桥接层），新攻击在 PX4 固件两个飞控模块内。二者不同进程、不同构建产物、不同命令话题，天然隔离：

```text
传感器 ──[gz_bridge 攻击点]──▶ EKF ──[状态攻击点]──▶ 控制器 ──▶ 执行器
         （现有：EKF 输入）        （新增：EKF 输出）
```

EKF 输入攻击污染 EKF 的输入，EKF 内部状态会被污染并持续；EKF 输出攻击污染 EKF 的输出下游，EKF 内部状态始终干净——攻击一停，控制器立即回到真值。这一对比本身就是两类攻击的实验区分维度（§9 #1）。

### 3.2 攻击分层

镜像总览 §3.1 的三层，但独立实现、独立命名空间：

```text
Layer A'（StateAttackPrimitive）「怎么篡改一个状态标量」  纯数学，零 PX4/gz/uORB 依赖
           ▲ 调用
Layer C'（StateAttackManager）  「哪个状态字段挂了什么攻击」 指令解析 + 路由 + 状态发布
           ▲ 调用
Layer B'（注入点 × 2）          「set_vehicle_states 之前 / Quatf q{v_att.q} 之前」
```

原语语义复用：原语枚举沿用原语分册 §4 的 13 种（`NONE=0`…`REPLAY=13`）与 `type` 码，数学定义一致；但代码独立——固件侧无法链接 gz_bridge 的 `attack/` 库，在 `src/lib/state_attack/` 独立实现一份。

### 3.3 管理器实例与数据流

状态攻击横跨两个飞控模块（mc_pos_control 与 mc_att_control 是不同 task），因此 `StateAttackManager` 实例化两次：两个模块各持一个成员，各自订阅同一条命令话题、各自维护同一份10 通道规格表（同一命令流广播，两表必然一致），但只应用属于自己注入点的通道子集：

| 模块 | 持有 | 订阅命令 | 应用通道 | 发布状态 |
|---|---|---|---|---|
| mc_pos_control | `_state_attack` | `state_attack_command` | 0–6（导航） | `state_attack_pos_status` |
| mc_att_control | `_state_attack` | `state_attack_command` | 7–9（姿态） | `state_attack_att_status` |

```text
ROS2 节点 ──ros2 topic pub──▶ /fmu/in/state_attack_command ──(DDS)──▶ uORB state_attack_command（广播）
                              ┌─────────────────────────────────────────────┴──────────┐
                              ▼                                                        ▼
   mc_pos_control 循环内 StateAttackManager::update()                       mc_att_control 循环StateAttackManager::update()
        │  on_command → set（通道 0–6）                                                  │  on_command → set（通道 7–9）
        ▼                                                                               ▼
   vehicle_local_position（真值）                                                    vehicle_attitude.q（真值）
        │  apply_position(local_pos) 就地污染 x/y/z/vx/vy/vz/heading                     │  apply_attitude(v_att) Euler 域就地污染
        ▼                                                                               ▼
   set_vehicle_states() → PositionControl → 姿态设定值                            Quatf q{v_att.q} → 速率设定值
        │                                                                               │
        ▼                                                                               ▼
   publish_status() → state_attack_pos_status                               publish_status() → state_attack_att_status
        └──────────────────────────────┬─────────────────────────────────────────────────────┘
                                       ▼
                                   logger → ulog（控制器所见 + 真值并列）
```

> **注**：管理器在各自模块的循环里 `update()` + `apply_position()`（照 `GZBridge` 持有 `AttackManager` 的既有模式），不起独立 task，也不加任何 uORB 中转跳。

## 4. 消息

### 4.1 命令

`StateAttackCommand.msg`（字段与 `AttackCommand` 相同，仅通道语义不同）：

```text
uint64 timestamp        # 系统启动以来时间（微秒）
uint8  channel          # StateChannel 枚举值（0..9）
uint8  type             # PrimitiveType 枚举值（0=NONE 清除；沿用原语分册）
float64[4] param        # a, b, c, d
uint64 t0_us            # 相对指令到达的启动延迟（µs）；0 = 立即
uint64 t1_us            # 相对启动的持续时间（µs）；0 = 无限
uint32 seed             # 随机原语种子；0 = 每通道默认
```

### 4.2 状态

两个状态话题，各由对应模块发布，各含每通道配置（active/type/param）与「控制器所见」值（`value`；真值话题里看不到攻击，§7.2）：

`StateAttackPosStatus.msg`（导航，mc_pos_control 发布）：

```text
uint64 timestamp          # 发布时刻（微秒）
uint64 timestamp_sample   # 对应 vehicle_local_position.timestamp_sample，对齐真值样本
uint8[7]  active          # 每导航通道此刻是否生效
uint8[7]  type           # 每导航通道当前原语类型
float64[7] param_a       # 每导航通道参数 a（b/c/d 同构）
float64[7] param_b
float64[7] param_c
float64[7] param_d
float64[7] value          # 每导航通道控制器所见值；未攻击=真值；单位随通道（§4.3）
```

`StateAttackAttStatus.msg`（姿态，mc_att_control 发布）：

```text
uint64 timestamp          # 发布时刻（微秒）
uint64 timestamp_sample   # 对应 vehicle_attitude.timestamp_sample，对齐真值样本
uint8[3]  active
uint8[3]  type
float64[3] param_a
float64[3] param_b
float64[3] param_c
float64[3] param_d
float64[3] value          # 每姿态通道控制器所见（roll/pitch/yaw，rad）；未攻击=真值
```

时间戳语义（`timestamp` 与 `timestamp_sample` 是两个不同字段）：

| 字段 | 含义 | 来源 |
|---|---|---|
| `timestamp` | 发布时刻 | `update()`/`publish_status()` 里的 `hrt_absolute_time()` |
| `timestamp_sample` | 被篡改的那份 EKF 真值样本的采样时刻——不是篡改时刻、也不是发布时刻 | 从 `vehicle_local_position.timestamp_sample` / `vehicle_attitude.timestamp_sample` 原样复制 |

`timestamp_sample` 是**对齐键**：真值话题与 status 发布频率、时机不同，各自的 `timestamp` 对不齐；只有 `timestamp_sample` 指向同一份 EKF 样本，据此把 `value` 与真值逐样本并排，算「控制器所见 − 真值」偏差（§7.3、§8 #10）。

### 4.3 通道

| 通道 | 名称 | 字段 | 单位 | 帧 / 方向 | 注入点 |
|---|---|---|---|---|---|
| 0 | `NAV_POS_X` | `vehicle_local_position.x` | m | NED 北 | mc_pos_control |
| 1 | `NAV_POS_Y` | `.y` | m | NED 东 | mc_pos_control |
| 2 | `NAV_POS_Z` | `.z` | m | NED 下 | mc_pos_control |
| 3 | `NAV_VEL_X` | `.vx` | m/s | NED 北 | mc_pos_control |
| 4 | `NAV_VEL_Y` | `.vy` | m/s | NED 东 | mc_pos_control |
| 5 | `NAV_VEL_Z` | `.vz` | m/s | NED 下 | mc_pos_control |
| 6 | `NAV_HEADING` | `.heading` | rad | 航向 −π..π | mc_pos_control |
| 7 | `ATT_ROLL` | `vehicle_attitude.q`（Euler 分解） | rad | 横滚 | mc_att_control |
| 8 | `ATT_PITCH` | 同上 | rad | 俯仰 | mc_att_control |
| 9 | `ATT_YAW` | 同上 | rad | 偏航 | mc_att_control |

10 个通道彼此独立（照 GPS 6 分量语义），可单独或任意组合攻击。`NAV_HEADING`（位置控制器用的 yaw）与 `ATT_YAW`（姿态控制器用的 yaw）分属两条链路，攻击其中一个或两个同时攻击，是实验维度。

> **注**：`value[ch]` 的单位与方向 = 该通道在表中「单位 / 帧」列（如 `value[0]` 是 NED 北向位置 m、`value[6]` 是航向 rad、`value[7]` 是横滚 rad）。

## 5. 注入点

### 5.1 导航状态（mc_pos_control）

注入点：`MulticopterPositionControl::Run()` 内 `_local_pos_sub.update()` 之后、`set_vehicle_states()` 之前（[MulticopterPositionControl.cpp:399/432](PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L399)）：

```cpp
vehicle_local_position_s vehicle_local_position;
if (_local_pos_sub.update(&vehicle_local_position)) {
    // ... 时间戳 / 模式 / 约束更新 ...

    _state_attack.apply_position(vehicle_local_position);              // ← 注入点：就地污染 7 个导航字段

    PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};   // ← 内部对速度做滤波（§5.3）
    // ...
}
```

`set_vehicle_states()` 只读 `x/y/z`、`vx/vy/vz`、`heading` 与有效标志位，是 EKF 状态灌进位置控制器的唯一入口；其中速度还会过滤波（notch → 低通，§5.4），加速度由滤波后速度微分再低通得到，位置 / 航向不滤波。就地污染只改 mc_pos_control 一个消费者所见，EKF / navigator / commander 读到的仍是真值。

注入点放在 `set_vehicle_states()` **之前**：被污染的速度会先经滤波再进控制器——故障最终是「原样直达」还是「被滤波抹平」取决于通道是否走滤波。这个 before / after 的区别是故障注入实验的一个独立观测维度，故单列 §5.3 说明；滤波器类型与默认状态见 §5.4。

`apply_position()` 内部：7 通道各经 `apply_channel(ch, field, now)` 调 `StateAttackLibrary::apply(field, spec, now, state)` → 写回字段；被污染值由 `update()` 在下一周期快照进 status（§7.2 时序注）。

### 5.2 机体姿态（mc_att_control）

注入点：`_vehicle_attitude_sub.update()` 之后、`Quatf q{v_att.q}` 构造之前（[mc_att_control_main.cpp:247/253](PX4-Autopilot/src/modules/mc_att_control/mc_att_control_main.cpp#L247)）：

```cpp
if (_vehicle_attitude_sub.update(&v_att)) {
    _state_attack.apply_attitude(v_att);                     // ← 注入点：Euler 域就地污染 q
    const float dt = math::constrain((v_att.timestamp_sample - _last_run) * 1e-6f, 0.0002f, 0.02f);
    const Quatf q{v_att.q};                                  // 用被污染后的姿态
    // ...
}
```

四元数归一化：`vehicle_attitude.q` 是单位四元数，直接改 `q` 的某个分量会破坏单位模长。因此姿态攻击在欧拉角域进行，再由欧拉角重构四元数（欧拉角 → 四元数必然单位模长）：

```cpp
void StateAttackManager<StatusMsg>::apply_attitude(vehicle_attitude_s &v)
{
    const uint64_t now = hrt_absolute_time();
    const matrix::Eulerf e{matrix::Quatf(v.q)};            // q → (roll, pitch, yaw)
    double roll = e.phi(), pitch = e.theta(), yaw = e.psi();

    apply_channel(StateChannel::ATT_ROLL,  roll,  now);   // 3 通道各过一次原语变换
    apply_channel(StateChannel::ATT_PITCH, pitch, now);
    apply_channel(StateChannel::ATT_YAW,   yaw,   now);

    const matrix::Quatf q_out{matrix::Eulerf(roll, pitch, yaw)};  // Euler → 单位四元数
    v.q[0] = q_out(0); v.q[1] = q_out(1); v.q[2] = q_out(2); v.q[3] = q_out(3);
}
```

> **注**：姿态外环（mc_att_control）用 `q` 算姿态误差并出速率设定值；速率内环（mc_rate_control）读的 `vehicle_angular_velocity` **不受本攻击影响**。攻击姿态会让外环输出错误的速率设定值、内环去追这个错目标，从而形成真实控制响应；若要直接攻击速率内环，另开 `vehicle_angular_velocity` 注入点（§10）。

### 5.3 滤波与注入点前后（before / after）区别

`set_vehicle_states()` 只对速度做滤波（vx/vy/vz：notch → 低通，[MulticopterPositionControl.cpp:340/360](PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L340)），加速度由滤波后速度求微分再低通得到（`:343/363`，原始 `vehicle_local_position` 里没有加速度字段）；位置 x/y/z 与 heading 不滤波，直接拷贝。注意：速度的 notch / 低通默认禁用（`MPC_VEL_NF_FRQ`、`MPC_VEL_LP` 默认 0），仅加速度微分低通（`MPC_VELD_LP`=5 Hz）默认开启（§5.4）。姿态侧状态则完全不滤波——`apply_attitude` 之后 `Quatf q{v_att.q}` 直接使用（[mc_att_control_main.cpp:253/255](PX4-Autopilot/src/modules/mc_att_control/mc_att_control_main.cpp#L253)）；mc_att_control 里仅有的 `_man_roll_input_filter` / `_man_pitch_input_filter` 作用于手动摇杆输入（setpoint 侧），与状态无关。

因此「注入点放滤波前（本设计当前，§5.1/§5.2）还是滤波后（`set_vehicle_states` 之后、`_control.setState` 之前）」的区别只落在速度与加速度上：

| 通道 | 滤波前注入（当前） | 滤波后注入 | 区别 |
|---|---|---|---|
| position x/y/z（0–2） | 原样进控制器 | 原样进控制器 | 无（不滤波） |
| heading（6） | 原样进控制器 | 原样进控制器 | 无（不滤波） |
| velocity vx/vy/vz（3–5） | 被 notch+低通抹平（阶跃→斜坡、高频衰减、停止有尾；默认关，需开 `MPC_VEL_LP`/`MPC_VEL_NF_FRQ`） | 原样直达 | 有 |
| acceleration | 无法直接注入（仅作速度微分附带产生） | 可直接注入，甚至制造「速度↔加速度不一致」故障 | 有（且仅滤波后可打） |
| 姿态 roll/pitch/yaw（7–9） | 原样直达 | 原样直达 | 无（状态不滤波） |

要点：1）只有速度通道有 before / after 区别，且仅在显式开启 `MPC_VEL_LP` / `MPC_VEL_NF_FRQ` 后才成立；默认（滤波关）时速度与位置一样原样直通。开启后：直流 bias 低通稳态增益为 1，前后稳态等价、只差瞬态；高频噪声/振荡则衰减明显。2）加速度是唯一「仅滤波后可直接注入」的维度。3）姿态与位置/航向一样无滤波，注入即达、无滤波记忆。

对净化（sanitize）的含义：要彻底净化速度通道，净化必须放在滤波**之前**（`set_vehicle_states` 顶部），否则 notch/低通的内部状态已被污染，会留下衰减余尾；位置/航向/姿态无滤波，净化放哪效果一样。

> **注**：本设计采用**滤波前**注入——忠实于「状态本身错误、控制器正常处理」的故障注入语义，并保留「控制器滤波如何抹平故障」这一可观测维度；只有需要直接打加速度、或研究滤波平滑作用时才改到滤波后。

### 5.4 速度滤波器说明（类型 / 参数 / 默认状态）

`set_vehicle_states()` 速度链路上的三个滤波器（成员声明 [MulticopterPositionControl.hpp:201-208](PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.hpp#L201)，条件启用逻辑 [MulticopterPositionControl.cpp:88-118](PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L88)）：

| 滤波器 | 类型 | 作用 | 参数（默认） | 默认状态 |
|---|---|---|---|---|
| `_vel_xy/z_notch_filter` | `math::NotchFilter` | 陷波（带阻）：挖掉单一结构/桨共振频率，防止控制器「追振」 | `MPC_VEL_NF_FRQ`（0）、`MPC_VEL_NF_BW`（5） | 关（frq=0） |
| `_vel_xy/z_lp_filter` | `AlphaFilter`（一阶低通 / 指数平滑） | 去高频估计噪声，减少电机抖 | `MPC_VEL_LP`（0） | 关（=0） |
| `_vel_deriv_xy/z_lp_filter` | `AlphaFilter` | 平滑「微分出来的加速度」（微分放大噪声） | `MPC_VELD_LP`（5 Hz） | **开** |

- **陷波 Notch**：只挖一个很窄的频段（结构 / 螺旋桨共振频率），其余频率保留；该频率每架机架不同，需悬停试飞实测后填 `MPC_VEL_NF_FRQ`，故默认关。
- **低通 Low-pass**：挖高频宽带噪声，代价是相位滞后；EKF（卡尔曼滤波）本身已平滑速度，再加低通会伤响应，故默认关。
- **微分低通**：加速度是速度的差分，差分放大高频噪声，几乎人人会遇到，故默认开（5 Hz）。

对故障注入的含义：默认配置下速度链路基本原样直通（notch / 低通关，只剩 5 Hz 加速度平滑），速度攻击与位置 / 航向一样「注入即达」；只有在显式调大 `MPC_VEL_LP` / `MPC_VEL_NF_FRQ` 后，§5.3 描述的「被滤波抹平、滤波记忆」现象才会出现。

> 参数定义：[multicopter_position_control_params.c:89/104/118/132](PX4-Autopilot/src/modules/mc_pos_control/multicopter_position_control_params.c#L89)。

## 6. 控制

链路接线（照 `attack_command` 既有套路）：`msg/CMakeLists.txt` 注册三个 `.msg`；`dds_topics.yaml` 加 `/fmu/in/state_attack_command`（订阅）与 `/fmu/out/state_attack_pos_status`、`/fmu/out/state_attack_att_status`（发布）；ROS2 侧直接用 `px4_msgs`。

```bash
# 位置北向偏置 +10 m（channel 0 = NAV_POS_X，type 1 = BIAS，a = 10）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 1, param: [10.0, 0.0, 0.0, 0.0]}" --once

# 高度欺骗：让控制器以为高度比实际高 10 m（NED z 向下，NAV_POS_Z 偏置 −10）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 2, type: 1, param: [-10.0, 0.0, 0.0, 0.0]}" --once

# 冻结位置：控制器始终认为位置不变（type 10 = FREEZE）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 10}" --once

# 横滚偏置 +15°（channel 7 = ATT_ROLL，type 1 = BIAS，a = 0.2618 rad）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 7, type: 1, param: [0.2618, 0.0, 0.0, 0.0]}" --once

# 航向欺骗到 90°（channel 6 = NAV_HEADING，type 2 = SPOOF，a = π/2）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 6, type: 2, param: [1.5708, 0.0, 0.0, 0.0]}" --once

# 清除 NAV_POS_X 通道
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 0}" --once
```

随时 / 不改代码语义（与总览 §6 完全一致）：开 = 发 `type=X`；改参数 = 再发一条同通道新参数；关 = 发 `type=NONE`。从该通道下一个控制器循环起生效（mc_pos_control ~100–250 Hz、mc_att_control 同理，近乎即时）。运行时只动 ROS2 命令，不重新编译、不改任何代码。

> **注**：`dds_topics.yaml` 里这两条状态 publication **不要加低 `rate_limit`**（现有 `attack_status` 用 10 Hz 是因为它是配置心跳）。`value` 是被污染值的动态过程（NOISE / DRIFT / REPLAY 逐拍变化），降采样会丢细节；给 50–100 Hz 或不设，让 ROS2 拿到接近全率。

## 7. 日志

### 7.1 登记

```cpp
add_optional_topic("state_attack_command");       // 事件型，经 DDS 动态进来，无静态 advertiser
add_topic("state_attack_pos_status", 10);             // 导航，100 Hz 快照（10 ms，含被污染状态）
add_topic("state_attack_att_status", 10);             // 姿态，100 Hz 快照（10 ms，含被污染姿态）
```

### 7.2 关键

**关键点：真值话题里看不到攻击**

这是本攻击与现有攻击在日志分析上的本质区别：1）现有攻击改 `sensor_gps` / 电机指令，这些话题本身就是被篡改的，ulog 直接记录到了攻击面。2）本攻击改的是控制器内部的临时状态，ulog 里的 `vehicle_local_position` / `vehicle_attitude` 仍是 EKF 真值，不含被污染值。若不额外记录，事后无法复原「控制器当时以为自己在哪、姿态是什么」。

因此两个状态话题的 `value[]` 数组：每次 `update()` 把每通道控制器实际所用值（即上一周期 `apply_position()` / `apply_attitude()` 写入的 `last_value`）发出去，未攻击通道回填真值，与真值话题并列记录。

时序注（一周期滞后）：两个模块的循环里 `update()`（内含 `publish_status()`）都在 `apply_position()` / `apply_attitude()` 之前调用，因此 status 的 `value[]` 恒等于上一周期 `apply_*` 写入的 `last_value`，相对本周期原始样本滞后一个控制周期（约 10 ms）。这是刻意取舍：命令先被 `update()` 消费、同一周期即生效，代价是 status 慢一拍。若要逐拍对齐，把 `update()` 挪到 `apply_*` 之后即可，但新攻击命令会晚一周期生效。

### 7.3 分析

四条时间线对齐：

| 时间线 | 内容 | 话题 | 是否默认记录 |
|---|---|---|---|
| 攻击侧 | 何时、哪通道、挂了什么攻击 | `state_attack_command` / `state_attack_pos_status` / `state_attack_att_status` | 已登记（§7.1） |
| 控制器所见侧 | 控制器实际输入（被污染） | `state_attack_pos_status.value` / `state_attack_att_status.value` | 已登记 |
| 真值侧（EKF） | EKF 融合输出（未污染） | `vehicle_local_position` / `vehicle_attitude` | 默认记录 |
| 反应侧 | 飞控在攻击下怎么飞 | `actuator_outputs` / `vehicle_*_groundtruth` | 默认记录 |

攻击效应 = 控制器所见侧 − 真值侧（逐字段偏差），叠在 `actuator_outputs` 上对齐。逐样本对齐靠 `timestamp_sample`（同一份 EKF 样本的采样时刻，§4.2），而非各自的发布 `timestamp`；所有时间戳均在 `hrt_absolute_time()` 微秒单调时钟上。种子语义与配对运行法同总览 §7.2。

## 8. 决策

| # | 决策 | 理由 |
|---|---|---|
| 1 | 独立命令通道 `state_attack_command` | 与 `attack_command` 隔离，可同时或独立注入两种攻击面 |
| 2 | 独立代码 `src/lib/state_attack/`，不链接 gz_bridge | 固件侧无法链接仿真层库；物理隔离 |
| 3 | 原语语义复用、代码独立 | 数学定义一致（原语分册 §4），构建产物分离 |
| 4 | 两个注入点（导航 / 姿态），各选唯一咽喉点 | 一处覆盖 7 导航字段、一处覆盖 3 姿态角 |
| 5 | 管理器实例化两次、共享一条命令话题 | 两模块不同 task，无法共享内存对象；同命令流广播使两表一致 |
| 6 | 姿态在欧拉角域攻击、再重构四元数 | 直接改 `q` 分量破坏单位模长 |
| 7 | 状态话题分导航 / 姿态两个 | 两模块各自发布，含各自「控制器所见」 |
| 8 | 只污染控制器消费者 | 天然「隔离开」，EKF / navigator / commander 不受影响 |
| 9 | 状态消息用 `value[ch]` 数组而非平铺字段 | 与 `attack_status` 的全 per-channel 数组一致；发布是循环，语义在注入点 |
| 10 | 状态消息带 `timestamp_sample` | 与真值话题逐样本对齐（§7.3） |
| 11 | 状态 DDS 不设低 `rate_limit` | 保留被污染值动态（§6 注） |
| 12 | 状态攻击只做点变换，不做时间序列原语（DELAY / REPLAY） | 点攻击已覆盖主要注入需求；时间序列原语需每通道历史缓冲、内存开销大，暂不实施（§10），要实施照仿真侧 gz_bridge `attack/` 库 |

## 9. 限制

| # | 事项 | 说明 |
|---|---|---|
| 1 | 攻击不残留 EKF | 只污染控制器输入，EKF 内部干净；攻击一停控制器立即回真值。模拟持久状态偏差需靠 EKF 输入攻击 |
| 2 | DROP 不可实施（退化为直通） | `vehicle_local_position` / `vehicle_attitude` 是控制器内部读取的标量字段，攻击必须返回一个值给控制器，「丢弃一条消息」的语义不存在，因此 DROP 无对应实现。要模拟「数据中断」应改用 FREEZE（保持上拍值），或到 EKF 输入侧（gz_bridge 层）做真正的 DROP |
| 3 | 只覆盖两个控制器消费者 | navigator / commander 的地图、geofence、failsafe 仍读真值，不模拟「全飞控导航解被替换」 |
| 4 | 姿态只攻击外环 | 速率内环读 `vehicle_angular_velocity`，不受姿态攻击直接影响（§5.2 注、§10） |
| 5 | 有效标志位不攻击 | `xy_valid` 等仍按真值；诱导控制器判无效 / 降级列扩展 |
| 6 | 角度通道不环绕归一 | `heading` / roll / pitch / yaw 当普通标量处理，大偏置可超出 [-π, π]；下游是否环绕取决于消费者实现 |
| 7 | 姿态欧拉重构有 gimbal lock | pitch 攻击逼近 ±90° 时欧拉↔四元数不唯一，大俯仰攻击需谨慎 |
| 8 | 两模块启动时间有微小偏差 | 各管理器相对自己收到命令的当下解析 `t0_us`，绝对启动时刻偏差 ≤ 1 控制循环；严格同步可改用命令 `timestamp` 或绝对时刻 |

## 10. 扩展

| 想扩展 | 改哪层 | 动什么 |
|---|---|---|
| 攻击角速度（速率内环） | B'（mc_rate_control） | `vehicle_angular_velocity.xyz` 加注入点，通道 `RATE_X/Y/Z` |
| 攻击有效标志位 | A'/C' | 通道加 `VALID` 类，翻转 `xy_valid` 等诱导控制器降级 |
| 攻击其他消费者（navigator / commander） | B'（新注入点） | 在对应模块读 `vehicle_local_position` 处加同款 hook |
| 全飞控导航解替换 | B'（EKF 输出） | 另起模块发布被污染状态并改所有消费者订阅（侵入大，暂列） |
| DELAY / REPLAY 导航 / 姿态 | A' | 未实施（本设计只做点变换）。要实施照仿真侧 gz_bridge `attack/` 库的时间序列原语设计：每通道历史环形缓冲 + REPLAY 快照（`kTimeSeriesCapacity=4096`） |
| 更友好 ROS2 封装 | ROS2 侧 | `x500_plus` 加 `state_attack_controller` 节点（照 `wind_injector` 模式） |

## 11. 文件清单

```text
PX4-Autopilot/src/lib/state_attack/
├── StateAttackPrimitive.hpp/.cpp   # Layer A'：原语（纯数学，复用原语分册语义）
└── StateAttackManager.hpp/.cpp     # Layer C'：指令解析 + 通道路由 + 状态发布（含 apply_position / apply_attitude）

PX4-Autopilot/src/modules/mc_pos_control/
└── MulticopterPositionControl.cpp/.hpp   # 改：持有 _state_attack；注入 apply_position()；update()

PX4-Autopilot/src/modules/mc_att_control/
└── mc_att_control_main.cpp / mc_att_control.hpp   # 改：持有 _state_attack；注入 apply_attitude()；update()

PX4-Autopilot/msg/
├── StateAttackCommand.msg          # uORB 消息（输入）
├── StateAttackPosStatus.msg        # uORB 消息（导航状态输出）
└── StateAttackAttStatus.msg        # uORB 消息（姿态状态输出）

PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml   # 加 3 条（DDS 桥）
PX4-Autopilot/src/modules/logger/logged_topics.cpp           # 加 3 条（ulog 登记）
PX4-Autopilot/msg/CMakeLists.txt                             # 注册三个 .msg
```

## 12. 结论

本设计在 PX4 固件的 mc_pos_control 与 mc_att_control 内新增一类与现有 gz_bridge 攻击物理隔离的「EKF 后状态攻击」：独立命令通道 `state_attack_command`、独立固件侧库 `src/lib/state_attack/`，在两个唯一咽喉点（`set_vehicle_states()` 之前、`Quatf q{v_att.q}` 之前）就地污染 EKF 输出的导航状态 7 字段（位置 / 速度 / 航向）与姿态 3 欧拉角（横滚 / 俯仰 / 偏航），使控制器在 EKF 无感知的情况下信任被污染状态。姿态在欧拉角域攻击后重构单位四元数，规避归一化问题。被污染状态经两个状态话题显式记入 ulog，与 `vehicle_local_position` / `vehicle_attitude` 真值并列，配合 `hrt_absolute_time()` 统一时钟与确定性种子，实现「控制器所见 vs 真值 vs 反应」三线可复现、可对齐分析；运行时通过命令消息随时开 / 关 / 改参数，不改任何代码。
