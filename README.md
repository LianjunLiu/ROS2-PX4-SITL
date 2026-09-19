# ROS2-PX4-SITL

PX4 SITL（x500_plus + FAST-LIVO2）上的**无人机安全研究平台**：在仿真中对 GPS、电机、EKF 输出状态做可配置攻击注入，配合 LiDAR-惯性-视觉里程计与风速注入，研究攻击对飞控的影响与防御。

> ⚠️ **仅供授权安全研究使用。** 本平台在仿真环境中研究无人机攻击注入与防御，请勿用于真实飞行器或任何未经授权的目标。

## 概览

| 组件 | 说明 |
|---|---|
| **PX4-Autopilot v1.17.0** | SITL 飞控固件，含攻击注入改动（`make px4_sitl gz_x500_plus`） |
| **x500_plus 模型** | x500 + RealSense D455F + Livox MID-360S（gz-sim Harmonic） |
| **FAST-LIVO2** | LiDAR-惯性-视觉里程计，桥接给 PX4 EKF2 |
| **攻击注入系统** | 13 种原语 × 10 通道，作用于 EKF 输入侧（传感器 / 执行器） |
| **状态攻击系统** | 作用于 EKF 输出侧（导航状态 / 姿态），独立命令通道 |
| **风速注入** | 运行时向 gz-sim 注入可随时更改的风 |

## 攻击注入

三层架构，代码在 `PX4-Autopilot/src/modules/simulation/gz_bridge/attack/`：

- **Layer A（原语库）** `AttackPrimitive` — 纯数学、零 PX4/gz/uORB 依赖，13 种原语；
- **Layer B（注入点）** — GPS 6 通道 + 电机 4 通道，各一行调用；
- **Layer C（管理器）** `AttackManager` — 指令解析、路由、状态发布。

13 种原语（按类别）：完整性 `BIAS / SPOOF / NOISE / SCALING / DRIFT / OSCILLATION / RANDOM_WALK / QUANTIZE / CLAMP`，可用性 `FREEZE / DROP`，时间序列 `DELAY / REPLAY`。

| 攻击面 | 对象 | 注入点 | 通道 |
|---|---|---|---|
| 传感器攻击 | GPS（位置 / 速度） | `GZBridge::navSatCallback` → `sensor_gps` | 6 |
| 执行器攻击 | 4 个电机 | `GZMixingInterfaceESC::updateOutputs` → `gz.msgs.Actuators` | 4 |
| 状态攻击（EKF 输出侧） | 导航状态 + 姿态 | `mc_pos_control` + `mc_att_control` | 10 |

指令经 `ROS2 → DDS → uORB` 下达，全过程记入 ulog，时间戳统一于 `hrt_absolute_time()` 微秒单调时钟，配合确定性种子与配对运行实现可复现、可对齐分析。详见 `Docs/`。

## 仓库结构

本仓库是「伞仓」：自身只存文档、配置与 10 个子模块的指针，代码分散在各子模块。

| 子模块 | 来源 | 说明 |
|---|---|---|
| `PX4-Autopilot` | fork（`ROS2` 分支） | 飞控固件 + 攻击注入 |
| `src/px4_msgs` | fork（`ROS2` 分支） | 自定义 `AttackCommand` / `AttackStatus` 消息 |
| `src/FAST-LIVO2` | fork（`ROS2` 分支） | LiDAR-惯性-视觉里程计 |
| `src/livox_ros_driver2` | fork（`ROS2` 分支） | 雷达 ROS2 驱动 |
| `src/px4_ros_com` | 上游 PX4 | PX4 ROS2 接口 |
| `src/rpg_vikit` | 上游 | FAST-LIVO2 依赖 |
| `Livox-SDK2` | 上游 | 雷达 SDK |
| `Sophus` | 上游（tag 1.22.10） | 李群库 |
| `Dynamic_World_Generator` | 上游 | 动态地图生成 |
| `Micro-XRCE-DDS-Agent` | 上游 | uXRCE-DDS 桥 |

此外 `PX4-Autopilot/Tools/simulation/gz` 是一个**嵌套子模块**，指向 fork `ROS2-PX4-gazebo-models`（含 x500_plus 模型与 Penglai 世界）。

克隆：

```bash
git clone --recurse-submodules git@github.com:LianjunLiu/ROS2-PX4-SITL.git
# 已克隆但缺子模块时：
git submodule update --init --recursive
```

## 环境

- Ubuntu 22.04 + ROS2 Humble + gz-sim Harmonic
- 依赖安装（含 gz-harmonic、NuttX 工具链等）：

```bash
bash PX4-Autopilot/Tools/setup/ubuntu.sh
```

> 仅跑 SITL 无需 ESP32 Xtensa 工具链（那是 `esp32` 板目标专用）。

### 版本

| 组件 | 版本 |
|---|---|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| gz-sim | 8.15.0（Harmonic） |
| Python | 3.10.12 |
| cmake | 3.22.1 |
| GCC | 11.4.0 |
| PX4-Autopilot | v1.17.0 + 攻击注入提交（`v1.17.0-1-g0d60a3e`） |
| Micro-XRCE-DDS-Agent | v3.0.2 |
| Sophus | 1.22.10（子模块钉在发行标签） |
| 其余子模块 | 见各自 gitlink（子模块指针即精确版本） |

## 构建

> **路径约定**：本仓库顶层目录为 `ROS2-PX4-SITL`。部分命令与 `Docs/` 内文档里写作 `/home/liu/Desktop/ROS2`（早期名称），两者指向同一目录，可互换理解。

```bash
cd /home/liu/Desktop/ROS2-PX4-SITL

# 1) ROS2 工作区（只编 src/；切勿裸 `colcon build`，否则会连带编译 PX4 固件）
colcon build --base-paths src --symlink-install

# 2) PX4 固件（SITL）
cd PX4-Autopilot
make px4_sitl
```

## 运行

三个终端，按顺序：

```bash
# 终端 1：启动 SITL + gz-sim（x500_plus 模型，Penglai 世界，起飞点偏移）
cd /home/liu/Desktop/ROS2-PX4-SITL/PX4-Autopilot
PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
```

```bash
# 终端 2：启动 uXRCE-DDS 代理（ROS2 ↔ PX4 桥）
MicroXRCEAgent udp4 -p 8888
```

```bash
# 终端 3：启动 x500_plus 桥接（传感器 / TF / 深度修复 / 风速注入 / LIVO→PX4）
source install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
```

## 攻击用法

### 传感器 / 执行器攻击（EKF 输入侧）

```bash
# GPS 纬度欺骗到 31.23°（channel 0 = GPS_LAT，type 2 = SPOOF）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand \
  "{channel: 0, type: 2, param: [31.23, 0.0, 0.0, 0.0]}" --once

# 延迟 1 秒开始、持续 5 秒的 GPS 纬度偏置（type 1 = BIAS，+0.001°）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand \
  "{channel: 0, type: 1, param: [0.001, 0.0, 0.0, 0.0], t0_us: 1000000, t1_us: 5000000}" --once

# GPS 噪声（type 3 = NOISE，σ=0.0001°；seed 覆盖默认种子）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand \
  "{channel: 0, type: 3, param: [0.0001, 0.0, 0.0, 0.0], seed: 42}" --once

# 关闭该通道攻击
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 0}" --once
```

### 导航状态 / 姿态攻击（EKF 输出侧）

命令话题独立于上面的传感器 / 执行器攻击（`/fmu/in/state_attack_command`），作用于 EKF 融合之后的导航状态与姿态。仅支持点变换原语（`DELAY` / `REPLAY` / `DROP` 暂未实施）。

```bash
# 位置北向偏置 +10 m（channel 0 = NAV_POS_X，type 1 = BIAS）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand \
  "{channel: 0, type: 1, param: [10.0, 0.0, 0.0, 0.0]}" --once

# 高度欺骗：让控制器以为比实际高 10 m（NED z 向下，NAV_POS_Z 偏置 −10）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand \
  "{channel: 2, type: 1, param: [-10.0, 0.0, 0.0, 0.0]}" --once

# 横滚偏置 +15°（channel 7 = ATT_ROLL，type 1 = BIAS，0.2618 rad）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand \
  "{channel: 7, type: 1, param: [0.2618, 0.0, 0.0, 0.0]}" --once

# 航向欺骗到 90°（channel 6 = NAV_HEADING，type 2 = SPOOF，π/2）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand \
  "{channel: 6, type: 2, param: [1.5708, 0.0, 0.0, 0.0]}" --once

# 清除该通道
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 0}" --once
```

通道编号：

- `attack_command`：`0–5` = GPS 纬度 / 经度 / 高度 / 北速 / 东速 / 地速，`6–9` = 电机 0–3。
- `state_attack_command`：`0–6` = 导航位置 x/y/z、速度 vx/vy/vz、航向，`7–9` = 横滚 / 俯仰 / 偏航。

`type` 枚举见 `Docs/attack_primitive_reference.md`。

## 风速注入

```bash
ros2 run x500_plus wind_injector
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 3.0, y: 0.0, z: 0.0}" --once   # 3 m/s 朝 +X
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" --once   # 停风
```

## 文档

| 文档 | 内容 |
|---|---|
| `Docs/ROS2-PX4-SITL.md` | 环境搭建与运行全过程 |
| `Docs/attack_injection_design.md` | 攻击注入系统总览（架构 / 协议 / 注入点 / 日志） |
| `Docs/attack_primitive_reference.md` | 13 种原语公式、参数、边界 |
| `Docs/attack_manager_reference.md` | 管理器指令解析、种子、状态 |
| `Docs/state_attack_design.md` | post-EKF 状态攻击设计 |
| `Docs/fast_livo2_px4_integration.md` | FAST-LIVO2 坐标系、外参、PX4 桥接 |
| `Docs/git_commit_guide.md` | 提交改动到 GitHub 的流程指南（三层 git 结构） |

## 许可证

本仓库自身的原创内容（`README`、`Docs/`、`src/x500_plus/`、配置文件）采用 [MIT 许可证](LICENSE)。各子模块遵循其上游许可证（BSD-3-Clause / GPL-2.0 / MIT / Apache-2.0 等），以各子模块仓库内的 LICENSE 文件为准。

## 授权声明

本项目仅用于**授权的无人机安全研究**——在仿真环境中复现、评估攻击注入，为防御与检测研究提供可复现的实验平台。任何面向真实飞行器或未经授权系统的使用均与本项目无关，由使用者自行承担全部责任。
