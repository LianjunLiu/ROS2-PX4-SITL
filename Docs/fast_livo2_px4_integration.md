# FAST-LIVO2 坐标系与外参

> FAST-LIVO2（LiDAR-惯性-视觉里程计）在 x500_plus 仿真平台上的坐标系、相机外参与 PX4 桥接设计。全文围绕一个问题——雷达点如何正确投成彩色像素，以及之前为什么投不进。关联代码：`src/FAST-LIVO2/config/sim_x500_plus.yaml`、`config/camera_x500_plus.yaml`、`src/x500_plus/x500_plus/livo_to_px4.py`、`PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.sdf`。版本 v5 · 状态 RGB 点云已修复（§三）· 已知问题 10° 俯仰偏置（§二）。

一个雷达点变成一颗彩色像素，要穿过四套坐标系、三段变换：

```text
  世界 camera_init        IMU mid360s = 雷达        相机 vikit          像素
 …●─────────────────────▶●─────────────────────▶●─────────────▶●
    对齐 §二                 外参 §三                内参 §四
```

三段变换各占一节：对齐（§二，世界 ← IMU）、外参（§三，雷达 → 相机，本次点云为空的病根）、内参（§四，相机 → 像素）。另有两条补充链路：雷达与 IMU 共帧共位（外参单位阵，§一）；世界系喂给 PX4 还差一次翻 z（§五）。

## 一、坐标系

### 1.1 约定

| 坐标系 | 记号 / frame_id | 约定 |
|---|---|---|
| 世界系 | `camera_init` | LIVO 的固定世界系；z 上、航向任意；历史命名，非相机帧 |
| 机体系 | `base_link` | PX4 身体 FRD，z 下 |
| IMU 系 = 雷达系 | `mid360s_link` | FAST-LIVO2 的「body」；雷达与 IMU 共帧共位 |
| 相机系（gz-sim） | — | 光轴 +X、左 +Y、上 +Z |
| 相机系（vikit） | — | 光轴 +Z、右 +X、下 +Y（FAST-LIVO2 针孔模型） |

两套相机系是本文题眼——gz-sim 与 FAST-LIVO2 对「相机朝哪」的定义差 90°（§3.2）。

### 1.2 几何

x500_plus 在基础 x500 上 `include merge` 两个传感器 link（[model.sdf](../PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.sdf)）：

| 部件 | 相对 `base_link` | 说明 |
|---|---|---|
| `realsense_link` | `0.15 0 0.018 0 0.2618 0` | 相机前移 0.15 m、低头 15° |
| `realsense_rgb` | link 内 `0.0126 0 0` | 相机光心 x = 0.15 + 0.0126 = **0.1626 m** |
| `mid360s_link` | `0 0 0.11 0 0.1745 0` | 雷达低头 10°、质心上方 0.11 m |
| `mid360s_lidar` / `mid360s_imu` | link 内无 pose | 雷达与 IMU 共帧共位，一起低头 10° |

由此得两个外参来源：**相对俯仰** = 相机比雷达多低头 5°（进 `Rcl` 记 `Ry(-5°)`，§3.3）；**相对平移** = 雷达原点 − 相机光心（进 `Pcl`，§3.3）。订阅话题：`imu_topic=/mid360s/imu`、`lid_topic=/mid360s/points/points`、`img_topic=/realsensed455f/color/image_raw`。

## 二、对齐

### 2.1 重力

`gravityAlignment()`（[LIVMapper.cpp](../src/FAST-LIVO2/src/LIVMapper.cpp)）：

```cpp
if (!p_imu->imu_need_init && !gravity_align_finished)   // 只进一次
{
    V3D ez(0,0,-1), gz(_state.gravity);
    G_q_I0 = FromTwoVectors(gz, ez);        // 把实测重力转到 (0,0,-1)
    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    gravity_align_finished = true;          // 冻住
}
```

- 输入只有 IMU 实测重力（`_state.gravity`，由 `mean_acc` 初始化），不知道雷达/相机的 10°/15° 偏转角；
- 只做一件事：把世界系 z 转到朝上，然后一次性冻住；此后世界系恒定；
- 不能持续对齐：世界系必须是惯性系，跟着重力/机体转就没有绝对参考了。

> **注**：重力对齐管「世界系 z 朝上」，外参管「倾斜量测投到 IMU 帧」——两套正交机制。相机投影用的是外参，与重力对齐是否开启无关。

### 2.2 偏置

整机两个 IMU：

| IMU | 谁在用 | 位置 |
|---|---|---|
| 飞控 IMU（x500 自带） | PX4 自己的 EKF | 质心附近 |
| `mid360s_imu`（x500_plus 新增） | FAST-LIVO2 | 雷达处 `(0,0,0.11)`，俯仰 10° |

FAST-LIVO2 只用 `mid360s_imu`，故 LIVO 的「body」= 雷达共装 IMU 系，不是飞控机体/质心。`mid360s_imu` 相对 `base_link` 俯仰 10°，重力对齐扳平了世界系，却把这个 10° 留在了姿态里：LIVO 姿态 = `rot_end` = IMU → 世界，相对机体恒带 10° 俯仰偏置（静偏置，动态时原样存在）。

影响分两类，泾渭分明：

| 融合内容 | 受影响吗 | 原因 |
|---|---|---|
| 位置（`EKF2_EV_CTRL` bit0/1，当前方案） | **否** | 位置是点，IMU 朝向不影响其坐标 |
| 姿态 / 偏航（bit3，未开） | **是（真错 10°）** | PX4 把发布姿态当机体姿态 |

> **已知问题（暂不修复）**：将来若开偏航融合，把这 −10° 折进 `BODY_ROT_RPY`（约 `(π, -0.1745, 0)`，符号实测标定）；IMU 在质心上方 0.11 m，如需可填 `EKF2_EV_POS_Z` 补偿杆臂。

## 三、外参

现象：SITL 里 `/cloud_registered` 彩色点云为空（`colored = 0`），而同一套算法跑真实 ROSBAG 能正常出彩色点云——据此锁定是 SITL 的外参/参数问题，而非算法本身。诊断输出 `[ Publish ] RGB points: total 35863, front 33671, inframe 0`（94% 的点在相机前方，却 0 个投进画面），指向相机朝向整体错位。

### 3.1 约定

[vio.cpp](../src/FAST-LIVO2/src/vio.cpp) 把外参组装成「世界 → 相机」变换：

```text
p_cam = Rcl * p_lidar + Pcl
```

- `Rcl`：雷达系 → 相机系的旋转；
- `Pcl`：雷达原点在相机系下的坐标（`t_cl`），非「相机在雷达系」（`t_lc`）；`Pcl = -Rcl·t_lc`；
- 此处的「相机系」是 vikit 针孔模型系（光轴 +Z）——点最终投到这个系再算像素（§四）。

### 3.2 根因

| | 光轴 | x | y | z |
|---|---|---|---|---|
| gz-sim `camera` | **+X**（前） | 前 | 左 | 上 |
| vikit 针孔模型 | **+Z**（前） | 右 | 下 | 前 |

两系差一次固定旋转：

```text
           gz-sim → vikit
R_gz2vik = [ 0  -1  0 ]
           [ 0   0 -1 ]
           [ 1   0  0 ]
```

**真实数据集里这个旋转本就带在 `Rcl` 里**（如 `avia.yaml` 的 `Rcl ≈ [[0,-1,0],[0,0,-1],[1,0,0]]`，因 livox 雷达 +X 前向、ROS 相机 +Z 前向，标定外参天然含此 90°）。SITL 旧配置只写相对俯仰 `Rcl = Ry(-5°)`，**漏掉 90°**，于是 FAST-LIVO2 以为相机朝 +Z、实际朝 +X，相机姿态错 90°，所有点都投不进画面——即诊断输出 `front 33671, inframe 0` 的直接原因。

### 3.3 推导

相机比雷达多俯仰 5°（相机 −15°、雷达 −10°），gz-sim 系里相对旋转为 `Ry(-5°)`；再转到 vikit 系：

```text
Rcl = R_gz2vik · Ry(-5°)

Ry(-5°) = [ 0.996195  0  -0.087156 ]
          [ 0         1   0         ]
          [ 0.087156  0   0.996195 ]

=> Rcl = [ 0        -1        0        ]
         [ -0.087156 0       -0.996195 ]
         [ 0.996195  0       -0.087156 ]
```

自检（雷达前/右/上三点各投到哪）：

| 雷达系点 | `Rcl·p` | 含义 |
|---|---|---|
| 前 `(d,0,0)` | `(0, -0.0872d, 0.996d)` | vikit 里 5° 朝上、几乎全在光轴 +Z |
| 右 `(0,-d,0)` | `(d, 0, 0)` | vikit +X = 右 |
| 上 `(0,0,d)` | `(0, -0.996d, -0.087d)` | vikit −Y = 上 |

`Pcl`（雷达原点在 vikit 相机系）分两步：

```text
p_base  = (0 - 0.1626, 0, 0.11 - 0.018) = (-0.1626, 0, 0.092)   // 相机在 x=0.1626
p_gzcam = Ry(-15°) · p_base              = (-0.1809, 0, 0.0468)  // 相机系俯仰 15°
Pcl     = R_gz2vik · p_gzcam             = (0, -0.0468, -0.1809)
```

物理意义：雷达在相机正后方约 0.181 m、上方约 0.047 m（vikit 里「后」=−Z、「上」=−Y）。

**最终配置**（[sim_x500_plus.yaml](../src/FAST-LIVO2/config/sim_x500_plus.yaml) `extrin_calib`）：

```yaml
Rcl: [0.0, -1.0, 0.0,
      -0.087156, 0.0, -0.996195,
      0.996195, 0.0, -0.087156]
Pcl: [0.0, -0.04678, -0.18087]
```

### 3.4 排除

| 候选 | 判定 | 结论 |
|---|---|---|
| 内参 `fx = 674` | 由 87° HFOV 推出，正确（§四） | 排除 |
| `Pcl` 符号反 | 只造成 ~0.3 m 平移，不足以 `inframe = 0` | 排除 |
| `Rcl` 缺 90° | 相机姿态错 90°，点全投不进 | **根因** |

> **注**：`Pcl` 符号错只会造成平移误差、不至于 `inframe = 0`；致命的是 `Rcl` 缺 90°。两个一起修，才能既「有颜色」又「位置准」。

## 四、内参

[config/camera_x500_plus.yaml](../src/FAST-LIVO2/config/camera_x500_plus.yaml)：

```yaml
cam_model: Pinhole
cam_width: 1280      cam_height: 800
cam_fx: 674.4439     cam_fy: 674.4439     cam_cx: 640.0     cam_cy: 400.0
cam_d0..d3: 0.0      # 无畸变（gz-sim 相机是纯针孔）
```

- 像素 = `(u, v) = (fx·x/z + cx, fy·y/z + cy)`，`(x,y,z)` 是 vikit 相机系（+Z 光轴）坐标——这正是 §3.2 必须先转到 +Z 光轴系的原因；
- `fx = fy`（方形像素）、`cx/cy` 居 1280×800 中心；
- 内参来自 gz-sim 相机 FOV：`fx = (W/2) / tan(HFOV/2) = 640 / tan(43.5°) ≈ 674`，与 `model.sdf` 的 `horizontal_fov 1.5184 rad`（87°）一致——内参本身正确，从来不是点云为空的原因。

## 五、桥接

### 5.1 翻转

FAST-LIVO2 **不发布** `px4_msgs/VehicleOdometry`，只发布 SLAM 原生话题（`/aft_mapped_to_init`、`/path`、遗留 `/mavros/vision_pose/pose`）。ROS2 + PX4 栈要的原生话题是 `fmu/in/vehicle_visual_odometry`。桥接节点 [livo_to_px4.py](../src/x500_plus/x500_plus/livo_to_px4.py) 订阅 `/aft_mapped_to_init`，做一次固定旋转后发布 `VehicleOdometry`：

- LIVO 世界系 z 上、航向任意（FLU）；PX4 `POSE_FRAME_FRD` z 下、航向任意（FRD）；
- 两者差一次固定 180°（绕 x：`(x,y,z)→(x,-y,-z)`），即 `BODY_ROT_RPY = (π,0,0)`；
- 「任意恒定航向」正是 `POSE_FRAME_FRD` 的语义（constant arbitrary heading offset from True North），由 EKF2 自估，节点**不做**航向对齐。

节点还负责：时间戳 ns→µs、姿态用 Hamilton 顺序 `(w,x,y,z)`、速度/角速度填 NaN（不融合）、协方差给下界 `position_variance = 0.05 m²`。

> **注**：默认 `BODY_ROT_RPY = (π,0,0)` 假设 LIVO body 轴「x 朝前」。若实测方向反了：SITL 悬停后给前向指令，对比真值 `/model/x500_plus_0/odometry`，镜像/反向则调 roll/pitch/yaw 直到方向一致。

### 5.2 使能

`vehicle_visual_odometry` 的 uORB 订阅默认存在，但 EKF2 **默认不融合**（`EKF2_EV_CTRL` 默认 0）：

```text
EKF2_EV_CTRL    = 3    # bit0 水平位置 + bit1 垂直位置；要速度=7，要偏航=15
EKF2_EV_POS_X/Y/Z       # VI 传感器焦点在机体系的位置（杆臂），当前可先 0
```

本方案用 `vehicle_visual_odometry`（相对帧，SLAM/VIO 用），非 `vehicle_mocap_odometry`（绝对坐标，动捕用）。

## 六、参数

### 6.1 传感器

| 组 | 参数 | 值 | 意义 |
|---|---|---|---|
| `common` | `img_en` / `lidar_en` | `1` / `1` | 都开 = LIVO 模式（LIO+VIO 交替） |
| `extrin_calib` | `extrinsic_T` / `extrinsic_R` | `[0,0,0]` / 单位阵 | 雷达↔IMU，共帧共位 |
| | `Rcl` / `Pcl` | §3.3 | 雷达→相机外参（本次修复核心） |
| `preprocess` | `lidar_type` | `8`（MID360S） | 雷达型号，决定点解析 |
| | `scan_line` | `40` | 竖直线数，与 gpu_lidar 40 线一致 |
| | `blind` | `0.8` | 盲区半径（m），< 0.8 m 的点丢弃（机身自遮挡） |
| | `point_filter_num` | `2` | 每隔 2 点取 1（`i % N == 0`，50% 降采样） |
| | `filter_size_surf` | `0.5` | LIO 面点体素降采样 leaf size（m） |
| `vio` | `patch_size` / `patch_pyrimid_level` | `8` / `4` | 光度 patch 边长 / 图像金字塔层数 |
| | `exposure_estimate_en` | `true` | 在线估曝光（抗 SITL 光照漂移） |
| `imu` | `acc_cov` / `gyr_cov` | `0.5` / `0.3` | 加计/陀螺量测噪声协方差 |
| | `b_acc_cov` / `b_gyr_cov` | `1e-4` | 零偏随机游走协方差 |
| `lio` | `voxel_size` / `max_layer` | `0.5` / `2` | 体素地图分辨率 / octree 层数 |
| | `dept_err` / `beam_err` | `0.02` / `0.05` | 点到平面 ICP 深度/光束误差权重 |
| | `min_eigen_value` | `0.0025` | 退化检测最小特征值 |
| `uav` | `gravity_align_en` | `true` | 重力对齐（§2.1） |
| `publish` | `dense_map_en` | `true` | 发布彩色稠密地图 `/cloud_registered` |
| | `pub_scan_num` | `1` | 每 N 帧发布一次 |
| | `blind_rgb_points` | `0.0` | RGB 点盲区（0 = 不去除近处点） |

**几个值的来历**：

- `scan_line = 40`、`lidar_type = 8`：与 gpu_lidar 的 40 线、MID360S 点格式严格对应，否则点按错误的环形结构去畸变。
- `blind = 0.8`：x500 轴距 ~0.5 m，0.8 m 内的点多为机身/桨/地面杂点，直接剔除。
- `filter_size_surf = 0.5`、`voxel_size = 0.5`：SITL 室内 1800×40 点云，0.5 m 体素把 3.6 万原始点压到 ~2000 有效面点，够 ICP 收敛又省算力。
- `exposure_estimate_en = true`：gz-sim 相机无真实曝光，画面亮度随场景变，在线估曝光稳住光度误差。

### 6.2 相机

见 §四。`cam_fx/fy` 由 gz-sim 相机 HFOV 决定，`cam_cx/cy` 是图像中心，`cam_d0..d3 = 0` 因 gz-sim 是纯针孔无畸变。

## 七、结论

一条链路、四个坐标系、三段变换：LIVO 输出固定惯性系 `camera_init`（z 上、航向任意、发布 IMU 位姿），重力对齐一次性扳平世界系并冻结；雷达点到像素的关键在外参 `Rcl = R_gz2vik · Ry(-5°)`——必须含 gz-sim(+X 光轴)→vikit(+Z 光轴) 的 90° 约定旋转，`Pcl` 为雷达原点在 vikit 相机系的坐标，二者一起修好才恢复彩色点云；内参 `fx ≈ 674` 由 87° HFOV 推出、本身正确；世界系再经一次翻 z（FRD）桥接给 PX4，航向交 EKF2、另需开 `EKF2_EV_CTRL`。遗留一个 10° 俯仰静偏置——只影响姿态融合、不影响位置融合，当前不修。
