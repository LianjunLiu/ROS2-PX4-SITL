# data

存放 SITL 实验的 rosbag 数据（GB 级，网盘下载或运行时生成），**不上传仓库**——顶层 `.gitignore` 忽略本目录所有内容，仅保留这份 README。

## 下载数据

`Red_Sculpture` 示例 bag 从 Google Drive 下载：

<https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA>

把 `.bag` 文件下载到本目录（`data/`）。

## 转换格式

用 `rosbags` 把 `.bag` 转成 rosbag2 格式：

```bash
pip install rosbags
rosbags-convert --src Red_Sculpture.bag --dst Red_Sculpture
```

转换产物：

- `Red_Sculpture.bag` —— 原始 bag 文件（下载所得）
- `Red_Sculpture/` —— rosbag2 目录格式（`*.db3` 消息数据 + `metadata.yaml` 元数据）

## 修正元数据

`rosbags-convert` 生成的 `Red_Sculpture/metadata.yaml` 有两处需要手改，否则 `ros2 bag play` 会因 QoS / 类型解析失败：

1. 每个话题的 `offered_qos_profiles` 改为空字符串 `''`；
2. `/livox/lidar` 的 `type` 改回 `livox_ros_driver2/msg/CustomMsg`（自定义消息，`rosbags` 转换时类型名可能被改写）。

修正后的关键内容如下：

```yaml
rosbag2_bagfile_information:
  duration:
    nanoseconds: 101866084793
  storage_identifier: sqlite3
  topics_with_message_count:
  - message_count: 20799
    topic_metadata:
      name: /livox/imu
      offered_qos_profiles: ''
      serialization_format: cdr
      type: sensor_msgs/msg/Imu
  - message_count: 1020
    topic_metadata:
      name: /livox/lidar
      offered_qos_profiles: ''
      serialization_format: cdr
      type: livox_ros_driver2/msg/CustomMsg
  - message_count: 1020
    topic_metadata:
      name: /left_camera/image
      offered_qos_profiles: ''
      serialization_format: cdr
      type: sensor_msgs/msg/Image
```

三个话题对应 FAST-LIVO2 的 LiDAR-惯性-视觉输入：`/livox/imu`（`Imu`）、`/livox/lidar`（`CustomMsg`）、`/left_camera/image`（`Image`）。

## 回放测试

> 路径约定：下文命令中的顶层目录写作 `ROS2-PX4-SITL`，与早期目录名 `ROS2` 指向同一位置。

### 终端 1：播放 bag

```bash
ros2 bag play /home/liu/Desktop/ROS2-PX4-SITL/data/Red_Sculpture      # 立即播放
ros2 bag play -p /home/liu/Desktop/ROS2-PX4-SITL/data/Red_Sculpture   # 暂停启动，按空格开始
```

> 查看话题 / 时长 / 大小：`ros2 bag info /home/liu/Desktop/ROS2-PX4-SITL/data/Red_Sculpture`

### 终端 2：启动 FAST-LIVO2

```bash
source /home/liu/Desktop/ROS2-PX4-SITL/install/setup.bash
ros2 run fast_livo fastlivo_mapping --ros-args \
  --params-file /home/liu/Desktop/ROS2-PX4-SITL/install/fast_livo/share/fast_livo/config/avia.yaml \
  --params-file /home/liu/Desktop/ROS2-PX4-SITL/install/fast_livo/share/fast_livo/config/camera_pinhole.yaml
```

### 终端 3：打开 RViz

```bash
rviz2 -d /home/liu/Desktop/ROS2-PX4-SITL/install/fast_livo/share/fast_livo/rviz_cfg/fast_livo2.rviz
```
