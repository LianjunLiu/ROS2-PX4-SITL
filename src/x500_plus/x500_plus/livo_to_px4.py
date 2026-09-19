#!/usr/bin/env python3
"""
LIVO → PX4 视觉里程计桥接节点。

把 FAST-LIVO2 的 LIO 里程计转成 PX4 EKF2 能融合的 vehicle_visual_odometry：

  FAST-LIVO2 在 /aft_mapped_to_init (nav_msgs/Odometry) 发布位姿，坐标系是
  camera_init —— 重力对齐后 z 朝上、航向 = 相机初始朝向（任意航向）。

  PX4 EKF2 通过 ROS2 桥订阅 fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry)。
  我们用 POSE_FRAME_FRD（Forward-Right-Down，z 朝下，允许「任意恒定航向偏移」），
  航向对齐交给 EKF2 自己估，本节点只需做一次固定的坐标翻转。

标定 BODY_ROT_RPY：
  默认 (pi, 0, 0)，即绕 x 轴转 180°，把 camera_init（Forward-Left-Up）
  翻到 FRD（Forward-Right-Down）。若你的 IMU 安装轴不是「x 朝前」，
  在 SITL 里起飞悬停、给前向俯仰指令，看 PX4 里无人机是否真的朝前动：
  镜像 / 反向就调整这里的 roll/pitch/yaw（单位 rad）。

PX4 侧还需手动开启（默认关）：
  EKF2_EV_CTRL = 3      （bit0 水平位置 + bit1 垂直位置）
  EKF2_EV_POS_X/Y/Z     （VI 传感器焦点在机体系的位置，杆臂补偿）

已知问题（暂不修复）：
  mid360s_imu 与雷达共装在 mid360s_link 里，相对 base_link 俯仰 10°（model.sdf 中
  pitch 0.1745 rad，无 roll/yaw）。因此 LIVO 发布的姿态（IMU→世界）相对机体恒带
  10° 俯仰偏置，静止与动态时都原样存在（是安装偏转，不是动态误差）：
    - 位置融合（EKF2_EV_CTRL bit0/1）：不受影响 —— 位置是点，IMU 朝向不影响其坐标。
    - 姿态/偏航融合（bit3）：会真错 10°。将来若开启，需把这 -10° 折进 BODY_ROT_RPY
      （约 (pi, -0.1745, 0)，具体符号以实测标定为准）。
  另：IMU 位于质心上方 0.11m（base_link 下 (0,0,0.11)），如需可填 EKF2_EV_POS_Z 做杆臂补偿。
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node

# 订阅 FAST-LIVO2 的 LIO 里程计
LIVO_ODOM_TOPIC = '/aft_mapped_to_init'
# 发布到 PX4 的 ROS2 桥接话题（uXRCE-DDS → uORB vehicle_visual_odometry）
PX4_TOPIC = 'fmu/in/vehicle_visual_odometry'

# camera_init → FRD 的固定旋转（roll, pitch, yaw, 单位 rad）
BODY_ROT_RPY = (math.pi, 0.0, 0.0)

# LIVO 不输出协方差，这里给常数下界
POSITION_VARIANCE = 0.05      # m^2   (~0.22 m 标准差)
ORIENTATION_VARIANCE = 0.05   # rad^2


def rpy_to_quat(roll, pitch, yaw):
    """欧拉角 → 四元数，返回 (w, x, y, z)。"""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def quat_multiply(q1, q2):
    """Hamilton 四元数乘法 q1 ⊗ q2，输入/输出均为 (w, x, y, z)。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def rotate_vector(q, v):
    """用四元数 q=(w,x,y,z) 主动旋转向量 v=(x,y,z)，返回旋转后的向量。"""
    q_conj = (q[0], -q[1], -q[2], -q[3])
    v_q = (0.0, v[0], v[1], v[2])
    r = quat_multiply(quat_multiply(q, v_q), q_conj)
    return (r[1], r[2], r[3])


class LivoToPx4(Node):
    def __init__(self):
        super().__init__('livo_to_px4')
        self.q_body_rot = rpy_to_quat(*BODY_ROT_RPY)

        self.publisher = self.create_publisher(VehicleOdometry, PX4_TOPIC, 10)
        self.create_subscription(Odometry, LIVO_ODOM_TOPIC, self.on_odometry, 10)
        self.get_logger().info(f'livo_to_px4: {LIVO_ODOM_TOPIC} -> {PX4_TOPIC}')

    def on_odometry(self, msg):
        # 位置：固定旋转 camera_init -> FRD
        p = msg.pose.pose.position
        px, py, pz = rotate_vector(self.q_body_rot, (p.x, p.y, p.z))

        # 姿态：q_out = q_body_rot ⊗ q_in
        q_in = msg.pose.pose.orientation
        q_out = quat_multiply(self.q_body_rot, (q_in.w, q_in.x, q_in.y, q_in.z))

        out = VehicleOdometry()
        # 时间戳：ROS 时间(ns) → PX4 时间(us)。SITL 下两者都从 /clock 同步。
        out.timestamp = int(msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec / 1_000)
        out.timestamp_sample = out.timestamp

        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = [px, py, pz]
        out.q = [q_out[0], q_out[1], q_out[2], q_out[3]]

        # 速度 / 角速度不融合，填 NaN
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [float('nan')] * 3
        out.angular_velocity = [float('nan')] * 3

        out.position_variance = [POSITION_VARIANCE] * 3
        out.orientation_variance = [ORIENTATION_VARIANCE] * 3
        out.velocity_variance = [float('nan')] * 3

        out.reset_counter = 0
        out.quality = 0
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = LivoToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
