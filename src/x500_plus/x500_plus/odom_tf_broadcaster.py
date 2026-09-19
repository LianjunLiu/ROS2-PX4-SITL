#!/usr/bin/env python3
"""
里程计 TF 广播节点：把 gz 的无人机里程计转成动态 TF，让点云 / 相机随无人机一起运动。

原理：
  gz 侧 OdometryPublisher 发布 /model/x500_plus_0/odometry (gz.msgs.Odometry)，由 parameter_bridge 转成 nav_msgs/Odometry。本节点订阅它，把无人机在gz 世界坐标系里的位姿广播成 world -> base_link 的动态 TF（发布到 /tf）。

TF 树：
  world -> base_link (动态, 本节点) -> realsense_link / mid360s_link (静态 TF)

这样 RViz2 把 Fixed Frame 设为 world 后，点云 / 图像会随无人机一起运动。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# 订阅的里程计话题（gz -> ROS2 桥接后）
ODOMETRY_TOPIC = '/model/x500_plus_0/odometry'


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, ODOMETRY_TOPIC, self.on_odometry, 10)
        self.get_logger().info(f'odom_tf_broadcaster: {ODOMETRY_TOPIC} -> /tf (world -> base_link)')

    def on_odometry(self, msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
