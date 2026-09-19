#!/usr/bin/env python3
"""
深度图修复节点：把 gz 深度图中超量程产生的 inf/NaN 清成 0，再重新发布，供 RViz2 显示。

背景：
  gz 的 depth_camera 对「超出 far 量程 / 无回波」的像素输出 +inf，RViz2 对 32FC1 图做 min/max 归一化时遇到 inf 会把整幅图渲染成黑色。

输入 / 输出：
  /realsensed455f/depth/image_raw  (sensor_msgs/Image, 32FC1, 单位米)  <- gz 桥接
  /realsensed455f/depth/image_fix  (sensor_msgs/Image, 32FC1, 单位米)  -> RViz2

性能要点（否则会延迟 / 掉帧）：
  直接把 bytes 赋给 Image.data 时，rclpy 会用 Python 逐字节转 array.array，1280x720x4 = 3.7MB 要 ~300ms/帧。必须先转 array.array('B') 再赋值（C 级 memcpy）。
"""

import rclpy
import numpy as np
from array import array
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# 话题名
DEPTH_RAW_TOPIC = '/realsensed455f/depth/image_raw'
DEPTH_FIX_TOPIC = '/realsensed455f/depth/image_fix'


class DepthFixer(Node):
    def __init__(self):
        super().__init__('depth_fixer')

        # 订阅端用 BEST_EFFORT：对上游 QoS 最宽容，桥接发 RELIABLE / BEST_EFFORT 都能收。
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # 发布端必须 RELIABLE：RViz2 的 Image 显示默认 RELIABLE 订阅，BEST_EFFORT 发布者会被它拒收（RELIABILITY_QOS_POLICY 不兼容）。
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, DEPTH_RAW_TOPIC, self.on_depth_image, sub_qos)
        self.depth_pub = self.create_publisher(Image, DEPTH_FIX_TOPIC, pub_qos)
        self.get_logger().info(f'depth_fixer: {DEPTH_RAW_TOPIC} -> {DEPTH_FIX_TOPIC}')

    def on_depth_image(self, msg):
        fixed = np.nan_to_num(np.frombuffer(msg.data, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)  # inf / NaN -> 0，其余值保持（单位米）

        out = Image()
        out.header = msg.header  # 保留 frame_id = realsense_link
        out.height = msg.height
        out.width = msg.width
        out.encoding = msg.encoding  # 32FC1
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = array('B', fixed.tobytes())  # 关键：先转 array.array('B') 再赋值，避免 rclpy 逐字节慢转换
        self.depth_pub.publish(out)


def main():
    rclpy.init()
    node = DepthFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
