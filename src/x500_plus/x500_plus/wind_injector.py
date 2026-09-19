#!/usr/bin/env python3
"""
风速注入节点：运行时向 gz-sim 注入可随时更改的风。

原理：
  PX4 的 gz_bridge 通过 server.config 已经加载了 WindEffects 插件（gz-sim-wind-effects-system，噪声/阵风全为 0），它订阅 /world/<world>/wind 话题接收 gz.msgs.Wind，并把风作用到带 <enable_wind>true</enable_wind> 的 link 上（x500_base 的 base_link 已开启）。

  本节点订阅 ROS2 话题 /wind_cmd（geometry_msgs/Vector3，单位 m/s，world ENU 坐标系），收到后转成 gz.msgs.Wind 一次性发布到 /world/<world>/wind。WindEffects 会锁存最后风速，直到下一次更新。

用法：
  ros2 run x500_plus wind_injector
  ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 3.0, y: 0.0, z: 0.0}" --once   # 3 m/s 朝 +X
  ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" --once   # 停风
"""

import os
# protobuf 7.x 与 gz-msgs10 的旧生成代码不兼容，切纯 Python 实现绕过（仅本进程生效，不影响 mavsdk 等）
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from px4_msgs.msg import WindCommand
import gz.transport13 as gz_transport
from gz.msgs10.wind_pb2 import Wind


# gz 世界名（与 Penglai.sdf 的 <world name="..."> 一致），可用 -p world:=xxx 覆盖
WORLD = 'Penglai'

# 订阅的风速指令话题（m/s，world ENU）
WIND_CMD_TOPIC = '/wind_cmd'


class WindInjector(Node):
    def __init__(self):
        super().__init__('wind_injector')
        self.declare_parameter('world', WORLD)
        world = self.get_parameter('world').value
        self.wind_topic = f'/world/{world}/wind'

        self.create_subscription(Vector3, WIND_CMD_TOPIC, self.on_wind_cmd, 10)
        
        # 进程内直接发 gz.msgs.Wind（取代 subprocess 调 gz topic）
        self._gz_node = gz_transport.Node()
        self._gz_wind_pub = self._gz_node.advertise(self.wind_topic, Wind)

        self.wind_pub = self.create_publisher(WindCommand, '/fmu/in/wind_command', 10)

        self.get_logger().info(f'wind_injector ready: {WIND_CMD_TOPIC} -> {self.wind_topic} (Vector3 m/s); also logged via /fmu/in/wind_command')

    def on_wind_cmd(self, msg):
        # 进程内发布 gz.msgs.Wind 给 WindEffects 插件（linear_velocity + enable_wind）
        w = Wind()
        w.linear_velocity.x = msg.x
        w.linear_velocity.y = msg.y
        w.linear_velocity.z = msg.z
        w.enable_wind = True
        self._gz_wind_pub.publish(w)

        # 同一命令另发一份进 uORB，供 logger 记入 ulog（DDS 桥 /fmu/in/wind_command）
        wc = WindCommand()
        wc.velocity = [msg.x, msg.y, msg.z]
        wc.enable_wind = True
        self.wind_pub.publish(wc)

        self.get_logger().info(f'wind set to ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}) m/s')


def main():
    rclpy.init()
    node = WindInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
