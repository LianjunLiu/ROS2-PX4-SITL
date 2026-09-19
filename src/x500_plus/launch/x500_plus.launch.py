"""x500_plus: bridge gz-sim sensors to ROS2 + static/dynamic TF + depth image fix."""
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('x500_plus')
    bridge_cfg = os.path.join(pkg_share, 'config', 'x500_plus_bridge.yaml')

    # 1) 传感器 + 里程计桥接（gz -> ROS2），从 config/x500_plus_bridge.yaml 读取。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_cfg}],
        output='screen',
    )

    # 2) 动态 TF：world -> base_link
    odom_tf_broadcaster = Node(
        package='x500_plus',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
    )
    
    # 3) 静态 TF：把两个自定义传感器 link 挂到 base_link 下。数值来自 model.sdf 的 <pose relative_to="base_link">，顺序为 x y z yaw pitch roll。
    #    realsense_link: 0.15 0 0.018，pitch 0.2618 rad (15°)   mid360s_link: 0 0 0.11，pitch 0.1745 rad (10°)
    tf_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_realsense',
        arguments=['0.15', '0', '0.018', '0', '0.2618', '0', 'base_link', 'realsense_link'],
        output='screen',
    )
    tf_mid360s = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_mid360s',
        arguments=['0', '0', '0.11', '0', '0.1745', '0', 'base_link', 'mid360s_link'],
        output='screen',
    )

    # 4) 深度图修复：inf -> 0
    depth_fixer = Node(
        package='x500_plus',
        executable='depth_fixer',
        name='depth_fixer',
        output='screen',
    )

    # 5) 风速注入（ROS2 -> gz）：订阅 /wind_cmd，转成 gz.msgs.Wind 发到 /world/Penglai/wind
    wind_injector = Node(
        package='x500_plus',
        executable='wind_injector',
        name='wind_injector',
        output='screen',
    )

    # 6) FAST-LIVO2 里程计 -> PX4 vehicle_visual_odometry（相机初始化位姿 -> FRD）
    livo_to_px4 = Node(
        package='x500_plus',
        executable='livo_to_px4',
        name='livo_to_px4',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        bridge, odom_tf_broadcaster, tf_realsense, tf_mid360s, depth_fixer, wind_injector, livo_to_px4,
    ])
