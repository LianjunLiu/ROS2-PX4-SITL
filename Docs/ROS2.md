# 一、构建地图

软件：Dynamic_World_Generator

# 二、构建仿真环境

## 2.1 修改 server.config

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config

~~~xml
<server_config>
  <plugins>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin entity_name="*" entity_type="world" filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>1</time_for_rise>
          <sin>
            <amplitude_percent>0</amplitude_percent>
            <period>1</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>1</time_for_rise>
          <sin>
            <amplitude>0</amplitude>
            <period>1</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0</stddev>
        </noise>
      </vertical>
    </plugin>
    <plugin entity_name="*" entity_type="world" filename="libOpticalFlowSystem.so" name="custom::OpticalFlowSystem"/>
    <plugin entity_name="*" entity_type="world" filename="libGstCameraSystem.so" name="custom::GstCameraSystem"/>
    <!-- <plugin entity_name="*" entity_type="world" filename="libTemplatePlugin.so" name="custom::TemplateSystem"/> -->
  </plugins>
</server_config>

~~~

## 2.2 修改  World 模型

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/worlds/Penglai.sdf

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/worlds/Penglai.sdf

~~~xml
<?xml version="1.0" encoding="utf-8"?>
<sdf version="1.9">
  <world name="Penglai">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>39.9883</latitude_deg>
      <longitude_deg>116.3503</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode/>
            </friction>
            <bounce/>
            <contact/>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 -0 0</pose>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <enable_wind>false</enable_wind>
      </link>
      <pose>0 0 0 0 -0 0</pose>
      <self_collide>false</self_collide>
    </model>
    <model name="wall_1">
      <static>true</static>
      <type>wall</type>
      <pose>0.000000 -10.000000 0.500000 0 0 0.000000</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_2">
      <static>true</static>
      <type>wall</type>
      <pose>10.000000 0.000000 0.500000 0 0 1.570796</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_3">
      <static>true</static>
      <type>wall</type>
      <pose>0.000000 10.000000 0.500000 0 0 3.141593</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="wall_4">
      <static>true</static>
      <type>wall</type>
      <pose>-10.000000 0.000000 0.500000 0 0 -1.570796</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20.000000 0.100000 1.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_1">
      <static>true</static>
      <type>box</type>
      <pose>-4.300000 4.900000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_2">
      <static>true</static>
      <type>box</type>
      <pose>-4.500000 6.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_3">
      <static>true</static>
      <type>box</type>
      <pose>-5.900000 4.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_4">
      <static>true</static>
      <type>box</type>
      <pose>-2.400000 3.800000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_5">
      <static>true</static>
      <type>box</type>
      <pose>-1.800000 5.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_6">
      <static>true</static>
      <type>box</type>
      <pose>0.700000 6.100000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_7">
      <static>true</static>
      <type>box</type>
      <pose>0.200000 4.100000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_8">
      <static>true</static>
      <type>box</type>
      <pose>2.400000 5.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_9">
      <static>true</static>
      <type>box</type>
      <pose>4.200000 6.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_10">
      <static>true</static>
      <type>box</type>
      <pose>5.900000 5.200000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_11">
      <static>true</static>
      <type>box</type>
      <pose>4.000000 4.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_12">
      <static>true</static>
      <type>box</type>
      <pose>6.800000 4.100000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_13">
      <static>true</static>
      <type>box</type>
      <pose>7.400000 4.800000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_14">
      <static>true</static>
      <type>box</type>
      <pose>8.700000 5.100000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_15">
      <static>true</static>
      <type>box</type>
      <pose>7.600000 6.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_16">
      <static>true</static>
      <type>box</type>
      <pose>8.800000 3.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_17">
      <static>true</static>
      <type>box</type>
      <pose>-6.900000 6.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_18">
      <static>true</static>
      <type>box</type>
      <pose>-8.000000 5.000000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_19">
      <static>true</static>
      <type>box</type>
      <pose>-8.400000 6.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_20">
      <static>true</static>
      <type>box</type>
      <pose>-8.200000 3.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_21">
      <static>true</static>
      <type>box</type>
      <pose>-5.300000 3.600000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_22">
      <static>true</static>
      <type>box</type>
      <pose>-2.100000 6.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_23">
      <static>true</static>
      <type>box</type>
      <pose>-0.500000 3.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_24">
      <static>true</static>
      <type>box</type>
      <pose>2.500000 3.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_25">
      <static>true</static>
      <type>box</type>
      <pose>5.500000 3.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_26">
      <static>true</static>
      <type>box</type>
      <pose>2.100000 6.900000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_27">
      <static>true</static>
      <type>box</type>
      <pose>6.200000 6.800000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_28">
      <static>true</static>
      <type>box</type>
      <pose>-6.900000 3.200000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_29">
      <static>true</static>
      <type>box</type>
      <pose>7.200000 2.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_30">
      <static>true</static>
      <type>box</type>
      <pose>9.300000 7.000000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_31">
      <static>true</static>
      <type>box</type>
      <pose>-8.500000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_32">
      <static>true</static>
      <type>box</type>
      <pose>-6.800000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_33">
      <static>true</static>
      <type>box</type>
      <pose>-7.700000 -0.000000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_34">
      <static>true</static>
      <type>box</type>
      <pose>-5.000000 1.900000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_35">
      <static>true</static>
      <type>box</type>
      <pose>-5.100000 0.800000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_36">
      <static>true</static>
      <type>box</type>
      <pose>-1.800000 1.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_37">
      <static>true</static>
      <type>box</type>
      <pose>-3.300000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_38">
      <static>true</static>
      <type>box</type>
      <pose>-3.600000 0.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_39">
      <static>true</static>
      <type>box</type>
      <pose>-5.100000 -0.200000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_40">
      <static>true</static>
      <type>box</type>
      <pose>-6.000000 0.400000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_41">
      <static>true</static>
      <type>box</type>
      <pose>-8.800000 0.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_42">
      <static>true</static>
      <type>box</type>
      <pose>-9.000000 -0.500000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_43">
      <static>true</static>
      <type>box</type>
      <pose>-6.500000 -0.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_44">
      <static>true</static>
      <type>box</type>
      <pose>-7.000000 0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_45">
      <static>true</static>
      <type>box</type>
      <pose>-7.700000 -0.800000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_46">
      <static>true</static>
      <type>box</type>
      <pose>-4.800000 -0.800000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_47">
      <static>true</static>
      <type>box</type>
      <pose>-3.500000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_48">
      <static>true</static>
      <type>box</type>
      <pose>-1.700000 -0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_49">
      <static>true</static>
      <type>box</type>
      <pose>-1.900000 0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_50">
      <static>true</static>
      <type>box</type>
      <pose>-0.300000 1.100000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_51">
      <static>true</static>
      <type>box</type>
      <pose>0.300000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_52">
      <static>true</static>
      <type>box</type>
      <pose>0.400000 2.400000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_53">
      <static>true</static>
      <type>box</type>
      <pose>1.900000 2.000000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_54">
      <static>true</static>
      <type>box</type>
      <pose>2.100000 1.200000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_55">
      <static>true</static>
      <type>box</type>
      <pose>1.300000 0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_56">
      <static>true</static>
      <type>box</type>
      <pose>0.900000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_57">
      <static>true</static>
      <type>box</type>
      <pose>0.200000 0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_58">
      <static>true</static>
      <type>box</type>
      <pose>-0.200000 -0.500000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_59">
      <static>true</static>
      <type>box</type>
      <pose>2.400000 -0.500000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_60">
      <static>true</static>
      <type>box</type>
      <pose>3.100000 0.200000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_61">
      <static>true</static>
      <type>box</type>
      <pose>3.700000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_62">
      <static>true</static>
      <type>box</type>
      <pose>4.000000 2.400000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_63">
      <static>true</static>
      <type>box</type>
      <pose>5.300000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_64">
      <static>true</static>
      <type>box</type>
      <pose>4.500000 0.800000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_65">
      <static>true</static>
      <type>box</type>
      <pose>4.600000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_66">
      <static>true</static>
      <type>box</type>
      <pose>6.200000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_67">
      <static>true</static>
      <type>box</type>
      <pose>6.000000 0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_68">
      <static>true</static>
      <type>box</type>
      <pose>7.000000 1.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_69">
      <static>true</static>
      <type>box</type>
      <pose>7.400000 0.400000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_70">
      <static>true</static>
      <type>box</type>
      <pose>8.000000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_71">
      <static>true</static>
      <type>box</type>
      <pose>8.800000 1.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_72">
      <static>true</static>
      <type>box</type>
      <pose>9.300000 -0.300000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_73">
      <static>true</static>
      <type>box</type>
      <pose>8.300000 0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_74">
      <static>true</static>
      <type>box</type>
      <pose>8.400000 2.000000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_75">
      <static>true</static>
      <type>box</type>
      <pose>-1.500000 2.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_76">
      <static>true</static>
      <type>box</type>
      <pose>-3.000000 2.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_77">
      <static>true</static>
      <type>box</type>
      <pose>-7.900000 2.700000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_78">
      <static>true</static>
      <type>box</type>
      <pose>-4.700000 2.800000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_79">
      <static>true</static>
      <type>box</type>
      <pose>-0.900000 0.600000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_80">
      <static>true</static>
      <type>box</type>
      <pose>7.400000 -0.900000 6.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.250000 0.250000 12.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_81">
      <static>true</static>
      <type>box</type>
      <pose>-8.400000 -2.000000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_82">
      <static>true</static>
      <type>box</type>
      <pose>-6.600000 -3.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_83">
      <static>true</static>
      <type>box</type>
      <pose>-4.700000 -4.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_84">
      <static>true</static>
      <type>box</type>
      <pose>-4.700000 -1.900000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_85">
      <static>true</static>
      <type>box</type>
      <pose>-8.600000 -4.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_86">
      <static>true</static>
      <type>box</type>
      <pose>-3.300000 -3.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_87">
      <static>true</static>
      <type>box</type>
      <pose>-0.800000 -1.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_88">
      <static>true</static>
      <type>box</type>
      <pose>-0.300000 -3.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_89">
      <static>true</static>
      <type>box</type>
      <pose>2.100000 -4.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_90">
      <static>true</static>
      <type>box</type>
      <pose>6.000000 -4.200000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_91">
      <static>true</static>
      <type>box</type>
      <pose>8.200000 -2.000000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_92">
      <static>true</static>
      <type>box</type>
      <pose>9.000000 -4.300000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_93">
      <static>true</static>
      <type>box</type>
      <pose>3.900000 -2.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_94">
      <static>true</static>
      <type>box</type>
      <pose>1.700000 -2.500000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_95">
      <static>true</static>
      <type>box</type>
      <pose>6.100000 -1.600000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_96">
      <static>true</static>
      <type>box</type>
      <pose>-1.900000 -4.900000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_97">
      <static>true</static>
      <type>box</type>
      <pose>-6.700000 -5.400000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_98">
      <static>true</static>
      <type>box</type>
      <pose>1.300000 -5.600000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_99">
      <static>true</static>
      <type>box</type>
      <pose>5.300000 -5.700000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <model name="box_100">
      <static>true</static>
      <type>box</type>
      <pose>-3.600000 -5.800000 3.000000 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.500000 0.500000 6.000000</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>

~~~

## 2.3 修改 Model 模型

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_plus
文件位置：	/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf
文件位置：	/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.config
文件位置：	/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.sdf

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf

~~~xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_base'>
    <pose>0 0 .24 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.02166666666666667</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.02166666666666667</iyy>
          <iyz>0</iyz>
          <izz>0.04000000000000001</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <enable_wind>true</enable_wind>
      <velocity_decay />
      <visual name="base_link_visual">
        <pose>0 0 .025 0 0 3.141592654</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/NXP-HGD-CF.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_0">
        <pose>0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_1">
        <pose>-0.174 0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_2">
        <pose>0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="5010_motor_base_3">
        <pose>-0.174 -0.174 .032 0 0 -.45</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Base.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <visual name="NXP_FMUK66_FRONT">
        <pose>0.047 .001 .043 1 0 1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="NXP_FMUK66_TOP">
        <pose>-0.023 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.013 .007</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/nxp.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <visual name="RDDRONE_FMUK66_TOP">
        <pose>-.03 0 .0515 0 0 -1.57</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>.032 .0034</size>
          </plane>
        </geometry>
        <material>
          <diffuse>1.0 1.0 1.0</diffuse>
          <specular>1.0 1.0 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://x500_base/materials/textures/rd.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="base_link_collision_0">
        <pose>0 0 .007 0 0 0</pose>
        <geometry>
          <box>
            <size>0.35355339059327373 0.35355339059327373 0.05</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_1">
        <pose>0 -0.098 -.123 -0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_2">
        <pose>0 0.098 -.123 0.35 0 0</pose>
        <geometry>
          <box>
            <size>0.015 0.015 0.21</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_3">
        <pose>0 -0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
      <collision name="base_link_collision_4">
        <pose>0 0.132 -.2195 0 0 0</pose>
        <geometry>
          <box>
            <size>0.25 0.015 0.015</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure>
          <!-- Noise modeled after BMP390 -->
          <pressure>
            <noise type="gaussian">
              <mean>0</mean>
              <stddev>3</stddev>
            </noise>
          </pressure>
        </air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <!-- TODO: update to fix units and coordinate system when we move past Harmonic -->
          <!-- See https://github.com/gazebosim/gz-sim/pull/2460 -->
          <!-- 3mgauss RMS: NOTE: noise is in tesla but sensor reports data in gauss -->
          <!-- Noise modeled after IIS2MDC -->
          <x>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <stddev>0.0001</stddev>
            </noise>
          </z>
        </magnetometer>
      </sensor>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <!-- Noise modeled after IIM42653 -->
            <!-- 0.05 deg/s converted to rad/s -->
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <!-- Noise modeled after IIM42653 -->
            <!-- X & Y axis: 0.65 mg-rms converted to m/ss -->
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00637</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00637</stddev>
              </noise>
            </y>
            <!-- Z axis: 0.70 mg-rms converted to m/ss-->
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00686</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>
    <link name="rotor_0">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay />
      <pose>0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
        <mass>0.016076923076923075</mass>
        <inertia>
          <ixx>3.8464910483993325e-07</ixx>
          <iyy>2.6115851691700804e-05</iyy>
          <izz>2.649858234714004e-05</izz>
        </inertia>
      </inertial>
      <visual name="rotor_0_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_0_visual_motor_bell">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_0_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_0_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_0</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_1">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay />
      <pose>-0.174 0.174 0.06 0 0 0</pose>
      <inertial>
        <mass>0.016076923076923075</mass>
        <inertia>
          <ixx>3.8464910483993325e-07</ixx>
          <iyy>2.6115851691700804e-05</iyy>
          <izz>2.649858234714004e-05</izz>
        </inertia>
      </inertial>
      <visual name="rotor_1_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_ccw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_1_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_1_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_1_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_2">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay />
      <pose>0.174 0.174 0.06 0 0 0</pose>
      <inertial>
        <mass>0.016076923076923075</mass>
        <inertia>
          <ixx>3.8464910483993325e-07</ixx>
          <iyy>2.6115851691700804e-05</iyy>
          <izz>2.649858234714004e-05</izz>
        </inertia>
      </inertial>
      <visual name="rotor_2_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_2_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_2_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_2_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_2</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
    <link name="rotor_3">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay />
      <pose>-0.174 -0.174 0.06 0 0 0</pose>
      <inertial>
        <mass>0.016076923076923075</mass>
        <inertia>
          <ixx>3.8464910483993325e-07</ixx>
          <iyy>2.6115851691700804e-05</iyy>
          <izz>2.649858234714004e-05</izz>
        </inertia>
      </inertial>
      <visual name="rotor_3_visual">
        <pose>-0.022 -0.14638461538461536 -0.016 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>0.8461538461538461 0.8461538461538461 0.8461538461538461</scale>
            <uri>model://x500_base/meshes/1345_prop_cw.stl</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      <visual name="rotor_3_visual_motor_top">
        <pose>0 0 -.032 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>model://x500_base/meshes/5010Bell.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="rotor_3_collision">
        <pose>0 0 0 0 0 0 </pose>
        <geometry>
          <box>
            <size>0.2792307692307692 0.016923076923076923 0.0008461538461538462</size>
          </box>
        </geometry>
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
            </ode>
          </contact>
          <friction>
            <ode />
          </friction>
        </surface>
      </collision>
    </link>
    <joint name="rotor_3_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_3</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e+16</lower>
          <upper>1e+16</upper>
        </limit>
        <dynamics>
          <spring_reference>0</spring_reference>
          <spring_stiffness>0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.config

~~~xml
<?xml version="1.0"?>
<model>
  <name>x500_plus</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>PX4 simulation</name>
    <email>px4@example.com</email>
  </author>
  <description>X500 with a RealSense D455F RGB-D camera and a MID-360S 3D lidar approximation.</description>
</model>
~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500_plus/model.sdf

~~~xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="x500_plus">
    <include merge="true">
      <uri>x500</uri>
    </include>
    <!-- 发布 /model/<name>/odometry (gz.msgs.Odometry)，用于 ROS2 侧动态 TF -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
    </plugin>
    <link name="realsense_link">
      <pose relative_to="base_link">0.15 0 0.018 0 0.2618 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="realsense_visual">
        <geometry>
          <box>
            <size>0.025 0.09 0.025</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.08 0.08 0.08 1</diffuse>
        </material>
      </visual>
      <collision name="realsense_collision">
        <geometry>
          <box>
            <size>0.025 0.09 0.025</size>
          </box>
        </geometry>
      </collision>
      <sensor name="realsense_rgb" type="camera">
        <gz_frame_id>realsense_link</gz_frame_id>
        <pose>0.0126 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.5184</horizontal_fov>
          <image>
            <width>1280</width>
            <height>800</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>realsensed455f/color/image_raw</topic>
      </sensor>
      <sensor name="realsense_depth" type="depth_camera">
        <gz_frame_id>realsense_link</gz_frame_id>
        <pose>0.0126 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.5184</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>R_FLOAT32</format>
          </image>
          <clip>
            <near>0.52</near>
            <far>6</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>realsensed455f/depth/image_raw</topic>
      </sensor>
    </link>
    <joint name="realsense_joint" type="fixed">
      <parent>base_link</parent>
      <child>realsense_link</child>
    </joint>
    <link name="mid360s_link">
      <pose relative_to="base_link">0 0 0.11 0 0.1745 0</pose>
      <inertial>
        <mass>0.265</mass>
        <inertia>
          <ixx>0.0002</ixx>
          <iyy>0.0002</iyy>
          <izz>0.0002</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="mid360s_visual">
        <geometry>
          <cylinder>
            <radius>0.045</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <collision name="mid360s_collision">
        <geometry>
          <cylinder>
            <radius>0.045</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <sensor name="mid360s_lidar" type="gpu_lidar">
        <gz_frame_id>mid360s_link</gz_frame_id>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1800</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159265</min_angle>
              <max_angle>3.14159265</max_angle>
            </horizontal>
            <vertical>
              <samples>40</samples>
              <resolution>1</resolution>
              <min_angle>-0.122173</min_angle>
              <max_angle>0.907571</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>100</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <topic>mid360s/points</topic>
      </sensor>
      <sensor name="mid360s_imu" type="imu">
        <gz_frame_id>mid360s_link</gz_frame_id>
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <topic>mid360s/imu</topic>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.0008726646</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00637</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00637</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.00686</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>
    <joint name="mid360s_joint" type="fixed">
      <parent>base_link</parent>
      <child>mid360s_link</child>
    </joint>
  </model>
</sdf>

~~~



## 2.4 构建 Model 模型

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
文件位置：	/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_plus
文件位置：	/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_plus

~~~c++
#!/bin/sh
#
# @name Gazebo x500 plus (RealSense D455F + MID-360S)
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_plus}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000

param set-default MPC_THR_HOVER 0.60
param set-default NAV_DLL_ACT 2

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_romfs_files(

	1010_gazebo-classic_iris_opt_flow
	1010_gazebo-classic_iris_opt_flow.post
	1011_gazebo-classic_iris_irlock
	1012_gazebo-classic_iris_rplidar
	1013_gazebo-classic_iris_vision
	1013_gazebo-classic_iris_vision.post
	1015_gazebo-classic_iris_depth_camera
	1016_gazebo-classic_iris_downward_depth_camera
	1017_gazebo-classic_iris_opt_flow_mockup
	1019_gazebo-classic_iris_dual_gps
	1021_gazebo-classic_uuv_hippocampus
	1022_gazebo-classic_uuv_bluerov2_heavy
	1030_gazebo-classic_plane
	1031_gazebo-classic_plane_cam
	1031_gazebo-classic_plane_cam.post
	1032_gazebo-classic_plane_catapult
	1033_jsbsim_rascal
	1034_flightgear_rascal-electric
	1035_gazebo-classic_techpod
	1036_jsbsim_malolo
	1037_gazebo-classic_believer
	1038_gazebo-classic_glider
	1039_gazebo-classic_advanced_plane
	1040_gazebo-classic_standard_vtol
	1041_gazebo-classic_tailsitter
	1042_gazebo-classic_tiltrotor
	1043_gazebo-classic_standard_vtol_drop
	1044_gazebo-classic_plane_lidar
	1045_gazebo-classic_quadtailsitter
	1062_flightgear_tf-r1
	1070_gazebo-classic_boat

	2507_gazebo-classic_cloudship

	3010_jsbsim_quadrotor_x
	3011_jsbsim_hexarotor_x

	4001_gz_x500
	4002_gz_x500_depth
	4003_gz_rc_cessna
	4004_gz_standard_vtol
	4005_gz_x500_vision
	4006_gz_px4vision
	4008_gz_advanced_plane
	4009_gz_r1_rover
	4010_gz_x500_mono_cam
	4011_gz_lawnmower
	4013_gz_x500_lidar_2d
	4014_gz_x500_mono_cam_down
	4016_gz_x500_lidar_down
	4017_gz_x500_lidar_front
	4018_gz_quadtailsitter
	4019_gz_x500_gimbal
	4020_gz_tiltrotor
	4021_gz_x500_flow
	4022_gz_x500_plus

	6011_gazebo-classic_typhoon_h480
	6011_gazebo-classic_typhoon_h480.post

	8011_gz_omnicopter

	10015_gazebo-classic_iris
	10016_none_iris
	10017_jmavsim_iris
	10018_gazebo-classic_iris_foggy_lidar
	10019_gazebo-classic_omnicopter
	10030_gazebo-classic_px4vision

	10040_sihsim_quadx
	10041_sihsim_airplane
	10042_sihsim_xvert
	10043_sihsim_standard_vtol
	10044_sihsim_hex
	10045_sihsim_rover_ackermann

	17001_flightgear_tf-g1
	17002_flightgear_tf-g2

	50000_gz_rover_differential
	51000_gz_rover_ackermann
	52000_gz_rover_mecanum

	60002_gz_uuv_bluerov2_heavy

	70000_gz_atmos

	# [22000, 22999] Reserve for custom models
)

~~~

## 2.5 桥接 ROS2 消息

文件位置：/home/liu/Desktop/ROS2/src
文件位置：	/home/liu/Desktop/ROS2/src/x500_plus
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/config
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/config/x500_plus_bridge.yaml
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/launch
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/resource
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/resource/x500_plus
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/x500_plus
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/__init__.py
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/depth_fixer.py
文件位置：			/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/odom_tf_broadcaster.py
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/package.xml
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/setup.cfg
文件位置：		/home/liu/Desktop/ROS2/src/x500_plus/setup.py

文件：/home/liu/Desktop/ROS2/src/x500_plus/config/x500_plus_bridge.yaml

~~~yaml
# ============================================================
# x500_plus 传感器桥接清单（Gazebo gz-sim -> ROS2）
#
# 这是「需要桥接哪些消息」的完整清单，供 parameter_bridge 使用。
# 当前 launch (x500_plus.launch.py) 通过 config_file 参数直接读取本文件，
# 本文件即为桥接的唯一真源（无需再手动传位置参数）。
# 未启用的条目先注释掉，需要时取消注释即可。
# ============================================================

# ---------- A. 传感器数据 ----------

# 彩色相机
- ros_topic_name: "/realsensed455f/color/image_raw"
  gz_topic_name: "/realsensed455f/color/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# 彩色相机内参（标定 / 深度对齐用，与图像配对）
# - ros_topic_name: "/realsensed455f/color/camera_info"
#   gz_topic_name: "/realsensed455f/color/camera_info"
#   ros_type_name: "sensor_msgs/msg/CameraInfo"
#   gz_type_name: "gz.msgs.CameraInfo"
#   direction: GZ_TO_ROS

# 深度相机（R_FLOAT32 -> 32FC1，单位米；超量程为 +inf，需 depth_fixer 清洗）
- ros_topic_name: "/realsensed455f/depth/image_raw"
  gz_topic_name: "/realsensed455f/depth/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# 深度相机内参
# - ros_topic_name: "/realsensed455f/depth/camera_info"
#   gz_topic_name: "/realsensed455f/depth/camera_info"
#   ros_type_name: "sensor_msgs/msg/CameraInfo"
#   gz_type_name: "gz.msgs.CameraInfo"
#   direction: GZ_TO_ROS

# 深度相机导出的点云
# - ros_topic_name: "/realsensed455f/depth/points"
#   gz_topic_name: "/realsensed455f/depth/image_raw/points"
#   ros_type_name: "sensor_msgs/msg/PointCloud2"
#   gz_type_name: "gz.msgs.PointCloudPacked"
#   direction: GZ_TO_ROS

# MID-360S 雷达 3D 点云（gz 侧 gpu_lidar 自动发布在 <topic>/points）
- ros_topic_name: "/mid360s/points/points"
  gz_topic_name: "/mid360s/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

# MID-360S 内置 IMU（FAST-LIVO 惯导输入，frame_id = mid360s_link，200 Hz）
- ros_topic_name: "/mid360s/imu"
  gz_topic_name: "/mid360s/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

# MID-360S 雷达 2D scan（LaserScan，gz 侧在 <topic> 本身）
# - ros_topic_name: "/mid360s/scan"
#   gz_topic_name: "/mid360s/points"
#   ros_type_name: "sensor_msgs/msg/LaserScan"
#   gz_type_name: "gz.msgs.LaserScan"
#   direction: GZ_TO_ROS

# ---------- B. 里程计 / 动态 TF ----------

# 无人机里程计 -> odom_tf_broadcaster.py 转成 world->base_link 动态 TF
- ros_topic_name: "/model/x500_plus_0/odometry"
  gz_topic_name: "/model/x500_plus_0/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# 带协方差的里程计（下游若需要协方差）
# - ros_topic_name: "/model/x500_plus_0/odometry_with_covariance"
#   gz_topic_name: "/model/x500_plus_0/odometry_with_covariance"
#   ros_type_name: "nav_msgs/msg/Odometry"
#   gz_type_name: "gz.msgs.OdometryWithCovariance"
#   direction: GZ_TO_ROS

# ---------- C. 可选 ----------

# 仿真时钟（RViz2 需要跟随仿真时间时启用）
# - ros_topic_name: "/clock"
#   gz_topic_name: "/clock"
#   ros_type_name: "rosgraph_msgs/msg/Clock"
#   gz_type_name: "gz.msgs.Clock"
#   direction: GZ_TO_ROS

# ------------------------------------------------------------
# 备注：gz 里还有一批 PX4 飞控内部传感器话题（IMU / magnetometer /
# navsat / air_pressure / optical_flow / air_speed 等，都在
# /world/Penglai/model/x500_plus_0/link/.../sensor/... 下），
# 它们属于飞控内部闭环，不在「x500_plus 传感器可视化」范围内，故未列出。
# 若将来要桥接，类型对应：gz.msgs.IMU -> sensor_msgs/msg/Imu 等。
# ------------------------------------------------------------

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py

~~~python
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

    return LaunchDescription([
        bridge, odom_tf_broadcaster, tf_realsense, tf_mid360s, depth_fixer,
    ])

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/resource/x500_plus

~~~

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/__init__.py

~~~python
~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/depth_fixer.py

~~~python
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

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/odom_tf_broadcaster.py

~~~python
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

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/package.xml

~~~xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>x500_plus</name>
  <version>0.1.0</version>
  <description>x500_plus gz-sim to ROS2 bridge: RealSense D455F + Livox MID-360S sensors, TF and depth image fix.</description>
  <maintainer email="liu@todo.todo">liu</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ament_index_python</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/setup.cfg

~~~
[develop]
script_dir=$base/lib/x500_plus
[install]
install_scripts=$base/lib/x500_plus

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/setup.py

~~~python
import os
from glob import glob
from setuptools import setup

package_name = 'x500_plus'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu',
    maintainer_email='liu@todo.todo',
    description='x500_plus gz-sim to ROS2 bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = x500_plus.odom_tf_broadcaster:main',
            'depth_fixer = x500_plus.depth_fixer:main',
        ],
    },
)

~~~

## 2.6 安装 ROS2 包

构建 x500_plus 包

~~~bash
cd /home/liu/Desktop/ROS2
colcon build --base-paths src --symlink-install
source /home/liu/Desktop/ROS2/install/setup.bash
# 参考命令
ros2 launch x500_plus x500_plus.launch.py  # 一键起全部
ros2 run x500_plus odom_tf_broadcaster     # 单独跑某个节点
ros2 run x500_plus depth_fixer             # 单独跑某个节点
~~~

构建 px4_msgs px4_ros_com 包

~~~bash
cd /home/liu/Desktop/ROS2/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd /home/liu/Desktop/ROS2
colcon build --base-paths src --symlink-install
source /home/liu/Desktop/ROS2/install/setup.bash
~~~

追加到  ~/.bashrc

~~~bash
grep -q "ROS2/install/setup.bash" ~/.bashrc || echo "source /home/liu/Desktop/ROS2/install/setup.bash" >> ~/.bashrc
~~~

## 2.7 安装 Micro-XRCE-DDS-Agent 代理

构建并安装 Micro-XRCE-DDS-Agent 代理

~~~bash
cd /home/liu/Desktop/ROS2
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
# 参考命令
MicroXRCEAgent udp4 -p 8888  # 软件仿真
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600  # 真机串口
~~~

## 2.8 调整最大转速

无人机默认设置的最大转速（包括仿真模型能够提供的最大转速和电调能够提供的最大转速）限制在1000，强风情况下不能给无人机提供足够的升力，为此需要更改无人机默认设置的最大转速（注意需要同时修改仿真模型能够提供的最大转速和电调能够提供的最大转速）。下面给出修改示例：

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_plus
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_plus）为前述添加过的文件，这里主要修改 param set-default SIM_GZ_EC_MAX1 1200、param set-default SIM_GZ_EC_MAX2 1200、param set-default SIM_GZ_EC_MAX3 1200、param set-default SIM_GZ_EC_MAX4 1200 四个参数。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf

~~~xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500'>
    <include merge='true'>
      <uri>model://x500_base</uri>
    </include>
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_0_joint</jointName>
      <linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1200.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_1_joint</jointName>
      <linkName>rotor_1</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1200.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>1</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_2_joint</jointName>
      <linkName>rotor_2</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1200.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>2</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_3_joint</jointName>
      <linkName>rotor_3</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1200.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>3</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
  </model>
</sdf>

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_plus

~~~c++
#!/bin/sh
#
# @name Gazebo x500 plus (RealSense D455F + MID-360S)
#
# @type Quadrotor
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_plus}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

param set-default CA_ROTOR0_PX 0.13
param set-default CA_ROTOR0_PY 0.22
param set-default CA_ROTOR0_KM  0.05

param set-default CA_ROTOR1_PX -0.13
param set-default CA_ROTOR1_PY -0.20
param set-default CA_ROTOR1_KM  0.05

param set-default CA_ROTOR2_PX 0.13
param set-default CA_ROTOR2_PY -0.22
param set-default CA_ROTOR2_KM -0.05

param set-default CA_ROTOR3_PX -0.13
param set-default CA_ROTOR3_PY 0.20
param set-default CA_ROTOR3_KM -0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1200
param set-default SIM_GZ_EC_MAX2 1200
param set-default SIM_GZ_EC_MAX3 1200
param set-default SIM_GZ_EC_MAX4 1200

param set-default MPC_THR_HOVER 0.60
param set-default NAV_DLL_ACT 2

~~~

## 2.9 启动仿真环境

开启 QGC 地面站：

~~~bash
cd /home/liu/Desktop/ROS2/QGroundControl && ./QGroundControl-x86_64.AppImage
~~~

开启 SITL 仿真：

~~~bash
cd /home/liu/Desktop/ROS2/PX4-Autopilot && PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
~~~

开启 Micro-XRCE-DDS-Agent 代理：

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

开启 ROS2 消息桥接：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
~~~

开启 Rviz2 可视化软件：

~~~bash
rviz2
~~~

#   三、实施风扰注入

## 3.1 构建 node 文件

文件位置：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/wind_injector.py

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/wind_injector.py

~~~python
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

import rclpy
import subprocess
from rclpy.node import Node
from geometry_msgs.msg import Vector3


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
        self.get_logger().info(f'wind_injector ready: {WIND_CMD_TOPIC} -> {self.wind_topic} (Vector3 m/s)')

    def on_wind_cmd(self, msg):
        # gz topic 一次性发布 gz.msgs.Wind（linear_velocity + enable_wind）
        payload = (f'linear_velocity: {{x: {msg.x}, y: {msg.y}, z: {msg.z}}}, enable_wind: true')
        cmd = ['gz', 'topic', '-t', self.wind_topic, '-m', 'gz.msgs.Wind', '-p', payload]
        subprocess.run(cmd, check=False)
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

~~~

## 3.2 修改 launch 文件

文件位置：/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py
注：文件（/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py ）为前述文件。

文件：/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py

~~~python
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

    return LaunchDescription([
        bridge, odom_tf_broadcaster, tf_realsense, tf_mid360s, depth_fixer, wind_injector,
    ])

~~~

## 3.3 安装 ROS2 包

重新构建 x500_plus 包

~~~bash
cd /home/liu/Desktop/ROS2
colcon build --base-paths src --symlink-install
source /home/liu/Desktop/ROS2/install/setup.bash
# 参考命令
ros2 launch x500_plus x500_plus.launch.py  # 一键起全部
ros2 run x500_plus odom_tf_broadcaster     # 单独跑某个节点
ros2 run x500_plus depth_fixer             # 单独跑某个节点
ros2 run x500_plus wind_injector           # 单独跑某个节点
~~~

## 3.4 风扰注入示例

风速控制（CLI）

~~~bash
# 起风
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 6.0, y: 0.0, z: 0.0}" --once
# 改风（随时可改，覆盖上一次）
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 3.0, z: 0.0}" --once
# 停风
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" --once
~~~

风速控制（Python）

~~~python
#!/usr/bin/env python3
"""完整示例。

前提：x500_plus.launch.py 已把 wind_injector 节点拉起（它在订阅 /wind_cmd）。
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class WindController(Node):
    """封装风速控制：发 Vector3 到 /wind_cmd，由 wind_injector 转成 gz.msgs.Wind。"""

    def __init__(self):
        super().__init__('wind_controller')
        # 发布器只建一次，之后反复 publish 即可
        self.wind_pub = self.create_publisher(Vector3, '/wind_cmd', 10)

    def set_wind(self, x, y, z):
        """起风 / 改风：风速 = (x, y, z) m/s，world ENU 系。"""
        self.wind_pub.publish(Vector3(x=float(x), y=float(y), z=float(z)))
        self.get_logger().info(f'wind -> ({x}, {y}, {z}) m/s')

    def stop_wind(self):
        """停风：发零矢量。"""
        self.set_wind(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    node = WindController()

    node.set_wind(3.0, 0.0, 0.0)   # 1) 起风：3 m/s 朝 +X
    time.sleep(10.0)               # 2) 保持 10 秒

    node.set_wind(0.0, 2.0, 0.0)   # 3) 改风：2 m/s 朝 +Y
    time.sleep(10.0)

    node.stop_wind()               # 4) 停风

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

~~~

## 3.5 风扰仿真测试

开启 QGC 地面站：

~~~bash
cd /home/liu/Desktop/ROS2/QGroundControl && ./QGroundControl-x86_64.AppImage
~~~

开启 SITL 仿真：

~~~bash
cd /home/liu/Desktop/ROS2/PX4-Autopilot && PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
~~~

开启 Micro-XRCE-DDS-Agent 代理：

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

开启 ROS2 消息桥接：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
~~~

开启 Rviz2 可视化软件：

~~~bash
rviz2
~~~

开启风扰：

~~~bash
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 6.0, y: 0.0, z: 0.0}" --once
~~~

改变风扰：

~~~bash
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 3.0, z: 0.0}" --once
~~~

关闭风扰：

~~~bash
ros2 topic pub /wind_cmd geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}" --once
~~~

## 3.6* 无人机端 msg 更新

注：* 表示这段修改发生在 4.11 之后。

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/WindCommand.msg

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/WindCommand.msg

~~~c++
# Wind injection command (ROS2 -> PX4 -> ulog).
#
# Mirrors the gz.msgs.Wind that the ROS2 wind_injector node publishes to
# gz-sim, so the applied wind is also recorded in the ulog for reproducibility.
# This topic is log-only: nothing in PX4 consumes it; the wind itself is applied
# directly by the gz-sim WindEffects plugin.

uint64 timestamp		# time since system start (microseconds)

float64[3] velocity		# wind velocity [m/s], world ENU (x/y/z)
bool enable_wind		# true = wind enabled

~~~

## 3.7* 服务器端 msg 更新

注：* 表示这段修改发生在 4.11 之后。

文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/WindCommand.msg

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/WindCommand.msg

~~~c++
# Wind injection command (ROS2 -> PX4 -> ulog).
#
# Mirrors the gz.msgs.Wind that the ROS2 wind_injector node publishes to
# gz-sim, so the applied wind is also recorded in the ulog for reproducibility.
# This topic is log-only: nothing in PX4 consumes it; the wind itself is applied
# directly by the gz-sim WindEffects plugin.

uint64 timestamp		# time since system start (microseconds)

float64[3] velocity		# wind velocity [m/s], world ENU (x/y/z)
bool enable_wind		# true = wind enabled

~~~

## 3.8* 修改 msg 配置文件

注：* 表示这段修改发生在 4.11 之后。

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt

~~~c++
############################################################################
#
#   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

include(px4_list_make_absolute)

set(msg_files
	ActionRequest.msg
	ActuatorArmed.msg
	ActuatorControlsStatus.msg
	ActuatorOutputs.msg
	ActuatorServosTrim.msg
	ActuatorTest.msg
	AdcReport.msg
	Airspeed.msg
	AirspeedWind.msg
	AttackCommand.msg
	AttackStatus.msg
	AutotuneAttitudeControlStatus.msg
	BatteryInfo.msg
	ButtonEvent.msg
	CameraCapture.msg
	CameraStatus.msg
	CameraTrigger.msg
	CanInterfaceStatus.msg
	CellularStatus.msg
	CollisionConstraints.msg
	ControlAllocatorStatus.msg
	Cpuload.msg
	DatamanRequest.msg
	DatamanResponse.msg
	DebugArray.msg
	DebugKeyValue.msg
	DebugValue.msg
	DebugVect.msg
	DifferentialPressure.msg
	DistanceSensor.msg
	DistanceSensorModeChangeRequest.msg
	DronecanNodeStatus.msg
	Ekf2Timestamps.msg
	EscReport.msg
	EscStatus.msg
	EstimatorAidSource1d.msg
	EstimatorAidSource2d.msg
	EstimatorAidSource3d.msg
	EstimatorBias.msg
	EstimatorBias3d.msg
	EstimatorEventFlags.msg
	EstimatorGpsStatus.msg
	EstimatorInnovations.msg
	EstimatorSelectorStatus.msg
	EstimatorSensorBias.msg
	EstimatorStates.msg
	EstimatorStatus.msg
	EstimatorStatusFlags.msg
	versioned/Event.msg
	FigureEightStatus.msg
	FailsafeFlags.msg
	FailureDetectorStatus.msg
	FlightPhaseEstimation.msg
	FollowTarget.msg
	FollowTargetEstimator.msg
	FollowTargetStatus.msg
	FuelTankStatus.msg
	FixedWingLateralGuidanceStatus.msg
	FixedWingLateralStatus.msg
	FixedWingRunwayControl.msg
	GeneratorStatus.msg
	GeofenceResult.msg
	GeofenceStatus.msg
	GimbalControls.msg
	GimbalDeviceAttitudeStatus.msg
	GimbalDeviceInformation.msg
	GimbalDeviceSetAttitude.msg
	GimbalManagerInformation.msg
	GimbalManagerSetAttitude.msg
	GimbalManagerSetManualControl.msg
	GimbalManagerStatus.msg
	GpioConfig.msg
	GpioIn.msg
	GpioOut.msg
	GpioRequest.msg
	GpsDump.msg
	GpsInjectData.msg
	Gripper.msg
	HealthReport.msg
	HeaterStatus.msg
	HoverThrustEstimate.msg
	InputRc.msg
	InternalCombustionEngineControl.msg
	InternalCombustionEngineStatus.msg
	IridiumsbdStatus.msg
	IrlockReport.msg
	LandingGear.msg
	LandingGearWheel.msg
	LandingTargetInnovations.msg
	LandingTargetPose.msg
	LaunchDetectionStatus.msg
	LedControl.msg
	LoggerStatus.msg
	LogMessage.msg
	MagnetometerBiasEstimate.msg
	MagWorkerData.msg
	ManualControlSwitches.msg
	MavlinkLog.msg
	MavlinkTunnel.msg
	MessageFormatRequest.msg
	MessageFormatResponse.msg
	Mission.msg
	MissionResult.msg
	MountOrientation.msg
	NavigatorMissionItem.msg
	NavigatorStatus.msg
	NeuralControl.msg
	NormalizedUnsignedSetpoint.msg
	ObstacleDistance.msg
	OffboardControlMode.msg
	OnboardComputerStatus.msg
	OpenDroneIdArmStatus.msg
	OpenDroneIdOperatorId.msg
	OpenDroneIdSelfId.msg
	OpenDroneIdSystem.msg
	OrbitStatus.msg
	OrbTest.msg
	OrbTestLarge.msg
	OrbTestMedium.msg
	ParameterResetRequest.msg
	ParameterSetUsedRequest.msg
	ParameterSetValueRequest.msg
	ParameterSetValueResponse.msg
	ParameterUpdate.msg
	Ping.msg
	PositionControllerLandingStatus.msg
	PositionControllerStatus.msg
	PositionSetpoint.msg
	PositionSetpointTriplet.msg
	PowerButtonState.msg
	PowerMonitor.msg
	PpsCapture.msg
	PurePursuitStatus.msg
	PwmInput.msg
	Px4ioStatus.msg
	QshellReq.msg
	QshellRetval.msg
	RadioStatus.msg
	RateCtrlStatus.msg
	RcChannels.msg
	RcParameterMap.msg
	RoverAttitudeSetpoint.msg
	RoverAttitudeStatus.msg
	RoverPositionSetpoint.msg
	RoverRateSetpoint.msg
	RoverRateStatus.msg
	RoverSpeedSetpoint.msg
	RoverSpeedStatus.msg
	RoverSteeringSetpoint.msg
	RoverThrottleSetpoint.msg
	Rpm.msg
	RtlStatus.msg
	RtlTimeEstimate.msg
	SatelliteInfo.msg
	SensorAccel.msg
	SensorAccelFifo.msg
	SensorBaro.msg
	SensorCombined.msg
	SensorCorrection.msg
	SensorGnssRelative.msg
	SensorGnssStatus.msg
	SensorGps.msg
	SensorGyro.msg
	SensorGyroFft.msg
	SensorGyroFifo.msg
	SensorHygrometer.msg
	SensorMag.msg
	SensorOpticalFlow.msg
	SensorPreflightMag.msg
	SensorSelection.msg
	SensorsStatus.msg
	SensorsStatusImu.msg
	SensorUwb.msg
	SensorAirflow.msg
	SystemPower.msg
	TakeoffStatus.msg
	TaskStackInfo.msg
	TecsStatus.msg
	TelemetryStatus.msg
	TiltrotorExtraControls.msg
	TimesyncStatus.msg
	TrajectorySetpoint6dof.msg
	TransponderReport.msg
	TuneControl.msg
	UavcanParameterRequest.msg
	UavcanParameterValue.msg
	UlogStream.msg
	UlogStreamAck.msg
	VehicleAcceleration.msg
	VehicleAirData.msg
	VehicleAngularAccelerationSetpoint.msg
	VehicleConstraints.msg
	VehicleImu.msg
	VehicleImuStatus.msg
	VehicleLocalPositionSetpoint.msg
	VehicleMagnetometer.msg
	VehicleOpticalFlow.msg
	VehicleOpticalFlowVel.msg
	VehicleRoi.msg
	VehicleThrustSetpoint.msg
	VehicleTorqueSetpoint.msg
	VelocityLimits.msg
	WheelEncoders.msg
	WindCommand.msg
	YawEstimatorStatus.msg
	versioned/ActuatorMotors.msg
	versioned/ActuatorServos.msg
	versioned/AirspeedValidated.msg
	versioned/ArmingCheckReply.msg
	versioned/ArmingCheckRequest.msg
	versioned/BatteryStatus.msg
	versioned/ConfigOverrides.msg
	versioned/FixedWingLateralSetpoint.msg
	versioned/FixedWingLongitudinalSetpoint.msg
	versioned/GotoSetpoint.msg
	versioned/HomePosition.msg
	versioned/LateralControlConfiguration.msg
	versioned/LongitudinalControlConfiguration.msg
	versioned/ManualControlSetpoint.msg
	versioned/ModeCompleted.msg
	versioned/RegisterExtComponentReply.msg
	versioned/RegisterExtComponentRequest.msg
	versioned/TrajectorySetpoint.msg
	versioned/UnregisterExtComponent.msg
	versioned/VehicleAngularVelocity.msg
	versioned/VehicleAttitude.msg
	versioned/VehicleAttitudeSetpoint.msg
	versioned/VehicleCommandAck.msg
	versioned/VehicleCommand.msg
	versioned/VehicleControlMode.msg
	versioned/VehicleGlobalPosition.msg
	versioned/VehicleLandDetected.msg
	versioned/VehicleLocalPosition.msg
	versioned/VehicleOdometry.msg
	versioned/VehicleRatesSetpoint.msg
	versioned/VehicleStatus.msg
	versioned/VtolVehicleStatus.msg
	versioned/Wind.msg
)
list(SORT msg_files)

px4_list_make_absolute(msg_files ${CMAKE_CURRENT_SOURCE_DIR} ${msg_files})

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(ucdr_out_path ${PX4_BINARY_DIR}/uORB/ucdr)
set(msg_source_out_path ${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
set(uorb_ucdr_headers)
set(uorb_json_files)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
	list(APPEND uorb_ucdr_headers ${ucdr_out_path}/${msg}.h)
	list(APPEND uorb_json_files ${msg_source_out_path}/${msg}.json)
endforeach()

# set parent scope msg_files for ROS
set(msg_files ${msg_files} PARENT_SCOPE)

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.hpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

add_custom_command(
	OUTPUT
		${uorb_json_files}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--json
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.json.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB json files"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(uorb_json_files DEPENDS ${uorb_json_files})

set(uorb_message_fields_cpp_file ${msg_source_out_path}/uORBMessageFieldsGenerated.cpp)
set(uorb_message_fields_header_file ${msg_out_path}/uORBMessageFieldsGenerated.hpp)
add_custom_command(
	OUTPUT
		${uorb_message_fields_cpp_file}
		${uorb_message_fields_header_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
		-f ${uorb_json_files}
		--source-output-file ${uorb_message_fields_cpp_file}
		--header-output-file ${uorb_message_fields_header_file}
	DEPENDS
		${uorb_json_files}
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
	COMMENT "Generating uORB compressed fields"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)

# Generate microcdr headers
add_custom_command(
	OUTPUT ${uorb_ucdr_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${ucdr_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic ucdr headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_ucdr_headers DEPENDS ${uorb_ucdr_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp ${uorb_message_fields_cpp_file})
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)

if(CONFIG_LIB_CDRSTREAM)
	set(uorb_cdr_idl)
	set(uorb_cdr_msg)
	set(uorb_cdr_hash)
	set(uorb_cdr_idl_uorb)
	set(idl_include_path ${PX4_BINARY_DIR}/uORB/idl)
	set(idl_out_path ${idl_include_path}/px4/msg)
	set(idl_rihs01_out_path ${idl_include_path}/px4)
	set(idl_uorb_path ${PX4_BINARY_DIR}/msg/px4/msg)

	# Make sure that CycloneDDS has been checkout out
	execute_process(COMMAND git submodule sync src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )
	execute_process(COMMAND git submodule update --init --force src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )

	# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
	MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds
			-DCMAKE_C_COMPILER=/usr/bin/gcc
			-DBUILD_EXAMPLES=OFF
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

	# Copy .msg files
	foreach(msg_file ${msg_files})
		get_filename_component(msg ${msg_file} NAME_WE)
		configure_file(${msg_file} ${idl_out_path}/${msg}.msg COPYONLY)
		list(APPEND uorb_cdr_idl ${idl_out_path}/${msg}.idl)
		list(APPEND uorb_cdr_msg ${idl_out_path}/${msg}.msg)
		list(APPEND uorb_cdr_hash ${idl_out_path}/${msg}.json)
		list(APPEND uorb_cdr_idl_uorb ${idl_uorb_path}/${msg}.h)
	endforeach()

	# Generate IDL from .msg using rosidl_adapter
	# Note this a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_idl}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/msg2idl.py
			${uorb_cdr_msg}
		DEPENDS
			${uorb_cdr_msg}
			git_cyclonedds
		COMMENT "Generating IDL from uORB topic headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	file(CREATE_LINK ${idl_rihs01_out_path} ${idl_include_path}/px4_msgs SYMBOLIC)

	# Generate IDL from .msg using rosidl_adapter
	# Note this is a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_hash}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_parser:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_generator_type_description"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/idl2rihs01.py
			--output-dir ${idl_rihs01_out_path}
			${uorb_cdr_idl}
		DEPENDS
			${uorb_cdr_idl}
			git_cyclonedds
		COMMENT "Generating RIHS01 hashes from IDL"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	# Generate C definitions from IDL
	set(CYCLONEDDS_DIR ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds)
	include("${CYCLONEDDS_DIR}/cmake/Modules/Generate.cmake")
	idlc_generate(TARGET uorb_cdrstream
                  FEATURES "cdrstream-desc"
                  FILES ${uorb_cdr_idl}
                  INCLUDES ${idl_include_path}
                  BASE_DIR ${idl_include_path}
                  WARNINGS no-implicit-extensibility)
	target_link_libraries(uorb_cdrstream INTERFACE cdr)

	# Generate and overwrite IDL header with custom headers for uORB operatability
	# We typedef the IDL struct the uORB struct so that the IDL offset calculate
	# the offset of internal uORB struct for serialization/deserialization

	# In the future we might want to turn this around let the IDL struct be the leading ABI
	# However we need to remove the padding for logging and remove the re-ordering of fields

	add_custom_target(
		uorb_idl_header
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			--uorb-idl-header
			-f ${msg_files}
			-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
			-o ${idl_uorb_path}
			-e ${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream
		DEPENDS
			uorb_cdrstream
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream/uorb_idl_header.h.em
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
		COMMENT "Generating uORB compatible IDL headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)
	add_dependencies(uorb_msgs uorb_idl_header)

	# Compile all CDR compatible message defnitions
	target_link_libraries(uorb_msgs PRIVATE uorb_cdrstream )
endif()

if(CONFIG_MODULES_ZENOH)
	# Update kconfig file for topics
	execute_process(COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-config
			-f ${msg_files}
			-o ${PX4_SOURCE_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		)
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-pub-sub
			-f ${msg_files}
			-o ${PX4_BINARY_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
			--rihs ${idl_rihs01_out_path}
		DEPENDS
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh/uorb_pubsub_factory.hpp.em
			${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
		COMMENT "Generating Zenoh Topic Code"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)
		add_library(zenoh_topics ${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp)
		set_target_properties(zenoh_topics PROPERTIES LINKER_LANGUAGE CXX)
endif()

~~~

## 3.9* 修改无人机端 Micro-XRCE-DDS-Client 配置

注：* 表示这段修改发生在 4.11 之后。

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

~~~yaml
#####
#
# This file maps all the topics that are to be used on the uXRCE-DDS client.
#
#####
publications:

  - topic: /fmu/out/attack_status
    type: px4_msgs::msg::AttackStatus
    rate_limit: 10.

  - topic: /fmu/out/register_ext_component_reply
    type: px4_msgs::msg::RegisterExtComponentReply

  - topic: /fmu/out/arming_check_request
    type: px4_msgs::msg::ArmingCheckRequest
    rate_limit: 5.

  - topic: /fmu/out/mode_completed
    type: px4_msgs::msg::ModeCompleted
    rate_limit: 50.

  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
    rate_limit: 1.

  - topic: /fmu/out/collision_constraints
    type: px4_msgs::msg::CollisionConstraints
    rate_limit: 50.

  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags
    rate_limit: 5.

  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags
    rate_limit: 5.

  - topic: /fmu/out/manual_control_setpoint
    type: px4_msgs::msg::ManualControlSetpoint
    rate_limit: 25.

  - topic: /fmu/out/message_format_response
    type: px4_msgs::msg::MessageFormatResponse

  - topic: /fmu/out/position_setpoint_triplet
    type: px4_msgs::msg::PositionSetpointTriplet
    rate_limit: 5.

  - topic: /fmu/out/sensor_combined
    type: px4_msgs::msg::SensorCombined

  - topic: /fmu/out/timesync_status
    type: px4_msgs::msg::TimesyncStatus
    rate_limit: 10.

  - topic: /fmu/out/transponder_report
    type: px4_msgs::msg::TransponderReport
 
  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity
    rate_limit: 50.

  - topic: /fmu/out/vehicle_land_detected
    type: px4_msgs::msg::VehicleLandDetected
    rate_limit: 5.

  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude

  - topic: /fmu/out/vehicle_control_mode
    type: px4_msgs::msg::VehicleControlMode
    rate_limit: 50.

  - topic: /fmu/out/vehicle_command_ack
    type: px4_msgs::msg::VehicleCommandAck

  - topic: /fmu/out/vehicle_global_position
    type: px4_msgs::msg::VehicleGlobalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_gps_position
    type: px4_msgs::msg::SensorGps
    rate_limit: 50.

  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus
    rate_limit: 5.

  - topic: /fmu/out/airspeed_validated
    type: px4_msgs::msg::AirspeedValidated
    rate_limit: 50.

  - topic: /fmu/out/vtol_vehicle_status
    type: px4_msgs::msg::VtolVehicleStatus

  - topic: /fmu/out/home_position
    type: px4_msgs::msg::HomePosition
    rate_limit: 5.

  - topic: /fmu/out/wind
    type: px4_msgs::msg::Wind
    rate_limit: 1.

  - topic: /fmu/out/gimbal_device_attitude_status
    type: px4_msgs::msg::GimbalDeviceAttitudeStatus
    rate_limit: 20.
  
  - topic: /fmu/out/esc_status
    type: px4_msgs::msg::EscStatus

# Create uORB::Publication
subscriptions:
  - topic: /fmu/in/attack_command
    type: px4_msgs::msg::AttackCommand

  - topic: /fmu/in/wind_command
    type: px4_msgs::msg::WindCommand

  - topic: /fmu/in/register_ext_component_request
    type: px4_msgs::msg::RegisterExtComponentRequest

  - topic: /fmu/in/unregister_ext_component
    type: px4_msgs::msg::UnregisterExtComponent

  - topic: /fmu/in/config_overrides_request
    type: px4_msgs::msg::ConfigOverrides

  - topic: /fmu/in/arming_check_reply
    type: px4_msgs::msg::ArmingCheckReply

  - topic: /fmu/in/message_format_request
    type: px4_msgs::msg::MessageFormatRequest

  - topic: /fmu/in/mode_completed
    type: px4_msgs::msg::ModeCompleted

  - topic: /fmu/in/config_control_setpoints
    type: px4_msgs::msg::VehicleControlMode

  - topic: /fmu/in/distance_sensor
    type: px4_msgs::msg::DistanceSensor

  - topic: /fmu/in/manual_control_input
    type: px4_msgs::msg::ManualControlSetpoint

  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode

  - topic: /fmu/in/onboard_computer_status
    type: px4_msgs::msg::OnboardComputerStatus

  - topic: /fmu/in/obstacle_distance
    type: px4_msgs::msg::ObstacleDistance

  - topic: /fmu/in/sensor_optical_flow
    type: px4_msgs::msg::SensorOpticalFlow

  - topic: /fmu/in/goto_setpoint
    type: px4_msgs::msg::GotoSetpoint

  - topic: /fmu/in/telemetry_status
    type: px4_msgs::msg::TelemetryStatus

  - topic: /fmu/in/trajectory_setpoint
    type: px4_msgs::msg::TrajectorySetpoint

  - topic: /fmu/in/vehicle_attitude_setpoint
    type: px4_msgs::msg::VehicleAttitudeSetpoint

  - topic: /fmu/in/vehicle_mocap_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_command
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_command_mode_executor
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint

  - topic: /fmu/in/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint

  - topic: /fmu/in/actuator_motors
    type: px4_msgs::msg::ActuatorMotors

  - topic: /fmu/in/actuator_servos
    type: px4_msgs::msg::ActuatorServos

  - topic: /fmu/in/aux_global_position
    type: px4_msgs::msg::VehicleGlobalPosition

  - topic: /fmu/in/fixed_wing_longitudinal_setpoint
    type: px4_msgs::msg::FixedWingLongitudinalSetpoint

  - topic: /fmu/in/fixed_wing_lateral_setpoint
    type: px4_msgs::msg::FixedWingLateralSetpoint

  - topic: /fmu/in/longitudinal_control_configuration
    type: px4_msgs::msg::LongitudinalControlConfiguration

  - topic: /fmu/in/lateral_control_configuration
    type: px4_msgs::msg::LateralControlConfiguration

  - topic: /fmu/in/rover_position_setpoint
    type: px4_msgs::msg::RoverPositionSetpoint

  - topic: /fmu/in/rover_speed_setpoint
    type: px4_msgs::msg::RoverSpeedSetpoint

  - topic: /fmu/in/rover_attitude_setpoint
    type: px4_msgs::msg::RoverAttitudeSetpoint

  - topic: /fmu/in/rover_rate_setpoint
    type: px4_msgs::msg::RoverRateSetpoint

  - topic: /fmu/in/rover_throttle_setpoint
    type: px4_msgs::msg::RoverThrottleSetpoint

  - topic: /fmu/in/rover_steering_setpoint
    type: px4_msgs::msg::RoverSteeringSetpoint

  - topic: /fmu/in/landing_gear
    type: px4_msgs::msg::LandingGear

# Create uORB::PublicationMulti
subscriptions_multi:

~~~

## 3.10* 修改无人机端 LOGGER 配置

注：* 表示这段修改发生在 4.11 之后。

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "logged_topics.h"
#include "messages.h"

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

void LoggedTopics::add_default_topics()
{
	add_topic("action_request");
	add_topic("actuator_armed");
	add_optional_topic("actuator_controls_status_0", 300);
	add_topic("airspeed", 1000);
	add_optional_topic("airspeed_validated", 200);
	add_topic("attack_command");
	add_topic("attack_status", 100);
	add_optional_topic("autotune_attitude_control_status", 100);
	add_topic_multi("battery_info", 5000, 3);
	add_optional_topic("camera_capture");
	add_optional_topic("camera_trigger");
	add_topic("cellular_status", 200);
	add_topic("commander_state");
	add_topic("config_overrides");
	add_topic("cpuload");
	add_topic("distance_sensor_mode_change_request");
	add_topic_multi("dronecan_node_status", 250);
	add_optional_topic("external_ins_attitude");
	add_optional_topic("external_ins_global_position");
	add_optional_topic("external_ins_local_position");
	// add_optional_topic("esc_status", 250);
	add_topic("esc_status");
	add_topic("failure_detector_status", 100);
	add_topic("failsafe_flags");
	add_optional_topic("follow_target", 500);
	add_optional_topic("follow_target_estimator", 200);
	add_optional_topic("follow_target_status", 400);
	add_optional_topic("flaps_setpoint", 1000);
	add_optional_topic("flight_phase_estimation", 1000);
	add_optional_topic("fuel_tank_status", 10);
	add_topic("gimbal_manager_set_attitude", 500);
	add_optional_topic("generator_status");
	add_optional_topic("gps_dump");
	add_optional_topic("gimbal_controls", 200);
	add_optional_topic("gripper");
	add_optional_topic("heater_status");
	add_topic("home_position");
	add_topic("hover_thrust_estimate", 100);
	add_topic("input_rc", 500);
	add_optional_topic("internal_combustion_engine_control", 10);
	add_optional_topic("internal_combustion_engine_status", 10);
	add_optional_topic("iridiumsbd_status", 1000);
	add_optional_topic("irlock_report", 1000);
	add_optional_topic("landing_gear", 200);
	add_optional_topic("landing_gear_wheel", 100);
	add_optional_topic("landing_target_pose", 1000);
	add_optional_topic("launch_detection_status", 200);
	add_optional_topic("magnetometer_bias_estimate", 200);
	add_topic("manual_control_setpoint", 200);
	add_topic("manual_control_switches");
	add_topic("mission_result");
	add_topic("navigator_mission_item");
	add_topic("navigator_status");
	add_topic("offboard_control_mode", 100);
	add_topic("onboard_computer_status", 10);
	add_topic("parameter_update");
	add_topic("position_controller_status", 500);
	add_topic("position_controller_landing_status", 100);
	add_optional_topic("pure_pursuit_status", 100);
	add_topic("goto_setpoint", 200);
	add_topic("position_setpoint_triplet", 200);
	add_optional_topic("px4io_status");
	add_topic("radio_status");
	add_optional_topic("rover_attitude_setpoint", 100);
	add_optional_topic("rover_attitude_status", 100);
	add_optional_topic("rover_position_setpoint", 100);
	add_optional_topic("rover_rate_setpoint", 100);
	add_optional_topic("rover_rate_status", 100);
	add_optional_topic("rover_speed_setpoint", 100);
	add_optional_topic("rover_speed_status", 100);
	add_optional_topic("rover_steering_setpoint", 100);
	add_optional_topic("rover_throttle_setpoint", 100);
	add_topic("rtl_time_estimate", 1000);
	add_topic("rtl_status", 2000);
	add_optional_topic("sensor_airflow", 100);
	add_topic("sensor_combined");
	add_optional_topic("sensor_correction");
	add_optional_topic("sensor_gyro_fft", 50);
	add_topic("sensor_selection");
	add_topic("sensors_status_imu", 200);
	add_optional_topic("spoilers_setpoint", 1000);
	add_topic("system_power", 500);
	add_optional_topic("takeoff_status", 1000);
	add_optional_topic("tecs_status", 200);
	add_optional_topic("tiltrotor_extra_controls", 100);
	add_topic("trajectory_setpoint", 200);
	add_topic("transponder_report");
	add_topic("vehicle_acceleration", 50);
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_angular_velocity", 20);
	add_topic("vehicle_attitude", 50);
	add_topic("vehicle_attitude_setpoint", 50);
	add_topic("vehicle_command");
	add_topic("vehicle_command_ack");
	add_topic("vehicle_constraints", 1000);
	add_topic("vehicle_control_mode");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position", 100);
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 20);
	add_topic("vehicle_roi", 1000);
	add_topic("vehicle_status");
	add_optional_topic("vtol_vehicle_status", 200);
	add_topic("wind", 1000);
	add_topic("wind_command");
	add_topic("fixed_wing_lateral_setpoint");
	add_topic("fixed_wing_longitudinal_setpoint");
	add_topic("longitudinal_control_configuration");
	add_topic("lateral_control_configuration");
	add_optional_topic("fixed_wing_lateral_guidance_status", 100);
	add_optional_topic("fixed_wing_lateral_status", 100);
	add_optional_topic("fixed_wing_runway_control", 100);

	// multi topics
	add_optional_topic_multi("actuator_outputs", 100, 3);
	add_optional_topic_multi("airspeed_wind", 1000, 4);
	add_optional_topic_multi("control_allocator_status", 200, 2);
	add_optional_topic_multi("rate_ctrl_status", 200, 2);
	add_optional_topic_multi("sensor_hygrometer", 500, 4);
	add_optional_topic_multi("rpm", 200);
	add_topic_multi("timesync_status", 1000, 3);
	add_optional_topic_multi("telemetry_status", 1000, 4);

	// EKF multi topics
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 500; // 2 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	// important EKF topics (higher rate)
	add_optional_topic("estimator_selector_status", 10);
	add_optional_topic_multi("estimator_event_flags", 10);
	add_optional_topic_multi("estimator_optical_flow_vel", 200);
	add_optional_topic_multi("estimator_sensor_bias", 1000);
	add_optional_topic_multi("estimator_status", 200);
	add_optional_topic_multi("estimator_status_flags", 10);
	add_optional_topic_multi("yaw_estimator_status", 1000);

	// log all raw sensors at minimal rate (at least 1 Hz)
	add_topic_multi("battery_status", 200, 3);
	add_topic_multi("differential_pressure", 1000, 2);
	add_topic_multi("distance_sensor", 1000, 2);
	add_optional_topic_multi("sensor_accel", 1000, 4);
	add_topic_multi("sensor_baro", 1000, 4);
	add_topic_multi("sensor_gps", 1000, 2);
	add_topic_multi("sensor_gnss_relative", 1000, 1);
	add_optional_topic_multi("sensor_gyro", 1000, 4);
	add_topic_multi("sensor_mag", 1000, 4);
	add_topic_multi("sensor_optical_flow", 1000, 2);

	add_topic_multi("vehicle_imu", 500, 4);
	add_topic_multi("vehicle_imu_status", 1000, 4);
	add_optional_topic_multi("vehicle_magnetometer", 500, 4);
	add_topic("vehicle_optical_flow", 500);
	add_topic("aux_global_position", 500);
	//add_optional_topic("vehicle_optical_flow_vel", 100);
	add_optional_topic("pps_capture");

	// additional control allocation logging
	add_topic("actuator_motors", 100);
	add_topic("actuator_servos", 100);
	add_topic_multi("vehicle_thrust_setpoint", 20, 2);
	add_topic_multi("vehicle_torque_setpoint", 20, 2);

	// SYS_HITL: default ground truth logging for simulation
	int32_t sys_hitl = 0;
	param_get(param_find("SYS_HITL"), &sys_hitl);

	if (sys_hitl >= 1) {
		add_topic("vehicle_angular_velocity_groundtruth", 10);
		add_topic("vehicle_attitude_groundtruth", 10);
		add_topic("vehicle_global_position_groundtruth", 100);
		add_topic("vehicle_local_position_groundtruth", 20);
	}

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_optional_topic("vehicle_torque_setpoint_virtual_mc");
	add_optional_topic("vehicle_torque_setpoint_virtual_fw");
	add_optional_topic("vehicle_thrust_setpoint_virtual_mc");
	add_optional_topic("vehicle_thrust_setpoint_virtual_fw");
	add_topic("time_offset");
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_angular_velocity_groundtruth", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 20);

	// EKF replay
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 10; // 100 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	add_topic("vehicle_attitude");
	add_topic("vehicle_global_position");
	add_topic("vehicle_local_position");
	add_topic("wind");
	add_optional_topic_multi("yaw_estimator_status");

#endif /* CONFIG_ARCH_BOARD_PX4_SITL */

#ifdef CONFIG_BOARD_UAVCAN_INTERFACES
	add_topic_multi("can_interface_status", 100, CONFIG_BOARD_UAVCAN_INTERFACES);
#endif
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint");
	add_topic_multi("rate_ctrl_status", 20, 2);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");

	add_topic("esc_status", 5);
	add_topic("actuator_motors");
	add_topic("actuator_outputs_debug");
	add_topic("actuator_servos");
	add_topic_multi("vehicle_thrust_setpoint", 0, 2);
	add_topic_multi("vehicle_torque_setpoint", 0, 2);
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("actuator_test", 500);
	add_topic("neural_control", 50);
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("airspeed_validated");
	add_topic("vehicle_optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic("aux_global_position");
	add_topic_multi("distance_sensor");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic_multi("distance_sensor");
	add_topic("obstacle_distance_fused");
	add_topic("obstacle_distance");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_raw_imu_gyro_fifo()
{
	add_topic("sensor_gyro_fifo");
}

void LoggedTopics::add_raw_imu_accel_fifo()
{
	add_topic("sensor_accel_fifo");
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_torque_setpoint");
	add_topic("vehicle_acceleration");
	add_topic("actuator_motors");
}

void LoggedTopics::add_high_rate_sensors_topics()
{
	add_topic_multi("distance_sensor", 0, 4);
	add_topic_multi("sensor_optical_flow", 0, 2);
	add_topic_multi("sensor_gps", 0, 4);
	add_topic_multi("sensor_mag", 0, 4);
}

void LoggedTopics::add_mavlink_tunnel()
{
	add_topic("mavlink_tunnel");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %" PRIu32 " %" PRIu32, topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance, bool optional)
{
	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %" PRIu8, topic->o_name, instance);
		return false;
	}

	if (optional && orb_exists(topic, instance) != 0) {
		PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);

		if (instance == 0 && _subscriptions.num_excluded_optional_topic_ids < MAX_EXCLUDED_OPTIONAL_TOPICS_NUM) {
			_subscriptions.excluded_optional_topic_ids[_subscriptions.num_excluded_optional_topic_ids++] = topic->o_id;
		}

		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance, bool optional)
{
	interval_ms /= _rate_factor;

	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%" PRIu8 "), interval: %" PRIu16 ", already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance, optional);

				if (success) {
					PX4_DEBUG("logging topic: %s(%" PRIu8 "), interval: %" PRIu16, topics[i]->o_name, instance, interval_ms);
				}

				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms, uint8_t max_num_instances, bool optional)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < max_num_instances; instance++) {
		add_topic(name, interval_ms, instance, optional);
	}

	return true;
}

bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return _subscriptions.count > 0;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}

	if (profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO) {
		add_raw_imu_gyro_fifo();
	}

	if (profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) {
		add_raw_imu_accel_fifo();
	}

	if (profile & SDLogProfileMask::MAVLINK_TUNNEL) {
		add_mavlink_tunnel();
	}

	if (profile & SDLogProfileMask::HIGH_RATE_SENSORS) {
		add_high_rate_sensors_topics();
	}
}

~~~

第二次优化：

~~~python
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

~~~

## 3.11*  优化 node 文件

文件位置：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/wind_injector.py

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/wind_injector.py

第一次优化：

~~~python
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

import rclpy
import subprocess
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from px4_msgs.msg import WindCommand


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
        self.wind_pub = self.create_publisher(WindCommand, '/fmu/in/wind_command', 10)
        self.get_logger().info(f'wind_injector ready: {WIND_CMD_TOPIC} -> {self.wind_topic} (Vector3 m/s); also logged via /fmu/in/wind_command')

    def on_wind_cmd(self, msg):
        # gz topic 一次性发布 gz.msgs.Wind（linear_velocity + enable_wind）
        payload = (f'linear_velocity: {{x: {msg.x}, y: {msg.y}, z: {msg.z}}}, enable_wind: true')
        cmd = ['gz', 'topic', '-t', self.wind_topic, '-m', 'gz.msgs.Wind', '-p', payload]
        subprocess.run(cmd, check=False)

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

~~~

第二次优化：

~~~python
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

~~~

## 3.12 效果说明

文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config）设置了风速的上升时间为1秒，所以从发布风速控制到有明显的作用效果会有一点延迟，故风速发布不是立即生效的。

~~~xml
    <plugin entity_name="*" entity_type="world" filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>1</time_for_rise>  <!-- 风幅值，1s 时间常数 -->
          <sin>
            <amplitude_percent>0</amplitude_percent>
            <period>1</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>1</time_for_rise>  <!-- 风角度，1s 时间常数 -->
          <sin>
            <amplitude>0</amplitude>
            <period>1</period>
          </sin>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0</stddev>
        </noise>
      </vertical>
    </plugin>
~~~

# 四、仿真攻击注入

## 4.1 构建攻击原语库

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackPrimitive.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackPrimitive.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackPrimitive.hpp

~~~c++
/****************************************************************************
 *
 * Attack injection — primitive library.
 *
 * Pure "how to tamper with a single scalar value" logic. No PX4 / Gazebo / 
 * uORB dependency, so it compiles and unit-tests standalone.
 *
 * How attack injection works
 * --------------------------
 * The feature tampers with the data the flight controller consumes (GPS
 * fixes and motor outputs) so we can study how it reacts to a compromise.
 * Injection stays out of PX4's own flight code and lives entirely in the
 * gz_bridge process, split into three layers:
 *
 *   Layer A — stateless tampering math. Thirteen primitives
 *       (bias, spoof, noise, scaling, drift, oscillation, random walk,
 *       quantize, clamp, freeze, drop, delay, replay) turn one clean
 *       scalar into a tampered one, or decide to drop it. Eleven are
 *       point transforms; delay / replay are time-series transforms that
 *       read back through the channel's recorded history.
 *
 *   Layer B — thin injection points inside gz_bridge:
 *       GZBridge::navSatCallback     GPS lat/lon/alt + velocity N/E/D
 *       GZMixingInterfaceESC         the 4 motor outputs
 *       Each point feeds its "truth" value to the manager and publishes
 *       whatever comes back (or drops the message).
 *
 *   Layer C — AttackManager: routing + control + status. It owns a
 *       per-channel table of AttackSpec (configuration) + ChannelState
 *       (runtime), consumes uORB `attack_command`, applies attacks on
 *       demand, and publishes `attack_status` at 10 Hz.
 *
 * Data flow: an external node publishes attack_command -> AttackManager
 * resolves the relative window (t0 delay / t1 duration) to absolute hrt
 * time and stores the spec -> each injection point calls apply(truth) every
 * cycle -> the tampered value (or a drop) is what the flight controller
 * sees. Configuration is zero-parameter: it all travels as uORB messages and
 * is logged to ulog, so a run is reproducible from its log alone.
 *
 ****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace attack
{

/**
 * Attack shape: how a single scalar value is tampered with.
 *
 * The numeric value is the wire number carried by attack_command /
 * attack_status, so it must stay stable once released.
 */
enum class PrimitiveType : uint8_t {
	NONE = 0,      ///< Pass-through (attack disabled).
	BIAS,          ///< v = truth + a
	SPOOF,         ///< v = a
	NOISE,         ///< v = truth + N(0, a) (seeded)
	SCALING,       ///< v = truth * a
	DRIFT,         ///< v = truth + a * (t - start) (a = slope)
	OSCILLATION,   ///< v = truth + a * sin(b*(t - start) + c) (a = amplitude, b = angular rate, c = phase)
	RANDOM_WALK,   ///< v = v_last + N(0, a) (stateful)
	QUANTIZE,      ///< v = round(truth / a) * a (a = step)
	CLAMP,         ///< v = clamp(truth, a, b) (a = min, b = max)
	FREEZE,        ///< v = v_last (stateful)
	DROP,          ///< drop this message with probability a (a = drop probability)
	DELAY,         ///< v = value from a seconds ago (a = delay [s])
	REPLAY,        ///< v = loop of the a seconds before start (a = segment length [s])
};

/**
 * Injection-point vocabulary.
 *
 * A channel is a plain index; its meaning lives at the injection point
 * (GZBridge for GPS, GZMixingInterfaceESC for motors).
 */
enum class Channel : uint8_t {
	GPS_LAT = 0,        ///< GPS latitude [deg]
	GPS_LON,            ///< GPS longitude [deg]
	GPS_ALT,            ///< GPS altitude [m]
	GPS_VEL_N,          ///< GPS velocity north [m/s]
	GPS_VEL_E,          ///< GPS velocity east [m/s]
	GPS_VEL_D,          ///< GPS velocity down [m/s]
	MOTOR_0 = 6,        ///< motor 0 speed [rpm]
	MOTOR_1,            ///< motor 1 speed [rpm]
	MOTOR_2,            ///< motor 2 speed [rpm]
	MOTOR_3,            ///< motor 3 speed [rpm]
	NUM_CHANNELS = 10,  ///< number of channels (10)
};

/// Number of attack channels (GPS 6 + motors 4).
constexpr size_t kNumChannels = static_cast<size_t>(Channel::NUM_CHANNELS);

/// Capacity of each channel's time-series buffers (history + replay snapshot).
/// Bounds the max DELAY / REPLAY lookback to ~capacity / sample rate.
constexpr size_t kTimeSeriesCapacity = 4096;

/**
 * Array index of a channel in the per-channel tables (specs / state).
 */
constexpr size_t index_of(Channel ch)
{
	return static_cast<size_t>(ch);
}

/**
 * Motor channel for motor i (MOTOR_0 + i). Out-of-range i yields an invalid
 * channel that AttackManager::apply() safely treats as pass-through.
 */
constexpr Channel motor_channel(unsigned i)
{
	return static_cast<Channel>(static_cast<uint8_t>(Channel::MOTOR_0) + i);
}

/**
 * Serializable attack configuration for one channel.
 *
 * This is the unit that travels in attack_command and is logged for
 * reproducibility. It is a pure value; runtime state lives in ChannelState.
 */
struct AttackSpec {
	PrimitiveType type{PrimitiveType::NONE}; ///< Attack shape (NONE = disabled).
	double a{0.0};                           ///< Parameter a (meaning depends on type).
	double b{0.0};                           ///< Parameter b (meaning depends on type).
	double c{0.0};                           ///< Parameter c (meaning depends on type).
	double d{0.0};                           ///< Parameter d (meaning depends on type).
	uint64_t start_us{0};                    ///< Absolute hrt time the attack becomes active.
	uint64_t end_us{0};                      ///< Absolute hrt time the attack ends; 0 = no end.
	uint32_t seed{0};                        ///< RNG seed for stochastic primitives; 0 = per-channel default.
};

/**
 * Per-channel runtime state, owned by AttackManager (not serialized).
 */
struct ChannelState {
	double last_value{0.0};  ///< Last value that passed through (FREEZE / RANDOM_WALK).
	bool has_last{false};    ///< Whether last_value is valid.
	uint32_t rng_state{0};   ///< PRNG state, seeded by AttackManager::set().

	// Clean "truth" stream, recorded on every apply() call even while idle.
	// Never reset by set(): it is the channel's data, not the attack's state.
	uint64_t hist_time[kTimeSeriesCapacity]{};  ///< Sample timestamps [us], oldest..newest.
	double hist_value[kTimeSeriesCapacity]{};   ///< Sample values, same order as hist_time.
	size_t hist_head{0};                        ///< Next write slot.
	size_t hist_count{0};                       ///< Valid entries (<= capacity).

	// REPLAY loop snapshot, captured once at attack start and then cycled.
	uint64_t replay_offset[kTimeSeriesCapacity]{};  ///< Offset from (start - a), us.
	double replay_value[kTimeSeriesCapacity]{};     ///< Snapshot values.
	size_t replay_count{0};                         ///< Valid snapshot entries.
	bool replay_captured{false};                    ///< Whether the snapshot is valid.
};

/**
 * Result of applying an attack to one value.
 */
struct AttackResult {
	bool pass{true};   ///< false => drop the message (DROP only).
	double value{0.0}; ///< Tampered value; valid when pass is true.
};

/**
 * Stateless attack math.
 *
 * All per-channel state is passed in via ChannelState, so this class holds no
 * mutable state of its own.
 */
class AttackLibrary
{
public:
	/**
	 * Apply an attack to a scalar truth value.
	 *
	 * Every call first records (now_us, truth) into state's history (used
	 * by DELAY / REPLAY) before applying any transform.
	 *
	 * @param truth   the clean value to (possibly) tamper with
	 * @param spec    attack configuration for this channel
	 * @param now_us  current time, hrt_absolute_time() in microseconds
	 * @param state   per-channel runtime state (advanced by stochastic primitives)
	 * @return        pass=false means the message must be dropped (DROP only);
	 *                otherwise `value` is the tampered value
	 */
	static AttackResult apply(double truth, const AttackSpec &spec, uint64_t now_us, ChannelState &state);

	/**
	 * Whether an attack is currently within its active [start_us, end_us) window.
	 */
	static bool active(const AttackSpec &spec, uint64_t now_us);
};

} // namespace attack

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackPrimitive.cpp

~~~c++
/****************************************************************************
 *
 * Attack injection — primitive library, implementation.
 *
 ****************************************************************************/

#include <cmath>

#include "AttackPrimitive.hpp"


namespace attack
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/**
 * xorshift32 — small, fast, deterministic PRNG.
 *
 * @param state  PRNG state, advanced in place
 * @return       next 32-bit value
 */
uint32_t xorshift32(uint32_t &state)
{
	uint32_t x = (state == 0) ? 0x9E3779B9u : state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

/// Uniform random value in [0, 1).
double uniform(uint32_t &state)
{
	return static_cast<double>(xorshift32(state) >> 8) / 16777216.0;
}

/// Standard normal random value via Box-Muller.
double gaussian(uint32_t &state)
{
	double u1 = uniform(state);
	double u2 = uniform(state);

	if (u1 < 1e-12) {u1 = 1e-12;}

	return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

/// Seconds elapsed since the attack start (clamped to >= 0).
double elapsed_s(const AttackSpec &spec, uint64_t now_us)
{
	if (now_us <= spec.start_us) {return 0.0;}

	return static_cast<double>(now_us - spec.start_us) / 1e6;
}

/**
 * Append a sample to the channel's history ring buffer.
 */
void record_history(ChannelState &state, uint64_t now_us, double value)
{
	state.hist_time[state.hist_head] = now_us;
	state.hist_value[state.hist_head] = value;
	state.hist_head = (state.hist_head + 1) % kTimeSeriesCapacity;
	if (state.hist_count < kTimeSeriesCapacity) {state.hist_count++;}
}

/**
 * Value of the newest history sample with timestamp <= target_us.
 * Falls back to the oldest sample if target_us predates the history.
 * Returns false (leaving `out` untouched) when the history is empty.
 */
bool history_lookup(const ChannelState &state, uint64_t target_us, double &out)
{
	if (state.hist_count == 0) {
		return false;
	}

	// Walk backward from the newest sample; history is time-ordered.
	for (size_t i = 0; i < state.hist_count; i++) {
		const size_t idx = (state.hist_head + kTimeSeriesCapacity - 1 - i) % kTimeSeriesCapacity;

		if (state.hist_time[idx] <= target_us) {
			out = state.hist_value[idx];
			return true;
		}
	}

	// target_us predates the oldest sample: fall back to the oldest.
	const size_t oldest = (state.hist_head + kTimeSeriesCapacity - state.hist_count) % kTimeSeriesCapacity;
	out = state.hist_value[oldest];
	return true;
}

/**
 * Capture the [start - a, start] history segment into the REPLAY snapshot.
 * Offsets are relative to (start - a), so the replay lookup can index by
 * phase = (now - start) % a.
 */
void replay_capture(ChannelState &state, const AttackSpec &spec)
{
	const uint64_t period_us = static_cast<uint64_t>(spec.a * 1e6);
	const uint64_t lo = (period_us > spec.start_us) ? 0 : (spec.start_us - period_us);

	state.replay_count = 0;
	state.replay_captured = true;

	if (period_us == 0 || state.hist_count == 0) {
		return;
	}

	const size_t oldest = (state.hist_head + kTimeSeriesCapacity - state.hist_count) % kTimeSeriesCapacity;

	// Skip entries older than the segment start (t < lo).
	size_t begin = 0;

	while (begin < state.hist_count && state.hist_time[(oldest + begin) % kTimeSeriesCapacity] < lo) {
		begin++;
	}

	// Copy entries within [lo, start_us), oldest first. The sample at exactly
	// start_us is the attack's own current input, not "before start".
	for (size_t i = begin; i < state.hist_count; i++) {
		const size_t idx = (oldest + i) % kTimeSeriesCapacity;
		const uint64_t t = state.hist_time[idx];

		if (t >= spec.start_us) {
			break;
		}

		state.replay_offset[state.replay_count] = t - lo;
		state.replay_value[state.replay_count] = state.hist_value[idx];
		state.replay_count++;
	}
}

/**
 * Compute the tampered value for an already-active attack.
 *
 * The caller has already established that `type != NONE` and `now_us` is
 * within the [start_us, end_us) window.
 *
 * @return pass=false only for DROP; `value` is then irrelevant.
 */
AttackResult transform(PrimitiveType type, double truth, const AttackSpec &spec, uint64_t now_us, ChannelState &state)
{
	switch (type) {
	case PrimitiveType::BIAS:
		return AttackResult{true, truth + spec.a};

	case PrimitiveType::SPOOF:
		return AttackResult{true, spec.a};

	case PrimitiveType::NOISE:
		return AttackResult{true, truth + spec.a * gaussian(state.rng_state)};

	case PrimitiveType::SCALING:
		return AttackResult{true, truth * spec.a};

	case PrimitiveType::DRIFT:
		return AttackResult{true, truth + spec.a * elapsed_s(spec, now_us)};

	case PrimitiveType::OSCILLATION:
		return AttackResult{true, truth + spec.a * std::sin(spec.b * elapsed_s(spec, now_us) + spec.c)};

	case PrimitiveType::RANDOM_WALK:
		return AttackResult{true, (state.has_last ? state.last_value : truth) + spec.a * gaussian(state.rng_state)};

	case PrimitiveType::QUANTIZE:
		// Step must have a non-negligible magnitude, otherwise pass through.
		if (std::fabs(spec.a) > 1e-12) {return AttackResult{true, std::round(truth / spec.a) * spec.a};}
		return AttackResult{true, truth};

	case PrimitiveType::CLAMP:
		return AttackResult{true, (truth < spec.a) ? spec.a : ((truth > spec.b) ? spec.b : truth)};

	case PrimitiveType::FREEZE:
		return AttackResult{true, state.has_last ? state.last_value : truth};

	case PrimitiveType::DROP:
		return AttackResult{uniform(state.rng_state) >= spec.a, truth};

	case PrimitiveType::DELAY: {
		const uint64_t delay_us = static_cast<uint64_t>(spec.a * 1e6);
		const uint64_t target_us = (delay_us > now_us) ? 0 : (now_us - delay_us);
		double v = truth;
		history_lookup(state, target_us, v);
		return AttackResult{true, v};
	}

	case PrimitiveType::REPLAY: {
		const uint64_t period_us = static_cast<uint64_t>(spec.a * 1e6);

		if (period_us == 0) {
			return AttackResult{true, truth};  // a <= 0: nothing to replay.
		}

		if (!state.replay_captured) {
			replay_capture(state, spec);
		}

		const uint64_t phase_us = (now_us - spec.start_us) % period_us;
		double v = (state.replay_count > 0) ? state.replay_value[0] : truth;

		for (size_t i = 0; i < state.replay_count; i++) {
			if (state.replay_offset[i] <= phase_us) {
				v = state.replay_value[i];

			} else {
				break;
			}
		}

		return AttackResult{true, v};
	}

	case PrimitiveType::NONE:
		return AttackResult{true, truth};

	default:
		return AttackResult{true, truth};
	}
}

} // namespace

bool AttackLibrary::active(const AttackSpec &spec, uint64_t now_us)
{
	if (spec.type == PrimitiveType::NONE) {
		return false;
	}

	if (now_us < spec.start_us) {
		return false;
	}

	if (spec.end_us != 0 && now_us >= spec.end_us) {
		return false;
	}

	return true;
}

AttackResult AttackLibrary::apply(double truth, const AttackSpec &spec, uint64_t now_us, ChannelState &state)
{
	// Record the clean truth into history on every sample (even while idle) so
	// DELAY / REPLAY have pre-roll data to look back through.
	record_history(state, now_us, truth);

	// Disabled or outside the window: pass through, but keep last_value fresh
	// so FREEZE / RANDOM_WALK start from the current value when activated.
	if (!active(spec, now_us)) {
		state.last_value = truth;
		state.has_last = true;
		return AttackResult{true, truth};
	}

	const AttackResult result = transform(spec.type, truth, spec, now_us, state);

	// Remember the last value that passed through (FREEZE / RANDOM_WALK use it).
	if (result.pass) {
		state.last_value = result.value;
		state.has_last = true;
	}

	return result;
}

} // namespace attack

~~~

## 4.2 构建攻击管理库

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackManager.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackManager.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackManager.hpp

~~~c++
/****************************************************************************
 *
 * Attack injection — attack manager.
 *
 * Owns the per-channel attack table (Channel -> AttackSpec + ChannelState),
 * consumes uORB `attack_command`, applies attacks on demand, and publishes
 * uORB `attack_status` at 10 Hz.
 *
 * Threading: GPS injection runs on a gz-transport callback thread while the
 * motor injection and command handling run on the rate_ctrl work queue, so
 * every table access is guarded by a pthread mutex.
 *
 ****************************************************************************/

#pragma once

#include "AttackPrimitive.hpp"

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/attack_command.h>
#include <uORB/topics/attack_status.h>

#include <pthread.h>

namespace attack
{

/**
 * Routes attack commands to per-channel specifications and applies them.
 */
class AttackManager
{
public:
	AttackManager() = default;
	~AttackManager();

	// Owns a mutex and uORB handles: not copyable / movable.
	AttackManager(const AttackManager &) = delete;
	AttackManager &operator=(const AttackManager &) = delete;

	/**
	 * Initialize the mutex and advertise the status topic.
	 * @return true on success
	 */
	bool init();

	/**
	 * Apply the attack configured for `ch` to `value`.
	 *
	 * @param ch       channel to attack
	 * @param value    in: clean value; out: (possibly) tampered value
	 * @param now_us   current hrt time in microseconds
	 * @return         false => the message must be dropped (DROP primitive fired);
	 *                 true  => `value` is valid and should be used
	 */
	bool apply(Channel ch, double &value, uint64_t now_us);

	/**
	 * Mount / replace the attack spec for a channel (resets its runtime state,
	 * including reseeding the PRNG: `spec.seed` if non-zero, else a per-channel
	 * deterministic default).
	 */
	void set(Channel ch, const AttackSpec &spec);

	/**
	 * Remove the attack on a channel (equivalent to set() with type=NONE).
	 */
	void clear(Channel ch);

	/**
	 * Consume the latest attack_command and publish attack_status at 10 Hz.
	 * Call periodically from the main loop (~100 Hz).
	 */
	void update();

private:
	/**
	 * Resolve a relative-time command (t0_us / t1_us) into an absolute
	 * AttackSpec and route it to the channel.
	 */
	void on_command(const attack_command_s &cmd, uint64_t now_us);

	/**
	 * Snapshot the current table into an attack_status message and publish it.
	 */
	void publish_status(uint64_t now_us);

	AttackSpec   _specs[kNumChannels]{};  ///< Per-channel attack configuration.
	ChannelState _state[kNumChannels]{};  ///< Per-channel runtime state (stochastic primitives).

	uORB::Subscription                 _cmd_sub{ORB_ID(attack_command)};
	uORB::Publication<attack_status_s> _status_pub{ORB_ID(attack_status)};

	pthread_mutex_t _lock{};
	bool _lock_initialized{false};

	uint64_t _last_status_us{0};  ///< hrt time of the last status publish (10 Hz gating).
};

} // namespace attack

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/attack/AttackManager.cpp

~~~c++
/****************************************************************************
 *
 * Attack injection — attack manager, implementation.
 *
 ****************************************************************************/

#include "AttackManager.hpp"

#include <drivers/drv_hrt.h>

namespace attack
{
namespace
{

constexpr uint64_t STATUS_PERIOD_US = 100000;  // 10 Hz status publish

/**
 * Deterministic per-channel seed for `seed == 0` commands.
 *
 * The default is reproducible (an experiment reruns identically without
 * remembering a seed) but distinct per channel, so two channels attacked with
 * the same stochastic primitive don't share one identical noise stream and
 * become perfectly correlated (e.g. GPS lat/lon drifting along a straight
 * diagonal instead of a 2-D random walk).
 */
uint32_t default_seed(size_t idx)
{
	constexpr uint32_t kBase = 0x9E3779B9u;
	return kBase + static_cast<uint32_t>(idx);
}

/**
 * Build an AttackSpec from a uORB command, resolving the relative window
 * (t0_us delay, t1_us duration) to absolute hrt times.
 */
AttackSpec make_spec(const attack_command_s &cmd, uint64_t now_us)
{
	AttackSpec spec{};
	spec.type = static_cast<PrimitiveType>(cmd.type);
	spec.a = cmd.param[0];
	spec.b = cmd.param[1];
	spec.c = cmd.param[2];
	spec.d = cmd.param[3];
	spec.seed = cmd.seed;

	spec.start_us = now_us + cmd.t0_us;
	spec.end_us = (cmd.t1_us != 0) ? spec.start_us + cmd.t1_us : 0;
	return spec;
}

} // namespace

AttackManager::~AttackManager()
{
	if (_lock_initialized) {
		pthread_mutex_destroy(&_lock);
	}
}

bool AttackManager::init()
{
	if (pthread_mutex_init(&_lock, nullptr) != 0) {
		return false;
	}

	_lock_initialized = true;

	// Advertise up front so subscribers (logger / DDS) see the topic immediately.
	_status_pub.advertise();
	return true;
}

void AttackManager::set(Channel ch, const AttackSpec &spec)
{
	const size_t idx = index_of(ch);

	if (idx >= kNumChannels) {
		return;
	}

	pthread_mutex_lock(&_lock);

	_specs[idx] = spec;

	// Reset the runtime state. Seeding here (not inside apply()) keeps the
	// stochastic primitives reproducible: seed != 0 uses spec.seed, seed == 0
	// uses the per-channel deterministic default. The history buffer is left
	// intact: it is the channel's data stream, not the current attack's state,
	// and DELAY / REPLAY need its pre-roll.
	ChannelState &state = _state[idx];
	state.last_value = 0.0;
	state.has_last = false;
	state.rng_state = (spec.seed != 0) ? spec.seed : default_seed(idx);
	state.replay_captured = false;
	state.replay_count = 0;

	pthread_mutex_unlock(&_lock);
}

void AttackManager::clear(Channel ch)
{
	AttackSpec none{};  // type == NONE => pass through
	set(ch, none);
}

bool AttackManager::apply(Channel ch, double &value, uint64_t now_us)
{
	const size_t idx = index_of(ch);

	if (idx >= kNumChannels) {
		return true;  // Unknown channel: pass through untouched.
	}

	pthread_mutex_lock(&_lock);
	const AttackResult result = AttackLibrary::apply(value, _specs[idx], now_us, _state[idx]);
	pthread_mutex_unlock(&_lock);

	if (result.pass) {
		value = result.value;
	}

	return result.pass;
}

void AttackManager::update()
{
	const uint64_t now = hrt_absolute_time();

	// Consume the latest command (between polls, the newest one wins).
	attack_command_s cmd{};

	if (_cmd_sub.update(&cmd)) {
		on_command(cmd, now);
	}

	// Publish status at 10 Hz.
	if (now - _last_status_us >= STATUS_PERIOD_US) {
		publish_status(now);
		_last_status_us = now;
	}
}

void AttackManager::on_command(const attack_command_s &cmd, uint64_t now_us)
{
	if (cmd.channel >= kNumChannels) {
		return;  // Unknown channel.
	}

	set(static_cast<Channel>(cmd.channel), make_spec(cmd, now_us));
}

void AttackManager::publish_status(uint64_t now_us)
{
	attack_status_s status{};
	status.timestamp = now_us;

	pthread_mutex_lock(&_lock);

	for (size_t ch = 0; ch < kNumChannels; ch++) {
		status.active[ch] = AttackLibrary::active(_specs[ch], now_us) ? 1 : 0;
		status.type[ch] = static_cast<uint8_t>(_specs[ch].type);
		status.param_a[ch] = _specs[ch].a;
		status.param_b[ch] = _specs[ch].b;
		status.param_c[ch] = _specs[ch].c;
		status.param_d[ch] = _specs[ch].d;
	}

	pthread_mutex_unlock(&_lock);

	_status_pub.publish(status);
}

} // namespace attack

~~~

## 4.3 修改仿真编译配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

if(NOT DEFINED ENV{GZ_DISTRO} OR NOT "$ENV{GZ_DISTRO}" STREQUAL "harmonic")
    find_package(gz-transport NAMES gz-transport gz-transport14 gz-transport13)
else()
    find_package(gz-transport NAMES gz-transport13)
endif()

file(GLOB gz_worlds ${PX4_SOURCE_DIR}/Tools/simulation/gz/worlds/*.sdf)
file(GLOB gz_airframes ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*)

if (gz-transport_FOUND)
    if (gz-transport_VERSION VERSION_LESS "15")
        set(GZ_TRANSPORT_TARGET "gz-transport${gz-transport_VERSION_MAJOR}::core")
    else()
        set(GZ_TRANSPORT_TARGET "gz-transport::core")
    endif()
	px4_add_module(
		MODULE modules__simulation__gz_bridge
		MAIN gz_bridge
		COMPILE_FLAGS
			${MAX_CUSTOM_OPT_LEVEL}
		SRCS
			attack/AttackManager.cpp
			attack/AttackManager.hpp
			attack/AttackPrimitive.cpp
			attack/AttackPrimitive.hpp
			GZBridge.cpp
			GZBridge.hpp
			GZMixingInterfaceESC.cpp
			GZMixingInterfaceESC.hpp
			GZMixingInterfaceServo.cpp
			GZMixingInterfaceServo.hpp
			GZMixingInterfaceWheel.cpp
			GZMixingInterfaceWheel.hpp
			GZGimbal.cpp
			GZGimbal.hpp
		DEPENDS
			mixer_module
			px4_work_queue
			${GZ_TRANSPORT_TARGET}
		MODULE_CONFIG
			module.yaml
	)

    target_include_directories(modules__simulation__gz_bridge
        PUBLIC
            ${PX4_GZ_MSGS_BINARY_DIR}
    )

	target_include_directories(modules__simulation__gz_bridge PUBLIC px4_gz_msgs)
	target_link_libraries(modules__simulation__gz_bridge PUBLIC px4_gz_msgs)

	px4_add_git_submodule(TARGET git_gz PATH "${PX4_SOURCE_DIR}/Tools/simulation/gz")
	include(ExternalProject)
	ExternalProject_Add(gz
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/gz
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/build_gz
		INSTALL_COMMAND ""
		DEPENDS git_gz
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
	)

	# Below we setup the build targets for our worlds and models
	# Syntax: gz_<model_name>_<world_name>
	# Example: gz_x500_flow_forest
	foreach(gz_airframe IN LISTS gz_airframes)
		set(model_name)
		string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})

		foreach(world ${gz_worlds})
			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name STREQUAL "default")
				add_custom_target(gz_${model_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} GZ_IP=127.0.0.1 $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 px4_gz_plugins
				)
			else()
				add_custom_target(gz_${model_name}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E env PX4_SIM_MODEL=gz_${model_name} PX4_GZ_WORLD=${world_name} GZ_IP=127.0.0.1 $<TARGET_FILE:px4>
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 px4_gz_plugins
				)
			endif()
		endforeach()
	endforeach()

	# Setup the environment variables: PX4_GZ_MODELS, PX4_GZ_WORLDS, GZ_SIM_RESOURCE_PATH
	configure_file(gz_env.sh.in ${PX4_BINARY_DIR}/rootfs/gz_env.sh)

else()
	# Create fallback targets that provide helpful error messages when Gazebo dependencies are missing
	foreach(gz_airframe IN LISTS gz_airframes)
		set(model_name)
		string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})

		foreach(world ${gz_worlds})
			get_filename_component("world_name" ${world} NAME_WE)

			if(world_name STREQUAL "default")
				add_custom_target(gz_${model_name}
					COMMAND ${CMAKE_COMMAND} -E echo "ERROR: Gazebo simulation dependencies not found!"
					COMMAND ${CMAKE_COMMAND} -E echo "  - For installation instructions, see: https://gazebosim.org/docs/harmonic/install_ubuntu/"
					COMMAND ${CMAKE_COMMAND} -E false
					VERBATIM
				)
			else()
				add_custom_target(gz_${model_name}_${world_name}
					COMMAND ${CMAKE_COMMAND} -E echo "ERROR: Gazebo simulation dependencies not found!"
					COMMAND ${CMAKE_COMMAND} -E echo "  - For installation instructions, see: https://gazebosim.org/docs/harmonic/install_ubuntu/"
					COMMAND ${CMAKE_COMMAND} -E false
					VERBATIM
				)
			endif()
		endforeach()
	endforeach()

	message(STATUS "Gazebo simulation bridge module disabled: missing dependencies")
endif()

~~~

## 4.4 注入传感器攻击

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.hpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "GZMixingInterfaceESC.hpp"
#include "GZMixingInterfaceServo.hpp"
#include "GZMixingInterfaceWheel.hpp"
#include "GZGimbal.hpp"
#include "attack/AttackManager.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/geo/geo.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/air_speed.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry_with_covariance.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/scene.pb.h>
// Custom PX4 proto
#include <opticalflow.pb.h>

using namespace time_literals;

class GZBridge : public ModuleBase<GZBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GZBridge(const std::string &world, const std::string &model_name);
	~GZBridge() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	int init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void Run() override;

	bool subscribeClock(bool required);
	bool subscribePoseInfo(bool required);
	bool subscribeImu(bool required);
	bool subscribeMag(bool required);
	bool subscribeOdometry(bool required);
	bool subscribeLaserScan(bool required);
	bool subscribeDistanceSensor(bool required);
	bool subscribeAirspeed(bool required);
	bool subscribeAirPressure(bool required);
	bool subscribeNavsat(bool required);
	bool subscribeOpticalFlow(bool required);

	void clockCallback(const gz::msgs::Clock &msg);
	void airspeedCallback(const gz::msgs::AirSpeed &msg);
	void airPressureCallback(const gz::msgs::FluidPressure &msg);
	void imuCallback(const gz::msgs::IMU &msg);
	void poseInfoCallback(const gz::msgs::Pose_V &msg);
	void odometryCallback(const gz::msgs::OdometryWithCovariance &msg);
	void navSatCallback(const gz::msgs::NavSat &msg);
	void laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg);
	void laserScanCallback(const gz::msgs::LaserScan &msg);
	void opticalFlowCallback(const px4::msgs::OpticalFlow &msg);
	void magnetometerCallback(const gz::msgs::Magnetometer &msg);

	static void rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU);

	static float generate_wgn();

	void addGpsNoise(double &latitude, double &longitude, double &altitude,
			 float &vel_north, float &vel_east, float &vel_down);

	uORB::SubscriptionInterval                    _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<distance_sensor_s>          _distance_sensor_pub{ORB_ID(distance_sensor)};
	uORB::Publication<differential_pressure_s>    _differential_pressure_pub{ORB_ID(differential_pressure)};
	uORB::Publication<obstacle_distance_s>        _obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_ground_truth_pub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Publication<vehicle_attitude_s>         _attitude_ground_truth_pub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Publication<vehicle_global_position_s>  _gpos_ground_truth_pub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<vehicle_local_position_s>   _lpos_ground_truth_pub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::PublicationMulti<sensor_gps_s>          _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<sensor_baro_s>         _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_accel_s>        _sensor_accel_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_gyro_s>         _sensor_gyro_pub{ORB_ID(sensor_gyro)};
	uORB::PublicationMulti<sensor_mag_s>          _sensor_mag_pub{ORB_ID(sensor_mag)};
	uORB::PublicationMulti<vehicle_odometry_s>    _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::PublicationMulti<sensor_optical_flow_s> _optical_flow_pub{ORB_ID(sensor_optical_flow)};


	attack::AttackManager _attack;

	GZMixingInterfaceESC   _mixing_interface_esc{_node};
	GZMixingInterfaceServo _mixing_interface_servo{_node};
	GZMixingInterfaceWheel _mixing_interface_wheel{_node};

	GZGimbal _gimbal{_node};

	MapProjection _pos_ref{};
	double _alt_ref{};

	matrix::Vector3d _position_prev{};
	matrix::Vector3d _velocity_prev{};
	matrix::Vector3f _euler_prev{};
	hrt_abstime _timestamp_prev{};

	const std::string _world_name;
	const std::string _model_name;

	float _temperature{288.15};  // 15 degrees

	bool _realtime_clock_set{false};
	gz::transport::Node _node;

	// GPS noise model
	float _gps_pos_noise_n = 0.0f;
	float _gps_pos_noise_e = 0.0f;
	float _gps_pos_noise_d = 0.0f;
	float _gps_vel_noise_n = 0.0f;
	float _gps_vel_noise_e = 0.0f;
	float _gps_vel_noise_d = 0.0f;
	const float _pos_noise_amplitude = 0.8f;    // Position noise amplitude [m]
	const float _pos_random_walk = 0.01f;       // Position random walk coefficient
	const float _pos_markov_time = 0.95f;       // Position Markov process coefficient
	const float _vel_noise_amplitude = 0.05f;   // Velocity noise amplitude [m/s]
	const float _vel_noise_density = 0.2f;      // Velocity noise process density
	const float _vel_markov_time = 0.85f;       // Velocity Markov process coefficient

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SIM_GPS_USED>) _sim_gps_used,
		(ParamInt<px4::params::SIM_GZ_EN_LIDAR>) _sim_gz_en_lidar,
		(ParamInt<px4::params::SIM_GZ_EN_FLOW>) _sim_gz_en_flow,
		(ParamInt<px4::params::SIM_GZ_EN_ASPD>) _sim_gz_en_aspd,
		(ParamInt<px4::params::SIM_GZ_EN_BARO>) _sim_gz_en_baro,
		(ParamInt<px4::params::SIM_GZ_EN_ODOM>) _sim_gz_en_odom,
		(ParamInt<px4::params::SIM_GZ_EN_GPS>) _sim_gz_en_gps
	)
};

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "GZBridge.hpp"

#include <uORB/Subscription.hpp>

#include <lib/atmosphere/atmosphere.h>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <iostream>
#include <string>

GZBridge::GZBridge(const std::string &world, const std::string &model_name) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_world_name(world),
	_model_name(model_name)
{
	updateParams();
}

GZBridge::~GZBridge()
{
	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int GZBridge::init()
{
	// REQUIRED:
	if (!subscribeClock(true)) {
		return PX4_ERROR;
	}

	// We must wait for clock before subscribing to other topics. This is because
	// if we publish a 0 timestamp it screws up the EKF.
	while (1) {
		px4_usleep(25000);

		if (_realtime_clock_set) {
			px4_usleep(25000);
			break;
		}
	}

	if (!subscribePoseInfo(true)) {
		return PX4_ERROR;
	}

	if (!subscribeImu(true)) {
		return PX4_ERROR;
	}

	if (!subscribeMag(true)) {
		return PX4_ERROR;
	}

	// OPTIONAL:
	if (_sim_gz_en_gps.get()) {
		if (!subscribeNavsat(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_baro.get()) {
		if (!subscribeAirPressure(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_lidar.get()) {
		if (!subscribeDistanceSensor(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_aspd.get()) {
		if (!subscribeAirspeed(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_flow.get()) {
		if (!subscribeOpticalFlow(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_odom.get()) {
		if (!subscribeOdometry(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_lidar.get()) {
		if (!subscribeLaserScan(false)) {
			return PX4_ERROR;
		}
	}

	// Attack injection manager: init first, then wire it into the ESC interface.
	if (!_attack.init()) {
		PX4_ERR("failed to init attack manager");
		return PX4_ERROR;
	}

	_mixing_interface_esc.setAttack(&_attack);

	// ESC mixing interface
	if (!_mixing_interface_esc.init(_model_name)) {
		PX4_ERR("failed to init ESC output");
		return PX4_ERROR;
	}

	// Servo mixing interface
	if (!_mixing_interface_servo.init(_model_name)) {
		PX4_ERR("failed to init servo output");
		return PX4_ERROR;
	}

	// Wheel mixing interface
	if (!_mixing_interface_wheel.init(_model_name)) {
		PX4_ERR("failed to init motor output");
		return PX4_ERROR;
	}

	// Gimbal mixing interface
	if (!_gimbal.init(_world_name, _model_name)) {
		PX4_ERR("failed to init gimbal");
		return PX4_ERROR;
	}

	ScheduleNow();
	return OK;
}

void GZBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();

		_mixing_interface_esc.stop();
		_mixing_interface_servo.stop();
		_mixing_interface_wheel.stop();
		_gimbal.stop();

		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();

		_mixing_interface_esc.updateParams();
		_mixing_interface_servo.updateParams();
		_mixing_interface_wheel.updateParams();
		_gimbal.updateParams();
	}

	_attack.update();

	ScheduleDelayed(10_ms);
}

bool GZBridge::subscribeClock(bool required)
{
	std::string clock_topic = "/world/" + _world_name + "/clock";

	if (!_node.Subscribe(clock_topic, &GZBridge::clockCallback, this)) {
		PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribePoseInfo(bool required)
{
	std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

	if (!_node.Subscribe(world_pose_topic, &GZBridge::poseInfoCallback, this)) {
		PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeImu(bool required)
{
	std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

	if (!_node.Subscribe(imu_topic, &GZBridge::imuCallback, this)) {
		PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeMag(bool required)
{
	std::string mag_topic = "/world/" + _world_name + "/model/" + _model_name +
				"/link/base_link/sensor/magnetometer_sensor/magnetometer";

	if (!_node.Subscribe(mag_topic, &GZBridge::magnetometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", mag_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeOdometry(bool required)
{
	// odom: /world/$WORLD/model/$MODEL/link/base_link/odometry_with_covariance
	std::string odometry_topic = "/model/" + _model_name + "/odometry_with_covariance";

	if (!_node.Subscribe(odometry_topic, &GZBridge::odometryCallback, this)) {
		PX4_ERR("failed to subscribe to %s", odometry_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeLaserScan(bool required)
{
	std::string laser_scan_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/link/sensor/lidar_2d_v2/scan";

	if (!_node.Subscribe(laser_scan_topic, &GZBridge::laserScanCallback, this)) {
		PX4_WARN("failed to subscribe to %s", laser_scan_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeDistanceSensor(bool required)
{
	std::string lidar_sensor = "/world/" + _world_name + "/model/" + _model_name +
				   "/link/lidar_sensor_link/sensor/lidar/scan";

	if (!_node.Subscribe(lidar_sensor, &GZBridge::laserScantoLidarSensorCallback, this)) {
		PX4_WARN("failed to subscribe to %s", lidar_sensor.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeAirspeed(bool required)
{
	std::string airspeed_topic = "/world/" + _world_name + "/model/" + _model_name +
				     "/link/airspeed_link/sensor/air_speed/air_speed";

	if (!_node.Subscribe(airspeed_topic, &GZBridge::airspeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", airspeed_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeAirPressure(bool required)
{
	std::string air_pressure_topic = "/world/" + _world_name + "/model/" + _model_name +
					 "/link/base_link/sensor/air_pressure_sensor/air_pressure";

	if (!_node.Subscribe(air_pressure_topic, &GZBridge::airPressureCallback, this)) {
		PX4_ERR("failed to subscribe to %s", air_pressure_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeNavsat(bool required)
{
	std::string nav_sat_topic = "/world/" + _world_name + "/model/" + _model_name +
				    "/link/base_link/sensor/navsat_sensor/navsat";

	if (!_node.Subscribe(nav_sat_topic, &GZBridge::navSatCallback, this)) {
		PX4_ERR("failed to subscribe to %s", nav_sat_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeOpticalFlow(bool required)
{
	std::string flow_topic = "/world/" + _world_name + "/model/" + _model_name +
				 "/link/flow_link/sensor/optical_flow/optical_flow";

	if (!_node.Subscribe(flow_topic, &GZBridge::opticalFlowCallback, this)) {
		PX4_ERR("failed to subscribe to %s", flow_topic.c_str());
		return required ? false : true;
	}

	return true;
}

void GZBridge::clockCallback(const gz::msgs::Clock &msg)
{
	// NOTE: PX4-SITL time needs to stay in sync with gz, so this clock-sync will happen on every callback.
	struct timespec ts;
	ts.tv_sec = msg.sim().sec();
	ts.tv_nsec = msg.sim().nsec();

	if (!_realtime_clock_set) {
		// Set initial real time clock at startup
		px4_clock_settime(CLOCK_REALTIME, &ts);
		_realtime_clock_set = true;

	} else {
		// Keep monotonic clock synchronized
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}
}

void GZBridge::opticalFlowCallback(const px4::msgs::OpticalFlow &msg)
{
	sensor_optical_flow_s report = {};

	report.timestamp = hrt_absolute_time();
	report.timestamp_sample = msg.time_usec();
	report.pixel_flow[0] = msg.integrated_x();
	report.pixel_flow[1] = msg.integrated_y();
	report.quality = msg.quality();
	report.integration_timespan_us = msg.integration_time_us();

	// Static data
	device::Device::DeviceId id;
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.bus = 0;
	id.devid_s.address = 0;
	id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;
	report.device_id = id.devid;

	// values taken from PAW3902
	report.mode = sensor_optical_flow_s::MODE_LOWLIGHT;
	report.max_flow_rate = 7.4f;
	report.min_ground_distance = 0.f;
	report.max_ground_distance = 30.f;
	report.error_count = 0;

	// No delta angle
	// No distance
	// This means that delta angle will come from vehicle gyro
	// Distance will come from vehicle distance sensor

	_optical_flow_pub.publish(report);
}

void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_MAG_DEVTYPE_MAGSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 3; // TODO: any value other than 3 causes Commander to not use the mag.... wtf

	sensor_mag_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.temperature = this->_temperature;

	// FIMEX: once we're on jetty or later
	// The magnetometer plugin publishes in units of gauss and in a weird left handed coordinate system
	// https://github.com/gazebosim/gz-sim/pull/2460
	report.x = -msg.field_tesla().y();
	report.y = -msg.field_tesla().x();
	report.z = msg.field_tesla().z();

	_sensor_mag_pub.publish(report);
}

void GZBridge::airPressureCallback(const gz::msgs::FluidPressure &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_BARO_DEVTYPE_BAROSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_baro_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.pressure = msg.pressure();
	report.temperature = this->_temperature;
	_sensor_baro_pub.publish(report);
}

void GZBridge::airspeedCallback(const gz::msgs::AirSpeed &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	differential_pressure_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.differential_pressure_pa = msg.diff_pressure(); // hPa to Pa;
	report.temperature = static_cast<float>(msg.temperature()) + atmosphere::kAbsoluteNullCelsius; // K to C
	_differential_pressure_pub.publish(report);

	this->_temperature = report.temperature;
}

void GZBridge::imuCallback(const gz::msgs::IMU &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					     msg.linear_acceleration().x(),
					     msg.linear_acceleration().y(),
					     msg.linear_acceleration().z()));

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_IMU_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	// publish accel
	sensor_accel_s accel{};

	accel.timestamp_sample = timestamp;
	accel.timestamp = timestamp;
	accel.device_id = id.devid;

	accel.x = accel_b.X();
	accel.y = accel_b.Y();
	accel.z = accel_b.Z();
	accel.temperature = NAN;
	accel.samples = 1;
	_sensor_accel_pub.publish(accel);

	gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					    msg.angular_velocity().x(),
					    msg.angular_velocity().y(),
					    msg.angular_velocity().z()));

	// publish gyro
	sensor_gyro_s gyro{};
	gyro.timestamp_sample = timestamp;
	gyro.timestamp = timestamp;
	gyro.device_id = id.devid;
	gyro.x = gyro_b.X();
	gyro.y = gyro_b.Y();
	gyro.z = gyro_b.Z();
	gyro.temperature = NAN;
	gyro.samples = 1;
	_sensor_gyro_pub.publish(gyro);
}

void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	for (int p = 0; p < msg.pose_size(); p++) {
		if (msg.pose(p).name() == _model_name) {

			const double dt = math::constrain((timestamp - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = timestamp;

			gz::msgs::Vector3d pose_position = msg.pose(p).position();
			gz::msgs::Quaternion pose_orientation = msg.pose(p).orientation();

			// ground truth
			gz::math::Quaterniond q_gr = gz::math::Quaterniond(
							     pose_orientation.w(),
							     pose_orientation.x(),
							     pose_orientation.y(),
							     pose_orientation.z());

			gz::math::Quaterniond q_nb;
			GZBridge::rotateQuaternion(q_nb, q_gr);

			// publish attitude groundtruth
			vehicle_attitude_s vehicle_attitude_groundtruth{};
			vehicle_attitude_groundtruth.timestamp_sample = timestamp;
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = timestamp;
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
			vehicle_angular_velocity_groundtruth.timestamp_sample = timestamp;
			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = timestamp;
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.timestamp_sample = timestamp;
			// position ENU -> NED
			const matrix::Vector3d position{pose_position.y(), pose_position.x(), -pose_position.z()};
			const matrix::Vector3d velocity{(position - _position_prev) / dt};
			const matrix::Vector3d acceleration{(velocity - _velocity_prev) / dt};

			_position_prev = position;
			_velocity_prev = velocity;

			local_position_groundtruth.ax = acceleration(0);
			local_position_groundtruth.ay = acceleration(1);
			local_position_groundtruth.az = acceleration(2);
			local_position_groundtruth.vx = velocity(0);
			local_position_groundtruth.vy = velocity(1);
			local_position_groundtruth.vz = velocity(2);
			local_position_groundtruth.x = position(0);
			local_position_groundtruth.y = position(1);
			local_position_groundtruth.z = position(2);

			local_position_groundtruth.heading = euler.psi();

			if (_pos_ref.isInitialized()) {

				local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat(); // Reference point latitude in degrees
				local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon(); // Reference point longitude in degrees
				local_position_groundtruth.ref_alt = _alt_ref;
				local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
				local_position_groundtruth.xy_global = true;
				local_position_groundtruth.z_global = true;

			} else {
				local_position_groundtruth.ref_lat = static_cast<double>(NAN);
				local_position_groundtruth.ref_lon = static_cast<double>(NAN);
				local_position_groundtruth.ref_alt = NAN;
				local_position_groundtruth.ref_timestamp = 0;
				local_position_groundtruth.xy_global = false;
				local_position_groundtruth.z_global = false;
			}

			local_position_groundtruth.timestamp = timestamp;
			_lpos_ground_truth_pub.publish(local_position_groundtruth);
			return;
		}
	}
}

void GZBridge::odometryCallback(const gz::msgs::OdometryWithCovariance &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	vehicle_odometry_s report{};
	report.timestamp_sample = timestamp;
	report.timestamp = timestamp;

	// gz odometry position is in ENU frame and needs to be converted to NED
	report.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	report.position[0] = msg.pose_with_covariance().pose().position().y();
	report.position[1] = msg.pose_with_covariance().pose().position().x();
	report.position[2] = -msg.pose_with_covariance().pose().position().z();

	// gz odometry orientation is "body FLU->ENU" and needs to be converted in "body FRD->NED"
	gz::msgs::Quaternion pose_orientation = msg.pose_with_covariance().pose().orientation();
	gz::math::Quaterniond q_gr = gz::math::Quaterniond(
					     pose_orientation.w(),
					     pose_orientation.x(),
					     pose_orientation.y(),
					     pose_orientation.z());
	gz::math::Quaterniond q_nb;
	GZBridge::rotateQuaternion(q_nb, q_gr);
	report.q[0] = q_nb.W();
	report.q[1] = q_nb.X();
	report.q[2] = q_nb.Y();
	report.q[3] = q_nb.Z();

	// gz odometry linear velocity is in body FLU and needs to be converted in body FRD
	report.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
	report.velocity[0] = msg.twist_with_covariance().twist().linear().x();
	report.velocity[1] = -msg.twist_with_covariance().twist().linear().y();
	report.velocity[2] = -msg.twist_with_covariance().twist().linear().z();

	// gz odometry angular velocity is in body FLU and need to be converted in body FRD
	report.angular_velocity[0] = msg.twist_with_covariance().twist().angular().x();
	report.angular_velocity[1] = -msg.twist_with_covariance().twist().angular().y();
	report.angular_velocity[2] = -msg.twist_with_covariance().twist().angular().z();

	// VISION_POSITION_ESTIMATE covariance
	//  pose 6x6 cross-covariance matrix
	//  (states: x, y, z, roll, pitch, yaw).
	//  If unknown, assign NaN value to first element in the array.
	report.position_variance[0] = msg.pose_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.position_variance[1] = msg.pose_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.position_variance[2] = msg.pose_with_covariance().covariance().data(14); // Z  row 2, col 2

	report.orientation_variance[0] = msg.pose_with_covariance().covariance().data(21); // R  row 3, col 3
	report.orientation_variance[1] = msg.pose_with_covariance().covariance().data(28); // P  row 4, col 4
	report.orientation_variance[2] = msg.pose_with_covariance().covariance().data(35); // Y  row 5, col 5

	report.velocity_variance[0] = msg.twist_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.velocity_variance[1] = msg.twist_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.velocity_variance[2] = msg.twist_with_covariance().covariance().data(14); // Z  row 2, col 2

	// report.reset_counter = vpe.reset_counter;
	_visual_odometry_pub.publish(report);
}

float GZBridge::generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void GZBridge::addGpsNoise(double &latitude, double &longitude, double &altitude,
			   float &vel_north, float &vel_east, float &vel_down)
{
	_gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_n;

	_gps_pos_noise_e = _pos_markov_time * _gps_pos_noise_e +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_e;

	_gps_pos_noise_d = _pos_markov_time * _gps_pos_noise_d +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f -
			   0.02f * _gps_pos_noise_d;

	latitude += math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);
	longitude += math::degrees((double)_gps_pos_noise_e / CONSTANTS_RADIUS_OF_EARTH);
	altitude += (double)_gps_pos_noise_d;

	_gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_e = _vel_markov_time * _gps_vel_noise_e +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_d = _vel_markov_time * _gps_vel_noise_d +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

	vel_north += _gps_vel_noise_n;
	vel_east += _gps_vel_noise_e;
	vel_down += _gps_vel_noise_d;
}

void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// initialize gps position
	if (!_pos_ref.isInitialized()) {
		_pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
		_alt_ref = msg.altitude();
		return;
	}

	double latitude = msg.latitude_deg();
	double longitude = msg.longitude_deg();
	double altitude = msg.altitude();
	float vel_north = msg.velocity_north();
	float vel_east = msg.velocity_east();
	float vel_down = -msg.velocity_up();

	vehicle_global_position_s gps_truth{};

	// Publish GPS groundtruth
	gps_truth.timestamp = timestamp;
	gps_truth.timestamp_sample = timestamp;
	gps_truth.lat = latitude;
	gps_truth.lon = longitude;
	gps_truth.alt = altitude;
	_gpos_ground_truth_pub.publish(gps_truth);

	// Apply noise model (based on ublox F9P)
	addGpsNoise(latitude, longitude, altitude, vel_north, vel_east, vel_down);

	// Attack injection: tamper the GPS measurement before the EKF consumes it.
	// If any GPS channel requests a drop (DROP primitive), drop the whole message.
	double lat_att = latitude;
	double lon_att = longitude;
	double alt_att = altitude;
	double vn_att = static_cast<double>(vel_north);
	double ve_att = static_cast<double>(vel_east);
	double vd_att = static_cast<double>(vel_down);

	bool gps_publish = true;
	gps_publish &= _attack.apply(attack::Channel::GPS_LAT, lat_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_LON, lon_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_ALT, alt_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_N, vn_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_E, ve_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_D, vd_att, timestamp);

	if (!gps_publish) {
		return;
	}

	latitude = lat_att;
	longitude = lon_att;
	altitude = alt_att;
	vel_north = static_cast<float>(vn_att);
	vel_east = static_cast<float>(ve_att);
	vel_down = static_cast<float>(vd_att);

	// Device ID
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_gps_s sensor_gps{};

	if (_sim_gps_used.get() >= 4) {
		// fix
		sensor_gps.fix_type = 3; // 3D fix
		sensor_gps.s_variance_m_s = 0.4f;
		sensor_gps.c_variance_rad = 0.1f;
		sensor_gps.eph = 0.9f;
		sensor_gps.epv = 1.78f;
		sensor_gps.hdop = 0.7f;
		sensor_gps.vdop = 1.1f;

	} else {
		// no fix
		sensor_gps.fix_type = 0; // No fix
		sensor_gps.s_variance_m_s = 100.f;
		sensor_gps.c_variance_rad = 100.f;
		sensor_gps.eph = 100.f;
		sensor_gps.epv = 100.f;
		sensor_gps.hdop = 100.f;
		sensor_gps.vdop = 100.f;
	}

	sensor_gps.timestamp = timestamp;
	sensor_gps.timestamp_sample = timestamp;
	sensor_gps.time_utc_usec = 0;
	sensor_gps.device_id = id.devid;
	sensor_gps.latitude_deg = latitude;
	sensor_gps.longitude_deg = longitude;
	sensor_gps.altitude_msl_m = altitude;
	sensor_gps.altitude_ellipsoid_m = altitude;
	sensor_gps.noise_per_ms = 0;
	sensor_gps.jamming_indicator = 0;
	sensor_gps.vel_m_s = sqrtf(vel_north * vel_north + vel_east * vel_east);
	sensor_gps.vel_n_m_s = vel_north;
	sensor_gps.vel_e_m_s = vel_east;
	sensor_gps.vel_d_m_s = vel_down;
	sensor_gps.cog_rad = atan2(vel_east, vel_north);
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = NAN;
	sensor_gps.heading_accuracy = 0;
	sensor_gps.automatic_gain_control = 0;
	sensor_gps.jamming_state = 0;
	sensor_gps.spoofing_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = _sim_gps_used.get();

	_sensor_gps_pub.publish(sensor_gps);
}

void GZBridge::laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	distance_sensor_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = id.devid;
	report.min_distance = static_cast<float>(msg.range_min());
	report.max_distance = static_cast<float>(msg.range_max());
	report.current_distance = static_cast<float>(msg.ranges()[0]);
	report.variance = 0.0f;
	report.signal_quality = -1;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	gz::msgs::Quaternion pose_orientation = msg.world_pose().orientation();
	gz::math::Quaterniond q_sensor = gz::math::Quaterniond(
			pose_orientation.w(),
			pose_orientation.x(),
			pose_orientation.y(),
			pose_orientation.z());

	const gz::math::Quaterniond q_left(0.7071068, 0, 0, -0.7071068);

	const gz::math::Quaterniond q_front(0.7071068, 0.7071068, 0, 0);

	const gz::math::Quaterniond q_down(0, 1, 0, 0);

	if (q_sensor.Equal(q_front, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;

	} else if (q_sensor.Equal(q_down, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	} else if (q_sensor.Equal(q_left, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_LEFT_FACING;

	} else {
		report.orientation = distance_sensor_s::ROTATION_CUSTOM;
		report.q[0] = q_sensor.W();
		report.q[1] = q_sensor.X();
		report.q[2] = q_sensor.Y();
		report.q[3] = q_sensor.Z();
	}

	_distance_sensor_pub.publish(report);
}

void GZBridge::laserScanCallback(const gz::msgs::LaserScan &msg)
{
	static constexpr int SECTOR_SIZE_DEG = 5; // PX4 Collision Prevention uses 5 degree sectors

	double angle_min_deg = msg.angle_min() * 180 / M_PI;
	double angle_step_deg = msg.angle_step() * 180 / M_PI;

	int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
	int number_of_sectors = msg.ranges_size() / samples_per_sector;

	std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

	// Downsample -- take average of samples per sector
	for (int i = 0; i < number_of_sectors; i++) {

		double sum = 0;

		int samples_used_in_sector = 0;

		for (int j = 0; j < samples_per_sector; j++) {

			double distance = msg.ranges()[i * samples_per_sector + j];

			// inf values mean no object
			if (isinf(distance)) {
				continue;
			}

			sum += distance;
			samples_used_in_sector++;
		}

		// If all samples in a sector are inf then it means the sector is clear
		if (samples_used_in_sector == 0) {
			ds_array[i] = msg.range_max();

		} else {
			ds_array[i] = sum / samples_used_in_sector;
		}
	}

	// Publish to uORB
	obstacle_distance_s report {};

	// Initialize unknown
	for (auto &i : report.distances) {
		i = UINT16_MAX;
	}

	report.timestamp = hrt_absolute_time();
	report.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	report.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	report.min_distance = static_cast<uint16_t>(msg.range_min() * 100.);
	report.max_distance = static_cast<uint16_t>(msg.range_max() * 100.);
	report.angle_offset = static_cast<float>(angle_min_deg);
	report.increment = static_cast<float>(SECTOR_SIZE_DEG);

	// Map samples in FOV into sectors in ObstacleDistance
	int index = 0;

	// Iterate in reverse because array is FLU and we need FRD
	for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

		uint16_t distance_cm = (*i) * 100.;

		if (distance_cm >= report.max_distance) {
			report.distances[index] = report.max_distance + 1;

		} else if (distance_cm < report.min_distance) {
			report.distances[index] = 0;

		} else {
			report.distances[index] = distance_cm;
		}

		index++;
	}

	_obstacle_distance_pub.publish(report);
}

void GZBridge::rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU)
{
	// FLU (ROS) to FRD (PX4) static rotation
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	/**
	 * @brief Quaternion for rotation between ENU and NED frames
	 *
	 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
	 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
	 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
	 */
	static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

	// final rotation composition
	q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}

int GZBridge::task_spawn(int argc, char *argv[])
{
	std::string world_name;
	std::string model_name;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			world_name = myoptarg;
			break;

		case 'n':
			model_name = myoptarg;
			break;

		default:
			print_usage();
			return PX4_ERROR;
		}
	}

	PX4_INFO("world: %s, model: %s", world_name.c_str(), model_name.c_str());

	GZBridge *instance = new GZBridge(world_name, model_name);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int GZBridge::print_status()
{
	PX4_INFO_RAW("ESC outputs:\n");
	_mixing_interface_esc.mixingOutput().printStatus();

	PX4_INFO_RAW("Servo outputs:\n");
	_mixing_interface_servo.mixingOutput().printStatus();

	PX4_INFO_RAW("Wheel outputs:\n");
	_mixing_interface_wheel.mixingOutput().printStatus();

	return 0;
}

int GZBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GZBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gz_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
	return GZBridge::main(argc, argv);
}

~~~

## 4.5 注入执行器攻击

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp）为前述修改过的文件，前述文件已完成这里的修改，这里再次说明是为了区分两次修改对应的不同功能，本次修改主要添加 \_mixing_interface_esc.setAttack(&_attack);  这行代码， 将同一个攻击管理类实例的地址传到执行器仿真代码进行使用，以使得传感器和执行器的仿真共用一个攻击管理类实例。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.hpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/mixer_module/mixer_module.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/esc_status.h>

namespace attack {
class AttackManager;
}


// GZBridge mixing class for ESCs.
// It is separate from GZBridge to have separate WorkItems and therefore allowing independent scheduling
// All work items are expected to run on the same work queue.
class GZMixingInterfaceESC : public OutputModuleInterface
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	GZMixingInterfaceESC(gz::transport::Node &node) :
		OutputModuleInterface(MODULE_NAME "-actuators-esc", px4::wq_configurations::rate_ctrl),
		_node(node)
	{}

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	MixingOutput &mixingOutput() { return _mixing_output; }

	bool init(const std::string &model_name);

	/** Wire in the attack manager (owned by GZBridge); may be null to disable attacks. */
	void setAttack(attack::AttackManager *attack) { _attack = attack; }

	void stop()
	{
		_mixing_output.unregister();
		ScheduleClear();
	}

private:
	friend class GZBridge;

	void Run() override;

	void motorSpeedCallback(const gz::msgs::Actuators &actuators);

	gz::transport::Node &_node;
	pthread_mutex_t _node_mutex;

	attack::AttackManager *_attack{nullptr};
	uint16_t _last_motor[MAX_ACTUATORS]{};  ///< last value sent per motor (DROP holds this)

	MixingOutput _mixing_output{"SIM_GZ_EC", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	gz::transport::Node::Publisher _actuators_pub;

	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

};

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "GZMixingInterfaceESC.hpp"
#include "attack/AttackManager.hpp"

#include <lib/mathlib/mathlib.h>

bool GZMixingInterfaceESC::init(const std::string &model_name)
{

	// ESC feedback: /x500/command/motor_speed
	std::string motor_speed_topic = "/" + model_name + "/command/motor_speed";

	if (!_node.Subscribe(motor_speed_topic, &GZMixingInterfaceESC::motorSpeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", motor_speed_topic.c_str());
		return false;
	}

	// output eg /X500/command/motor_speed
	std::string actuator_topic = "/" + model_name + "/command/motor_speed";
	_actuators_pub = _node.Advertise<gz::msgs::Actuators>(actuator_topic);

	if (!_actuators_pub.Valid()) {
		PX4_ERR("failed to advertise %s", actuator_topic.c_str());
		return false;
	}

	_esc_status_pub.advertise();

	pthread_mutex_init(&_node_mutex, nullptr);

	ScheduleNow();

	return true;
}

bool GZMixingInterfaceESC::updateOutputs(uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	unsigned active_output_count = 0;

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	if (active_output_count > 0) {
		gz::msgs::Actuators rotor_velocity_message;
		rotor_velocity_message.mutable_velocity()->Resize(active_output_count, 0);

		for (unsigned i = 0; i < active_output_count; i++) {
			uint16_t value = outputs[i];

			if (_attack != nullptr) {
				const attack::Channel ch = attack::motor_channel(i);
				double v = static_cast<double>(outputs[i]);

				if (_attack->apply(ch, v, hrt_absolute_time())) {
					value = static_cast<uint16_t>(math::constrain(v, 0.0, 65535.0));

				} else {
					// DROP primitive fired: hold the last value sent to the ESC.
					value = _last_motor[i];
				}

				_last_motor[i] = value;
			}

			rotor_velocity_message.set_velocity(i, value);
		}

		if (_actuators_pub.Valid()) {
			return _actuators_pub.Publish(rotor_velocity_message);
		}
	}

	return false;
}

void GZMixingInterfaceESC::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void GZMixingInterfaceESC::motorSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	esc_status_s esc_status{};
	esc_status.esc_count = actuators.velocity_size();

	for (int i = 0; i < actuators.velocity_size(); i++) {
		esc_status.esc[i].timestamp = hrt_absolute_time();
		esc_status.esc[i].esc_rpm = actuators.velocity(i);
		esc_status.esc_online_flags |= 1 << i;

		if (actuators.velocity(i) > 0) {
			esc_status.esc_armed_flags |= 1 << i;
		}
	}

	if (esc_status.esc_count > 0) {
		esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(esc_status);
	}

	pthread_mutex_unlock(&_node_mutex);
}

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "GZBridge.hpp"

#include <uORB/Subscription.hpp>

#include <lib/atmosphere/atmosphere.h>
#include <lib/mathlib/mathlib.h>

#include <px4_platform_common/getopt.h>

#include <iostream>
#include <string>

GZBridge::GZBridge(const std::string &world, const std::string &model_name) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_world_name(world),
	_model_name(model_name)
{
	updateParams();
}

GZBridge::~GZBridge()
{
	for (auto &sub_topic : _node.SubscribedTopics()) {
		_node.Unsubscribe(sub_topic);
	}
}

int GZBridge::init()
{
	// REQUIRED:
	if (!subscribeClock(true)) {
		return PX4_ERROR;
	}

	// We must wait for clock before subscribing to other topics. This is because
	// if we publish a 0 timestamp it screws up the EKF.
	while (1) {
		px4_usleep(25000);

		if (_realtime_clock_set) {
			px4_usleep(25000);
			break;
		}
	}

	if (!subscribePoseInfo(true)) {
		return PX4_ERROR;
	}

	if (!subscribeImu(true)) {
		return PX4_ERROR;
	}

	if (!subscribeMag(true)) {
		return PX4_ERROR;
	}

	// OPTIONAL:
	if (_sim_gz_en_gps.get()) {
		if (!subscribeNavsat(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_baro.get()) {
		if (!subscribeAirPressure(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_lidar.get()) {
		if (!subscribeDistanceSensor(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_aspd.get()) {
		if (!subscribeAirspeed(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_flow.get()) {
		if (!subscribeOpticalFlow(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_odom.get()) {
		if (!subscribeOdometry(false)) {
			return PX4_ERROR;
		}
	}

	if (_sim_gz_en_lidar.get()) {
		if (!subscribeLaserScan(false)) {
			return PX4_ERROR;
		}
	}

	// Attack injection manager: init first, then wire it into the ESC interface.
	if (!_attack.init()) {
		PX4_ERR("failed to init attack manager");
		return PX4_ERROR;
	}

	_mixing_interface_esc.setAttack(&_attack);

	// ESC mixing interface
	if (!_mixing_interface_esc.init(_model_name)) {
		PX4_ERR("failed to init ESC output");
		return PX4_ERROR;
	}

	// Servo mixing interface
	if (!_mixing_interface_servo.init(_model_name)) {
		PX4_ERR("failed to init servo output");
		return PX4_ERROR;
	}

	// Wheel mixing interface
	if (!_mixing_interface_wheel.init(_model_name)) {
		PX4_ERR("failed to init motor output");
		return PX4_ERROR;
	}

	// Gimbal mixing interface
	if (!_gimbal.init(_world_name, _model_name)) {
		PX4_ERR("failed to init gimbal");
		return PX4_ERROR;
	}

	ScheduleNow();
	return OK;
}

void GZBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();

		_mixing_interface_esc.stop();
		_mixing_interface_servo.stop();
		_mixing_interface_wheel.stop();
		_gimbal.stop();

		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();

		_mixing_interface_esc.updateParams();
		_mixing_interface_servo.updateParams();
		_mixing_interface_wheel.updateParams();
		_gimbal.updateParams();
	}

	_attack.update();

	ScheduleDelayed(10_ms);
}

bool GZBridge::subscribeClock(bool required)
{
	std::string clock_topic = "/world/" + _world_name + "/clock";

	if (!_node.Subscribe(clock_topic, &GZBridge::clockCallback, this)) {
		PX4_ERR("failed to subscribe to %s", clock_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribePoseInfo(bool required)
{
	std::string world_pose_topic = "/world/" + _world_name + "/pose/info";

	if (!_node.Subscribe(world_pose_topic, &GZBridge::poseInfoCallback, this)) {
		PX4_ERR("failed to subscribe to %s", world_pose_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeImu(bool required)
{
	std::string imu_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/imu_sensor/imu";

	if (!_node.Subscribe(imu_topic, &GZBridge::imuCallback, this)) {
		PX4_ERR("failed to subscribe to %s", imu_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeMag(bool required)
{
	std::string mag_topic = "/world/" + _world_name + "/model/" + _model_name +
				"/link/base_link/sensor/magnetometer_sensor/magnetometer";

	if (!_node.Subscribe(mag_topic, &GZBridge::magnetometerCallback, this)) {
		PX4_ERR("failed to subscribe to %s", mag_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeOdometry(bool required)
{
	// odom: /world/$WORLD/model/$MODEL/link/base_link/odometry_with_covariance
	std::string odometry_topic = "/model/" + _model_name + "/odometry_with_covariance";

	if (!_node.Subscribe(odometry_topic, &GZBridge::odometryCallback, this)) {
		PX4_ERR("failed to subscribe to %s", odometry_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeLaserScan(bool required)
{
	std::string laser_scan_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/link/sensor/lidar_2d_v2/scan";

	if (!_node.Subscribe(laser_scan_topic, &GZBridge::laserScanCallback, this)) {
		PX4_WARN("failed to subscribe to %s", laser_scan_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeDistanceSensor(bool required)
{
	std::string lidar_sensor = "/world/" + _world_name + "/model/" + _model_name +
				   "/link/lidar_sensor_link/sensor/lidar/scan";

	if (!_node.Subscribe(lidar_sensor, &GZBridge::laserScantoLidarSensorCallback, this)) {
		PX4_WARN("failed to subscribe to %s", lidar_sensor.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeAirspeed(bool required)
{
	std::string airspeed_topic = "/world/" + _world_name + "/model/" + _model_name +
				     "/link/airspeed_link/sensor/air_speed/air_speed";

	if (!_node.Subscribe(airspeed_topic, &GZBridge::airspeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", airspeed_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeAirPressure(bool required)
{
	std::string air_pressure_topic = "/world/" + _world_name + "/model/" + _model_name +
					 "/link/base_link/sensor/air_pressure_sensor/air_pressure";

	if (!_node.Subscribe(air_pressure_topic, &GZBridge::airPressureCallback, this)) {
		PX4_ERR("failed to subscribe to %s", air_pressure_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeNavsat(bool required)
{
	std::string nav_sat_topic = "/world/" + _world_name + "/model/" + _model_name +
				    "/link/base_link/sensor/navsat_sensor/navsat";

	if (!_node.Subscribe(nav_sat_topic, &GZBridge::navSatCallback, this)) {
		PX4_ERR("failed to subscribe to %s", nav_sat_topic.c_str());
		return required ? false : true;
	}

	return true;
}

bool GZBridge::subscribeOpticalFlow(bool required)
{
	std::string flow_topic = "/world/" + _world_name + "/model/" + _model_name +
				 "/link/flow_link/sensor/optical_flow/optical_flow";

	if (!_node.Subscribe(flow_topic, &GZBridge::opticalFlowCallback, this)) {
		PX4_ERR("failed to subscribe to %s", flow_topic.c_str());
		return required ? false : true;
	}

	return true;
}

void GZBridge::clockCallback(const gz::msgs::Clock &msg)
{
	// NOTE: PX4-SITL time needs to stay in sync with gz, so this clock-sync will happen on every callback.
	struct timespec ts;
	ts.tv_sec = msg.sim().sec();
	ts.tv_nsec = msg.sim().nsec();

	if (!_realtime_clock_set) {
		// Set initial real time clock at startup
		px4_clock_settime(CLOCK_REALTIME, &ts);
		_realtime_clock_set = true;

	} else {
		// Keep monotonic clock synchronized
		px4_clock_settime(CLOCK_MONOTONIC, &ts);
	}
}

void GZBridge::opticalFlowCallback(const px4::msgs::OpticalFlow &msg)
{
	sensor_optical_flow_s report = {};

	report.timestamp = hrt_absolute_time();
	report.timestamp_sample = msg.time_usec();
	report.pixel_flow[0] = msg.integrated_x();
	report.pixel_flow[1] = msg.integrated_y();
	report.quality = msg.quality();
	report.integration_timespan_us = msg.integration_time_us();

	// Static data
	device::Device::DeviceId id;
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.bus = 0;
	id.devid_s.address = 0;
	id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;
	report.device_id = id.devid;

	// values taken from PAW3902
	report.mode = sensor_optical_flow_s::MODE_LOWLIGHT;
	report.max_flow_rate = 7.4f;
	report.min_ground_distance = 0.f;
	report.max_ground_distance = 30.f;
	report.error_count = 0;

	// No delta angle
	// No distance
	// This means that delta angle will come from vehicle gyro
	// Distance will come from vehicle distance sensor

	_optical_flow_pub.publish(report);
}

void GZBridge::magnetometerCallback(const gz::msgs::Magnetometer &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_MAG_DEVTYPE_MAGSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 3; // TODO: any value other than 3 causes Commander to not use the mag.... wtf

	sensor_mag_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.temperature = this->_temperature;

	// FIMEX: once we're on jetty or later
	// The magnetometer plugin publishes in units of gauss and in a weird left handed coordinate system
	// https://github.com/gazebosim/gz-sim/pull/2460
	report.x = -msg.field_tesla().y();
	report.y = -msg.field_tesla().x();
	report.z = msg.field_tesla().z();

	_sensor_mag_pub.publish(report);
}

void GZBridge::airPressureCallback(const gz::msgs::FluidPressure &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_BARO_DEVTYPE_BAROSIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_baro_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.pressure = msg.pressure();
	report.temperature = this->_temperature;
	_sensor_baro_pub.publish(report);
}

void GZBridge::airspeedCallback(const gz::msgs::AirSpeed &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIFF_PRESS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	differential_pressure_s report{};
	report.timestamp = timestamp;
	report.timestamp_sample = timestamp;
	report.device_id = id.devid;
	report.differential_pressure_pa = msg.diff_pressure(); // hPa to Pa;
	report.temperature = static_cast<float>(msg.temperature()) + atmosphere::kAbsoluteNullCelsius; // K to C
	_differential_pressure_pub.publish(report);

	this->_temperature = report.temperature;
}

void GZBridge::imuCallback(const gz::msgs::IMU &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// FLU -> FRD
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	gz::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					     msg.linear_acceleration().x(),
					     msg.linear_acceleration().y(),
					     msg.linear_acceleration().z()));

	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_IMU_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	// publish accel
	sensor_accel_s accel{};

	accel.timestamp_sample = timestamp;
	accel.timestamp = timestamp;
	accel.device_id = id.devid;

	accel.x = accel_b.X();
	accel.y = accel_b.Y();
	accel.z = accel_b.Z();
	accel.temperature = NAN;
	accel.samples = 1;
	_sensor_accel_pub.publish(accel);

	gz::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(gz::math::Vector3d(
					    msg.angular_velocity().x(),
					    msg.angular_velocity().y(),
					    msg.angular_velocity().z()));

	// publish gyro
	sensor_gyro_s gyro{};
	gyro.timestamp_sample = timestamp;
	gyro.timestamp = timestamp;
	gyro.device_id = id.devid;
	gyro.x = gyro_b.X();
	gyro.y = gyro_b.Y();
	gyro.z = gyro_b.Z();
	gyro.temperature = NAN;
	gyro.samples = 1;
	_sensor_gyro_pub.publish(gyro);
}

void GZBridge::poseInfoCallback(const gz::msgs::Pose_V &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	for (int p = 0; p < msg.pose_size(); p++) {
		if (msg.pose(p).name() == _model_name) {

			const double dt = math::constrain((timestamp - _timestamp_prev) * 1e-6, 0.001, 0.1);
			_timestamp_prev = timestamp;

			gz::msgs::Vector3d pose_position = msg.pose(p).position();
			gz::msgs::Quaternion pose_orientation = msg.pose(p).orientation();

			// ground truth
			gz::math::Quaterniond q_gr = gz::math::Quaterniond(
							     pose_orientation.w(),
							     pose_orientation.x(),
							     pose_orientation.y(),
							     pose_orientation.z());

			gz::math::Quaterniond q_nb;
			GZBridge::rotateQuaternion(q_nb, q_gr);

			// publish attitude groundtruth
			vehicle_attitude_s vehicle_attitude_groundtruth{};
			vehicle_attitude_groundtruth.timestamp_sample = timestamp;
			vehicle_attitude_groundtruth.q[0] = q_nb.W();
			vehicle_attitude_groundtruth.q[1] = q_nb.X();
			vehicle_attitude_groundtruth.q[2] = q_nb.Y();
			vehicle_attitude_groundtruth.q[3] = q_nb.Z();
			vehicle_attitude_groundtruth.timestamp = timestamp;
			_attitude_ground_truth_pub.publish(vehicle_attitude_groundtruth);

			// publish angular velocity groundtruth
			const matrix::Eulerf euler{matrix::Quatf(vehicle_attitude_groundtruth.q)};
			vehicle_angular_velocity_s vehicle_angular_velocity_groundtruth{};
			vehicle_angular_velocity_groundtruth.timestamp_sample = timestamp;
			const matrix::Vector3f angular_velocity = (euler - _euler_prev) / dt;
			_euler_prev = euler;
			angular_velocity.copyTo(vehicle_angular_velocity_groundtruth.xyz);

			vehicle_angular_velocity_groundtruth.timestamp = timestamp;
			_angular_velocity_ground_truth_pub.publish(vehicle_angular_velocity_groundtruth);

			vehicle_local_position_s local_position_groundtruth{};
			local_position_groundtruth.timestamp_sample = timestamp;
			// position ENU -> NED
			const matrix::Vector3d position{pose_position.y(), pose_position.x(), -pose_position.z()};
			const matrix::Vector3d velocity{(position - _position_prev) / dt};
			const matrix::Vector3d acceleration{(velocity - _velocity_prev) / dt};

			_position_prev = position;
			_velocity_prev = velocity;

			local_position_groundtruth.ax = acceleration(0);
			local_position_groundtruth.ay = acceleration(1);
			local_position_groundtruth.az = acceleration(2);
			local_position_groundtruth.vx = velocity(0);
			local_position_groundtruth.vy = velocity(1);
			local_position_groundtruth.vz = velocity(2);
			local_position_groundtruth.x = position(0);
			local_position_groundtruth.y = position(1);
			local_position_groundtruth.z = position(2);

			local_position_groundtruth.heading = euler.psi();

			if (_pos_ref.isInitialized()) {

				local_position_groundtruth.ref_lat = _pos_ref.getProjectionReferenceLat(); // Reference point latitude in degrees
				local_position_groundtruth.ref_lon = _pos_ref.getProjectionReferenceLon(); // Reference point longitude in degrees
				local_position_groundtruth.ref_alt = _alt_ref;
				local_position_groundtruth.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
				local_position_groundtruth.xy_global = true;
				local_position_groundtruth.z_global = true;

			} else {
				local_position_groundtruth.ref_lat = static_cast<double>(NAN);
				local_position_groundtruth.ref_lon = static_cast<double>(NAN);
				local_position_groundtruth.ref_alt = NAN;
				local_position_groundtruth.ref_timestamp = 0;
				local_position_groundtruth.xy_global = false;
				local_position_groundtruth.z_global = false;
			}

			local_position_groundtruth.timestamp = timestamp;
			_lpos_ground_truth_pub.publish(local_position_groundtruth);
			return;
		}
	}
}

void GZBridge::odometryCallback(const gz::msgs::OdometryWithCovariance &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	vehicle_odometry_s report{};
	report.timestamp_sample = timestamp;
	report.timestamp = timestamp;

	// gz odometry position is in ENU frame and needs to be converted to NED
	report.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	report.position[0] = msg.pose_with_covariance().pose().position().y();
	report.position[1] = msg.pose_with_covariance().pose().position().x();
	report.position[2] = -msg.pose_with_covariance().pose().position().z();

	// gz odometry orientation is "body FLU->ENU" and needs to be converted in "body FRD->NED"
	gz::msgs::Quaternion pose_orientation = msg.pose_with_covariance().pose().orientation();
	gz::math::Quaterniond q_gr = gz::math::Quaterniond(
					     pose_orientation.w(),
					     pose_orientation.x(),
					     pose_orientation.y(),
					     pose_orientation.z());
	gz::math::Quaterniond q_nb;
	GZBridge::rotateQuaternion(q_nb, q_gr);
	report.q[0] = q_nb.W();
	report.q[1] = q_nb.X();
	report.q[2] = q_nb.Y();
	report.q[3] = q_nb.Z();

	// gz odometry linear velocity is in body FLU and needs to be converted in body FRD
	report.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
	report.velocity[0] = msg.twist_with_covariance().twist().linear().x();
	report.velocity[1] = -msg.twist_with_covariance().twist().linear().y();
	report.velocity[2] = -msg.twist_with_covariance().twist().linear().z();

	// gz odometry angular velocity is in body FLU and need to be converted in body FRD
	report.angular_velocity[0] = msg.twist_with_covariance().twist().angular().x();
	report.angular_velocity[1] = -msg.twist_with_covariance().twist().angular().y();
	report.angular_velocity[2] = -msg.twist_with_covariance().twist().angular().z();

	// VISION_POSITION_ESTIMATE covariance
	//  pose 6x6 cross-covariance matrix
	//  (states: x, y, z, roll, pitch, yaw).
	//  If unknown, assign NaN value to first element in the array.
	report.position_variance[0] = msg.pose_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.position_variance[1] = msg.pose_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.position_variance[2] = msg.pose_with_covariance().covariance().data(14); // Z  row 2, col 2

	report.orientation_variance[0] = msg.pose_with_covariance().covariance().data(21); // R  row 3, col 3
	report.orientation_variance[1] = msg.pose_with_covariance().covariance().data(28); // P  row 4, col 4
	report.orientation_variance[2] = msg.pose_with_covariance().covariance().data(35); // Y  row 5, col 5

	report.velocity_variance[0] = msg.twist_with_covariance().covariance().data(7);  // Y  row 1, col 1
	report.velocity_variance[1] = msg.twist_with_covariance().covariance().data(0);  // X  row 0, col 0
	report.velocity_variance[2] = msg.twist_with_covariance().covariance().data(14); // Z  row 2, col 2

	// report.reset_counter = vpe.reset_counter;
	_visual_odometry_pub.publish(report);
}

float GZBridge::generate_wgn()
{
	// generate white Gaussian noise sample with std=1

	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

void GZBridge::addGpsNoise(double &latitude, double &longitude, double &altitude,
			   float &vel_north, float &vel_east, float &vel_down)
{
	_gps_pos_noise_n = _pos_markov_time * _gps_pos_noise_n +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_n;

	_gps_pos_noise_e = _pos_markov_time * _gps_pos_noise_e +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude -
			   0.02f * _gps_pos_noise_e;

	_gps_pos_noise_d = _pos_markov_time * _gps_pos_noise_d +
			   _pos_random_walk * generate_wgn() * _pos_noise_amplitude * 1.5f -
			   0.02f * _gps_pos_noise_d;

	latitude += math::degrees((double)_gps_pos_noise_n / CONSTANTS_RADIUS_OF_EARTH);
	longitude += math::degrees((double)_gps_pos_noise_e / CONSTANTS_RADIUS_OF_EARTH);
	altitude += (double)_gps_pos_noise_d;

	_gps_vel_noise_n = _vel_markov_time * _gps_vel_noise_n +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_e = _vel_markov_time * _gps_vel_noise_e +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude;

	_gps_vel_noise_d = _vel_markov_time * _gps_vel_noise_d +
			   _vel_noise_density * generate_wgn() * _vel_noise_amplitude * 1.2f;

	vel_north += _gps_vel_noise_n;
	vel_east += _gps_vel_noise_e;
	vel_down += _gps_vel_noise_d;
}

void GZBridge::navSatCallback(const gz::msgs::NavSat &msg)
{
	const uint64_t timestamp = hrt_absolute_time();

	// initialize gps position
	if (!_pos_ref.isInitialized()) {
		_pos_ref.initReference(msg.latitude_deg(), msg.longitude_deg(), timestamp);
		_alt_ref = msg.altitude();
		return;
	}

	double latitude = msg.latitude_deg();
	double longitude = msg.longitude_deg();
	double altitude = msg.altitude();
	float vel_north = msg.velocity_north();
	float vel_east = msg.velocity_east();
	float vel_down = -msg.velocity_up();

	vehicle_global_position_s gps_truth{};

	// Publish GPS groundtruth
	gps_truth.timestamp = timestamp;
	gps_truth.timestamp_sample = timestamp;
	gps_truth.lat = latitude;
	gps_truth.lon = longitude;
	gps_truth.alt = altitude;
	_gpos_ground_truth_pub.publish(gps_truth);

	// Apply noise model (based on ublox F9P)
	addGpsNoise(latitude, longitude, altitude, vel_north, vel_east, vel_down);

	// Attack injection: tamper the GPS measurement before the EKF consumes it.
	// If any GPS channel requests a drop (DROP primitive), drop the whole message.
	double lat_att = latitude;
	double lon_att = longitude;
	double alt_att = altitude;
	double vn_att = static_cast<double>(vel_north);
	double ve_att = static_cast<double>(vel_east);
	double vd_att = static_cast<double>(vel_down);

	bool gps_publish = true;
	gps_publish &= _attack.apply(attack::Channel::GPS_LAT, lat_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_LON, lon_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_ALT, alt_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_N, vn_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_E, ve_att, timestamp);
	gps_publish &= _attack.apply(attack::Channel::GPS_VEL_D, vd_att, timestamp);

	if (!gps_publish) {
		return;
	}

	latitude = lat_att;
	longitude = lon_att;
	altitude = alt_att;
	vel_north = static_cast<float>(vn_att);
	vel_east = static_cast<float>(ve_att);
	vel_down = static_cast<float>(vd_att);

	// Device ID
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	sensor_gps_s sensor_gps{};

	if (_sim_gps_used.get() >= 4) {
		// fix
		sensor_gps.fix_type = 3; // 3D fix
		sensor_gps.s_variance_m_s = 0.4f;
		sensor_gps.c_variance_rad = 0.1f;
		sensor_gps.eph = 0.9f;
		sensor_gps.epv = 1.78f;
		sensor_gps.hdop = 0.7f;
		sensor_gps.vdop = 1.1f;

	} else {
		// no fix
		sensor_gps.fix_type = 0; // No fix
		sensor_gps.s_variance_m_s = 100.f;
		sensor_gps.c_variance_rad = 100.f;
		sensor_gps.eph = 100.f;
		sensor_gps.epv = 100.f;
		sensor_gps.hdop = 100.f;
		sensor_gps.vdop = 100.f;
	}

	sensor_gps.timestamp = timestamp;
	sensor_gps.timestamp_sample = timestamp;
	sensor_gps.time_utc_usec = 0;
	sensor_gps.device_id = id.devid;
	sensor_gps.latitude_deg = latitude;
	sensor_gps.longitude_deg = longitude;
	sensor_gps.altitude_msl_m = altitude;
	sensor_gps.altitude_ellipsoid_m = altitude;
	sensor_gps.noise_per_ms = 0;
	sensor_gps.jamming_indicator = 0;
	sensor_gps.vel_m_s = sqrtf(vel_north * vel_north + vel_east * vel_east);
	sensor_gps.vel_n_m_s = vel_north;
	sensor_gps.vel_e_m_s = vel_east;
	sensor_gps.vel_d_m_s = vel_down;
	sensor_gps.cog_rad = atan2(vel_east, vel_north);
	sensor_gps.timestamp_time_relative = 0;
	sensor_gps.heading = NAN;
	sensor_gps.heading_offset = NAN;
	sensor_gps.heading_accuracy = 0;
	sensor_gps.automatic_gain_control = 0;
	sensor_gps.jamming_state = 0;
	sensor_gps.spoofing_state = 0;
	sensor_gps.vel_ned_valid = true;
	sensor_gps.satellites_used = _sim_gps_used.get();

	_sensor_gps_pub.publish(sensor_gps);
}

void GZBridge::laserScantoLidarSensorCallback(const gz::msgs::LaserScan &msg)
{
	device::Device::DeviceId id{};
	id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;
	id.devid_s.bus = 1;
	id.devid_s.address = 1;

	distance_sensor_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = id.devid;
	report.min_distance = static_cast<float>(msg.range_min());
	report.max_distance = static_cast<float>(msg.range_max());
	report.current_distance = static_cast<float>(msg.ranges()[0]);
	report.variance = 0.0f;
	report.signal_quality = -1;
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;

	gz::msgs::Quaternion pose_orientation = msg.world_pose().orientation();
	gz::math::Quaterniond q_sensor = gz::math::Quaterniond(
			pose_orientation.w(),
			pose_orientation.x(),
			pose_orientation.y(),
			pose_orientation.z());

	const gz::math::Quaterniond q_left(0.7071068, 0, 0, -0.7071068);

	const gz::math::Quaterniond q_front(0.7071068, 0.7071068, 0, 0);

	const gz::math::Quaterniond q_down(0, 1, 0, 0);

	if (q_sensor.Equal(q_front, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_FORWARD_FACING;

	} else if (q_sensor.Equal(q_down, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;

	} else if (q_sensor.Equal(q_left, 0.03)) {
		report.orientation = distance_sensor_s::ROTATION_LEFT_FACING;

	} else {
		report.orientation = distance_sensor_s::ROTATION_CUSTOM;
		report.q[0] = q_sensor.W();
		report.q[1] = q_sensor.X();
		report.q[2] = q_sensor.Y();
		report.q[3] = q_sensor.Z();
	}

	_distance_sensor_pub.publish(report);
}

void GZBridge::laserScanCallback(const gz::msgs::LaserScan &msg)
{
	static constexpr int SECTOR_SIZE_DEG = 5; // PX4 Collision Prevention uses 5 degree sectors

	double angle_min_deg = msg.angle_min() * 180 / M_PI;
	double angle_step_deg = msg.angle_step() * 180 / M_PI;

	int samples_per_sector = std::round(SECTOR_SIZE_DEG / angle_step_deg);
	int number_of_sectors = msg.ranges_size() / samples_per_sector;

	std::vector<double> ds_array(number_of_sectors, UINT16_MAX);

	// Downsample -- take average of samples per sector
	for (int i = 0; i < number_of_sectors; i++) {

		double sum = 0;

		int samples_used_in_sector = 0;

		for (int j = 0; j < samples_per_sector; j++) {

			double distance = msg.ranges()[i * samples_per_sector + j];

			// inf values mean no object
			if (isinf(distance)) {
				continue;
			}

			sum += distance;
			samples_used_in_sector++;
		}

		// If all samples in a sector are inf then it means the sector is clear
		if (samples_used_in_sector == 0) {
			ds_array[i] = msg.range_max();

		} else {
			ds_array[i] = sum / samples_used_in_sector;
		}
	}

	// Publish to uORB
	obstacle_distance_s report {};

	// Initialize unknown
	for (auto &i : report.distances) {
		i = UINT16_MAX;
	}

	report.timestamp = hrt_absolute_time();
	report.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	report.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	report.min_distance = static_cast<uint16_t>(msg.range_min() * 100.);
	report.max_distance = static_cast<uint16_t>(msg.range_max() * 100.);
	report.angle_offset = static_cast<float>(angle_min_deg);
	report.increment = static_cast<float>(SECTOR_SIZE_DEG);

	// Map samples in FOV into sectors in ObstacleDistance
	int index = 0;

	// Iterate in reverse because array is FLU and we need FRD
	for (std::vector<double>::reverse_iterator i = ds_array.rbegin(); i != ds_array.rend(); ++i) {

		uint16_t distance_cm = (*i) * 100.;

		if (distance_cm >= report.max_distance) {
			report.distances[index] = report.max_distance + 1;

		} else if (distance_cm < report.min_distance) {
			report.distances[index] = 0;

		} else {
			report.distances[index] = distance_cm;
		}

		index++;
	}

	_obstacle_distance_pub.publish(report);
}

void GZBridge::rotateQuaternion(gz::math::Quaterniond &q_FRD_to_NED, const gz::math::Quaterniond q_FLU_to_ENU)
{
	// FLU (ROS) to FRD (PX4) static rotation
	static const auto q_FLU_to_FRD = gz::math::Quaterniond(0, 1, 0, 0);

	/**
	 * @brief Quaternion for rotation between ENU and NED frames
	 *
	 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
	 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
	 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
	 */
	static const auto q_ENU_to_NED = gz::math::Quaterniond(0, 0.70711, 0.70711, 0);

	// final rotation composition
	q_FRD_to_NED = q_ENU_to_NED * q_FLU_to_ENU * q_FLU_to_FRD.Inverse();
}

int GZBridge::task_spawn(int argc, char *argv[])
{
	std::string world_name;
	std::string model_name;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "w:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'w':
			world_name = myoptarg;
			break;

		case 'n':
			model_name = myoptarg;
			break;

		default:
			print_usage();
			return PX4_ERROR;
		}
	}

	PX4_INFO("world: %s, model: %s", world_name.c_str(), model_name.c_str());

	GZBridge *instance = new GZBridge(world_name, model_name);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int GZBridge::print_status()
{
	PX4_INFO_RAW("ESC outputs:\n");
	_mixing_interface_esc.mixingOutput().printStatus();

	PX4_INFO_RAW("Servo outputs:\n");
	_mixing_interface_servo.mixingOutput().printStatus();

	PX4_INFO_RAW("Wheel outputs:\n");
	_mixing_interface_wheel.mixingOutput().printStatus();

	return 0;
}

int GZBridge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GZBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gz_bridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('w', nullptr, nullptr, "World name", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Model name", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gz_bridge_main(int argc, char *argv[])
{
	return GZBridge::main(argc, argv);
}

~~~

## 4.6 无人机端 msg 构建

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/AttackCommand.msg
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/AttackStatus.msg

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/AttackCommand.msg

~~~c++
# Attack injection command (ROS2 -> PX4).
# Instructs the AttackManager to mount / update / clear an attack on one channel.
#
# One message = one channel. Sending a message with type=NONE clears the channel.
# The next sample on that channel reflects the change (GPS ~5-10 Hz, motors high rate).

uint64 timestamp		# time since system start (microseconds)

uint8 channel			# Channel enum value (0..9): GPS_LAT..GPS_VEL_D, MOTOR_0..MOTOR_3
uint8 type			# PrimitiveType enum value (0=NONE -> clear the attack)

float64[4] param		# primitive parameters (a, b, c, d); meaning depends on 'type'

uint64 t0_us			# start delay relative to command arrival (microseconds); 0 = immediately
uint64 t1_us			# duration relative to start (microseconds); 0 = until cleared
uint32 seed			# RNG seed for stochastic primitives; 0 = random

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/AttackStatus.msg

~~~c++
# Attack injection status (PX4 -> ROS2 / logger).
# Snapshot of which channels are currently under attack, published at 10 Hz.

uint64 timestamp		# time since system start (microseconds)

uint8[10] active		# per channel: 1 = attack currently active (now in [start_us, end_us]), 0 = inactive
uint8[10] type			# per channel: current PrimitiveType enum value (meaningful when active)

float64[10] param_a		# per channel: primitive parameter a
float64[10] param_b		# per channel: primitive parameter b
float64[10] param_c		# per channel: primitive parameter c
float64[10] param_d		# per channel: primitive parameter d

~~~

## 4.7 服务器端 msg 构建

文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/AttackCommand.msg
文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/AttackStatus.msg

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/AttackCommand.msg

~~~c++
# Attack injection command (ROS2 -> PX4).
# Instructs the AttackManager to mount / update / clear an attack on one channel.
#
# One message = one channel. Sending type=NONE clears the channel.

uint64 timestamp # [us] Time since system start

uint8 channel # Channel enum value (0..9): GPS_LAT..GPS_VEL_D, MOTOR_0..MOTOR_3
uint8 type # PrimitiveType enum value (0=NONE -> clear the attack)

float64[4] param # primitive parameters (a, b, c, d); meaning depends on 'type'

uint64 t0_us # start delay relative to command arrival (us); 0 = immediately
uint64 t1_us # duration relative to start (us); 0 = until cleared
uint32 seed # RNG seed for stochastic primitives; 0 = random

~~~

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/AttackStatus.msg

~~~c++
# Attack injection status (PX4 -> ROS2 / logger).
# Snapshot of which channels are currently under attack, published at 10 Hz.

uint64 timestamp # [us] Time since system start

uint8[10] active # per channel: 1 = attack currently active, 0 = inactive
uint8[10] type # per channel: current PrimitiveType enum value (meaningful when active)

float64[10] param_a # per channel: primitive parameter a
float64[10] param_b # per channel: primitive parameter b
float64[10] param_c # per channel: primitive parameter c
float64[10] param_d # per channel: primitive parameter d

~~~

## 4.8 修改 msg 构建文件

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

include(px4_list_make_absolute)

set(msg_files
	ActionRequest.msg
	ActuatorArmed.msg
	ActuatorControlsStatus.msg
	ActuatorOutputs.msg
	ActuatorServosTrim.msg
	ActuatorTest.msg
	AdcReport.msg
	Airspeed.msg
	AirspeedWind.msg
	AttackCommand.msg
	AttackStatus.msg
	AutotuneAttitudeControlStatus.msg
	BatteryInfo.msg
	ButtonEvent.msg
	CameraCapture.msg
	CameraStatus.msg
	CameraTrigger.msg
	CanInterfaceStatus.msg
	CellularStatus.msg
	CollisionConstraints.msg
	ControlAllocatorStatus.msg
	Cpuload.msg
	DatamanRequest.msg
	DatamanResponse.msg
	DebugArray.msg
	DebugKeyValue.msg
	DebugValue.msg
	DebugVect.msg
	DifferentialPressure.msg
	DistanceSensor.msg
	DistanceSensorModeChangeRequest.msg
	DronecanNodeStatus.msg
	Ekf2Timestamps.msg
	EscReport.msg
	EscStatus.msg
	EstimatorAidSource1d.msg
	EstimatorAidSource2d.msg
	EstimatorAidSource3d.msg
	EstimatorBias.msg
	EstimatorBias3d.msg
	EstimatorEventFlags.msg
	EstimatorGpsStatus.msg
	EstimatorInnovations.msg
	EstimatorSelectorStatus.msg
	EstimatorSensorBias.msg
	EstimatorStates.msg
	EstimatorStatus.msg
	EstimatorStatusFlags.msg
	versioned/Event.msg
	FigureEightStatus.msg
	FailsafeFlags.msg
	FailureDetectorStatus.msg
	FlightPhaseEstimation.msg
	FollowTarget.msg
	FollowTargetEstimator.msg
	FollowTargetStatus.msg
	FuelTankStatus.msg
	FixedWingLateralGuidanceStatus.msg
	FixedWingLateralStatus.msg
	FixedWingRunwayControl.msg
	GeneratorStatus.msg
	GeofenceResult.msg
	GeofenceStatus.msg
	GimbalControls.msg
	GimbalDeviceAttitudeStatus.msg
	GimbalDeviceInformation.msg
	GimbalDeviceSetAttitude.msg
	GimbalManagerInformation.msg
	GimbalManagerSetAttitude.msg
	GimbalManagerSetManualControl.msg
	GimbalManagerStatus.msg
	GpioConfig.msg
	GpioIn.msg
	GpioOut.msg
	GpioRequest.msg
	GpsDump.msg
	GpsInjectData.msg
	Gripper.msg
	HealthReport.msg
	HeaterStatus.msg
	HoverThrustEstimate.msg
	InputRc.msg
	InternalCombustionEngineControl.msg
	InternalCombustionEngineStatus.msg
	IridiumsbdStatus.msg
	IrlockReport.msg
	LandingGear.msg
	LandingGearWheel.msg
	LandingTargetInnovations.msg
	LandingTargetPose.msg
	LaunchDetectionStatus.msg
	LedControl.msg
	LoggerStatus.msg
	LogMessage.msg
	MagnetometerBiasEstimate.msg
	MagWorkerData.msg
	ManualControlSwitches.msg
	MavlinkLog.msg
	MavlinkTunnel.msg
	MessageFormatRequest.msg
	MessageFormatResponse.msg
	Mission.msg
	MissionResult.msg
	MountOrientation.msg
	NavigatorMissionItem.msg
	NavigatorStatus.msg
	NeuralControl.msg
	NormalizedUnsignedSetpoint.msg
	ObstacleDistance.msg
	OffboardControlMode.msg
	OnboardComputerStatus.msg
	OpenDroneIdArmStatus.msg
	OpenDroneIdOperatorId.msg
	OpenDroneIdSelfId.msg
	OpenDroneIdSystem.msg
	OrbitStatus.msg
	OrbTest.msg
	OrbTestLarge.msg
	OrbTestMedium.msg
	ParameterResetRequest.msg
	ParameterSetUsedRequest.msg
	ParameterSetValueRequest.msg
	ParameterSetValueResponse.msg
	ParameterUpdate.msg
	Ping.msg
	PositionControllerLandingStatus.msg
	PositionControllerStatus.msg
	PositionSetpoint.msg
	PositionSetpointTriplet.msg
	PowerButtonState.msg
	PowerMonitor.msg
	PpsCapture.msg
	PurePursuitStatus.msg
	PwmInput.msg
	Px4ioStatus.msg
	QshellReq.msg
	QshellRetval.msg
	RadioStatus.msg
	RateCtrlStatus.msg
	RcChannels.msg
	RcParameterMap.msg
	RoverAttitudeSetpoint.msg
	RoverAttitudeStatus.msg
	RoverPositionSetpoint.msg
	RoverRateSetpoint.msg
	RoverRateStatus.msg
	RoverSpeedSetpoint.msg
	RoverSpeedStatus.msg
	RoverSteeringSetpoint.msg
	RoverThrottleSetpoint.msg
	Rpm.msg
	RtlStatus.msg
	RtlTimeEstimate.msg
	SatelliteInfo.msg
	SensorAccel.msg
	SensorAccelFifo.msg
	SensorBaro.msg
	SensorCombined.msg
	SensorCorrection.msg
	SensorGnssRelative.msg
	SensorGnssStatus.msg
	SensorGps.msg
	SensorGyro.msg
	SensorGyroFft.msg
	SensorGyroFifo.msg
	SensorHygrometer.msg
	SensorMag.msg
	SensorOpticalFlow.msg
	SensorPreflightMag.msg
	SensorSelection.msg
	SensorsStatus.msg
	SensorsStatusImu.msg
	SensorUwb.msg
	SensorAirflow.msg
	SystemPower.msg
	TakeoffStatus.msg
	TaskStackInfo.msg
	TecsStatus.msg
	TelemetryStatus.msg
	TiltrotorExtraControls.msg
	TimesyncStatus.msg
	TrajectorySetpoint6dof.msg
	TransponderReport.msg
	TuneControl.msg
	UavcanParameterRequest.msg
	UavcanParameterValue.msg
	UlogStream.msg
	UlogStreamAck.msg
	VehicleAcceleration.msg
	VehicleAirData.msg
	VehicleAngularAccelerationSetpoint.msg
	VehicleConstraints.msg
	VehicleImu.msg
	VehicleImuStatus.msg
	VehicleLocalPositionSetpoint.msg
	VehicleMagnetometer.msg
	VehicleOpticalFlow.msg
	VehicleOpticalFlowVel.msg
	VehicleRoi.msg
	VehicleThrustSetpoint.msg
	VehicleTorqueSetpoint.msg
	VelocityLimits.msg
	WheelEncoders.msg
	YawEstimatorStatus.msg
	versioned/ActuatorMotors.msg
	versioned/ActuatorServos.msg
	versioned/AirspeedValidated.msg
	versioned/ArmingCheckReply.msg
	versioned/ArmingCheckRequest.msg
	versioned/BatteryStatus.msg
	versioned/ConfigOverrides.msg
	versioned/FixedWingLateralSetpoint.msg
	versioned/FixedWingLongitudinalSetpoint.msg
	versioned/GotoSetpoint.msg
	versioned/HomePosition.msg
	versioned/LateralControlConfiguration.msg
	versioned/LongitudinalControlConfiguration.msg
	versioned/ManualControlSetpoint.msg
	versioned/ModeCompleted.msg
	versioned/RegisterExtComponentReply.msg
	versioned/RegisterExtComponentRequest.msg
	versioned/TrajectorySetpoint.msg
	versioned/UnregisterExtComponent.msg
	versioned/VehicleAngularVelocity.msg
	versioned/VehicleAttitude.msg
	versioned/VehicleAttitudeSetpoint.msg
	versioned/VehicleCommandAck.msg
	versioned/VehicleCommand.msg
	versioned/VehicleControlMode.msg
	versioned/VehicleGlobalPosition.msg
	versioned/VehicleLandDetected.msg
	versioned/VehicleLocalPosition.msg
	versioned/VehicleOdometry.msg
	versioned/VehicleRatesSetpoint.msg
	versioned/VehicleStatus.msg
	versioned/VtolVehicleStatus.msg
	versioned/Wind.msg
)
list(SORT msg_files)

px4_list_make_absolute(msg_files ${CMAKE_CURRENT_SOURCE_DIR} ${msg_files})

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(ucdr_out_path ${PX4_BINARY_DIR}/uORB/ucdr)
set(msg_source_out_path ${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
set(uorb_ucdr_headers)
set(uorb_json_files)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
	list(APPEND uorb_ucdr_headers ${ucdr_out_path}/${msg}.h)
	list(APPEND uorb_json_files ${msg_source_out_path}/${msg}.json)
endforeach()

# set parent scope msg_files for ROS
set(msg_files ${msg_files} PARENT_SCOPE)

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.hpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

add_custom_command(
	OUTPUT
		${uorb_json_files}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--json
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.json.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB json files"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(uorb_json_files DEPENDS ${uorb_json_files})

set(uorb_message_fields_cpp_file ${msg_source_out_path}/uORBMessageFieldsGenerated.cpp)
set(uorb_message_fields_header_file ${msg_out_path}/uORBMessageFieldsGenerated.hpp)
add_custom_command(
	OUTPUT
		${uorb_message_fields_cpp_file}
		${uorb_message_fields_header_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
		-f ${uorb_json_files}
		--source-output-file ${uorb_message_fields_cpp_file}
		--header-output-file ${uorb_message_fields_header_file}
	DEPENDS
		${uorb_json_files}
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
	COMMENT "Generating uORB compressed fields"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)

# Generate microcdr headers
add_custom_command(
	OUTPUT ${uorb_ucdr_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${ucdr_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic ucdr headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_ucdr_headers DEPENDS ${uorb_ucdr_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp ${uorb_message_fields_cpp_file})
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)

if(CONFIG_LIB_CDRSTREAM)
	set(uorb_cdr_idl)
	set(uorb_cdr_msg)
	set(uorb_cdr_hash)
	set(uorb_cdr_idl_uorb)
	set(idl_include_path ${PX4_BINARY_DIR}/uORB/idl)
	set(idl_out_path ${idl_include_path}/px4/msg)
	set(idl_rihs01_out_path ${idl_include_path}/px4)
	set(idl_uorb_path ${PX4_BINARY_DIR}/msg/px4/msg)

	# Make sure that CycloneDDS has been checkout out
	execute_process(COMMAND git submodule sync src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )
	execute_process(COMMAND git submodule update --init --force src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )

	# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
	MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds
			-DCMAKE_C_COMPILER=/usr/bin/gcc
			-DBUILD_EXAMPLES=OFF
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

	# Copy .msg files
	foreach(msg_file ${msg_files})
		get_filename_component(msg ${msg_file} NAME_WE)
		configure_file(${msg_file} ${idl_out_path}/${msg}.msg COPYONLY)
		list(APPEND uorb_cdr_idl ${idl_out_path}/${msg}.idl)
		list(APPEND uorb_cdr_msg ${idl_out_path}/${msg}.msg)
		list(APPEND uorb_cdr_hash ${idl_out_path}/${msg}.json)
		list(APPEND uorb_cdr_idl_uorb ${idl_uorb_path}/${msg}.h)
	endforeach()

	# Generate IDL from .msg using rosidl_adapter
	# Note this a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_idl}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/msg2idl.py
			${uorb_cdr_msg}
		DEPENDS
			${uorb_cdr_msg}
			git_cyclonedds
		COMMENT "Generating IDL from uORB topic headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	file(CREATE_LINK ${idl_rihs01_out_path} ${idl_include_path}/px4_msgs SYMBOLIC)

	# Generate IDL from .msg using rosidl_adapter
	# Note this is a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_hash}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_parser:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_generator_type_description"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/idl2rihs01.py
			--output-dir ${idl_rihs01_out_path}
			${uorb_cdr_idl}
		DEPENDS
			${uorb_cdr_idl}
			git_cyclonedds
		COMMENT "Generating RIHS01 hashes from IDL"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	# Generate C definitions from IDL
	set(CYCLONEDDS_DIR ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds)
	include("${CYCLONEDDS_DIR}/cmake/Modules/Generate.cmake")
	idlc_generate(TARGET uorb_cdrstream
                  FEATURES "cdrstream-desc"
                  FILES ${uorb_cdr_idl}
                  INCLUDES ${idl_include_path}
                  BASE_DIR ${idl_include_path}
                  WARNINGS no-implicit-extensibility)
	target_link_libraries(uorb_cdrstream INTERFACE cdr)

	# Generate and overwrite IDL header with custom headers for uORB operatability
	# We typedef the IDL struct the uORB struct so that the IDL offset calculate
	# the offset of internal uORB struct for serialization/deserialization

	# In the future we might want to turn this around let the IDL struct be the leading ABI
	# However we need to remove the padding for logging and remove the re-ordering of fields

	add_custom_target(
		uorb_idl_header
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			--uorb-idl-header
			-f ${msg_files}
			-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
			-o ${idl_uorb_path}
			-e ${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream
		DEPENDS
			uorb_cdrstream
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream/uorb_idl_header.h.em
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
		COMMENT "Generating uORB compatible IDL headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)
	add_dependencies(uorb_msgs uorb_idl_header)

	# Compile all CDR compatible message defnitions
	target_link_libraries(uorb_msgs PRIVATE uorb_cdrstream )
endif()

if(CONFIG_MODULES_ZENOH)
	# Update kconfig file for topics
	execute_process(COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-config
			-f ${msg_files}
			-o ${PX4_SOURCE_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		)
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-pub-sub
			-f ${msg_files}
			-o ${PX4_BINARY_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
			--rihs ${idl_rihs01_out_path}
		DEPENDS
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh/uorb_pubsub_factory.hpp.em
			${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
		COMMENT "Generating Zenoh Topic Code"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)
		add_library(zenoh_topics ${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp)
		set_target_properties(zenoh_topics PROPERTIES LINKER_LANGUAGE CXX)
endif()

~~~

## 4.9 修改无人机端 Micro-XRCE-DDS-Client 配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
注：这里额外添加了 /fmu/out/vehicle_angular_velocity /fmu/out/esc_status 以方便后续的实验。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

~~~yaml
#####
#
# This file maps all the topics that are to be used on the uXRCE-DDS client.
#
#####
publications:

  - topic: /fmu/out/attack_status
    type: px4_msgs::msg::AttackStatus
    rate_limit: 10.

  - topic: /fmu/out/register_ext_component_reply
    type: px4_msgs::msg::RegisterExtComponentReply

  - topic: /fmu/out/arming_check_request
    type: px4_msgs::msg::ArmingCheckRequest
    rate_limit: 5.

  - topic: /fmu/out/mode_completed
    type: px4_msgs::msg::ModeCompleted
    rate_limit: 50.

  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
    rate_limit: 1.

  - topic: /fmu/out/collision_constraints
    type: px4_msgs::msg::CollisionConstraints
    rate_limit: 50.

  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags
    rate_limit: 5.

  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags
    rate_limit: 5.

  - topic: /fmu/out/manual_control_setpoint
    type: px4_msgs::msg::ManualControlSetpoint
    rate_limit: 25.

  - topic: /fmu/out/message_format_response
    type: px4_msgs::msg::MessageFormatResponse

  - topic: /fmu/out/position_setpoint_triplet
    type: px4_msgs::msg::PositionSetpointTriplet
    rate_limit: 5.

  - topic: /fmu/out/sensor_combined
    type: px4_msgs::msg::SensorCombined

  - topic: /fmu/out/timesync_status
    type: px4_msgs::msg::TimesyncStatus
    rate_limit: 10.

  - topic: /fmu/out/transponder_report
    type: px4_msgs::msg::TransponderReport
 
  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity
    rate_limit: 50.

  - topic: /fmu/out/vehicle_land_detected
    type: px4_msgs::msg::VehicleLandDetected
    rate_limit: 5.

  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude

  - topic: /fmu/out/vehicle_control_mode
    type: px4_msgs::msg::VehicleControlMode
    rate_limit: 50.

  - topic: /fmu/out/vehicle_command_ack
    type: px4_msgs::msg::VehicleCommandAck

  - topic: /fmu/out/vehicle_global_position
    type: px4_msgs::msg::VehicleGlobalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_gps_position
    type: px4_msgs::msg::SensorGps
    rate_limit: 50.

  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus
    rate_limit: 5.

  - topic: /fmu/out/airspeed_validated
    type: px4_msgs::msg::AirspeedValidated
    rate_limit: 50.

  - topic: /fmu/out/vtol_vehicle_status
    type: px4_msgs::msg::VtolVehicleStatus

  - topic: /fmu/out/home_position
    type: px4_msgs::msg::HomePosition
    rate_limit: 5.

  - topic: /fmu/out/wind
    type: px4_msgs::msg::Wind
    rate_limit: 1.

  - topic: /fmu/out/gimbal_device_attitude_status
    type: px4_msgs::msg::GimbalDeviceAttitudeStatus
    rate_limit: 20.
  
  - topic: /fmu/out/esc_status
    type: px4_msgs::msg::EscStatus

# Create uORB::Publication
subscriptions:
  - topic: /fmu/in/attack_command
    type: px4_msgs::msg::AttackCommand

  - topic: /fmu/in/register_ext_component_request
    type: px4_msgs::msg::RegisterExtComponentRequest

  - topic: /fmu/in/unregister_ext_component
    type: px4_msgs::msg::UnregisterExtComponent

  - topic: /fmu/in/config_overrides_request
    type: px4_msgs::msg::ConfigOverrides

  - topic: /fmu/in/arming_check_reply
    type: px4_msgs::msg::ArmingCheckReply

  - topic: /fmu/in/message_format_request
    type: px4_msgs::msg::MessageFormatRequest

  - topic: /fmu/in/mode_completed
    type: px4_msgs::msg::ModeCompleted

  - topic: /fmu/in/config_control_setpoints
    type: px4_msgs::msg::VehicleControlMode

  - topic: /fmu/in/distance_sensor
    type: px4_msgs::msg::DistanceSensor

  - topic: /fmu/in/manual_control_input
    type: px4_msgs::msg::ManualControlSetpoint

  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode

  - topic: /fmu/in/onboard_computer_status
    type: px4_msgs::msg::OnboardComputerStatus

  - topic: /fmu/in/obstacle_distance
    type: px4_msgs::msg::ObstacleDistance

  - topic: /fmu/in/sensor_optical_flow
    type: px4_msgs::msg::SensorOpticalFlow

  - topic: /fmu/in/goto_setpoint
    type: px4_msgs::msg::GotoSetpoint

  - topic: /fmu/in/telemetry_status
    type: px4_msgs::msg::TelemetryStatus

  - topic: /fmu/in/trajectory_setpoint
    type: px4_msgs::msg::TrajectorySetpoint

  - topic: /fmu/in/vehicle_attitude_setpoint
    type: px4_msgs::msg::VehicleAttitudeSetpoint

  - topic: /fmu/in/vehicle_mocap_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_command
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_command_mode_executor
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint

  - topic: /fmu/in/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint

  - topic: /fmu/in/actuator_motors
    type: px4_msgs::msg::ActuatorMotors

  - topic: /fmu/in/actuator_servos
    type: px4_msgs::msg::ActuatorServos

  - topic: /fmu/in/aux_global_position
    type: px4_msgs::msg::VehicleGlobalPosition

  - topic: /fmu/in/fixed_wing_longitudinal_setpoint
    type: px4_msgs::msg::FixedWingLongitudinalSetpoint

  - topic: /fmu/in/fixed_wing_lateral_setpoint
    type: px4_msgs::msg::FixedWingLateralSetpoint

  - topic: /fmu/in/longitudinal_control_configuration
    type: px4_msgs::msg::LongitudinalControlConfiguration

  - topic: /fmu/in/lateral_control_configuration
    type: px4_msgs::msg::LateralControlConfiguration

  - topic: /fmu/in/rover_position_setpoint
    type: px4_msgs::msg::RoverPositionSetpoint

  - topic: /fmu/in/rover_speed_setpoint
    type: px4_msgs::msg::RoverSpeedSetpoint

  - topic: /fmu/in/rover_attitude_setpoint
    type: px4_msgs::msg::RoverAttitudeSetpoint

  - topic: /fmu/in/rover_rate_setpoint
    type: px4_msgs::msg::RoverRateSetpoint

  - topic: /fmu/in/rover_throttle_setpoint
    type: px4_msgs::msg::RoverThrottleSetpoint

  - topic: /fmu/in/rover_steering_setpoint
    type: px4_msgs::msg::RoverSteeringSetpoint

  - topic: /fmu/in/landing_gear
    type: px4_msgs::msg::LandingGear

# Create uORB::PublicationMulti
subscriptions_multi:

~~~

## 4.10 修改无人机端 LOGGER 配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "logged_topics.h"
#include "messages.h"

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

void LoggedTopics::add_default_topics()
{
	add_topic("action_request");
	add_topic("actuator_armed");
	add_optional_topic("actuator_controls_status_0", 300);
	add_topic("airspeed", 1000);
	add_optional_topic("airspeed_validated", 200);
	add_topic("attack_command");
	add_topic("attack_status", 100);
	add_optional_topic("autotune_attitude_control_status", 100);
	add_topic_multi("battery_info", 5000, 3);
	add_optional_topic("camera_capture");
	add_optional_topic("camera_trigger");
	add_topic("cellular_status", 200);
	add_topic("commander_state");
	add_topic("config_overrides");
	add_topic("cpuload");
	add_topic("distance_sensor_mode_change_request");
	add_topic_multi("dronecan_node_status", 250);
	add_optional_topic("external_ins_attitude");
	add_optional_topic("external_ins_global_position");
	add_optional_topic("external_ins_local_position");
	// add_optional_topic("esc_status", 250);
	add_topic("esc_status");
	add_topic("failure_detector_status", 100);
	add_topic("failsafe_flags");
	add_optional_topic("follow_target", 500);
	add_optional_topic("follow_target_estimator", 200);
	add_optional_topic("follow_target_status", 400);
	add_optional_topic("flaps_setpoint", 1000);
	add_optional_topic("flight_phase_estimation", 1000);
	add_optional_topic("fuel_tank_status", 10);
	add_topic("gimbal_manager_set_attitude", 500);
	add_optional_topic("generator_status");
	add_optional_topic("gps_dump");
	add_optional_topic("gimbal_controls", 200);
	add_optional_topic("gripper");
	add_optional_topic("heater_status");
	add_topic("home_position");
	add_topic("hover_thrust_estimate", 100);
	add_topic("input_rc", 500);
	add_optional_topic("internal_combustion_engine_control", 10);
	add_optional_topic("internal_combustion_engine_status", 10);
	add_optional_topic("iridiumsbd_status", 1000);
	add_optional_topic("irlock_report", 1000);
	add_optional_topic("landing_gear", 200);
	add_optional_topic("landing_gear_wheel", 100);
	add_optional_topic("landing_target_pose", 1000);
	add_optional_topic("launch_detection_status", 200);
	add_optional_topic("magnetometer_bias_estimate", 200);
	add_topic("manual_control_setpoint", 200);
	add_topic("manual_control_switches");
	add_topic("mission_result");
	add_topic("navigator_mission_item");
	add_topic("navigator_status");
	add_topic("offboard_control_mode", 100);
	add_topic("onboard_computer_status", 10);
	add_topic("parameter_update");
	add_topic("position_controller_status", 500);
	add_topic("position_controller_landing_status", 100);
	add_optional_topic("pure_pursuit_status", 100);
	add_topic("goto_setpoint", 200);
	add_topic("position_setpoint_triplet", 200);
	add_optional_topic("px4io_status");
	add_topic("radio_status");
	add_optional_topic("rover_attitude_setpoint", 100);
	add_optional_topic("rover_attitude_status", 100);
	add_optional_topic("rover_position_setpoint", 100);
	add_optional_topic("rover_rate_setpoint", 100);
	add_optional_topic("rover_rate_status", 100);
	add_optional_topic("rover_speed_setpoint", 100);
	add_optional_topic("rover_speed_status", 100);
	add_optional_topic("rover_steering_setpoint", 100);
	add_optional_topic("rover_throttle_setpoint", 100);
	add_topic("rtl_time_estimate", 1000);
	add_topic("rtl_status", 2000);
	add_optional_topic("sensor_airflow", 100);
	add_topic("sensor_combined");
	add_optional_topic("sensor_correction");
	add_optional_topic("sensor_gyro_fft", 50);
	add_topic("sensor_selection");
	add_topic("sensors_status_imu", 200);
	add_optional_topic("spoilers_setpoint", 1000);
	add_topic("system_power", 500);
	add_optional_topic("takeoff_status", 1000);
	add_optional_topic("tecs_status", 200);
	add_optional_topic("tiltrotor_extra_controls", 100);
	add_topic("trajectory_setpoint", 200);
	add_topic("transponder_report");
	add_topic("vehicle_acceleration", 50);
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_angular_velocity", 20);
	add_topic("vehicle_attitude", 50);
	add_topic("vehicle_attitude_setpoint", 50);
	add_topic("vehicle_command");
	add_topic("vehicle_command_ack");
	add_topic("vehicle_constraints", 1000);
	add_topic("vehicle_control_mode");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position", 100);
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 20);
	add_topic("vehicle_roi", 1000);
	add_topic("vehicle_status");
	add_optional_topic("vtol_vehicle_status", 200);
	add_topic("wind", 1000);
	add_topic("fixed_wing_lateral_setpoint");
	add_topic("fixed_wing_longitudinal_setpoint");
	add_topic("longitudinal_control_configuration");
	add_topic("lateral_control_configuration");
	add_optional_topic("fixed_wing_lateral_guidance_status", 100);
	add_optional_topic("fixed_wing_lateral_status", 100);
	add_optional_topic("fixed_wing_runway_control", 100);

	// multi topics
	add_optional_topic_multi("actuator_outputs", 100, 3);
	add_optional_topic_multi("airspeed_wind", 1000, 4);
	add_optional_topic_multi("control_allocator_status", 200, 2);
	add_optional_topic_multi("rate_ctrl_status", 200, 2);
	add_optional_topic_multi("sensor_hygrometer", 500, 4);
	add_optional_topic_multi("rpm", 200);
	add_topic_multi("timesync_status", 1000, 3);
	add_optional_topic_multi("telemetry_status", 1000, 4);

	// EKF multi topics
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 500; // 2 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	// important EKF topics (higher rate)
	add_optional_topic("estimator_selector_status", 10);
	add_optional_topic_multi("estimator_event_flags", 10);
	add_optional_topic_multi("estimator_optical_flow_vel", 200);
	add_optional_topic_multi("estimator_sensor_bias", 1000);
	add_optional_topic_multi("estimator_status", 200);
	add_optional_topic_multi("estimator_status_flags", 10);
	add_optional_topic_multi("yaw_estimator_status", 1000);

	// log all raw sensors at minimal rate (at least 1 Hz)
	add_topic_multi("battery_status", 200, 3);
	add_topic_multi("differential_pressure", 1000, 2);
	add_topic_multi("distance_sensor", 1000, 2);
	add_optional_topic_multi("sensor_accel", 1000, 4);
	add_topic_multi("sensor_baro", 1000, 4);
	add_topic_multi("sensor_gps", 1000, 2);
	add_topic_multi("sensor_gnss_relative", 1000, 1);
	add_optional_topic_multi("sensor_gyro", 1000, 4);
	add_topic_multi("sensor_mag", 1000, 4);
	add_topic_multi("sensor_optical_flow", 1000, 2);

	add_topic_multi("vehicle_imu", 500, 4);
	add_topic_multi("vehicle_imu_status", 1000, 4);
	add_optional_topic_multi("vehicle_magnetometer", 500, 4);
	add_topic("vehicle_optical_flow", 500);
	add_topic("aux_global_position", 500);
	//add_optional_topic("vehicle_optical_flow_vel", 100);
	add_optional_topic("pps_capture");

	// additional control allocation logging
	add_topic("actuator_motors", 100);
	add_topic("actuator_servos", 100);
	add_topic_multi("vehicle_thrust_setpoint", 20, 2);
	add_topic_multi("vehicle_torque_setpoint", 20, 2);

	// SYS_HITL: default ground truth logging for simulation
	int32_t sys_hitl = 0;
	param_get(param_find("SYS_HITL"), &sys_hitl);

	if (sys_hitl >= 1) {
		add_topic("vehicle_angular_velocity_groundtruth", 10);
		add_topic("vehicle_attitude_groundtruth", 10);
		add_topic("vehicle_global_position_groundtruth", 100);
		add_topic("vehicle_local_position_groundtruth", 20);
	}

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_optional_topic("vehicle_torque_setpoint_virtual_mc");
	add_optional_topic("vehicle_torque_setpoint_virtual_fw");
	add_optional_topic("vehicle_thrust_setpoint_virtual_mc");
	add_optional_topic("vehicle_thrust_setpoint_virtual_fw");
	add_topic("time_offset");
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_angular_velocity_groundtruth", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 20);

	// EKF replay
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 10; // 100 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	add_topic("vehicle_attitude");
	add_topic("vehicle_global_position");
	add_topic("vehicle_local_position");
	add_topic("wind");
	add_optional_topic_multi("yaw_estimator_status");

#endif /* CONFIG_ARCH_BOARD_PX4_SITL */

#ifdef CONFIG_BOARD_UAVCAN_INTERFACES
	add_topic_multi("can_interface_status", 100, CONFIG_BOARD_UAVCAN_INTERFACES);
#endif
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint");
	add_topic_multi("rate_ctrl_status", 20, 2);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");

	add_topic("esc_status", 5);
	add_topic("actuator_motors");
	add_topic("actuator_outputs_debug");
	add_topic("actuator_servos");
	add_topic_multi("vehicle_thrust_setpoint", 0, 2);
	add_topic_multi("vehicle_torque_setpoint", 0, 2);
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("actuator_test", 500);
	add_topic("neural_control", 50);
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("airspeed_validated");
	add_topic("vehicle_optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic("aux_global_position");
	add_topic_multi("distance_sensor");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic_multi("distance_sensor");
	add_topic("obstacle_distance_fused");
	add_topic("obstacle_distance");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_raw_imu_gyro_fifo()
{
	add_topic("sensor_gyro_fifo");
}

void LoggedTopics::add_raw_imu_accel_fifo()
{
	add_topic("sensor_accel_fifo");
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_torque_setpoint");
	add_topic("vehicle_acceleration");
	add_topic("actuator_motors");
}

void LoggedTopics::add_high_rate_sensors_topics()
{
	add_topic_multi("distance_sensor", 0, 4);
	add_topic_multi("sensor_optical_flow", 0, 2);
	add_topic_multi("sensor_gps", 0, 4);
	add_topic_multi("sensor_mag", 0, 4);
}

void LoggedTopics::add_mavlink_tunnel()
{
	add_topic("mavlink_tunnel");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %" PRIu32 " %" PRIu32, topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance, bool optional)
{
	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %" PRIu8, topic->o_name, instance);
		return false;
	}

	if (optional && orb_exists(topic, instance) != 0) {
		PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);

		if (instance == 0 && _subscriptions.num_excluded_optional_topic_ids < MAX_EXCLUDED_OPTIONAL_TOPICS_NUM) {
			_subscriptions.excluded_optional_topic_ids[_subscriptions.num_excluded_optional_topic_ids++] = topic->o_id;
		}

		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance, bool optional)
{
	interval_ms /= _rate_factor;

	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%" PRIu8 "), interval: %" PRIu16 ", already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance, optional);

				if (success) {
					PX4_DEBUG("logging topic: %s(%" PRIu8 "), interval: %" PRIu16, topics[i]->o_name, instance, interval_ms);
				}

				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms, uint8_t max_num_instances, bool optional)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < max_num_instances; instance++) {
		add_topic(name, interval_ms, instance, optional);
	}

	return true;
}

bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return _subscriptions.count > 0;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}

	if (profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO) {
		add_raw_imu_gyro_fifo();
	}

	if (profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) {
		add_raw_imu_accel_fifo();
	}

	if (profile & SDLogProfileMask::MAVLINK_TUNNEL) {
		add_mavlink_tunnel();
	}

	if (profile & SDLogProfileMask::HIGH_RATE_SENSORS) {
		add_high_rate_sensors_topics();
	}
}

~~~

## 4.11 攻击仿真测试

开启 QGC 地面站：

~~~bash
cd /home/liu/Desktop/ROS2/QGroundControl && ./QGroundControl-x86_64.AppImage
~~~

开启 SITL 仿真：

~~~bash
cd /home/liu/Desktop/ROS2/PX4-Autopilot && PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
~~~

开启 Micro-XRCE-DDS-Agent 代理：

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

开启 ROS2 消息桥接：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
~~~

实施攻击命令：

*通用模板：*

~~~bash
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: <通道号>, type: <原语号>, param: [a, b, c, d], t0_us: <延迟µs>, t1_us: <时长µs>, seed: <种子>}" --once
~~~

*字段说明：*

| 字段      | 含义                                                     | 默认 |
| --------- | -------------------------------------------------------- | ---- |
| channel   | 通道号（0–9）                                            | 必填 |
| type      | 原语号（0–13），0 = 清除攻击                             | 必填 |
| param     | [a, b, c, d]，含义随 type 变                             | 全 0 |
| t0_us     | 相对接收时刻的启动延迟（µs），0 = 立即                   | 0    |
| t1_us     | 相对启动的持续时间（µs），0 = 无限                       | 0    |
| seed      | 随机原语种子，0 = 随机，非 0 = 可复现                    | 0    |
| timestamp | 消息时间戳（管理器不用它定时机，用接收时的 hrt，可省略） | 0    |

*通道编号：*

| 号   | 通道         |
| ---- | ------------ |
| 0    | GPS 纬度     |
| 1    | GPS 经度     |
| 2    | GPS 高度     |
| 3    | GPS 北向速度 |
| 4    | GPS 东向速度 |
| 5    | GPS 地向速度 |
| 6    | 电机 0       |
| 7    | 电机 1       |
| 8    | 电机 2       |
| 9    | 电机 3       |

*原语编号：*

| 号   | 原语        | 公式                             | param                  |
| ---- | ----------- | -------------------------------- | ---------------------- |
| 0    | NONE        | 直通（清除）                     | —                      |
| 1    | BIAS        | v = truth + a                    | a=偏置                 |
| 2    | SPOOF       | v = a                            | a=目标值               |
| 3    | NOISE       | v = truth + N(0,a)               | a=σ                    |
| 4    | SCALING     | v = truth × a                    | a=系数                 |
| 5    | DRIFT       | v = truth + a·(t−start)          | a=斜率                 |
| 6    | OSCILLATION | v = truth + a·sin(b·(t−start)+c) | a=振幅 b=角频率 c=相位 |
| 7    | RANDOM_WALK | v = v_last + N(0,a)              | a=步长σ                |
| 8    | QUANTIZE    | v = round(truth/a)·a             | a=步长                 |
| 9    | CLAMP       | v = clamp(truth,a,b)             | a=min b=max            |
| 10   | FREEZE      | v = v_last                       | —                      |
| 11   | DROP        | 概率 a 丢本条                    | a=丢包率               |
| 12   | DELAY       | out(t)=in(t−a)                   | a=延迟秒               |
| 13   | REPLAY      | 循环回放开始前 a 秒              | a=片段秒               |

*命令示例：*

~~~bash
# 1 BIAS —— 纬度 +0.001°
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 1, param: [0.001, 0.0, 0.0, 0.0]}" --once

# 2 SPOOF —— 纬度欺骗到 31.23°
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 2, param: [31.23, 0.0, 0.0, 0.0]}" --once

# 3 NOISE —— 纬度加 σ=0.0001° 噪声（seed 42 可复现）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 3, param: [0.0001, 0.0, 0.0, 0.0], seed: 42}" --once

# 4 SCALING —— 电机 0 输出 ×0.5
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 6, type: 4, param: [0.5, 0.0, 0.0, 0.0]}" --once

# 5 DRIFT —— 纬度 0.0001°/s 线性漂移
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 5, param: [0.0001, 0.0, 0.0, 0.0]}" --once

# 6 OSCILLATION —— 纬度振幅 0.0001°、角频率 2 rad/s
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 6, param: [0.0001, 2.0, 0.0, 0.0]}" --once

# 7 RANDOM_WALK —— 电机 0 步长 σ=100
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 6, type: 7, param: [100.0, 0.0, 0.0, 0.0]}" --once

# 8 QUANTIZE —— 高度量化到 1 m 步长
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 2, type: 8, param: [1.0, 0.0, 0.0, 0.0]}" --once

# 9 CLAMP —— 电机 0 限幅到 [1000, 5000]
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 6, type: 9, param: [1000.0, 5000.0, 0.0, 0.0]}" --once

# 10 FREEZE —— 电机 0 冻结
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 6, type: 10}" --once

# 11 DROP —— GPS 丢包率 0.2（注意：任一 GPS 分量丢 → 整条 fix 不发布，即 GPS 六个分量中有一个被丢弃，所有六个分量这一周期都不发布）
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 11, param: [0.2, 0.0, 0.0, 0.0]}" --once

# 12 DELAY —— GPS 滞后 2 秒
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 12, param: [2.0, 0.0, 0.0, 0.0]}" --once

# 13 REPLAY —— 循环回放开始前 5 秒的 GPS 数据
ros2 topic pub /fmu/in/attack_command px4_msgs/msg/AttackCommand "{channel: 0, type: 13, param: [5.0, 0.0, 0.0, 0.0]}" --once
~~~

# 五、实机攻击注入

## 5.1 代码库接入构建系统

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/CMakeLists.txt

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/CMakeLists.txt

~~~c++
############################################################################
#
#   Copyright (c) 2017-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(adsb EXCLUDE_FROM_ALL)
add_subdirectory(airspeed EXCLUDE_FROM_ALL)
add_subdirectory(atmosphere EXCLUDE_FROM_ALL)
add_subdirectory(battery EXCLUDE_FROM_ALL)
add_subdirectory(button EXCLUDE_FROM_ALL)
add_subdirectory(cdev EXCLUDE_FROM_ALL)
add_subdirectory(cdrstream EXCLUDE_FROM_ALL)
add_subdirectory(circuit_breaker EXCLUDE_FROM_ALL)
add_subdirectory(collision_prevention EXCLUDE_FROM_ALL)
add_subdirectory(component_information EXCLUDE_FROM_ALL)
add_subdirectory(control_allocation EXCLUDE_FROM_ALL)
add_subdirectory(controllib EXCLUDE_FROM_ALL)
add_subdirectory(conversion EXCLUDE_FROM_ALL)
add_subdirectory(crc EXCLUDE_FROM_ALL)
add_subdirectory(crypto EXCLUDE_FROM_ALL)
add_subdirectory(dataman_client EXCLUDE_FROM_ALL)
add_subdirectory(drivers EXCLUDE_FROM_ALL)
add_subdirectory(field_sensor_bias_estimator EXCLUDE_FROM_ALL)
add_subdirectory(geo EXCLUDE_FROM_ALL)
add_subdirectory(heatshrink EXCLUDE_FROM_ALL)
add_subdirectory(hysteresis EXCLUDE_FROM_ALL)
add_subdirectory(lat_lon_alt EXCLUDE_FROM_ALL)
add_subdirectory(led EXCLUDE_FROM_ALL)
add_subdirectory(matrix EXCLUDE_FROM_ALL)
add_subdirectory(mathlib EXCLUDE_FROM_ALL)
add_subdirectory(mixer_module EXCLUDE_FROM_ALL)
add_subdirectory(motion_planning EXCLUDE_FROM_ALL)
add_subdirectory(npfg EXCLUDE_FROM_ALL)
add_subdirectory(perf EXCLUDE_FROM_ALL)
add_subdirectory(fw_performance_model EXCLUDE_FROM_ALL)
add_subdirectory(pid EXCLUDE_FROM_ALL)
add_subdirectory(pid_design EXCLUDE_FROM_ALL)
add_subdirectory(pure_pursuit EXCLUDE_FROM_ALL)
add_subdirectory(rate_control EXCLUDE_FROM_ALL)
add_subdirectory(rc EXCLUDE_FROM_ALL)
add_subdirectory(ringbuffer EXCLUDE_FROM_ALL)
add_subdirectory(rover_control EXCLUDE_FROM_ALL)
add_subdirectory(rtl EXCLUDE_FROM_ALL)
add_subdirectory(sensor_calibration EXCLUDE_FROM_ALL)
add_subdirectory(slew_rate EXCLUDE_FROM_ALL)
add_subdirectory(state_attack EXCLUDE_FROM_ALL)
add_subdirectory(sticks EXCLUDE_FROM_ALL)
add_subdirectory(stick_yaw EXCLUDE_FROM_ALL)
add_subdirectory(systemlib EXCLUDE_FROM_ALL)
add_subdirectory(system_identification EXCLUDE_FROM_ALL)
add_subdirectory(tecs EXCLUDE_FROM_ALL)
add_subdirectory(tensorflow_lite_micro EXCLUDE_FROM_ALL)
add_subdirectory(terrain_estimation EXCLUDE_FROM_ALL)
add_subdirectory(timesync EXCLUDE_FROM_ALL)
add_subdirectory(tinybson EXCLUDE_FROM_ALL)
add_subdirectory(tunes EXCLUDE_FROM_ALL)
add_subdirectory(variable_length_ringbuffer EXCLUDE_FROM_ALL)
add_subdirectory(version EXCLUDE_FROM_ALL)
add_subdirectory(weather_vane EXCLUDE_FROM_ALL)
add_subdirectory(wind_estimator EXCLUDE_FROM_ALL)
add_subdirectory(world_magnetic_model EXCLUDE_FROM_ALL)

~~~

## 5.2 构建状态攻击原语库

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackPrimitive.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackPrimitive.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackPrimitive.hpp

~~~c++
/****************************************************************************
 *
 * Post-EKF state attack — primitive library.
 *
 * Pure "how to tamper with a single state scalar" logic. No uORB / matrix
 * dependency, so it compiles and unit-tests standalone. This is the firmware
 * mirror of the gz_bridge `attack/` library (see attack_injection_design.md):
 * the primitive math is reused verbatim, but the code is independent because
 * firmware cannot link the simulation-side library.
 *
 * Scope vs. the gz_bridge library
 * -------------------------------
 * - The 13 wire type codes (NONE=0 .. REPLAY=13) are kept stable for
 *   protocol compatibility with attack_command semantics.
 * - The 11 point transforms (bias .. freeze) are fully implemented; DROP
 *   degrades to pass-through because an in-place state field has no message
 *   to drop (design §9 #2).
 * - DELAY / REPLAY are time-series transforms and are NOT implemented for the
 *   state attack (design §10): they would need a per-channel history buffer,
 *   so here they pass through. No history buffer is allocated, keeping the
 *   firmware memory footprint tiny (one scalar per channel). To implement
 *   them, follow the gz_bridge `attack/` library's time-series design.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

namespace state_attack
{

/**
 * Attack shape for one state scalar. The numeric value is the wire number
 * carried by state_attack_command / state_attack_*_status, so it must stay
 * stable once released.
 */
enum class StatePrimitiveType : uint8_t {
	NONE = 0,      ///< Pass-through (attack disabled).
	BIAS,          ///< v = truth + a
	SPOOF,         ///< v = a
	NOISE,         ///< v = truth + N(0, a) (seeded)
	SCALING,       ///< v = truth * a
	DRIFT,         ///< v = truth + a * (t - start) (a = slope)
	OSCILLATION,   ///< v = truth + a * sin(b*(t - start) + c)
	RANDOM_WALK,   ///< v = v_last + N(0, a) (stateful)
	QUANTIZE,      ///< v = round(truth / a) * a (a = step)
	CLAMP,         ///< v = clamp(truth, a, b) (a = min, b = max)
	FREEZE,        ///< v = v_last (stateful)
	DROP,          ///< pass-through: no message to drop here (design §9 #2)
	DELAY,         ///< NOT implemented (design §10) — pass-through
	REPLAY,        ///< NOT implemented (design §10) — pass-through
};

/**
 * State-channel vocabulary. A channel is a plain index; its meaning lives at
 * the injection point (mc_pos_control for nav, mc_att_control for attitude).
 */
enum class StateChannel : uint8_t {
	NAV_POS_X = 0,  ///< NED north position [m]
	NAV_POS_Y,      ///< NED east position [m]
	NAV_POS_Z,      ///< NED down position [m]
	NAV_VEL_X,      ///< NED north velocity [m/s]
	NAV_VEL_Y,      ///< NED east velocity [m/s]
	NAV_VEL_Z,      ///< NED down velocity [m/s]
	NAV_HEADING,    ///< heading [rad], -pi..pi
	ATT_ROLL = 7,   ///< roll [rad]
	ATT_PITCH,      ///< pitch [rad]
	ATT_YAW,        ///< yaw [rad]
	NUM_CHANNELS = 10,
};

/// Number of state-attack channels (nav 7 + attitude 3).
constexpr size_t kNumStateChannels = static_cast<size_t>(StateChannel::NUM_CHANNELS);

/// Array index of a channel in the per-channel tables.
constexpr size_t index_of(StateChannel ch)
{
	return static_cast<size_t>(ch);
}

/**
 * Serializable attack configuration for one channel (travels in
 * state_attack_command; logged for reproducibility). Pure value; runtime state
 * lives in StateChannelState.
 */
struct StateAttackSpec {
	StatePrimitiveType type{StatePrimitiveType::NONE};
	double a{0.0};
	double b{0.0};
	double c{0.0};
	double d{0.0};
	uint64_t start_us{0};   ///< Absolute hrt time the attack becomes active.
	uint64_t end_us{0};     ///< Absolute hrt time the attack ends; 0 = no end.
	uint32_t seed{0};       ///< RNG seed; 0 = per-channel deterministic default.
};

/**
 * Per-channel runtime state, owned by StateAttackManager (not serialized).
 * Intentionally small: no history buffer (DELAY / REPLAY are not implemented).
 */
struct StateChannelState {
	double last_value{0.0};   ///< Last value that passed through (FREEZE / RANDOM_WALK).
	bool has_last{false};     ///< Whether last_value is valid.
	uint32_t rng_state{0};    ///< PRNG state, seeded by StateAttackManager::set().
};

/**
 * Stateless attack math. All per-channel state is passed in via
 * StateChannelState, so this class holds no mutable state of its own.
 */
class StateAttackLibrary
{
public:
	/**
	 * Apply an attack to a scalar truth value, returning the (possibly)
	 * tampered value. There is no "drop" outcome: an in-place state field is
	 * always read, so DROP passes through.
	 */
	static double apply(double truth, const StateAttackSpec &spec, uint64_t now_us, StateChannelState &state);

	/// Whether an attack is currently within its active [start_us, end_us) window.
	static bool active(const StateAttackSpec &spec, uint64_t now_us);
};

} // namespace state_attack

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackPrimitive.cpp

~~~c++
/****************************************************************************
 *
 * Post-EKF state attack — primitive library, implementation.
 *
 ****************************************************************************/

#include <cmath>

#include "StateAttackPrimitive.hpp"


namespace state_attack
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/**
 * xorshift32 — small, fast, deterministic PRNG.
 */
uint32_t xorshift32(uint32_t &state)
{
	uint32_t x = (state == 0) ? 0x9E3779B9u : state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	state = x;
	return x;
}

/// Uniform random value in [0, 1).
double uniform(uint32_t &state)
{
	return static_cast<double>(xorshift32(state) >> 8) / 16777216.0;
}

/// Standard normal random value via Box-Muller.
double gaussian(uint32_t &state)
{
	double u1 = uniform(state);
	double u2 = uniform(state);

	if (u1 < 1e-12) {u1 = 1e-12;}

	return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

/// Seconds elapsed since the attack start (clamped to >= 0).
double elapsed_s(const StateAttackSpec &spec, uint64_t now_us)
{
	if (now_us <= spec.start_us) {return 0.0;}

	return static_cast<double>(now_us - spec.start_us) / 1e6;
}

/**
 * Compute the tampered value for an already-active attack. The caller has
 * already established that `type != NONE` and `now_us` is within the
 * [start_us, end_us) window.
 */
double transform(StatePrimitiveType type, double truth, const StateAttackSpec &spec, uint64_t now_us,
		 StateChannelState &state)
{
	switch (type) {
	case StatePrimitiveType::BIAS:
		return truth + spec.a;

	case StatePrimitiveType::SPOOF:
		return spec.a;

	case StatePrimitiveType::NOISE:
		return truth + spec.a * gaussian(state.rng_state);

	case StatePrimitiveType::SCALING:
		return truth * spec.a;

	case StatePrimitiveType::DRIFT:
		return truth + spec.a * elapsed_s(spec, now_us);

	case StatePrimitiveType::OSCILLATION:
		return truth + spec.a * std::sin(spec.b * elapsed_s(spec, now_us) + spec.c);

	case StatePrimitiveType::RANDOM_WALK:
		return (state.has_last ? state.last_value : truth) + spec.a * gaussian(state.rng_state);

	case StatePrimitiveType::QUANTIZE:
		// Step must have a non-negligible magnitude, otherwise pass through.
		if (std::fabs(spec.a) > 1e-12) {return std::round(truth / spec.a) * spec.a;}
		return truth;

	case StatePrimitiveType::CLAMP:
		return (truth < spec.a) ? spec.a : ((truth > spec.b) ? spec.b : truth);

	case StatePrimitiveType::FREEZE:
		return state.has_last ? state.last_value : truth;

	case StatePrimitiveType::DROP:
		// An in-place state field has no message to drop: degrade to no-op (design §9 #2).
		return truth;

	case StatePrimitiveType::DELAY:
		// Time-series primitives not implemented for the state attack (design §10).
		return truth;
		
	case StatePrimitiveType::REPLAY:
		// Time-series primitives not implemented for the state attack (design §10).
		return truth;

	case StatePrimitiveType::NONE:
	default:
		return truth;
	}
}

} // namespace

bool StateAttackLibrary::active(const StateAttackSpec &spec, uint64_t now_us)
{
	if (spec.type == StatePrimitiveType::NONE) {
		return false;
	}

	if (now_us < spec.start_us) {
		return false;
	}

	if (spec.end_us != 0 && now_us >= spec.end_us) {
		return false;
	}

	return true;
}

double StateAttackLibrary::apply(double truth, const StateAttackSpec &spec, uint64_t now_us, StateChannelState &state)
{
	// Disabled or outside the window: pass through, but keep last_value fresh
	// so FREEZE / RANDOM_WALK start from the current value when activated.
	if (!active(spec, now_us)) {
		state.last_value = truth;
		state.has_last = true;
		return truth;
	}

	const double result = transform(spec.type, truth, spec, now_us, state);

	state.last_value = result;
	state.has_last = true;

	return result;
}

} // namespace state_attack

~~~

## 5.3 构建状态攻击管理库

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackManager.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackManager.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackManager.hpp

~~~c++
/****************************************************************************
 *
 * Post-EKF state attack — manager.
 *
 * Owns the per-channel attack table (StateChannel -> StateAttackSpec +
 * StateChannelState), consumes uORB `state_attack_command`, applies attacks
 * on demand, and publishes a per-module status topic.
 *
 * One instance lives in each of the two controller modules, differing only in
 * the status message type it publishes and the channel subset it owns:
 *
 *   mc_pos_control: StateAttackManager<state_attack_pos_status_s> {…, 0, 7}
 *   mc_att_control: StateAttackManager<state_attack_att_status_s> {…, 7, 3}
 *
 * Both subscribe to the SAME broadcast `state_attack_command` and maintain the
 * same full 10-channel table, but each applies only its own channels. No
 * mutex: a manager runs entirely inside its module's control loop.
 *
 * The class is a template only so the status publication is type-safe against
 * the concrete status topic. The implementation lives in StateAttackManager.cpp
 * and is explicitly instantiated for the two status types actually used (see
 * the `extern template` declarations below); modules link those instantiations
 * instead of compiling their own copy.
 *
 ****************************************************************************/

#pragma once

#include "StateAttackPrimitive.hpp"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/state_attack_att_status.h>
#include <uORB/topics/state_attack_command.h>
#include <uORB/topics/state_attack_pos_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

namespace state_attack
{

template <typename StatusMsg>
class StateAttackManager
{
public:
	StateAttackManager(const orb_metadata *status_meta, uint8_t first_channel, uint8_t num_channels);

	/// Advertise the status topic up front so subscribers (logger / DDS) see it immediately.
	void init();

	/// Consume the latest command and publish the status topic. Call every loop.
	void update();

	/// In-place tamper the 7 nav fields (channels 0..6) of a local position sample.
	void apply_position(vehicle_local_position_s &v);

	/// In-place tamper the 3 attitude channels (7..9) in the Euler domain, then
	/// rebuild a unit quaternion (mutating q components directly would break the
	/// unit norm).
	void apply_attitude(vehicle_attitude_s &v);

private:
	void apply_channel(StateChannel ch, double &value, uint64_t now_us);
	void on_command(const state_attack_command_s &cmd, uint64_t now_us);
	void publish_status(uint64_t now_us);

	static uint32_t default_seed(size_t idx);

	StateAttackSpec _specs[kNumStateChannels]{};
	StateChannelState _state[kNumStateChannels]{};

	uORB::Subscription _cmd_sub{ORB_ID(state_attack_command)};
	uORB::Publication<StatusMsg> _status_pub;

	uint8_t _first_channel{0};
	uint8_t _num_channels{0};
	uint64_t _last_sample_us{0};
};

// Concrete instantiations are compiled in StateAttackManager.cpp. These extern
// declarations stop the two controller modules from instantiating their own
// copies; they link the state_attack library's definitions instead.
extern template class StateAttackManager<state_attack_pos_status_s>;
extern template class StateAttackManager<state_attack_att_status_s>;

} // namespace state_attack

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/StateAttackManager.cpp

~~~c++
/****************************************************************************
 *
 * Post-EKF state attack — manager, implementation.
 *
 * Explicitly instantiates the manager for the two status topics actually used
 * (see the matching `extern template` declarations in StateAttackManager.hpp).
 *
 ****************************************************************************/

#include "StateAttackManager.hpp"

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>

namespace state_attack
{

template <typename StatusMsg>
StateAttackManager<StatusMsg>::StateAttackManager(const orb_metadata *status_meta, uint8_t first_channel, uint8_t num_channels)
	: _status_pub{status_meta}, _first_channel(first_channel), _num_channels(num_channels)
{
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::init()
{
	_status_pub.advertise();
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::update()
{
	const uint64_t now = hrt_absolute_time();

	state_attack_command_s cmd{};

	if (_cmd_sub.update(&cmd)) {
		on_command(cmd, now);
	}

	publish_status(now);
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::apply_position(vehicle_local_position_s &v)
{
	const uint64_t now = hrt_absolute_time();
	_last_sample_us = v.timestamp_sample;

	double x = v.x;
	double y = v.y;
	double z = v.z;
	double vx = v.vx;
	double vy = v.vy;
	double vz = v.vz;
	double heading = v.heading;

	apply_channel(StateChannel::NAV_POS_X, x, now);
	apply_channel(StateChannel::NAV_POS_Y, y, now);
	apply_channel(StateChannel::NAV_POS_Z, z, now);
	apply_channel(StateChannel::NAV_VEL_X, vx, now);
	apply_channel(StateChannel::NAV_VEL_Y, vy, now);
	apply_channel(StateChannel::NAV_VEL_Z, vz, now);
	apply_channel(StateChannel::NAV_HEADING, heading, now);

	v.x = static_cast<float>(x);
	v.y = static_cast<float>(y);
	v.z = static_cast<float>(z);
	v.vx = static_cast<float>(vx);
	v.vy = static_cast<float>(vy);
	v.vz = static_cast<float>(vz);
	v.heading = static_cast<float>(heading);
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::apply_attitude(vehicle_attitude_s &v)
{
	const uint64_t now = hrt_absolute_time();
	_last_sample_us = v.timestamp_sample;

	const matrix::Quatf q_in{v.q};
	const matrix::Eulerf e{q_in};

	double roll = e.phi();
	double pitch = e.theta();
	double yaw = e.psi();

	apply_channel(StateChannel::ATT_ROLL, roll, now);
	apply_channel(StateChannel::ATT_PITCH, pitch, now);
	apply_channel(StateChannel::ATT_YAW, yaw, now);

	const matrix::Quatf q_out{
		matrix::Eulerf(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw))};

	v.q[0] = q_out(0);
	v.q[1] = q_out(1);
	v.q[2] = q_out(2);
	v.q[3] = q_out(3);
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::apply_channel(StateChannel ch, double &value, uint64_t now_us)
{
	const size_t idx = index_of(ch);

	if (idx < kNumStateChannels) {
		value = StateAttackLibrary::apply(value, _specs[idx], now_us, _state[idx]);
	}
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::on_command(const state_attack_command_s &cmd, uint64_t now_us)
{
	if (cmd.channel >= kNumStateChannels) {
		return;  // Unknown channel.
	}

	const size_t idx = cmd.channel;
	StateAttackSpec &spec = _specs[idx];
	spec.type = static_cast<StatePrimitiveType>(cmd.type);
	spec.a = cmd.param[0];
	spec.b = cmd.param[1];
	spec.c = cmd.param[2];
	spec.d = cmd.param[3];
	spec.seed = cmd.seed;
	spec.start_us = now_us + cmd.t0_us;
	spec.end_us = (cmd.t1_us != 0) ? spec.start_us + cmd.t1_us : 0;

	// Reset runtime state (reseed the PRNG). seed != 0 uses spec.seed,
	// seed == 0 uses the per-channel deterministic default so reruns match.
	StateChannelState &state = _state[idx];
	state.last_value = 0.0;
	state.has_last = false;
	state.rng_state = (spec.seed != 0) ? spec.seed : default_seed(idx);
}

template <typename StatusMsg>
void StateAttackManager<StatusMsg>::publish_status(uint64_t now_us)
{
	StatusMsg status{};
	status.timestamp = now_us;
	status.timestamp_sample = _last_sample_us;

	for (uint8_t i = 0; i < _num_channels; i++) {
		const size_t idx = _first_channel + i;
		status.active[i] = StateAttackLibrary::active(_specs[idx], now_us) ? 1 : 0;
		status.type[i] = static_cast<uint8_t>(_specs[idx].type);
		status.param_a[i] = _specs[idx].a;
		status.param_b[i] = _specs[idx].b;
		status.param_c[i] = _specs[idx].c;
		status.param_d[i] = _specs[idx].d;
		status.value[i] = _state[idx].last_value;  // controller-seen (unattacked = truth)
	}

	_status_pub.publish(status);
}

template <typename StatusMsg>
uint32_t StateAttackManager<StatusMsg>::default_seed(size_t idx)
{
	return 0x9E3779B9u + static_cast<uint32_t>(idx);
}

// Explicit instantiation for the two status topics actually used by the
// controller modules (see the extern template declarations in the header).
template class StateAttackManager<state_attack_pos_status_s>;
template class StateAttackManager<state_attack_att_status_s>;

} // namespace state_attack

~~~

## 5.4 构建状态攻击编译配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/CMakeLists.txt

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/lib/state_attack/CMakeLists.txt

~~~c++
############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_library(state_attack
	StateAttackPrimitive.cpp
	StateAttackPrimitive.hpp
	StateAttackManager.cpp
	StateAttackManager.hpp
)

target_include_directories(state_attack PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

~~~

## 5.5 修改位置控制逻辑

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/CMakeLists.txt
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(GotoControl)
add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

px4_add_module(
	MODULE modules__mc_pos_control
	MAIN mc_pos_control
	COMPILE_FLAGS
	SRCS
		MulticopterPositionControl.cpp
		MulticopterPositionControl.hpp
	DEPENDS
		GotoControl
		PositionControl
		Takeoff
		controllib
		geo
		SlewRate
		state_attack
	)

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.hpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Multicopter position controller.
 */

#pragma once

#include "PositionControl/PositionControl.hpp"
#include "Takeoff/Takeoff.hpp"
#include "GotoControl/GotoControl.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/NotchFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <lib/state_attack/StateAttackManager.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/state_attack_pos_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

using namespace time_literals;

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	MulticopterPositionControl(bool vtol = false);
	~MulticopterPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	TakeoffHandling _takeoff; /**< state machine and ramp to bring the vehicle off the ground without jumps */

	state_attack::StateAttackManager<state_attack_pos_status_s> _state_attack{ORB_ID(state_attack_pos_status), 0, 7};

	orb_advert_t _mavlink_log_pub{nullptr};

	uORB::PublicationData<takeoff_status_s>              _takeoff_status_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_attitude_setpoint_s>	     _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};	/**< vehicle local position setpoint publication */

	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};	/**< vehicle local position */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_constraints_sub{ORB_ID(vehicle_constraints)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	hrt_abstime _time_stamp_last_loop{0};		/**< time stamp of last loop iteration */
	hrt_abstime _time_position_control_enabled{0};

	trajectory_setpoint_s _setpoint{PositionControl::empty_trajectory_setpoint};
	trajectory_setpoint_s _last_valid_setpoint{PositionControl::empty_trajectory_setpoint};
	vehicle_control_mode_s _vehicle_control_mode{};

	vehicle_constraints_s _vehicle_constraints {
		.timestamp = 0,
		.speed_up = NAN,
		.speed_down = NAN,
		.want_takeoff = false,
	};

	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	DEFINE_PARAMETERS(
		// Position Control
		(ParamFloat<px4::params::MPC_XY_P>)         _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>)          _param_mpc_z_p,
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>)  _param_mpc_z_v_auto_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>)  _param_mpc_z_v_auto_dn,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>)  _param_mpc_tiltmax_air,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamBool<px4::params::MPC_USE_HTE>)       _param_mpc_use_hte,
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>)  _param_mpc_acc_decouple,

		(ParamFloat<px4::params::MPC_VEL_LP>)       _param_mpc_vel_lp,
		(ParamFloat<px4::params::MPC_VEL_NF_FRQ>)   _param_mpc_vel_nf_frq,
		(ParamFloat<px4::params::MPC_VEL_NF_BW>)    _param_mpc_vel_nf_bw,
		(ParamFloat<px4::params::MPC_VELD_LP>)      _param_mpc_veld_lp,

		// Takeoff / Land
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time, /**< time to let motors spool up after arming */
		(ParamBool<px4::params::COM_THROW_EN>)      _param_com_throw_en, /**< throw launch enabled  */
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>)   _param_mpc_tko_ramp_t,   /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_TKO_SPEED>)    _param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_SPEED>)   _param_mpc_land_speed,

		(ParamFloat<px4::params::MPC_VEL_MANUAL>)   _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_VEL_MAN_BACK>) _param_mpc_vel_man_back,
		(ParamFloat<px4::params::MPC_VEL_MAN_SIDE>) _param_mpc_vel_man_side,
		(ParamFloat<px4::params::MPC_XY_CRUISE>)    _param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_LAND_ALT2>)    _param_mpc_land_alt2,    /**< downwards speed limited below this altitude */
		(ParamInt<px4::params::MPC_ALT_MODE>)       _param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>)  _param_mpc_tiltmax_lnd,  /**< maximum tilt for landing and smooth takeoff */
		(ParamFloat<px4::params::MPC_THR_MIN>)      _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>)  _param_mpc_thr_xy_marg,

		(ParamFloat<px4::params::SYS_VEHICLE_RESP>) _param_sys_vehicle_resp,
		(ParamFloat<px4::params::MPC_ACC_HOR>)      _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>)   _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>)  _param_mpc_acc_hor_max,
		(ParamFloat<px4::params::MPC_JERK_AUTO>)    _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_JERK_MAX>)     _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,

		(ParamFloat<px4::params::MPC_XY_VEL_ALL>)   _param_mpc_xy_vel_all,
		(ParamFloat<px4::params::MPC_Z_VEL_ALL>)    _param_mpc_z_vel_all,

		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc
	);

	math::WelfordMean<float> _sample_interval_s{};

	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
	AlphaFilter<float> _vel_z_lp_filter{};

	math::NotchFilter<matrix::Vector2f> _vel_xy_notch_filter{};
	math::NotchFilter<float> _vel_z_notch_filter{};

	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
	AlphaFilter<float> _vel_deriv_z_lp_filter{};

	GotoControl _goto_control; ///< class for handling smooth goto position setpoints
	PositionControl _control; ///< class for core PID position control

	hrt_abstime _last_warn{0}; /**< timer when the last warn message was sent out */

	bool _hover_thrust_initialized{false};

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;

	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off and tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;

	static constexpr float MAX_SAFE_TILT_DEG = 89.f; // Numerical issues above this value due to tanf

	SlewRate<float> _tilt_limit_slew_rate;

	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	void parameters_update(bool force);

	/**
	 * Check for validity of positon/velocity states.
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, const float dt_s);

	/**
	 * Generate setpoint to bridge no executable setpoint being available.
	 * Used to handle transitions where no proper setpoint was generated yet and when the received setpoint is invalid.
	 * This should only happen briefly when transitioning and never during mode operation or by design.
	 */
	trajectory_setpoint_s generateFailsafeSetpoint(const hrt_abstime &now, const PositionControlStates &states, bool warn);

	/**
	 * @brief adjust existing (or older) setpoint with any EKF reset deltas and update the local counters
	 *
	 * @param[in] vehicle_local_position struct containing EKF reset deltas and counters
	 * @param[out] setpoint trajectory setpoint struct to be adjusted
	 */
	void adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
					trajectory_setpoint_s &setpoint);
};

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	_sample_interval_s.update(0.01f); // 100 Hz default
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	_state_attack.init();

	return true;
}

void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();

		float sample_freq_hz = 1.f / _sample_interval_s.mean();

		// velocity notch filter
		if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
			_vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());
			_vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());

		} else {
			_vel_xy_notch_filter.disable();
			_vel_z_notch_filter.disable();
		}

		// velocity xy/z low pass filter
		if (_param_mpc_vel_lp.get() > 0.f) {
			_vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());
			_vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());

		} else {
			// disable filtering
			_vel_xy_lp_filter.setAlpha(1.f);
			_vel_z_lp_filter.setAlpha(1.f);
		}

		// velocity derivative xy/z low pass filter
		if (_param_mpc_veld_lp.get() > 0.f) {
			_vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());
			_vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());

		} else {
			// disable filtering
			_vel_deriv_xy_lp_filter.setAlpha(1.f);
			_vel_deriv_z_lp_filter.setAlpha(1.f);
		}



		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());
		_goto_control.setParamMpcAccHor(_param_mpc_acc_hor.get());
		_goto_control.setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
		_goto_control.setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
		_goto_control.setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
		_goto_control.setParamMpcXyCruise(_param_mpc_xy_cruise.get());
		_goto_control.setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
		_goto_control.setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
		_goto_control.setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
		_goto_control.setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
		_goto_control.setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const float dt_s)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();

		// vel xy notch filter, then low pass filter
		states.velocity.xy() = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));

		// vel xy derivative low pass filter
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt_s);

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_xy_lp_filter.reset({});
		_vel_xy_notch_filter.reset();
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {

		const float vel_z_prev = _vel_z_lp_filter.getState();

		// vel z notch filter, then low pass filter
		states.velocity(2) = _vel_z_lp_filter.update(_vel_z_notch_filter.apply(vehicle_local_position.vz));

		// vel z derivative low pass filter
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((_vel_z_lp_filter.getState() - vel_z_prev) / dt_s);

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_z_lp_filter.reset({});
		_vel_z_notch_filter.reset();
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);

	_state_attack.update();

	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		_sample_interval_s.update(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		_state_attack.apply_position(vehicle_local_position);

		PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};

		// If a goto setpoint is available this publishes a trajectory setpoint to go there
		// If trajectory_setpoint is published elsewhere, do not use the goto setpoint
		const bool goto_setpoint_enable = _vehicle_control_mode.flag_multicopter_position_control_enabled
						  && !_trajectory_setpoint_sub.updated();

		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample, goto_setpoint_enable)) {
			_goto_control.update(dt, states.position, states.yaw);
		}

		_trajectory_setpoint_sub.update(&_setpoint);

		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			bool skip_takeoff = _param_com_throw_en.get();
			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
			}

			// limit tilt during takeoff ramupup
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			float max_speed_xy = _param_mpc_xy_vel_max.get();

			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}

			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			_control.setInputSetpoint(_setpoint);

			// update states
			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}

			if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
			    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
				// Horizontal velocity is not controlled, reset the integrators to avoid
				// over-corrections when starting again.
				_control.resetIntegralXY();
			}

			_control.setState(states);

			const hrt_abstime now = hrt_absolute_time();

			// Run position control
			if (_control.update(dt)) {

				// Valid control update - store for fallback
				_last_valid_setpoint = _setpoint;

			} else {

				// Initial update failed - Try fallback if within timeout
				if (now < _last_valid_setpoint.timestamp + 200_ms) {
					// Use last valid setpoint
					adjustSetpointForEKFResets(vehicle_local_position, _last_valid_setpoint);
					_control.setInputSetpoint(_last_valid_setpoint);
				}

				// Still failing / not within timeout - Go to failsafe
				if (!_control.update(dt)) {

					_vehicle_constraints = {0, NAN, NAN, false, {}}; // reset constraints

					_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
					_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());

					_control.update(dt);
				}
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
			_control.resetIntegral();
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
}

trajectory_setpoint_s MulticopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// don't move along xy
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// descend with land speed since we can't stop
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// don't move along z if we can stop in all dimensions
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		// emergency descend with a bit below hover thrust
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

void MulticopterPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}

	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + Vector2f(vehicle_local_position.delta_vxy));
		_vel_xy_notch_filter.reset();
	}

	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + vehicle_local_position.delta_vz);
		_vel_z_notch_filter.reset();
	}

	// save latest reset counters
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}

~~~

## 5.6 修改姿态控制逻辑

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/CMakeLists.txt
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/mc_att_control.hpp
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/mc_att_control_main.cpp

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2015-2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(AttitudeControl)

px4_add_module(
	MODULE modules__mc_att_control
	MAIN mc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		mc_att_control_main.cpp
		mc_att_control.hpp
	DEPENDS
		AttitudeControl
		mathlib
		px4_work_queue
		StickYaw
		state_attack
	)

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/mc_att_control.hpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/state_attack_att_status.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/stick_yaw/StickYaw.hpp>
#include <lib/state_attack/StateAttackManager.hpp>

#include <AttitudeControl.hpp>

using namespace time_literals;

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	AttitudeControl _attitude_control; /**< class for attitude control calculations */
	StickYaw _stick_yaw{this};

	state_attack::StateAttackManager<state_attack_att_status_s> _state_attack{ORB_ID(state_attack_att_status), 7, 3};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< rate setpoint publication */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub;

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */

	float _hover_thrust_estimate{NAN};
	SlewRate<float> _hover_thrust_slew_rate{.5f};

	float _yaw_setpoint_stabilized{0.f};
	bool _heading_good_for_control{true}; // initialized true to have heading lock when local position never published
	float _unaided_heading{NAN}; // initialized NAN to not distract heading lock when local position never published
	float _man_tilt_max{0.f};			/**< maximum tilt allowed for manual flight [rad] */

	SlewRate<float> _manual_throttle_minimum{0.f}; ///< 0 when landed and ramped to MPC_MANTHR_MIN in air
	SlewRate<float> _manual_throttle_maximum{0.f}; ///< 0 when disarmed ramped to 1 when spooled up
	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	bool _spooled_up{false}; ///< used to make sure the vehicle cannot take off during the spoolup time
	bool _landed{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};

	uint8_t _quat_reset_counter{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MAN_DEADZONE>) _param_man_deadzone,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	)
};

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/mc_att_control/mc_att_control_main.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();
	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	_manual_throttle_maximum.setSlewRate(0.5f);
	// Rate of change 5% per second -> 6 seconds to ramp 30% if hover thrust parameter is off
	_hover_thrust_slew_rate.setSlewRate(0.05f);
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_state_attack.init();

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// Update from hover thrust parameter if there's no valid estimate in use
	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	// throttle_stick_input is in range [-1, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // rescale to hover thrust param at 0 stick input
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 or other: rescale to HTE value
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Avoid accumulating absolute yaw error with arming stick gesture
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture || !_heading_good_for_control) {
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, .6f, _param_man_deadzone.get());
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _yaw_setpoint_stabilized, yaw_stick_input, yaw, dt,
				       _unaided_heading);

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(roll*roll + pitch*pitch)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// we want to fly towards the direction of (roll, pitch)
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// Make sure there's a valid attitude quaternion with no yaw error when yaw is unlocked (NAN)
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		// Modify the setpoints for roll and pitch such that they reflect the user's intention even
		// if a large yaw error(yaw_sp - yaw) is present. In the presence of a yaw error constructing
		// an attitude setpoint from the yaw setpoint will lead to unexpected attitude behaviour from
		// the user's view as the tilt will not be aligned with the heading of the vehicle.

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// Align the desired tilt with the yaw setpoint
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// Update hover thrust for stick scaling
	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hover_thrust_estimate;

		if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
			if (hover_thrust_estimate.valid) {
				_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

			} else {
				// Possibly bad estimate before it got invalid, slew back to parameter
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	// run controller on attitude updates
	_state_attack.update();

	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		_state_attack.apply_attitude(v_att);

		const Quatf q{v_att.q};

		/* check for updates in other topics */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
				_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// during transitions VTOL module generates attitude setpoints
		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
					  && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt);

			} else {
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_yaw_setpoint_stabilized = NAN;
				_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
			}

			// Check for new attitude setpoint
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Check for a heading reset
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				const float delta_psi = Eulerf(delta_q_reset).psi();

				// Only offset the yaw setpoint when the heading is locked
				if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
					_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
				}

				_stick_yaw.ekfResetHandler(delta_psi);

				if (v_att.timestamp > _last_attitude_setpoint) {
					// adapt existing attitude setpoint unless it was generated after the current attitude estimate
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				_quat_reset_counter = v_att.quat_reset_counter;
			}

			Vector3f rates_sp = _attitude_control.update(q);

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// publish rate setpoint
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_yaw_setpoint_stabilized = NAN;
			_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
		}

		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}

~~~

## 5.7 无人机端 msg 构建

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackCommand.msg
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackPosStatus.msg
文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackAttStatus.msg

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackCommand.msg

~~~c++
# Post-EKF state attack command (ROS2 -> PX4).
# Instructs the StateAttackManager to mount / update / clear an attack on one state channel.
#
# One message = one channel. Sending a message with type=NONE clears the channel.
# Channels 0..6 = nav state (mc_pos_control), 7..9 = attitude (mc_att_control).
# The next controller loop on that channel reflects the change (~100-250 Hz).

uint64 timestamp		# time since system start (microseconds)

uint8 channel			# StateChannel enum value (0..9): NAV_POS_X/Y/Z, NAV_VEL_X/Y/Z, NAV_HEADING, ATT_ROLL/PITCH/YAW
uint8 type			# PrimitiveType enum value (0=NONE -> clear the attack)

float64[4] param		# primitive parameters (a, b, c, d); meaning depends on 'type'

uint64 t0_us			# start delay relative to command arrival (microseconds); 0 = immediately
uint64 t1_us			# duration relative to start (microseconds); 0 = until cleared
uint32 seed			# RNG seed for stochastic primitives; 0 = random

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackPosStatus.msg

~~~c++
# Post-EKF nav state attack status (PX4 -> ROS2 / logger).
# Per-channel snapshot of what the position controller actually saw (attacked-or-truth),
# published by mc_pos_control every loop.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# vehicle_local_position.timestamp_sample this value[] was derived from

uint8[7] active			# per channel: 1 = attack currently active, 0 = inactive
uint8[7] type			# per channel: current PrimitiveType enum value (meaningful when active)

float64[7] param_a		# per channel: primitive parameter a
float64[7] param_b		# per channel: primitive parameter b
float64[7] param_c		# per channel: primitive parameter c
float64[7] param_d		# per channel: primitive parameter d

float64[7] value		# per channel: controller-seen value (unattacked = truth); units per channel (see state_attack_design.md §4.3)

~~~

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/StateAttackAttStatus.msg

~~~c++
# Post-EKF attitude attack status (PX4 -> ROS2 / logger).
# Per-channel snapshot of what the attitude controller actually saw (attacked-or-truth Euler angles),
# published by mc_att_control every loop.

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample		# vehicle_attitude.timestamp_sample this value[] was derived from

uint8[3] active			# per channel: 1 = attack currently active, 0 = inactive
uint8[3] type			# per channel: current PrimitiveType enum value (meaningful when active)

float64[3] param_a		# per channel: primitive parameter a
float64[3] param_b		# per channel: primitive parameter b
float64[3] param_c		# per channel: primitive parameter c
float64[3] param_d		# per channel: primitive parameter d

float64[3] value		# per channel: controller-seen Euler angle (roll/pitch/yaw, rad); unattacked = truth

~~~

## 5.8 服务器端 msg 构建

文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackCommand.msg
文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackPosStatus.msg
文件位置：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackAttStatus.msg

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackCommand.msg

~~~c++
# Post-EKF state attack command (ROS2 -> PX4).
# Instructs the StateAttackManager to mount / update / clear an attack on one state channel.
# Channels 0..6 = nav state, 7..9 = attitude. Sending type=NONE clears the channel.

uint64 timestamp # [us] Time since system start

uint8 channel # StateChannel enum value (0..9): NAV_POS_X/Y/Z, NAV_VEL_X/Y/Z, NAV_HEADING, ATT_ROLL/PITCH/YAW
uint8 type # PrimitiveType enum value (0=NONE -> clear the attack)

float64[4] param # primitive parameters (a, b, c, d); meaning depends on 'type'

uint64 t0_us # start delay relative to command arrival (us); 0 = immediately
uint64 t1_us # duration relative to start (us); 0 = until cleared
uint32 seed # RNG seed for stochastic primitives; 0 = random

~~~

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackPosStatus.msg

~~~c++
# Post-EKF nav state attack status (PX4 -> ROS2 / logger).
# Per-channel snapshot of what the position controller actually saw (attacked-or-truth).

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] vehicle_local_position.timestamp_sample this value[] was derived from

uint8[7] active # per channel: 1 = attack currently active, 0 = inactive
uint8[7] type # per channel: current PrimitiveType enum value (meaningful when active)

float64[7] param_a # per channel: primitive parameter a
float64[7] param_b # per channel: primitive parameter b
float64[7] param_c # per channel: primitive parameter c
float64[7] param_d # per channel: primitive parameter d

float64[7] value # per channel: controller-seen value (unattacked = truth); units per channel

~~~

文件：/home/liu/Desktop/ROS2/src/px4_msgs/msg/StateAttackAttStatus.msg

~~~c++
# Post-EKF attitude attack status (PX4 -> ROS2 / logger).
# Per-channel snapshot of what the attitude controller actually saw (attacked-or-truth Euler angles).

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] vehicle_attitude.timestamp_sample this value[] was derived from

uint8[3] active # per channel: 1 = attack currently active, 0 = inactive
uint8[3] type # per channel: current PrimitiveType enum value (meaningful when active)

float64[3] param_a # per channel: primitive parameter a
float64[3] param_b # per channel: primitive parameter b
float64[3] param_c # per channel: primitive parameter c
float64[3] param_d # per channel: primitive parameter d

float64[3] value # per channel: controller-seen Euler angle (roll/pitch/yaw, rad); unattacked = truth

~~~

## 5.9 修改 msg 构建文件

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/msg/CMakeLists.txt

~~~
############################################################################
#
#   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

include(px4_list_make_absolute)

set(msg_files
	ActionRequest.msg
	ActuatorArmed.msg
	ActuatorControlsStatus.msg
	ActuatorOutputs.msg
	ActuatorServosTrim.msg
	ActuatorTest.msg
	AdcReport.msg
	Airspeed.msg
	AirspeedWind.msg
	AttackCommand.msg
	AttackStatus.msg
	AutotuneAttitudeControlStatus.msg
	BatteryInfo.msg
	ButtonEvent.msg
	CameraCapture.msg
	CameraStatus.msg
	CameraTrigger.msg
	CanInterfaceStatus.msg
	CellularStatus.msg
	CollisionConstraints.msg
	ControlAllocatorStatus.msg
	Cpuload.msg
	DatamanRequest.msg
	DatamanResponse.msg
	DebugArray.msg
	DebugKeyValue.msg
	DebugValue.msg
	DebugVect.msg
	DifferentialPressure.msg
	DistanceSensor.msg
	DistanceSensorModeChangeRequest.msg
	DronecanNodeStatus.msg
	Ekf2Timestamps.msg
	EscReport.msg
	EscStatus.msg
	EstimatorAidSource1d.msg
	EstimatorAidSource2d.msg
	EstimatorAidSource3d.msg
	EstimatorBias.msg
	EstimatorBias3d.msg
	EstimatorEventFlags.msg
	EstimatorGpsStatus.msg
	EstimatorInnovations.msg
	EstimatorSelectorStatus.msg
	EstimatorSensorBias.msg
	EstimatorStates.msg
	EstimatorStatus.msg
	EstimatorStatusFlags.msg
	versioned/Event.msg
	FigureEightStatus.msg
	FailsafeFlags.msg
	FailureDetectorStatus.msg
	FlightPhaseEstimation.msg
	FollowTarget.msg
	FollowTargetEstimator.msg
	FollowTargetStatus.msg
	FuelTankStatus.msg
	FixedWingLateralGuidanceStatus.msg
	FixedWingLateralStatus.msg
	FixedWingRunwayControl.msg
	GeneratorStatus.msg
	GeofenceResult.msg
	GeofenceStatus.msg
	GimbalControls.msg
	GimbalDeviceAttitudeStatus.msg
	GimbalDeviceInformation.msg
	GimbalDeviceSetAttitude.msg
	GimbalManagerInformation.msg
	GimbalManagerSetAttitude.msg
	GimbalManagerSetManualControl.msg
	GimbalManagerStatus.msg
	GpioConfig.msg
	GpioIn.msg
	GpioOut.msg
	GpioRequest.msg
	GpsDump.msg
	GpsInjectData.msg
	Gripper.msg
	HealthReport.msg
	HeaterStatus.msg
	HoverThrustEstimate.msg
	InputRc.msg
	InternalCombustionEngineControl.msg
	InternalCombustionEngineStatus.msg
	IridiumsbdStatus.msg
	IrlockReport.msg
	LandingGear.msg
	LandingGearWheel.msg
	LandingTargetInnovations.msg
	LandingTargetPose.msg
	LaunchDetectionStatus.msg
	LedControl.msg
	LoggerStatus.msg
	LogMessage.msg
	MagnetometerBiasEstimate.msg
	MagWorkerData.msg
	ManualControlSwitches.msg
	MavlinkLog.msg
	MavlinkTunnel.msg
	MessageFormatRequest.msg
	MessageFormatResponse.msg
	Mission.msg
	MissionResult.msg
	MountOrientation.msg
	NavigatorMissionItem.msg
	NavigatorStatus.msg
	NeuralControl.msg
	NormalizedUnsignedSetpoint.msg
	ObstacleDistance.msg
	OffboardControlMode.msg
	OnboardComputerStatus.msg
	OpenDroneIdArmStatus.msg
	OpenDroneIdOperatorId.msg
	OpenDroneIdSelfId.msg
	OpenDroneIdSystem.msg
	OrbitStatus.msg
	OrbTest.msg
	OrbTestLarge.msg
	OrbTestMedium.msg
	ParameterResetRequest.msg
	ParameterSetUsedRequest.msg
	ParameterSetValueRequest.msg
	ParameterSetValueResponse.msg
	ParameterUpdate.msg
	Ping.msg
	PositionControllerLandingStatus.msg
	PositionControllerStatus.msg
	PositionSetpoint.msg
	PositionSetpointTriplet.msg
	PowerButtonState.msg
	PowerMonitor.msg
	PpsCapture.msg
	PurePursuitStatus.msg
	PwmInput.msg
	Px4ioStatus.msg
	QshellReq.msg
	QshellRetval.msg
	RadioStatus.msg
	RateCtrlStatus.msg
	RcChannels.msg
	RcParameterMap.msg
	RoverAttitudeSetpoint.msg
	RoverAttitudeStatus.msg
	RoverPositionSetpoint.msg
	RoverRateSetpoint.msg
	RoverRateStatus.msg
	RoverSpeedSetpoint.msg
	RoverSpeedStatus.msg
	RoverSteeringSetpoint.msg
	RoverThrottleSetpoint.msg
	Rpm.msg
	RtlStatus.msg
	RtlTimeEstimate.msg
	SatelliteInfo.msg
	SensorAccel.msg
	SensorAccelFifo.msg
	SensorBaro.msg
	SensorCombined.msg
	SensorCorrection.msg
	SensorGnssRelative.msg
	SensorGnssStatus.msg
	SensorGps.msg
	SensorGyro.msg
	SensorGyroFft.msg
	SensorGyroFifo.msg
	SensorHygrometer.msg
	SensorMag.msg
	SensorOpticalFlow.msg
	SensorPreflightMag.msg
	SensorSelection.msg
	SensorsStatus.msg
	SensorsStatusImu.msg
	SensorUwb.msg
	SensorAirflow.msg
	StateAttackAttStatus.msg
	StateAttackCommand.msg
	StateAttackPosStatus.msg
	SystemPower.msg
	TakeoffStatus.msg
	TaskStackInfo.msg
	TecsStatus.msg
	TelemetryStatus.msg
	TiltrotorExtraControls.msg
	TimesyncStatus.msg
	TrajectorySetpoint6dof.msg
	TransponderReport.msg
	TuneControl.msg
	UavcanParameterRequest.msg
	UavcanParameterValue.msg
	UlogStream.msg
	UlogStreamAck.msg
	VehicleAcceleration.msg
	VehicleAirData.msg
	VehicleAngularAccelerationSetpoint.msg
	VehicleConstraints.msg
	VehicleImu.msg
	VehicleImuStatus.msg
	VehicleLocalPositionSetpoint.msg
	VehicleMagnetometer.msg
	VehicleOpticalFlow.msg
	VehicleOpticalFlowVel.msg
	VehicleRoi.msg
	VehicleThrustSetpoint.msg
	VehicleTorqueSetpoint.msg
	VelocityLimits.msg
	WheelEncoders.msg
	WindCommand.msg
	YawEstimatorStatus.msg
	versioned/ActuatorMotors.msg
	versioned/ActuatorServos.msg
	versioned/AirspeedValidated.msg
	versioned/ArmingCheckReply.msg
	versioned/ArmingCheckRequest.msg
	versioned/BatteryStatus.msg
	versioned/ConfigOverrides.msg
	versioned/FixedWingLateralSetpoint.msg
	versioned/FixedWingLongitudinalSetpoint.msg
	versioned/GotoSetpoint.msg
	versioned/HomePosition.msg
	versioned/LateralControlConfiguration.msg
	versioned/LongitudinalControlConfiguration.msg
	versioned/ManualControlSetpoint.msg
	versioned/ModeCompleted.msg
	versioned/RegisterExtComponentReply.msg
	versioned/RegisterExtComponentRequest.msg
	versioned/TrajectorySetpoint.msg
	versioned/UnregisterExtComponent.msg
	versioned/VehicleAngularVelocity.msg
	versioned/VehicleAttitude.msg
	versioned/VehicleAttitudeSetpoint.msg
	versioned/VehicleCommandAck.msg
	versioned/VehicleCommand.msg
	versioned/VehicleControlMode.msg
	versioned/VehicleGlobalPosition.msg
	versioned/VehicleLandDetected.msg
	versioned/VehicleLocalPosition.msg
	versioned/VehicleOdometry.msg
	versioned/VehicleRatesSetpoint.msg
	versioned/VehicleStatus.msg
	versioned/VtolVehicleStatus.msg
	versioned/Wind.msg
)
list(SORT msg_files)

px4_list_make_absolute(msg_files ${CMAKE_CURRENT_SOURCE_DIR} ${msg_files})

if(NOT EXTERNAL_MODULES_LOCATION STREQUAL "")
	# Check that the msg directory and the CMakeLists.txt file exists
	if(EXISTS ${EXTERNAL_MODULES_LOCATION}/msg/CMakeLists.txt)
		add_subdirectory(${EXTERNAL_MODULES_LOCATION}/msg external_msg)

		# Add each of the external message files to the global msg_files list
		foreach(external_msg_file ${config_msg_list_external})
			list(APPEND msg_files ${EXTERNAL_MODULES_LOCATION}/msg/${external_msg_file})
		endforeach()
	endif()
endif()

# headers
set(msg_out_path ${PX4_BINARY_DIR}/uORB/topics)
set(ucdr_out_path ${PX4_BINARY_DIR}/uORB/ucdr)
set(msg_source_out_path ${CMAKE_CURRENT_BINARY_DIR}/topics_sources)

set(uorb_headers)
set(uorb_sources)
set(uorb_ucdr_headers)
set(uorb_json_files)
foreach(msg_file ${msg_files})
	get_filename_component(msg ${msg_file} NAME_WE)

	# Pascal case to snake case (MsgFile -> msg_file)
	string(REGEX REPLACE "(.)([A-Z][a-z]+)" "\\1_\\2" msg "${msg}")
	string(REGEX REPLACE "([a-z0-9])([A-Z])" "\\1_\\2" msg "${msg}")
	string(TOLOWER "${msg}" msg)

	list(APPEND uorb_headers ${msg_out_path}/${msg}.h)
	list(APPEND uorb_sources ${msg_source_out_path}/${msg}.cpp)
	list(APPEND uorb_ucdr_headers ${ucdr_out_path}/${msg}.h)
	list(APPEND uorb_json_files ${msg_source_out_path}/${msg}.json)
endforeach()

# set parent scope msg_files for ROS
set(msg_files ${msg_files} PARENT_SCOPE)

# Generate uORB headers
add_custom_command(
	OUTPUT
		${uorb_headers}
		${msg_out_path}/uORBTopics.hpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.hpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_headers DEPENDS ${uorb_headers})

add_custom_command(
	OUTPUT
		${uorb_json_files}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--json
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.json.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB json files"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(uorb_json_files DEPENDS ${uorb_json_files})

set(uorb_message_fields_cpp_file ${msg_source_out_path}/uORBMessageFieldsGenerated.cpp)
set(uorb_message_fields_header_file ${msg_out_path}/uORBMessageFieldsGenerated.hpp)
add_custom_command(
	OUTPUT
		${uorb_message_fields_cpp_file}
		${uorb_message_fields_header_file}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
		-f ${uorb_json_files}
		--source-output-file ${uorb_message_fields_cpp_file}
		--header-output-file ${uorb_message_fields_header_file}
	DEPENDS
		${uorb_json_files}
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_compressed_fields.py
	COMMENT "Generating uORB compressed fields"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)

# Generate microcdr headers
add_custom_command(
	OUTPUT ${uorb_ucdr_headers}
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--headers
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${ucdr_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/ucdr/msg.h.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic ucdr headers"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)
add_custom_target(uorb_ucdr_headers DEPENDS ${uorb_ucdr_headers})

# Generate uORB sources
add_custom_command(
	OUTPUT
		${uorb_sources}
		${msg_source_out_path}/uORBTopics.cpp
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		--sources
		-f ${msg_files}
		-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
		-o ${msg_source_out_path}
		-e ${PX4_SOURCE_DIR}/Tools/msg/templates/uorb
	DEPENDS
		${msg_files}
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/msg.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/templates/uorb/uORBTopics.cpp.em
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
		${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
	COMMENT "Generating uORB topic sources"
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
	)

add_library(uorb_msgs ${uorb_headers} ${msg_out_path}/uORBTopics.hpp ${uorb_sources} ${msg_source_out_path}/uORBTopics.cpp ${uorb_message_fields_cpp_file})
target_link_libraries(uorb_msgs PRIVATE m)
add_dependencies(uorb_msgs prebuild_targets uorb_headers)

if(CONFIG_LIB_CDRSTREAM)
	set(uorb_cdr_idl)
	set(uorb_cdr_msg)
	set(uorb_cdr_hash)
	set(uorb_cdr_idl_uorb)
	set(idl_include_path ${PX4_BINARY_DIR}/uORB/idl)
	set(idl_out_path ${idl_include_path}/px4/msg)
	set(idl_rihs01_out_path ${idl_include_path}/px4)
	set(idl_uorb_path ${PX4_BINARY_DIR}/msg/px4/msg)

	# Make sure that CycloneDDS has been checkout out
	execute_process(COMMAND git submodule sync src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )
	execute_process(COMMAND git submodule update --init --force src/lib/cdrstream/cyclonedds
			WORKING_DIRECTORY ${PX4_SOURCE_DIR} )

	# CycloneDDS-tools doesn't ship with the cdrstream-desc feature thus we've to compile idlc from source
	MESSAGE(STATUS "Configuring idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds
			-DCMAKE_C_COMPILER=/usr/bin/gcc
			-DBUILD_EXAMPLES=OFF
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	MESSAGE(STATUS "Building idlc :" ${CMAKE_CURRENT_BINARY_DIR}/idlc)
	execute_process(COMMAND ${CMAKE_COMMAND} --build . --target idlc
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idlc
			RESULT_VARIABLE CMD_ERROR
			OUTPUT_FILE CMD_OUTPUT )
	list(APPEND CMAKE_PROGRAM_PATH "${CMAKE_CURRENT_BINARY_DIR}/idlc/bin")

	# Copy .msg files
	foreach(msg_file ${msg_files})
		get_filename_component(msg ${msg_file} NAME_WE)
		configure_file(${msg_file} ${idl_out_path}/${msg}.msg COPYONLY)
		list(APPEND uorb_cdr_idl ${idl_out_path}/${msg}.idl)
		list(APPEND uorb_cdr_msg ${idl_out_path}/${msg}.msg)
		list(APPEND uorb_cdr_hash ${idl_out_path}/${msg}.json)
		list(APPEND uorb_cdr_idl_uorb ${idl_uorb_path}/${msg}.h)
	endforeach()

	# Generate IDL from .msg using rosidl_adapter
	# Note this a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_idl}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/msg2idl.py
			${uorb_cdr_msg}
		DEPENDS
			${uorb_cdr_msg}
			git_cyclonedds
		COMMENT "Generating IDL from uORB topic headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	file(CREATE_LINK ${idl_rihs01_out_path} ${idl_include_path}/px4_msgs SYMBOLIC)

	# Generate IDL from .msg using rosidl_adapter
	# Note this is a submodule inside PX4 hence no ROS2 installation required
	add_custom_command(
		OUTPUT ${uorb_cdr_hash}
		COMMAND ${CMAKE_COMMAND}
		        -E env "PYTHONPATH=${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_adapter:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_cli:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_parser:${PX4_SOURCE_DIR}/src/lib/cdrstream/rosidl/rosidl_generator_type_description"
			${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/src/lib/cdrstream/idl2rihs01.py
			--output-dir ${idl_rihs01_out_path}
			${uorb_cdr_idl}
		DEPENDS
			${uorb_cdr_idl}
			git_cyclonedds
		COMMENT "Generating RIHS01 hashes from IDL"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)

	# Generate C definitions from IDL
	set(CYCLONEDDS_DIR ${PX4_SOURCE_DIR}/src/lib/cdrstream/cyclonedds)
	include("${CYCLONEDDS_DIR}/cmake/Modules/Generate.cmake")
	idlc_generate(TARGET uorb_cdrstream
                  FEATURES "cdrstream-desc"
                  FILES ${uorb_cdr_idl}
                  INCLUDES ${idl_include_path}
                  BASE_DIR ${idl_include_path}
                  WARNINGS no-implicit-extensibility)
	target_link_libraries(uorb_cdrstream INTERFACE cdr)

	# Generate and overwrite IDL header with custom headers for uORB operatability
	# We typedef the IDL struct the uORB struct so that the IDL offset calculate
	# the offset of internal uORB struct for serialization/deserialization

	# In the future we might want to turn this around let the IDL struct be the leading ABI
	# However we need to remove the padding for logging and remove the re-ordering of fields

	add_custom_target(
		uorb_idl_header
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			--uorb-idl-header
			-f ${msg_files}
			-i ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/versioned
			-o ${idl_uorb_path}
			-e ${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream
		DEPENDS
			uorb_cdrstream
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/msg/templates/cdrstream/uorb_idl_header.h.em
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_files.py
			${PX4_SOURCE_DIR}/Tools/msg/px_generate_uorb_topic_helper.py
		COMMENT "Generating uORB compatible IDL headers"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
	)
	add_dependencies(uorb_msgs uorb_idl_header)

	# Compile all CDR compatible message defnitions
	target_link_libraries(uorb_msgs PRIVATE uorb_cdrstream )
endif()

if(CONFIG_MODULES_ZENOH)
	# Update kconfig file for topics
	execute_process(COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-config
			-f ${msg_files}
			-o ${PX4_SOURCE_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
		)
	add_custom_command(
		OUTPUT
			${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
			--zenoh-pub-sub
			-f ${msg_files}
			-o ${PX4_BINARY_DIR}/src/modules/zenoh/
			-e ${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh
			--rihs ${idl_rihs01_out_path}
		DEPENDS
			${msg_files}
			${uorb_cdr_hash}
			${PX4_SOURCE_DIR}/Tools/zenoh/templates/zenoh/uorb_pubsub_factory.hpp.em
			${PX4_SOURCE_DIR}/Tools/zenoh/px_generate_zenoh_topic_files.py
		COMMENT "Generating Zenoh Topic Code"
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		VERBATIM
		)
		add_library(zenoh_topics ${PX4_BINARY_DIR}/src/modules/zenoh/uorb_pubsub_factory.hpp)
		set_target_properties(zenoh_topics PROPERTIES LINKER_LANGUAGE CXX)
endif()

~~~

## 5.10 修改无人机端 Micro-XRCE-DDS-Client 配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

~~~yaml
#####
#
# This file maps all the topics that are to be used on the uXRCE-DDS client.
#
#####
publications:

  - topic: /fmu/out/attack_status
    type: px4_msgs::msg::AttackStatus
    rate_limit: 10.

  - topic: /fmu/out/state_attack_pos_status
    type: px4_msgs::msg::StateAttackPosStatus
    rate_limit: 100.

  - topic: /fmu/out/state_attack_att_status
    type: px4_msgs::msg::StateAttackAttStatus
    rate_limit: 100.

  - topic: /fmu/out/register_ext_component_reply
    type: px4_msgs::msg::RegisterExtComponentReply

  - topic: /fmu/out/arming_check_request
    type: px4_msgs::msg::ArmingCheckRequest
    rate_limit: 5.

  - topic: /fmu/out/mode_completed
    type: px4_msgs::msg::ModeCompleted
    rate_limit: 50.

  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
    rate_limit: 1.

  - topic: /fmu/out/collision_constraints
    type: px4_msgs::msg::CollisionConstraints
    rate_limit: 50.

  - topic: /fmu/out/estimator_status_flags
    type: px4_msgs::msg::EstimatorStatusFlags
    rate_limit: 5.

  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags
    rate_limit: 5.

  - topic: /fmu/out/manual_control_setpoint
    type: px4_msgs::msg::ManualControlSetpoint
    rate_limit: 25.

  - topic: /fmu/out/message_format_response
    type: px4_msgs::msg::MessageFormatResponse

  - topic: /fmu/out/position_setpoint_triplet
    type: px4_msgs::msg::PositionSetpointTriplet
    rate_limit: 5.

  - topic: /fmu/out/sensor_combined
    type: px4_msgs::msg::SensorCombined

  - topic: /fmu/out/timesync_status
    type: px4_msgs::msg::TimesyncStatus
    rate_limit: 10.

  - topic: /fmu/out/transponder_report
    type: px4_msgs::msg::TransponderReport
 
  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity
    rate_limit: 50.

  - topic: /fmu/out/vehicle_land_detected
    type: px4_msgs::msg::VehicleLandDetected
    rate_limit: 5.

  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude

  - topic: /fmu/out/vehicle_control_mode
    type: px4_msgs::msg::VehicleControlMode
    rate_limit: 50.

  - topic: /fmu/out/vehicle_command_ack
    type: px4_msgs::msg::VehicleCommandAck

  - topic: /fmu/out/vehicle_global_position
    type: px4_msgs::msg::VehicleGlobalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_gps_position
    type: px4_msgs::msg::SensorGps
    rate_limit: 50.

  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus
    rate_limit: 5.

  - topic: /fmu/out/airspeed_validated
    type: px4_msgs::msg::AirspeedValidated
    rate_limit: 50.

  - topic: /fmu/out/vtol_vehicle_status
    type: px4_msgs::msg::VtolVehicleStatus

  - topic: /fmu/out/home_position
    type: px4_msgs::msg::HomePosition
    rate_limit: 5.

  - topic: /fmu/out/wind
    type: px4_msgs::msg::Wind
    rate_limit: 1.

  - topic: /fmu/out/gimbal_device_attitude_status
    type: px4_msgs::msg::GimbalDeviceAttitudeStatus
    rate_limit: 20.
  
  - topic: /fmu/out/esc_status
    type: px4_msgs::msg::EscStatus

# Create uORB::Publication
subscriptions:
  - topic: /fmu/in/attack_command
    type: px4_msgs::msg::AttackCommand

  - topic: /fmu/in/state_attack_command
    type: px4_msgs::msg::StateAttackCommand

  - topic: /fmu/in/wind_command
    type: px4_msgs::msg::WindCommand

  - topic: /fmu/in/register_ext_component_request
    type: px4_msgs::msg::RegisterExtComponentRequest

  - topic: /fmu/in/unregister_ext_component
    type: px4_msgs::msg::UnregisterExtComponent

  - topic: /fmu/in/config_overrides_request
    type: px4_msgs::msg::ConfigOverrides

  - topic: /fmu/in/arming_check_reply
    type: px4_msgs::msg::ArmingCheckReply

  - topic: /fmu/in/message_format_request
    type: px4_msgs::msg::MessageFormatRequest

  - topic: /fmu/in/mode_completed
    type: px4_msgs::msg::ModeCompleted

  - topic: /fmu/in/config_control_setpoints
    type: px4_msgs::msg::VehicleControlMode

  - topic: /fmu/in/distance_sensor
    type: px4_msgs::msg::DistanceSensor

  - topic: /fmu/in/manual_control_input
    type: px4_msgs::msg::ManualControlSetpoint

  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode

  - topic: /fmu/in/onboard_computer_status
    type: px4_msgs::msg::OnboardComputerStatus

  - topic: /fmu/in/obstacle_distance
    type: px4_msgs::msg::ObstacleDistance

  - topic: /fmu/in/sensor_optical_flow
    type: px4_msgs::msg::SensorOpticalFlow

  - topic: /fmu/in/goto_setpoint
    type: px4_msgs::msg::GotoSetpoint

  - topic: /fmu/in/telemetry_status
    type: px4_msgs::msg::TelemetryStatus

  - topic: /fmu/in/trajectory_setpoint
    type: px4_msgs::msg::TrajectorySetpoint

  - topic: /fmu/in/vehicle_attitude_setpoint
    type: px4_msgs::msg::VehicleAttitudeSetpoint

  - topic: /fmu/in/vehicle_mocap_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry

  - topic: /fmu/in/vehicle_command
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_command_mode_executor
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_thrust_setpoint
    type: px4_msgs::msg::VehicleThrustSetpoint

  - topic: /fmu/in/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint

  - topic: /fmu/in/actuator_motors
    type: px4_msgs::msg::ActuatorMotors

  - topic: /fmu/in/actuator_servos
    type: px4_msgs::msg::ActuatorServos

  - topic: /fmu/in/aux_global_position
    type: px4_msgs::msg::VehicleGlobalPosition

  - topic: /fmu/in/fixed_wing_longitudinal_setpoint
    type: px4_msgs::msg::FixedWingLongitudinalSetpoint

  - topic: /fmu/in/fixed_wing_lateral_setpoint
    type: px4_msgs::msg::FixedWingLateralSetpoint

  - topic: /fmu/in/longitudinal_control_configuration
    type: px4_msgs::msg::LongitudinalControlConfiguration

  - topic: /fmu/in/lateral_control_configuration
    type: px4_msgs::msg::LateralControlConfiguration

  - topic: /fmu/in/rover_position_setpoint
    type: px4_msgs::msg::RoverPositionSetpoint

  - topic: /fmu/in/rover_speed_setpoint
    type: px4_msgs::msg::RoverSpeedSetpoint

  - topic: /fmu/in/rover_attitude_setpoint
    type: px4_msgs::msg::RoverAttitudeSetpoint

  - topic: /fmu/in/rover_rate_setpoint
    type: px4_msgs::msg::RoverRateSetpoint

  - topic: /fmu/in/rover_throttle_setpoint
    type: px4_msgs::msg::RoverThrottleSetpoint

  - topic: /fmu/in/rover_steering_setpoint
    type: px4_msgs::msg::RoverSteeringSetpoint

  - topic: /fmu/in/landing_gear
    type: px4_msgs::msg::LandingGear

# Create uORB::PublicationMulti
subscriptions_multi:

~~~

## 5.11 修改无人机端 LOGGER 配置

文件位置：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp
注：文件（/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp）为前述修改过的文件。

文件：/home/liu/Desktop/ROS2/PX4-Autopilot/src/modules/logger/logged_topics.cpp

~~~c++
/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "logged_topics.h"
#include "messages.h"

#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/uORBTopics.hpp>

#include <string.h>

using namespace px4::logger;

void LoggedTopics::add_default_topics()
{
	add_topic("action_request");
	add_topic("actuator_armed");
	add_optional_topic("actuator_controls_status_0", 300);
	add_topic("airspeed", 1000);
	add_optional_topic("airspeed_validated", 200);
	add_topic("attack_command");
	add_topic("attack_status", 100);
	add_optional_topic("autotune_attitude_control_status", 100);
	add_topic_multi("battery_info", 5000, 3);
	add_optional_topic("camera_capture");
	add_optional_topic("camera_trigger");
	add_topic("cellular_status", 200);
	add_topic("commander_state");
	add_topic("config_overrides");
	add_topic("cpuload");
	add_topic("distance_sensor_mode_change_request");
	add_topic_multi("dronecan_node_status", 250);
	add_optional_topic("external_ins_attitude");
	add_optional_topic("external_ins_global_position");
	add_optional_topic("external_ins_local_position");
	// add_optional_topic("esc_status", 250);
	add_topic("esc_status");
	add_topic("failure_detector_status", 100);
	add_topic("failsafe_flags");
	add_optional_topic("follow_target", 500);
	add_optional_topic("follow_target_estimator", 200);
	add_optional_topic("follow_target_status", 400);
	add_optional_topic("flaps_setpoint", 1000);
	add_optional_topic("flight_phase_estimation", 1000);
	add_optional_topic("fuel_tank_status", 10);
	add_topic("gimbal_manager_set_attitude", 500);
	add_optional_topic("generator_status");
	add_optional_topic("gps_dump");
	add_optional_topic("gimbal_controls", 200);
	add_optional_topic("gripper");
	add_optional_topic("heater_status");
	add_topic("home_position");
	add_topic("hover_thrust_estimate", 100);
	add_topic("input_rc", 500);
	add_optional_topic("internal_combustion_engine_control", 10);
	add_optional_topic("internal_combustion_engine_status", 10);
	add_optional_topic("iridiumsbd_status", 1000);
	add_optional_topic("irlock_report", 1000);
	add_optional_topic("landing_gear", 200);
	add_optional_topic("landing_gear_wheel", 100);
	add_optional_topic("landing_target_pose", 1000);
	add_optional_topic("launch_detection_status", 200);
	add_optional_topic("magnetometer_bias_estimate", 200);
	add_topic("manual_control_setpoint", 200);
	add_topic("manual_control_switches");
	add_topic("mission_result");
	add_topic("navigator_mission_item");
	add_topic("navigator_status");
	add_topic("offboard_control_mode", 100);
	add_topic("onboard_computer_status", 10);
	add_topic("parameter_update");
	add_topic("position_controller_status", 500);
	add_topic("position_controller_landing_status", 100);
	add_optional_topic("pure_pursuit_status", 100);
	add_topic("goto_setpoint", 200);
	add_topic("position_setpoint_triplet", 200);
	add_optional_topic("px4io_status");
	add_topic("radio_status");
	add_optional_topic("rover_attitude_setpoint", 100);
	add_optional_topic("rover_attitude_status", 100);
	add_optional_topic("rover_position_setpoint", 100);
	add_optional_topic("rover_rate_setpoint", 100);
	add_optional_topic("rover_rate_status", 100);
	add_optional_topic("rover_speed_setpoint", 100);
	add_optional_topic("rover_speed_status", 100);
	add_optional_topic("rover_steering_setpoint", 100);
	add_optional_topic("rover_throttle_setpoint", 100);
	add_topic("rtl_time_estimate", 1000);
	add_topic("rtl_status", 2000);
	add_optional_topic("sensor_airflow", 100);
	add_topic("sensor_combined");
	add_optional_topic("sensor_correction");
	add_optional_topic("sensor_gyro_fft", 50);
	add_topic("sensor_selection");
	add_topic("sensors_status_imu", 200);
	add_optional_topic("spoilers_setpoint", 1000);
	add_topic("state_attack_command");
	add_topic("state_attack_pos_status", 10);
	add_topic("state_attack_att_status", 10);
	add_topic("system_power", 500);
	add_optional_topic("takeoff_status", 1000);
	add_optional_topic("tecs_status", 200);
	add_optional_topic("tiltrotor_extra_controls", 100);
	add_topic("trajectory_setpoint", 200);
	add_topic("transponder_report");
	add_topic("vehicle_acceleration", 50);
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_angular_velocity", 20);
	add_topic("vehicle_attitude", 50);
	add_topic("vehicle_attitude_setpoint", 50);
	add_topic("vehicle_command");
	add_topic("vehicle_command_ack");
	add_topic("vehicle_constraints", 1000);
	add_topic("vehicle_control_mode");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position", 100);
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 20);
	add_topic("vehicle_roi", 1000);
	add_topic("vehicle_status");
	add_optional_topic("vtol_vehicle_status", 200);
	add_topic("wind", 1000);
	add_topic("wind_command");
	add_topic("fixed_wing_lateral_setpoint");
	add_topic("fixed_wing_longitudinal_setpoint");
	add_topic("longitudinal_control_configuration");
	add_topic("lateral_control_configuration");
	add_optional_topic("fixed_wing_lateral_guidance_status", 100);
	add_optional_topic("fixed_wing_lateral_status", 100);
	add_optional_topic("fixed_wing_runway_control", 100);

	// multi topics
	add_optional_topic_multi("actuator_outputs", 100, 3);
	add_optional_topic_multi("airspeed_wind", 1000, 4);
	add_optional_topic_multi("control_allocator_status", 200, 2);
	add_optional_topic_multi("rate_ctrl_status", 200, 2);
	add_optional_topic_multi("sensor_hygrometer", 500, 4);
	add_optional_topic_multi("rpm", 200);
	add_topic_multi("timesync_status", 1000, 3);
	add_optional_topic_multi("telemetry_status", 1000, 4);

	// EKF multi topics
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 500; // 2 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	// important EKF topics (higher rate)
	add_optional_topic("estimator_selector_status", 10);
	add_optional_topic_multi("estimator_event_flags", 10);
	add_optional_topic_multi("estimator_optical_flow_vel", 200);
	add_optional_topic_multi("estimator_sensor_bias", 1000);
	add_optional_topic_multi("estimator_status", 200);
	add_optional_topic_multi("estimator_status_flags", 10);
	add_optional_topic_multi("yaw_estimator_status", 1000);

	// log all raw sensors at minimal rate (at least 1 Hz)
	add_topic_multi("battery_status", 200, 3);
	add_topic_multi("differential_pressure", 1000, 2);
	add_topic_multi("distance_sensor", 1000, 2);
	add_optional_topic_multi("sensor_accel", 1000, 4);
	add_topic_multi("sensor_baro", 1000, 4);
	add_topic_multi("sensor_gps", 1000, 2);
	add_topic_multi("sensor_gnss_relative", 1000, 1);
	add_optional_topic_multi("sensor_gyro", 1000, 4);
	add_topic_multi("sensor_mag", 1000, 4);
	add_topic_multi("sensor_optical_flow", 1000, 2);

	add_topic_multi("vehicle_imu", 500, 4);
	add_topic_multi("vehicle_imu_status", 1000, 4);
	add_optional_topic_multi("vehicle_magnetometer", 500, 4);
	add_topic("vehicle_optical_flow", 500);
	add_topic("aux_global_position", 500);
	//add_optional_topic("vehicle_optical_flow_vel", 100);
	add_optional_topic("pps_capture");

	// additional control allocation logging
	add_topic("actuator_motors", 100);
	add_topic("actuator_servos", 100);
	add_topic_multi("vehicle_thrust_setpoint", 20, 2);
	add_topic_multi("vehicle_torque_setpoint", 20, 2);

	// SYS_HITL: default ground truth logging for simulation
	int32_t sys_hitl = 0;
	param_get(param_find("SYS_HITL"), &sys_hitl);

	if (sys_hitl >= 1) {
		add_topic("vehicle_angular_velocity_groundtruth", 10);
		add_topic("vehicle_attitude_groundtruth", 10);
		add_topic("vehicle_global_position_groundtruth", 100);
		add_topic("vehicle_local_position_groundtruth", 20);
	}

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_optional_topic("vehicle_torque_setpoint_virtual_mc");
	add_optional_topic("vehicle_torque_setpoint_virtual_fw");
	add_optional_topic("vehicle_thrust_setpoint_virtual_mc");
	add_optional_topic("vehicle_thrust_setpoint_virtual_fw");
	add_topic("time_offset");
	add_topic("vehicle_angular_velocity", 10);
	add_topic("vehicle_angular_velocity_groundtruth", 10);
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 20);

	// EKF replay
	{
		// optionally log all estimator* topics at minimal rate
		const uint16_t kEKFVerboseIntervalMilliseconds = 10; // 100 Hz
		const struct orb_metadata *const *topic_list = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strncmp(topic_list[i]->o_name, "estimator", 9) == 0) {
				add_optional_topic_multi(topic_list[i]->o_name, kEKFVerboseIntervalMilliseconds);
			}
		}
	}

	add_topic("vehicle_attitude");
	add_topic("vehicle_global_position");
	add_topic("vehicle_local_position");
	add_topic("wind");
	add_optional_topic_multi("yaw_estimator_status");

#endif /* CONFIG_ARCH_BOARD_PX4_SITL */

#ifdef CONFIG_BOARD_UAVCAN_INTERFACES
	add_topic_multi("can_interface_status", 100, CONFIG_BOARD_UAVCAN_INTERFACES);
#endif
}

void LoggedTopics::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("manual_control_setpoint");
	add_topic_multi("rate_ctrl_status", 20, 2);
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");

	add_topic("esc_status", 5);
	add_topic("actuator_motors");
	add_topic("actuator_outputs_debug");
	add_topic("actuator_servos");
	add_topic_multi("vehicle_thrust_setpoint", 0, 2);
	add_topic_multi("vehicle_torque_setpoint", 0, 2);
}

void LoggedTopics::add_debug_topics()
{
	add_topic("debug_array");
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
	add_topic_multi("satellite_info", 1000, 2);
	add_topic("mag_worker_data");
	add_topic("sensor_preflight_mag", 500);
	add_topic("actuator_test", 500);
	add_topic("neural_control", 50);
}

void LoggedTopics::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("airspeed_validated");
	add_topic("vehicle_optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_visual_odometry");
	add_topic("aux_global_position");
	add_topic_multi("distance_sensor");
}

void LoggedTopics::add_thermal_calibration_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_sensor_comparison_topics()
{
	add_topic_multi("sensor_accel", 100, 4);
	add_topic_multi("sensor_baro", 100, 4);
	add_topic_multi("sensor_gyro", 100, 4);
	add_topic_multi("sensor_mag", 100, 4);
}

void LoggedTopics::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic_multi("distance_sensor");
	add_topic("obstacle_distance_fused");
	add_topic("obstacle_distance");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_visual_odometry", 30);
}

void LoggedTopics::add_raw_imu_gyro_fifo()
{
	add_topic("sensor_gyro_fifo");
}

void LoggedTopics::add_raw_imu_accel_fifo()
{
	add_topic("sensor_accel_fifo");
}

void LoggedTopics::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("sensor_combined");
	add_topic("vehicle_angular_velocity");
	add_topic("vehicle_torque_setpoint");
	add_topic("vehicle_acceleration");
	add_topic("actuator_motors");
}

void LoggedTopics::add_high_rate_sensors_topics()
{
	add_topic_multi("distance_sensor", 0, 4);
	add_topic_multi("sensor_optical_flow", 0, 2);
	add_topic_multi("sensor_gps", 0, 4);
	add_topic_multi("sensor_mag", 0, 4);
}

void LoggedTopics::add_mavlink_tunnel()
{
	add_topic("mavlink_tunnel");
}

int LoggedTopics::add_topics_from_file(const char *fname)
{
	int ntopics = 0;

	/* open the topic list file */
	FILE *fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {
		/* get a line, bail on error/EOF */
		char line[80];
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[ <interval>[ <instance>]]
		char topic_name[80];
		uint32_t interval_ms = 0;
		uint32_t instance = 0;
		int nfields = sscanf(line, "%s %" PRIu32 " %" PRIu32, topic_name, &interval_ms, &instance);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval_ms */
			if ((nfields > 2 && add_topic(topic_name, interval_ms, instance))
			    || add_topic_multi(topic_name, interval_ms)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

void LoggedTopics::initialize_mission_topics(MissionLogType mission_log_type)
{
	if (mission_log_type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);

	} else if (mission_log_type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void LoggedTopics::add_mission_topic(const char *name, uint16_t interval_ms)
{
	if (add_topic(name, interval_ms)) {
		++_num_mission_subs;
	}
}

bool LoggedTopics::add_topic(const orb_metadata *topic, uint16_t interval_ms, uint8_t instance, bool optional)
{
	if (_subscriptions.count >= MAX_TOPICS_NUM) {
		PX4_WARN("Too many subscriptions, failed to add: %s %" PRIu8, topic->o_name, instance);
		return false;
	}

	if (optional && orb_exists(topic, instance) != 0) {
		PX4_DEBUG("Not adding non-existing optional topic %s %i", topic->o_name, instance);

		if (instance == 0 && _subscriptions.num_excluded_optional_topic_ids < MAX_EXCLUDED_OPTIONAL_TOPICS_NUM) {
			_subscriptions.excluded_optional_topic_ids[_subscriptions.num_excluded_optional_topic_ids++] = topic->o_id;
		}

		return false;
	}

	RequestedSubscription &sub = _subscriptions.sub[_subscriptions.count++];
	sub.interval_ms = interval_ms;
	sub.instance = instance;
	sub.id = static_cast<ORB_ID>(topic->o_id);
	return true;
}

bool LoggedTopics::add_topic(const char *name, uint16_t interval_ms, uint8_t instance, bool optional)
{
	interval_ms /= _rate_factor;

	const orb_metadata *const *topics = orb_get_topics();
	bool success = false;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (int j = 0; j < _subscriptions.count; ++j) {
				if (_subscriptions.sub[j].id == static_cast<ORB_ID>(topics[i]->o_id) &&
				    _subscriptions.sub[j].instance == instance) {

					PX4_DEBUG("logging topic %s(%" PRIu8 "), interval: %" PRIu16 ", already added, only setting interval",
						  topics[i]->o_name, instance, interval_ms);

					_subscriptions.sub[j].interval_ms = interval_ms;
					success = true;
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				success = add_topic(topics[i], interval_ms, instance, optional);

				if (success) {
					PX4_DEBUG("logging topic: %s(%" PRIu8 "), interval: %" PRIu16, topics[i]->o_name, instance, interval_ms);
				}

				break;
			}
		}
	}

	return success;
}

bool LoggedTopics::add_topic_multi(const char *name, uint16_t interval_ms, uint8_t max_num_instances, bool optional)
{
	// add all possible instances
	for (uint8_t instance = 0; instance < max_num_instances; instance++) {
		add_topic(name, interval_ms, instance, optional);
	}

	return true;
}

bool LoggedTopics::initialize_logged_topics(SDLogProfileMask profile)
{
	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics(profile);
	}

	return _subscriptions.count > 0;
}

void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)
{
	// load appropriate topics for profile
	// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
	if (profile & SDLogProfileMask::DEFAULT) {
		add_default_topics();
	}

	if (profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
		add_estimator_replay_topics();
	}

	if (profile & SDLogProfileMask::THERMAL_CALIBRATION) {
		add_thermal_calibration_topics();
	}

	if (profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
		add_system_identification_topics();
	}

	if (profile & SDLogProfileMask::HIGH_RATE) {
		add_high_rate_topics();
	}

	if (profile & SDLogProfileMask::DEBUG_TOPICS) {
		add_debug_topics();
	}

	if (profile & SDLogProfileMask::SENSOR_COMPARISON) {
		add_sensor_comparison_topics();
	}

	if (profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}

	if (profile & SDLogProfileMask::RAW_IMU_GYRO_FIFO) {
		add_raw_imu_gyro_fifo();
	}

	if (profile & SDLogProfileMask::RAW_IMU_ACCEL_FIFO) {
		add_raw_imu_accel_fifo();
	}

	if (profile & SDLogProfileMask::MAVLINK_TUNNEL) {
		add_mavlink_tunnel();
	}

	if (profile & SDLogProfileMask::HIGH_RATE_SENSORS) {
		add_high_rate_sensors_topics();
	}
}

~~~

## 5.12 攻击仿真测试

开启 QGC 地面站：

~~~bash
cd /home/liu/Desktop/ROS2/QGroundControl && ./QGroundControl-x86_64.AppImage
~~~

开启 SITL 仿真：

~~~bash
cd /home/liu/Desktop/ROS2/PX4-Autopilot && PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
~~~

开启 Micro-XRCE-DDS-Agent 代理：

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

开启 ROS2 消息桥接：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
~~~

实施攻击命令：

*通用模板：*

~~~bash
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: <CH>, type: <TYPE>, param: [a, b, c, d], t0_us: 0, t1_us: 0, seed: 0}" --once
~~~

*字段说明：*

| 字段        | 类型         | 含义                                             |
| ----------- | ------------ | ------------------------------------------------ |
| `timestamp` | `uint64`     | 系统启动以来时间（µs）。启动时刻不看它，一般留 0 |
| `channel`   | `uint8`      | 攻击哪个状态通道（0–9，见下）                    |
| `type`      | `uint8`      | 原语类型（0=NONE 清除，见下）                    |
| `param`     | `float64[4]` | 原语参数 a/b/c/d，含义随 `type`                  |
| `t0_us`     | `uint64`     | 相对到达时刻的启动延迟（µs）；0 = 立即           |
| `t1_us`     | `uint64`     | 相对启动的持续时间（µs）；0 = 直到被清除         |
| `seed`      | `uint32`     | 随机原语种子；0 = 每通道确定性默认值             |

*通道说明：*

| 值   | 通道          | 物理量         | 归属           |
| ---- | ------------- | -------------- | -------------- |
| 0    | `NAV_POS_X`   | NED 北位置 (m) | mc_pos_control |
| 1    | `NAV_POS_Y`   | NED 东位置 (m) | 〃             |
| 2    | `NAV_POS_Z`   | NED 下位置 (m) | 〃             |
| 3    | `NAV_VEL_X`   | 北速度 (m/s)   | 〃             |
| 4    | `NAV_VEL_Y`   | 东速度 (m/s)   | 〃             |
| 5    | `NAV_VEL_Z`   | 下速度 (m/s)   | 〃             |
| 6    | `NAV_HEADING` | 航向 (rad)     | 〃             |
| 7    | `ATT_ROLL`    | 横滚 (rad)     | mc_att_control |
| 8    | `ATT_PITCH`   | 俯仰 (rad)     | 〃             |
| 9    | `ATT_YAW`     | 偏航 (rad)     | 〃             |

*类型说明：*

| 值   | 原语          | 公式 / 效果                           | param 用到的             |
| ---- | ------------- | ------------------------------------- | ------------------------ |
| 0    | `NONE`        | **清除该通道**                        | —                        |
| 1    | `BIAS`        | `truth + a`                           | a = 偏置量               |
| 2    | `SPOOF`       | `= a`（固定值欺骗）                   | a = 目标值               |
| 3    | `NOISE`       | `truth + N(0,a)`                      | a = 标准差               |
| 4    | `SCALING`     | `truth * a`                           | a = 缩放系数             |
| 5    | `DRIFT`       | `truth + a·(t−start)`                 | a = 漂移斜率             |
| 6    | `OSCILLATION` | `truth + a·sin(b·(t−start)+c)`        | a=幅值, b=角频率, c=相位 |
| 7    | `RANDOM_WALK` | `v_last + N(0,a)`                     | a = 每步标准差           |
| 8    | `QUANTIZE`    | `round(truth/a)·a`                    | a = 量化步长             |
| 9    | `CLAMP`       | `clamp(truth, a, b)`                  | a=下限, b=上限           |
| 10   | `FREEZE`      | 保持激活瞬间的值                      | —                        |
| 11   | `DROP`        | **直通**（就地标量无消息可丢，§9 #2） | —                        |
| 12   | `DELAY`       | **直通**（未实施，§10）               | —                        |
| 13   | `REPLAY`      | **直通**（未实施，§10）               | —                        |

*命令示例：*

~~~bash
# 位置北向偏置 +10 m（BIAS）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 1, param: [10.0, 0.0, 0.0, 0.0]}" --once

# 高度欺骗：让控制器以为比实际高 10 m（NED z 向下，偏置 −10）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 2, type: 1, param: [-10.0, 0.0, 0.0, 0.0]}" --once

# 航向欺骗到 90°（SPOOF a=π/2）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 6, type: 2, param: [1.5708, 0.0, 0.0, 0.0]}" --once

# 横滚偏置 +15°（0.2618 rad）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 7, type: 1, param: [0.2618, 0.0, 0.0, 0.0]}" --once

# 冻结位置（FREEZE）
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 10}" --once

# 清除某通道
ros2 topic pub /fmu/in/state_attack_command px4_msgs/msg/StateAttackCommand "{channel: 0, type: 0}" --once
~~~

# 六、配置 FAST LIVO2

## 6.1 参考教程

https://github.com/Robotic-Developer-Road/FAST-LIVO2

**PCL && Eigen && OpenCV**

​	PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/).

​	Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

​	OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

上面三个软件自行安装，注意版本符合要求即可。

## 6.2 Sophus 安装

~~~bash
cd /home/liu/Desktop/ROS2
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc)
sudo make install
~~~

## 6.3 rpg_vikit 下载

~~~bash
cd /home/liu/Desktop/ROS2/src
git clone https://github.com/Robotic-Developer-Road/rpg_vikit.git
~~~

## 6.4 Livox-SDK2 安装

~~~bash
cd /home/liu/Desktop/ROS2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
~~~

## 6.5 livox_ros_driver2 下载

~~~bash
cd /home/liu/Desktop/ROS2/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cp -r /home/liu/Desktop/ROS2/src/ws_livox/src/livox_ros_driver2 /home/liu/Desktop/ROS2/src/
rm -rf /home/liu/Desktop/ROS2/src/ws_livox
~~~

由于修改了 livox_ros_driver2 文件夹的路径，为了能够在本项目进行编译，因此进行了如下修改，让 CMakeLists.txt 在找不到变量时自动指向源码目录。

1）打开 `/home/liu/Desktop/ROS2/src/livox_ros_driver2/CMakeLists.txt`。

2）找到引用 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES` 的那一行（通常在 `target_include_directories` 附近，大约在 300 行左右）。

3）将它替换为以下内容（直接指向 `livox_interfaces2` 的源码路径，通常是相对路径）：

```cmake
${PROJECT_SOURCE_DIR}/../livox_interfaces2
```

## 6.6 FAST-LIVO2 下载

~~~bash
cd /home/liu/Desktop/ROS2/src
git clone https://github.com/Robotic-Developer-Road/FAST-LIVO2.git
~~~

## 6.7 修改 ROS2 消息桥接

文件位置：/home/liu/Desktop/ROS2/src/x500_plus/config/x500_plus_bridge.yaml

文件：/home/liu/Desktop/ROS2/src/x500_plus/config/x500_plus_bridge.yaml

~~~yaml
# ============================================================
# x500_plus 传感器桥接清单（Gazebo gz-sim -> ROS2）
#
# 这是「需要桥接哪些消息」的完整清单，供 parameter_bridge 使用。
# 当前 launch (x500_plus.launch.py) 通过 config_file 参数直接读取本文件，
# 本文件即为桥接的唯一真源（无需再手动传位置参数）。
# 未启用的条目先注释掉，需要时取消注释即可。
# ============================================================

# ---------- A. 传感器数据 ----------

# 彩色相机
- ros_topic_name: "/realsensed455f/color/image_raw"
  gz_topic_name: "/realsensed455f/color/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# 彩色相机内参（标定 / 深度对齐用，与图像配对）
# - ros_topic_name: "/realsensed455f/color/camera_info"
#   gz_topic_name: "/realsensed455f/color/camera_info"
#   ros_type_name: "sensor_msgs/msg/CameraInfo"
#   gz_type_name: "gz.msgs.CameraInfo"
#   direction: GZ_TO_ROS

# 深度相机（R_FLOAT32 -> 32FC1，单位米；超量程为 +inf，需 depth_fixer 清洗）
- ros_topic_name: "/realsensed455f/depth/image_raw"
  gz_topic_name: "/realsensed455f/depth/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# 深度相机内参
# - ros_topic_name: "/realsensed455f/depth/camera_info"
#   gz_topic_name: "/realsensed455f/depth/camera_info"
#   ros_type_name: "sensor_msgs/msg/CameraInfo"
#   gz_type_name: "gz.msgs.CameraInfo"
#   direction: GZ_TO_ROS

# 深度相机导出的点云
# - ros_topic_name: "/realsensed455f/depth/points"
#   gz_topic_name: "/realsensed455f/depth/image_raw/points"
#   ros_type_name: "sensor_msgs/msg/PointCloud2"
#   gz_type_name: "gz.msgs.PointCloudPacked"
#   direction: GZ_TO_ROS

# MID-360S 雷达 3D 点云（gz 侧 gpu_lidar 自动发布在 <topic>/points）
- ros_topic_name: "/mid360s/points/points"
  gz_topic_name: "/mid360s/points/points"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

# MID-360S 内置 IMU（FAST-LIVO 惯导输入，frame_id = mid360s_link，200 Hz）
- ros_topic_name: "/mid360s/imu"
  gz_topic_name: "/mid360s/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

# MID-360S 雷达 2D scan（LaserScan，gz 侧在 <topic> 本身）
# - ros_topic_name: "/mid360s/scan"
#   gz_topic_name: "/mid360s/points"
#   ros_type_name: "sensor_msgs/msg/LaserScan"
#   gz_type_name: "gz.msgs.LaserScan"
#   direction: GZ_TO_ROS

# ---------- B. 里程计 / 动态 TF ----------

# 无人机里程计 -> odom_tf_broadcaster.py 转成 world->base_link 动态 TF
- ros_topic_name: "/model/x500_plus_0/odometry"
  gz_topic_name: "/model/x500_plus_0/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# 带协方差的里程计（下游若需要协方差）
# - ros_topic_name: "/model/x500_plus_0/odometry_with_covariance"
#   gz_topic_name: "/model/x500_plus_0/odometry_with_covariance"
#   ros_type_name: "nav_msgs/msg/Odometry"
#   gz_type_name: "gz.msgs.OdometryWithCovariance"
#   direction: GZ_TO_ROS

# ---------- C. 仿真时钟 ----------

# 仿真时钟（RViz2 需要跟随仿真时间时启用）
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

# ------------------------------------------------------------
# 备注：gz 里还有一批 PX4 飞控内部传感器话题（IMU / magnetometer /
# navsat / air_pressure / optical_flow / air_speed 等，都在
# /world/Penglai/model/x500_plus_0/link/.../sensor/... 下），
# 它们属于飞控内部闭环，不在「x500_plus 传感器可视化」范围内，故未列出。
# 若将来要桥接，类型对应：gz.msgs.IMU -> sensor_msgs/msg/Imu 等。
# ------------------------------------------------------------

~~~

## 6.8 新建配置文件

文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/config/camera_x500_plus.yaml
文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/config/sim_x500_plus.yaml

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/config/camera_x500_plus.yaml

~~~yaml
/**:
  ros__parameters:
    cam_model: Pinhole
    cam_width: 1280
    cam_height: 800
    scale: 1.0
    cam_fx: 674.4439
    cam_fy: 674.4439
    cam_cx: 640.0
    cam_cy: 400.0
    cam_d0: 0.0
    cam_d1: 0.0
    cam_d2: 0.0
    cam_d3: 0.0


~~~

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/config/sim_x500_plus.yaml

~~~yaml
/**:
  ros__parameters:
    common:
      img_topic: "/realsensed455f/color/image_raw"
      lid_topic: "/mid360s/points/points"
      imu_topic: "/mid360s/imu"
      img_en: 1
      lidar_en: 1
      ros_driver_bug_fix: false

    extrin_calib:
      # imu <-> lidar are co-located in mid360s_link -> identity
      extrinsic_T: [0.0, 0.0, 0.0]
      extrinsic_R: [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
      # lidar -> camera: p_cam = Rcl * p_lidar + Pcl
      # gz-sim lidar frame (+X fwd, +Y left, +Z up) -> vikit camera frame (+Z fwd, +X right, +Y down).
      # This ~90deg convention rotation is REQUIRED: the gz-sim camera looks along +X, FAST-LIVO2
      # assumes the camera looks along +Z (real datasets carry the same rotation in their Rcl).
      # Then composed with the 5deg extra camera pitch (camera -15deg vs lidar -10deg).
      Rcl: [0.0, -1.0, 0.0,
            -0.087156, 0.0, -0.996195,
            0.996195, 0.0, -0.087156]
      # lidar origin expressed in the (vikit) camera frame
      Pcl: [0.0, -0.04678, -0.18087]

    time_offset:
      imu_time_offset: 0.0
      img_time_offset: 0.0
      exposure_time_init: 0.0

    preprocess:
      point_filter_num: 2
      filter_size_surf: 0.5
      lidar_type: 8 # MID360S (sensor_msgs/PointCloud2: x/y/z/intensity/ring)
      scan_line: 40
      blind: 0.8

    vio:
      max_iterations: 5
      outlier_threshold: 1000
      img_point_cov: 100
      patch_size: 8
      patch_pyrimid_level: 4
      normal_en: true
      raycast_en: false
      inverse_composition_en: false
      exposure_estimate_en: true
      inv_expo_cov: 0.1

    imu:
      imu_en: true
      imu_int_frame: 30
      acc_cov: 0.5
      gyr_cov: 0.3
      b_acc_cov: 0.0001
      b_gyr_cov: 0.0001

    lio:
      max_iterations: 5
      dept_err: 0.02
      beam_err: 0.05
      min_eigen_value: 0.0025
      voxel_size: 0.5
      max_layer: 2
      max_points_num: 50
      layer_init_num: [5, 5, 5, 5, 5]

    local_map:
      map_sliding_en: false
      half_map_size: 100
      sliding_thresh: 8.0

    uav:
      imu_rate_odom: false
      gravity_align_en: true # imu frame (mid360s_link) is pitched +10deg vs body -> level the map

    publish:
      dense_map_en: true
      pub_effect_point_en: false
      pub_plane_en: false
      pub_scan_num: 1
      blind_rgb_points: 0.0

    evo:
      seq_name: "sim_x500_plus"
      pose_output_en: false

    pcd_save:
      pcd_save_en: false
      type: 0
      colmap_output_en: false
      filter_size_pcd: 0.15
      interval: -1

    image_save:
      img_save_en: false
      interval: 1


~~~

## 6.9 修改配置文件

文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/include/common_lib.h
文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/include/preprocess.h
文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/src/preprocess.cpp

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/include/common_lib.h

~~~c++
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <utils/so3_math.h>
#include <utils/types.h>
#include <utils/color.h>
#include <utils/utils.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

using namespace std;
// using namespace Eigen;   // avoid cmake error: reference to ‘Matrix’ is ambiguous
using namespace Sophus;

#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
#define G_m_s2 (9.81)   // Gravaty const in GuangDong/China
#define DIM_STATE (19)  // Dimension of states (Let Dim(SO(3)) = 3)
#define INIT_COV (0.01)
#define SIZE_LARGE (500)
#define SIZE_SMALL (100)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

enum LID_TYPE
{
  AVIA = 1,
  VELO16 = 2,
  OUST64 = 3,
  L515 = 4,
  XT32 = 5,
  PANDAR128 = 6,
  ROBOSENSE = 7,
  MID360S = 8
};
enum SLAM_MODE
{
  ONLY_LO = 0,
  ONLY_LIO = 1,
  LIVO = 2
};
enum EKF_STATE
{
  WAIT = 0,
  VIO = 1,
  LIO = 2,
  LO = 3
};

struct MeasureGroup
{
  double vio_time;
  double lio_time;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
  cv::Mat img;
  MeasureGroup()
  {
    vio_time = 0.0;
    lio_time = 0.0;
  };
};

struct LidarMeasureGroup
{
  double lidar_frame_beg_time;
  double lidar_frame_end_time;
  double last_lio_update_time;
  PointCloudXYZI::Ptr lidar;
  PointCloudXYZI::Ptr pcl_proc_cur;
  PointCloudXYZI::Ptr pcl_proc_next;
  deque<struct MeasureGroup> measures;
  EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup()
  {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    lio_vio_flg = WAIT;
    this->lidar.reset(new PointCloudXYZI());
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};

typedef struct pointWithVar
{
  Eigen::Vector3d point_b;     // point in the lidar body frame
  Eigen::Vector3d point_i;     // point in the imu body frame
  Eigen::Vector3d point_w;     // point in the world frame
  Eigen::Matrix3d var_nostate; // the var removed the state covarience
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  pointWithVar()
  {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;


struct StatesGroup
{
  StatesGroup()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
    this->bias_g = V3D::Zero();
    this->bias_a = V3D::Zero();
    this->gravity = V3D::Zero();
    this->inv_expo_time = 1.0;
    this->cov = MD(DIM_STATE, DIM_STATE)::Identity() * INIT_COV;
    this->cov(6, 6) = 0.00001;
    this->cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;
  };

  StatesGroup(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  };

  StatesGroup &operator=(const StatesGroup &b)
  {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  };

  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    StatesGroup a;
    a.rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);

    a.cov = this->cov;
    return a;
  };

  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
  {
    this->rot_end = this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
  };

  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b)
  {
    Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    return a;
  };

  void resetpose()
  {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
  }

  M3D rot_end;                              // the estimated attitude (rotation matrix) at the end lidar point
  V3D pos_end;                              // the estimated position at the end lidar point (world frame)
  V3D vel_end;                              // the estimated velocity at the end lidar point (world frame)
  double inv_expo_time;                     // the estimated inverse exposure time (no scale)
  V3D bias_g;                               // gyroscope bias
  V3D bias_a;                               // accelerator bias
  V3D gravity;                              // the estimated gravity acceleration
  Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance
};

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return move(rot_kp);
}

#endif
~~~

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/include/preprocess.h

~~~c++
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

enum LiDARFeature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
enum Surround
{
  Prev,
  Next
};
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  LiDARFeature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Hesai_Pandar128 ***/
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

/*** Robosense_Airy ***/
namespace robosense_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
/*****************/

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
  vector<orgtype> typess[128]; // maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;
  
  double blind, blind_sqr;
  bool feature_enabled, given_offset_time;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_full;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_surf;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_corn;

private:
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void mid360_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_
~~~

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/src/preprocess.cpp

~~~c++
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess() : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case L515:
    l515_handler(msg);
    break;

  case XT32:
    xt32_handler(msg);
    break;

  case PANDAR128:
    Pandar128_handler(msg);
    break;

  case ROBOSENSE:
    robosense_handler(msg);
    break;

  case MID360S:
    mid360_handler(msg);
    break;

  default:
    printf("Error LiDAR Type: %d \n", lidar_type);
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  printf("[ Preprocess ] Input point number: %d \n", plsize);
  // printf("point_filter_num: %d\n", point_filter_num);

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    for (uint i = 1; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        bool is_new = false;
        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count++;
    double t0 = omp_get_wtime();
    for (int j = 0; j < N_SCANS; j++)
    {
      if (pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for (uint i = 0; i < plsize; i++)
    {
      if ((msg->points[i].line < N_SCANS)) // && ((msg->points[i].tag & 0x30) == 0x10))
      {
        valid_num++;

        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points

        if (i == 0)
          pl_full[i].curvature = fabs(pl_full[i].curvature) < 1.0 ? pl_full[i].curvature : 0.0;
        else
        {
          // if(fabs(pl_full[i].curvature - pl_full[i - 1].curvature) > 1.0) ROS_ERROR("time jump: %f", fabs(pl_full[i].curvature - pl_full[i - 1].curvature));
          pl_full[i].curvature = fabs(pl_full[i].curvature - pl_full[i - 1].curvature) < 1.0
                                     ? pl_full[i].curvature
                                     : pl_full[i - 1].curvature + 0.004166667f; // float(100/24000)
        }

        if (valid_num % point_filter_num == 0)
        {
          if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z >= blind_sqr)
          {
            pl_surf.push_back(pl_full[i]);
            // if (i % 100 == 0 || i == 0) printf("pl_full[i].curvature: %f \n",
            // pl_full[i].curvature);
          }
        }
      }
    }
  }
  printf("[ Preprocess ] Output point number: %zu \n", pl_surf.points.size());
}

void Preprocess::l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<pcl::PointXYZRGB> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = stamp2Sec(msg->header.stamp);
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind_sqr) continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.normal_x = pl_orig.points[i].r;
    added_pt.normal_y = pl_orig.points[i].g;
    added_pt.normal_z = pl_orig.points[i].b;

    added_pt.curvature = 0.0;
    pl_surf.points.push_back(added_pt);
  }

  cout << "pl size:: " << pl_orig.points.size() << endl;
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind * blind) continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0.0;
    added_pt.normal_y = 0.0;
    added_pt.normal_z = 0.0;
    added_pt.curvature = 0.0;
    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind_sqr) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if (pl_orig.points[i].ring < N_SCANS) { pl_buff[pl_orig.points[i].ring].push_back(added_pt); }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = stamp2Sec(msg->header.stamp);
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < blind_sqr) continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;

      // cout<<added_pt.curvature<<endl;

      pl_surf.points.push_back(added_pt);
    }
    std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
      return a.curvature < b.curvature;
    });
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].time > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // added_pt.curvature = pl_orig.points[i].t;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
        }
      }
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();

  pcl::PointCloud<Pandar128_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = static_cast<float>(pl_orig.points[i].intensity) / 255.0f;
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

    if (i % point_filter_num == 0)
    {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
      {
        pl_surf.points.push_back(added_pt);
        // printf("time mode: %d time: %d \n", given_offset_time,
        // pl_orig.points[i].t);
      }
    }
  }

  // define a lambda function for the comparison
  auto comparePoints = [](const PointType& a, const PointType& b) -> bool
  {
    return a.curvature < b.curvature;
  };
  
  // sort the points using the comparison function
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), comparePoints);
  
  // cout << GREEN << "pl_surf.points[0].timestamp: " << pl_surf.points[0].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[1000].timestamp: " << pl_surf.points[1000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[5000].timestamp: " << pl_surf.points[5000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[10000].timestamp: " << pl_surf.points[10000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[20000].timestamp: " << pl_surf.points[20000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[30000].timestamp: " << pl_surf.points[30000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[31000].timestamp: " << pl_surf.points[31000].curvature << RESET << endl;
}

void Preprocess::xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<xt32_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  pl_surf.reserve(plsize);

  bool is_first[MAX_LINE_NUM];
  double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
  double omega_l = 3.61;                 // scan angular velocity
  float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
  float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].timestamp > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  double time_head = pl_orig.points[0].timestamp;

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].timestamp / 1000.0; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;

      // printf("added_pt.curvature: %lf %lf \n", added_pt.curvature,
      // pl_orig.points[i].timestamp);

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
        }
      }
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();

  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; ++i)
  {
    if (i % point_filter_num != 0) continue;

    const auto& pt = pl_orig.points[i];
    const double x = pt.x, y = pt.y, z = pt.z;
    const double dist_sqr = x * x + y * y + z * z;
    const bool is_valid = (dist_sqr >= blind_sqr) && !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    if (!is_valid) continue;

    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = (pt.timestamp - time_head) * 1000.0;
    pl_surf.points.push_back(added_pt);
  }
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
    return a.curvature < b.curvature;
  });
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while (types[head].range < blind_sqr)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr) { continue; }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)
    {
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex) { types[j].ftype = Real_Plane; }
        else { types[j].ftype = Poss_Plane; }
      }

      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707) { types[i].ftype = Edge_Plane; }
        else { types[i].ftype = Real_Plane; }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr || types[i].ftype >= Real_Plane) { continue; }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) { continue; }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1) { m = 1; }

      if (types[i + m].range < blind_sqr)
      {
        if (types[i].range > inf_bound) { types[i].edj[j] = Nr_inf; }
        else { types[i].edj[j] = Nr_blind; }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit) { types[i].edj[j] = Nr_180; }
      else if (types[i].angle[j] > jump_down_limit) { types[i].edj[j] = Nr_zero; }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor) { types[i].ftype = Wire; }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr || types[i - 1].range < blind_sqr || types[i + 1].range < blind_sqr) { continue; }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) { continue; }

    if (types[i].ftype == Nor)
    {
      if (types[i - 1].dista > types[i].dista) { ratio = types[i - 1].dista / types[i].dista; }
      else { ratio = types[i].dista / types[i - 1].dista; }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor) { types[i - 1].ftype = Real_Plane; }
        if (types[i + 1].ftype == Nor) { types[i + 1].ftype = Real_Plane; }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1) { last_surface = j; }

      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) { pl_corn.push_back(pl[j]); }
      if (last_surface != -1)
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis) { break; }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid) { leng_wid = lw; }
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == AVIA)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind_sqr || types[i - 2].range < blind_sqr) { return false; }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind_sqr || types[i + 2].range < blind_sqr) { return false; }
  }
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb) { return false; }

  return true;
}
~~~

## 6.10 声明节点参数

文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/src/LIVMapper.cpp

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/src/LIVMapper.cpp

~~~c++
/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <vikit/camera_loader.h>

using namespace Sophus;
LIVMapper::LIVMapper(rclcpp::Node::SharedPtr &node, std::string node_name)
    : node(std::make_shared<rclcpp::Node>(node_name)),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);
  
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(this->node);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this->node, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents(this->node);          // initialize components errors
  path.header.stamp = this->node->now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(rclcpp::Node::SharedPtr &node)
{
  // declare parameters
  this->node->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->node->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->node->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->node->declare_parameter<int>("common.img_en", 1);
  this->node->declare_parameter<int>("common.lidar_en", 1);
  this->node->declare_parameter<std::string>("common.img_topic", "/left_camera/image");

  this->node->declare_parameter<bool>("vio.normal_en", true);
  this->node->declare_parameter<bool>("vio.inverse_composition_en", false);
  this->node->declare_parameter<int>("vio.max_iterations", 5);
  this->node->declare_parameter<int>("vio.img_point_cov", 100);
  this->node->declare_parameter<bool>("vio.raycast_en", false);
  this->node->declare_parameter<bool>("vio.exposure_estimate_en", true);
  this->node->declare_parameter<double>("vio.inv_expo_cov", 0.1);
  this->node->declare_parameter<int>("vio.grid_size", 5);
  this->node->declare_parameter<int>("vio.grid_n_height", 17);
  this->node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
  this->node->declare_parameter<int>("vio.patch_size", 8);
  this->node->declare_parameter<int>("vio.outlier_threshold", 100);
  this->node->declare_parameter<double>("time_offset.exposure_time_init", 0.0);
  this->node->declare_parameter<double>("time_offset.img_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->node->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->node->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->node->declare_parameter<bool>("uav.gravity_align_en", false);

  this->node->declare_parameter<std::string>("evo.seq_name", "01");
  this->node->declare_parameter<bool>("evo.pose_output_en", false);
  this->node->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->node->declare_parameter<double>("imu.acc_cov", 1.0);
  this->node->declare_parameter<int>("imu.imu_int_frame", 30);
  this->node->declare_parameter<bool>("imu.imu_en", true);
  this->node->declare_parameter<bool>("imu.gravity_est_en", true);
  this->node->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->node->declare_parameter<double>("preprocess.blind", 0.01);
    this->node->declare_parameter<bool>("preprocess.hilti_en", false);
  this->node->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->node->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->node->declare_parameter<int>("preprocess.scan_line",6);
  this->node->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->node->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->node->declare_parameter<int>("pcd_save.interval", -1);
  this->node->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->node->declare_parameter<bool>("image_save.img_save_en", false);
  this->node->declare_parameter<int>("image_save.interval", 1);

  this->node->declare_parameter<int>("pcd_save.type", 0);
  this->node->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->node->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_T", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.extrinsic_R", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Pcl", vector<double>{});
  this->node->declare_parameter<vector<double>>("extrin_calib.Rcl", vector<double>{});
  this->node->declare_parameter<double>("debug.plot_time", -10);
  this->node->declare_parameter<int>("debug.frame_cnt", 6);

  this->node->declare_parameter<double>("publish.blind_rgb_points", 0.01);
  this->node->declare_parameter<int>("publish.pub_scan_num", 1);
  this->node->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->node->declare_parameter<bool>("publish.dense_map_en", false);

  // camera intrinsics: declared here so vikit's camera_loader (getRemoteParam)
  // finds them locally without a separate parameter_blackboard node. Defaults
  // match the gz-sim RealSense D455 (1280x800, no distortion).
  this->node->declare_parameter<std::string>("cam_model", "Pinhole");
  this->node->declare_parameter<int>("cam_width", 1280);
  this->node->declare_parameter<int>("cam_height", 800);
  this->node->declare_parameter<double>("scale", 1.0);
  this->node->declare_parameter<double>("cam_fx", 674.4439);
  this->node->declare_parameter<double>("cam_fy", 674.4439);
  this->node->declare_parameter<double>("cam_cx", 640.0);
  this->node->declare_parameter<double>("cam_cy", 400.0);
  this->node->declare_parameter<double>("cam_d0", 0.0);
  this->node->declare_parameter<double>("cam_d1", 0.0);
  this->node->declare_parameter<double>("cam_d2", 0.0);
  this->node->declare_parameter<double>("cam_d3", 0.0);

  // get parameter
  this->node->get_parameter("common.lid_topic", lid_topic);
  this->node->get_parameter("common.imu_topic", imu_topic);
  this->node->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->node->get_parameter("common.img_en", img_en);
  this->node->get_parameter("common.lidar_en", lidar_en);
  this->node->get_parameter("common.img_topic", img_topic);

  this->node->get_parameter("vio.normal_en", normal_en);
  this->node->get_parameter("vio.inverse_composition_en", inverse_composition_en);
  this->node->get_parameter("vio.max_iterations", max_iterations);
  this->node->get_parameter("vio.img_point_cov", IMG_POINT_COV);
  this->node->get_parameter("vio.raycast_en", raycast_en);
  this->node->get_parameter("vio.exposure_estimate_en", exposure_estimate_en);
  this->node->get_parameter("vio.inv_expo_cov", inv_expo_cov);
  this->node->get_parameter("vio.grid_size", grid_size);
  this->node->get_parameter("vio.grid_n_height", grid_n_height);
  this->node->get_parameter("vio.patch_pyrimid_level", patch_pyrimid_level);
  this->node->get_parameter("vio.patch_size", patch_size);
  this->node->get_parameter("vio.outlier_threshold", outlier_threshold);
  this->node->get_parameter("time_offset.exposure_time_init", exposure_time_init);
  this->node->get_parameter("time_offset.img_time_offset", img_time_offset);
  this->node->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->node->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->node->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->node->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->node->get_parameter("evo.seq_name", seq_name);
  this->node->get_parameter("evo.pose_output_en", pose_output_en);
  this->node->get_parameter("imu.gyr_cov", gyr_cov);
  this->node->get_parameter("imu.acc_cov", acc_cov);
  this->node->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->node->get_parameter("imu.imu_en", imu_en);
  this->node->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->node->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->node->get_parameter("preprocess.blind", p_pre->blind);
  this->node->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->node->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->node->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->node->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->node->get_parameter("pcd_save.interval", pcd_save_interval);
  this->node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->node->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->node->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->node->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->node->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->node->get_parameter("extrin_calib.Pcl", cameraextrinT);
  this->node->get_parameter("extrin_calib.Rcl", cameraextrinR);
  this->node->get_parameter("debug.plot_time", plot_time);
  this->node->get_parameter("debug.frame_cnt", frame_cnt);

  this->node->get_parameter("publish.blind_rgb_points", blind_rgb_points);
  this->node->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->node->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->node->get_parameter("publish.dense_map_en", dense_map_en);
}

void LIVMapper::initializeComponents(rclcpp::Node::SharedPtr &node) 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  
  // extrinT.assign({0.04165, 0.02326, -0.0284});
  // extrinR.assign({1, 0, 0, 0, 1, 0, 0, 0, 1});
  // cameraextrinT.assign({0.0194384, 0.104689,-0.0251952});
  // cameraextrinR.assign({0.00610193,-0.999863,-0.0154172,-0.00615449,0.0153796,-0.999863,0.999962,0.00619598,-0.0060598});

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs(this->node, "parameter_blackboard", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers(rclcpp::Node::SharedPtr &node, image_transport::ImageTransport &it_)
{
  image_transport::ImageTransport it(this->node);
  if (p_pre->lidar_type == AVIA) {
    sub_pcl = this->node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000, std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1));
  } else {
    sub_pcl = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1));
  }
  sub_imu = this->node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1));
  sub_img = this->node->create_subscription<sensor_msgs::msg::Image>(img_topic, 200000, std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1));
  
  pubLaserCloudFullRes = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 100);
  pubSubVisualMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = this->node->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->node->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = this->node->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->node->create_wall_timer(0.004s, std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_= this->node->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->node->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run(rclcpp::Node::SharedPtr &node) 
{
  rclcpp::Rate rate(5000);
  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(this->node);
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制 propagate 频率和 IMU 频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && stamp2Sec(prop_imu_buffer.front().header.stamp) < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = stamp2Sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = stamp2Sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = stamp2Sec(msg->header.stamp) + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(),"lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg(new livox_ros_driver2::msg::CustomMsg(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - stamp2Sec(msg->header.stamp)) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - stamp2Sec(msg->header.stamp);
    RCLCPP_INFO(this->node->get_logger(), "\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->node->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->node->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  RCLCPP_INFO(this->node->get_logger(), "get point cloud at time: %.6f", stamp2Sec(msg->header.stamp));
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->node->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  RCLCPP_INFO(this->node->get_logger(), "get imu at time: %.6f", stamp2Sec(msg_in->header.stamp));
  sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
  msg->header.stamp = sec2Stamp(stamp2Sec(msg->header.stamp) - imu_time_offset);
  double timestamp = stamp2Sec(msg->header.stamp);

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->node->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = sec2Stamp(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->node->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  {
    RCLCPP_WARN(this->node->get_logger(), "imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
  return img;
}

// static int i = 0;
void LIVMapper::img_cbk(const sensor_msgs::msg::Image::ConstSharedPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image(*msg_in));
  // if ((abs(stamp2Sec(msg->header.stamp) - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->node->get_logger(), "img jumps %.3f\n", stamp2Sec(msg->header.stamp) - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  stamp2Sec(msg->header.stamp);
  double msg_header_time = stamp2Sec(msg->header.stamp) + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  RCLCPP_INFO(this->node->get_logger(), "Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    RCLCPP_ERROR(this->node->get_logger(), "image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    RCLCPP_WARN(this->node->get_logger(), "Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = stamp2Sec(imu_buffer.back()->header.stamp);

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        RCLCPP_ERROR(this->node->get_logger(), "[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // RCLCPP_ERROR(this->node->get_logger(), "lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (stamp2Sec(imu_buffer.front()->header.stamp) > m.lio_time) break;

        if (stamp2Sec(imu_buffer.front()->header.stamp) > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // stamp2Sec(imu_buffer.front()->header.stamp));
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = stamp2Sec(imu_buffer.front()->header.stamp);

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = stamp2Sec(imu_buffer.front()->header.stamp);
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   stamp2Sec(imu_buffer.front()->header.stamp));
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->node->get_logger(), "out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->node->get_clock()->now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  if (slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO)
  {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  if (slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO)
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = this->node->get_clock()->now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = this->node->get_clock()->now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap->publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->node->get_clock()->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = this->node->get_clock()->now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  br = std::make_shared<tf2_ros::TransformBroadcaster>(this->node);
  tf2::Transform transform;
  tf2::Quaternion q;
  transform.setOrigin(tf2::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br->sendTransform(geometry_msgs::msg::TransformStamped(createTransformStamped(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped")));
  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = this->node->get_clock()->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}
~~~

## 6.11 构建 launch 文件

文件位置：/home/liu/Desktop/ROS2/src/FAST-LIVO2/launch/mapping_sim_x500_plus.launch.py

文件：/home/liu/Desktop/ROS2/src/FAST-LIVO2/launch/mapping_sim_x500_plus.launch.py

~~~ python
#!/usr/bin/python3
# FAST-LIVO2 for gz-sim x500_plus (Livox MID360S + RealSense D455F)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_livo")

    lidar_config_cmd = os.path.join(pkg_share, "config", "sim_x500_plus.yaml")
    camera_config_cmd = os.path.join(pkg_share, "config", "camera_x500_plus.yaml")
    rviz_config_cmd = os.path.join(pkg_share, "rviz_cfg", "fast_livo2.rviz")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Launch RViz2 with the bundled FAST-LIVO2 config (fast_livo2.rviz)",
    )
    lidar_config_arg = DeclareLaunchArgument(
        "lidar_params_file",
        default_value=lidar_config_cmd,
        description="Full path to the FAST-LIVO2 parameters file",
    )
    camera_config_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_config_cmd,
        description="Full path to the camera model parameters file",
    )

    lidar_params_file = LaunchConfiguration("lidar_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        use_rviz_arg,
        lidar_config_arg,
        camera_config_arg,

        # camera params are loaded into the same node (no demo_nodes_cpp
        # parameter_blackboard required; vikit getRemoteParam finds them locally)
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                lidar_params_file,
                camera_params_file,
                {"use_sim_time": True},
            ],
            output="screen",
        ),

        # optional visualization: ros2 launch fast_livo mapping_sim_x500_plus.launch.py use_rviz:=true
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_cmd],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_rviz),
        ),
    ])

~~~

## 6.12 构建坐标转换节点

文件位置：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/livo_to_px4.py
文件位置：/home/liu/Desktop/ROS2/src/x500_plus/package.xml
文件位置：/home/liu/Desktop/ROS2/src/x500_plus/setup.py
文件位置：/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py

文件：/home/liu/Desktop/ROS2/src/x500_plus/x500_plus/livo_to_px4.py

~~~python
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

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/package.xml

~~~xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>x500_plus</name>
  <version>0.1.0</version>
  <description>x500_plus gz-sim to ROS2 bridge: RealSense D455F + Livox MID-360S sensors, TF and depth image fix.</description>
  <maintainer email="liu@todo.todo">liu</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>px4_msgs</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ament_index_python</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/setup.py

~~~python
import os
from glob import glob
from setuptools import setup

package_name = 'x500_plus'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu',
    maintainer_email='liu@todo.todo',
    description='x500_plus gz-sim to ROS2 bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = x500_plus.odom_tf_broadcaster:main',
            'depth_fixer = x500_plus.depth_fixer:main',
            'wind_injector = x500_plus.wind_injector:main',
            'livo_to_px4 = x500_plus.livo_to_px4:main',
        ],
    },
)

~~~

文件：/home/liu/Desktop/ROS2/src/x500_plus/launch/x500_plus.launch.py

~~~python
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

~~~

## 6.13 安装 ROS2 包

这里可以将前面安装的 ROS2 包都删除，重新统一构建一下。

~~~bash
cd /home/liu/Desktop/ROS2
colcon build --base-paths src --symlink-install
~~~

## 6.14 启动 FAST-LIVO2

~~~bash
# 算法
ros2 launch fast_livo mapping_sim_x500_plus.launch.py

# 算法 + rviz
ros2 launch fast_livo mapping_sim_x500_plus.launch.py use_rviz:=true
~~~

## 6.15 FAST-LIVO2 算法离线测试

下载 rosbag 文件：
https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA

将 .bag 文件下载至 /home/liu/Desktop/ROS2/data/

转换 rosbag 文件：

~~~bash
pip install rosbags
rosbags-convert --src Red_Sculpture.bag --dst /home/liu/Desktop/ROS2/data/Red_Sculpture
~~~

修改 metadata.yaml 文件：（主要是：offered_qos_profiles: ''，type: livox_ros_driver2/msg/CustomMsg）

~~~yaml
rosbag2_bagfile_information:
  compression_format: ''
  compression_mode: ''
  custom_data: null
  duration:
    nanoseconds: 101866084793
  files:
  - duration:
      nanoseconds: 101866084793
    message_count: 22839
    path: Red_Sculpture.db3
    starting_time:
      nanoseconds_since_epoch: 1701587194733379123
  message_count: 22839
  relative_file_paths:
  - Red_Sculpture.db3
  ros_distro: rosbags
  starting_time:
    nanoseconds_since_epoch: 1701587194733379123
  storage_identifier: sqlite3
  topics_with_message_count:
  - message_count: 20799
    topic_metadata:
      name: /livox/imu
      offered_qos_profiles: ''
      serialization_format: cdr
      type: sensor_msgs/msg/Imu
      type_description_hash: 
        RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b
  - message_count: 1020
    topic_metadata:
      name: /livox/lidar
      offered_qos_profiles: ''
      serialization_format: cdr
      type: livox_ros_driver2/msg/CustomMsg
      type_description_hash: 
        RIHS01_94041b4794f52c1d81def2989107fc898a62dacb7a39d5dbe80d4b55e538bf6d
  - message_count: 1020
    topic_metadata:
      name: /left_camera/image
      offered_qos_profiles: ''
      serialization_format: cdr
      type: sensor_msgs/msg/Image
      type_description_hash: 
        RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47
  version: 9

~~~

播放 rosbag 文件：

~~~bash
ros2 bag play /home/liu/Desktop/ROS2/data/Red_Sculpture  # 立即播放
ros2 bag play -p /home/liu/Desktop/ROS2/data/Red_Sculpture  # 暂停启动，按空格开始
~~~

开启 FAST-LIVO2

~~~bash
ros2 run fast_livo fastlivo_mapping --ros-args --params-file /home/liu/Desktop/ROS2/install/fast_livo/share/fast_livo/config/avia.yaml --params-file /home/liu/Desktop/ROS2/install/fast_livo/share/fast_livo/config/camera_pinhole.yaml
~~~

开启可视化：

~~~bash
rviz2 -d /home/liu/Desktop/ROS2/install/fast_livo/share/fast_livo/rviz_cfg/fast_livo2.rviz
~~~

## 6.16 FAST-LIVO2 算法仿真测试

编译本地 ROS2 包：

~~~bash
cd /home/liu/Desktop/ROS2 && colcon build --base-paths src --symlink-install
~~~

开启 QGC 地面站：

~~~bash
cd /home/liu/Desktop/ROS2/QGroundControl && ./QGroundControl-x86_64.AppImage
~~~

开启 SITL 仿真：

~~~bash
cd /home/liu/Desktop/ROS2/PX4-Autopilot && PX4_GZ_WORLD=Penglai PX4_GZ_MODEL_POSE="0,-8,0,0,0,0" make px4_sitl gz_x500_plus
~~~

开启 Micro-XRCE-DDS-Agent 代理：

~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

开启 ROS2 消息桥接：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash
ros2 launch x500_plus x500_plus.launch.py
~~~

启动 FAST-LIVO2：

~~~bash
source /home/liu/Desktop/ROS2/install/setup.bash

# 算法
ros2 launch fast_livo mapping_sim_x500_plus.launch.py

# 算法 + rviz
ros2 launch fast_livo mapping_sim_x500_plus.launch.py use_rviz:=true
~~~
