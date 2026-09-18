"""mqtt_target_bridge 启动（MQTT → ROS2 位姿桥）。

用法:
  ros2 launch ros_mqtt_bridge mqtt_bridge.launch.py
  ros2 launch ros_mqtt_bridge mqtt_bridge.launch.py config:=<abs path>.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ros_mqtt_bridge")
    default_config = os.path.join(share, "config", "mqtt_bridge.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config, description="参数文件路径"
    )

    target_bridge = Node(
        package="ros_mqtt_bridge",
        executable="mqtt_target_bridge",
        name="mqtt_target_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, target_bridge])
