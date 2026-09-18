"""mqtt_control_bridge 启动（ROS 制导 → MQTT 出站，按模式截断）。

用法:
  ros2 launch ros_mqtt_bridge mqtt_control_bridge.launch.py
  # 安全联调（出站落到 beam_test/*，不碰真机话题）
  ros2 launch ros_mqtt_bridge mqtt_control_bridge.launch.py config:=<share>/config/mqtt_control_bridge_test.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ros_mqtt_bridge")
    default_config = os.path.join(share, "config", "mqtt_control_bridge.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config, description="参数文件路径"
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run", default_value="false", description="true = 只打印不外发"
    )

    control_bridge = Node(
        package="ros_mqtt_bridge",
        executable="mqtt_control_bridge",
        name="mqtt_control_bridge",
        output="screen",
        parameters=[LaunchConfiguration("config"), {"mqtt.dry_run": LaunchConfiguration("dry_run")}],
    )

    return LaunchDescription([config_arg, dry_run_arg, control_bridge])
