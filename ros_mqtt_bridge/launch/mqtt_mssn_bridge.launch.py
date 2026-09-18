"""mqtt_mssn_bridge 启动（aoa/ai_guide/* 入站：模式 + 导航栈/控制桥启停）。

用法:
  ros2 launch ros_mqtt_bridge mqtt_mssn_bridge.launch.py
  # 安全联调（beam_test/* 入站 + 不启停任何进程）
  ros2 launch ros_mqtt_bridge mqtt_mssn_bridge.launch.py config:=<share>/config/mqtt_mssn_bridge_test.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ros_mqtt_bridge")
    default_config = os.path.join(share, "config", "mqtt_mssn_bridge.yaml")

    config_arg = DeclareLaunchArgument(
        "config", default_value=default_config, description="参数文件路径"
    )
    process_control_arg = DeclareLaunchArgument(
        "enable_process_control",
        default_value="true",
        description="false = 只转发模式，不启停进程",
    )

    mssn_bridge = Node(
        package="ros_mqtt_bridge",
        executable="mqtt_mssn_bridge",
        name="mqtt_mssn_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {"mssn.enable_process_control": LaunchConfiguration("enable_process_control")},
        ],
    )

    return LaunchDescription([config_arg, process_control_arg, mssn_bridge])
