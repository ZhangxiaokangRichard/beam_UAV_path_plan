"""uav_guide 启动文件（ROS2 Humble）：一个 launch 拉起三个节点。

  uav_guide_loop_node   规划（APPROACH/TRACKING → aoa/uav/planed_path）
  uav_guide_point_node  setpoint 制导（→ aoa/uav/setpoint）
  uav_guidance_node     cruise 制导（→ aoa/uav/guidance）

参数 `with_planner`（默认 true）可一并拉起 beam_dubins 的规划服务。
三个节点**互不依赖**：模式截断由 ros_mqtt_bridge 的 mqtt_control_bridge 负责。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(get_package_share_directory("uav_guide"), "config", "uav_guide.yaml")
    planner_launch = os.path.join(
        get_package_share_directory("beam_dubins"), "launch", "planner.launch.py"
    )

    with_planner = LaunchConfiguration("with_planner")

    nodes = [
        Node(
            package="uav_guide",
            executable="uav_guide_loop_node",
            name="uav_guide_loop_node",
            output="screen",
            parameters=[params_file],
            emulate_tty=True,
        ),
        Node(
            package="uav_guide",
            executable="uav_guide_point_node",
            name="uav_guide_point_node",
            output="screen",
            parameters=[params_file],
            emulate_tty=True,
        ),
        Node(
            package="uav_guide",
            executable="uav_guidance_node",
            name="uav_guidance_node",
            output="screen",
            parameters=[params_file],
            emulate_tty=True,
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_planner",
                default_value="true",
                description="是否同时启动 beam_dubins 的 planner_server",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(planner_launch),
                condition=IfCondition(with_planner),
            ),
            *nodes,
        ]
    )
