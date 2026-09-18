"""Beam Dubins 规划服务启动文件（ROS2 Humble）。

等价于 1.0.1 的 launch/planner.launch：
  - 加载 config/beam_dubins.yaml 到 planner_server 节点参数
  - 启动 planner_server（提供 aoa/beam_dubins/plan_path 服务）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(
        get_package_share_directory("beam_dubins"), "config", "beam_dubins.yaml"
    )

    planner_server = Node(
        package="beam_dubins",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription([planner_server])
