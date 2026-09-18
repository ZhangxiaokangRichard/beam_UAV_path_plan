"""uav_viz_node + rviz2 启动（P7 可视化）。

用法:
  ros2 launch uav_guide_env viz.launch.py
  ros2 launch uav_guide_env viz.launch.py rviz:=false            # 只起 Marker 节点
  ros2 launch uav_guide_env viz.launch.py config:=<abs path>.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("uav_guide_env")
    default_config = os.path.join(share, "config", "viz.yaml")
    default_rviz = os.path.join(share, "config", "rviz", "beam_dubins_3d.rviz")

    config_arg = DeclareLaunchArgument("config", default_value=default_config,
                                      description="viz 参数文件")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true",
                                     description="是否同时启动 rviz2")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=default_rviz,
                                            description="rviz2 配置文件")

    viz_node = Node(
        package="uav_guide_env",
        executable="uav_viz_node",
        name="uav_viz_node",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([config_arg, rviz_arg, rviz_config_arg, viz_node, rviz])
