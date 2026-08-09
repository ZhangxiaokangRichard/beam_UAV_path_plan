#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV 引导主循环节点（真实模式）：复用 guide_core.GuideCore 引导主循环。

与 simulation_loop（仿真模式）共用同一套主循环逻辑（发射/规划/DWA/state_update），
唯一区别是 uav 状态的输出端：
  - 真实模式（本节点）：发布 uav/uav_state（nav_msgs/Odometry，与 /target/state 同接口）
    → udp_sender 订阅并编码 JSON-UDP → 发送到真实外部
  - 仿真模式（simulation_loop）：广播 TF + 航迹 → rviz 仿真

启动：由 uav_guide.launch 的 mode=real 参数二选一启动（默认）。
"""

from __future__ import annotations

import math

import rospy
from nav_msgs.msg import Odometry
import tf

from uav_guide.guide_core import GuideCore


class UavGuideLoopNode:
    def __init__(self):
        rospy.init_node("uav_guide_loop")
        config = rospy.get_param("~", {})

        output = config.get("output", {})
        self._uav_state_topic = output.get("uav_state_topic", "/uav/state")
        self._speed_mps = float(config.get("dynamics", {}).get("speed_mps", 50.0))
        self._uav_state_pub = rospy.Publisher(self._uav_state_topic, Odometry, queue_size=10)

        # 引导主循环核心（发射/规划/DWA/state_update，与仿真节点共用）
        self._core = GuideCore(config, on_state=self._on_state)
        rospy.loginfo("[uav_guide_loop] real 模式：uav_state=%s → udp_sender → 真实外部",
                      self._uav_state_topic)

    # ── GuideCore 状态回调：发布 uav/uav_state（与 target/state 同接口） ──

    def _on_state(self, state) -> None:
        x, y, z, yaw, pitch, roll = state
        o = Odometry()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = "map"
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        o.pose.pose.position.z = z
        # 完整姿态四元数（roll/pitch/yaw，与 target/state 同接口）
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        o.pose.pose.orientation.x = q[0]
        o.pose.pose.orientation.y = q[1]
        o.pose.pose.orientation.z = q[2]
        o.pose.pose.orientation.w = q[3]
        o.twist.twist.linear.x = self._speed_mps * math.cos(yaw)
        o.twist.twist.linear.y = self._speed_mps * math.sin(yaw)
        self._uav_state_pub.publish(o)


def main():
    try:
        UavGuideLoopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
