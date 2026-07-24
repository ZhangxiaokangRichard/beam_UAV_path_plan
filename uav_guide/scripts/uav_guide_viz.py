#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 /uav/guide_point 与 /uav/planned_path，打印摘要。"""

from __future__ import annotations

import rospy

from uav_guide.msg import UavGuidePoint, UavPlannedPath


def main():
    rospy.init_node("uav_guide_viz")

    def guide_cb(msg: UavGuidePoint):
        rospy.loginfo_throttle(
            2.0,
            "[guide] lon=%.6f lat=%.6f alt=%.1f radius=%.1f",
            msg.target_longitude,
            msg.target_latitude,
            msg.target_altitude,
            msg.target_radius,
        )

    def path_cb(msg: UavPlannedPath):
        if msg.success:
            rospy.loginfo_throttle(2.0, "[path] pts=%d cost=%.1f", len(msg.path), msg.cost)

    rospy.Subscriber("/uav/guide_point", UavGuidePoint, guide_cb, queue_size=1)
    rospy.Subscriber("/uav/planned_path", UavPlannedPath, path_cb, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
