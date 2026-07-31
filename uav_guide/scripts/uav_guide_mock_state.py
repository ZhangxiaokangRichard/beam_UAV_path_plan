#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地测试用 mock：发布 /uav/state 与 /target/state。"""

from __future__ import annotations

import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


def make_odom(x, y, z, yaw, frame_id="map", child="base"):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    q = quaternion_from_euler(0.0, 0.0, yaw)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    msg.twist.twist.linear.x = 10.0
    return msg


def main():
    rospy.init_node("uav_guide_mock_state")
    rate_hz = float(rospy.get_param("~rate_hz", 2.0))
    intercept_mode = rospy.get_param("~intercept_mode", "head-on")

    uav_x = float(rospy.get_param("~uav_x", 1000.0))
    uav_y = float(rospy.get_param("~uav_y", 1000.0))
    uav_z = float(rospy.get_param("~uav_z", 500.0))
    uav_yaw = float(rospy.get_param("~uav_yaw", 0.0))
    uav_speed = float(rospy.get_param("~uav_speed", 10.0))

    tgt_x = float(rospy.get_param("~target_x", 4500.0))
    tgt_y = float(rospy.get_param("~target_y", 3000.0))
    tgt_z = float(rospy.get_param("~target_z", 600.0))
    tgt_yaw = float(rospy.get_param("~target_yaw", math.pi / 2))
    tgt_speed = float(rospy.get_param("~target_speed", 2.0))

    pub_uav = rospy.Publisher("/uav/state", Odometry, queue_size=1)
    pub_tgt = rospy.Publisher("/target/state", Odometry, queue_size=1)

    dt = 1.0 / max(rate_hz, 0.1)
    rate = rospy.Rate(rate_hz)
    rospy.loginfo(
        "[uav_guide_mock_state] rate=%.1f Hz mode=%s uav=(%.0f,%.0f) target=(%.0f,%.0f)",
        rate_hz,
        intercept_mode,
        uav_x,
        uav_y,
        tgt_x,
        tgt_y,
    )

    while not rospy.is_shutdown():
        uav_x += uav_speed * math.cos(uav_yaw) * dt
        uav_y += uav_speed * math.sin(uav_yaw) * dt
        tgt_x += tgt_speed * math.cos(tgt_yaw) * dt
        tgt_y += tgt_speed * math.sin(tgt_yaw) * dt

        pub_uav.publish(make_odom(uav_x, uav_y, uav_z, uav_yaw, child="uav"))
        pub_tgt.publish(make_odom(tgt_x, tgt_y, tgt_z, tgt_yaw, child="target"))
        rate.sleep()


if __name__ == "__main__":
    main()
