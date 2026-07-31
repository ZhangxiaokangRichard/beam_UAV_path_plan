#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
引导点发布：5 Hz 发布 /uav/guide_point。
引导点仅按路径几何：远弧圆心或 750m 偏移（与 APPROACH/TRACKING 无关）。
"""

from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from uav_guide.field_extractor import extract_state_5d
from uav_guide.geodesy import Geodesy
from uav_guide.guide_point_builder import GuidePointBuilder
from uav_guide.intercept_mode_manager import normalize_intercept_mode
from uav_guide.msg import UavGuidePoint, UavPlannedPath
from uav_guide.state_adapter import StateAdapter


class UavGuidePointNode:
    def __init__(self):
        rospy.init_node("uav_guide_point")
        self._config = rospy.get_param("~", {})

        origin_lat = rospy.get_param("/geodesy/origin_latitude_deg", 0.0)
        origin_lon = rospy.get_param("/geodesy/origin_longitude_deg", 0.0)
        origin_alt = rospy.get_param("/geodesy/origin_altitude_m", 0.0)
        if abs(origin_lat) < 1e-12 and abs(origin_lon) < 1e-12:
            geo_cfg = self._config.get("geodesy", {})
            origin_lat = float(geo_cfg.get("origin_latitude_deg", 0.0))
            origin_lon = float(geo_cfg.get("origin_longitude_deg", 0.0))
            origin_alt = float(geo_cfg.get("origin_altitude_m", 0.0))

        self._geodesy = Geodesy(origin_lat, origin_lon, origin_alt)
        self._builder = GuidePointBuilder(self._config, self._geodesy)

        gp_cfg = self._config.get("guide_point", {})
        self._publish_rate = float(gp_cfg.get("publish_rate_hz", 5.0))
        self._guide_topic = gp_cfg.get("topic", "/uav/guide_point")
        self._pub = rospy.Publisher(self._guide_topic, UavGuidePoint, queue_size=1)

        self._planned_path = None
        self._uav_state = None
        self._target_state = None
        self._intercept_mode = normalize_intercept_mode(
            self._config.get("intercept_mode", "head-on")
        )

        status_topic = str(
            self._config.get("intercept_mode_status_topic", "/uav_guide/intercept_mode")
        ).strip()
        rospy.Subscriber(status_topic, String, self._intercept_mode_cb, queue_size=1)

        fields = self._config.get("inputs", {}).get(
            "odometry_fields", StateAdapter._default_odometry_fields()
        )
        self._fields = fields

        path_topic = self._config.get("output", {}).get("planned_path_topic", "/uav/planned_path")
        rospy.Subscriber(path_topic, UavPlannedPath, self._planned_path_cb, queue_size=1)
        rospy.Subscriber(
            self._config.get("inputs", {}).get("uav_topic", "/uav/state"),
            Odometry,
            self._uav_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            self._config.get("inputs", {}).get("target_topic", "/target/state"),
            Odometry,
            self._target_cb,
            queue_size=10,
        )

        rospy.loginfo("[uav_guide_point] publishing %s at %.1f Hz", self._guide_topic, self._publish_rate)

    def _planned_path_cb(self, msg: UavPlannedPath) -> None:
        if msg.success and msg.path:
            self._planned_path = list(msg.path)

    def _uav_cb(self, msg: Odometry) -> None:
        try:
            self._uav_state = extract_state_5d(self._fields, msg)
        except Exception:
            pass

    def _target_cb(self, msg: Odometry) -> None:
        try:
            self._target_state = extract_state_5d(self._fields, msg)
        except Exception:
            pass

    def _intercept_mode_cb(self, msg: String) -> None:
        self._intercept_mode = normalize_intercept_mode(msg.data)

    def spin(self) -> None:
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            self._publish_guide_point()
            rate.sleep()

    def _publish_guide_point(self) -> None:
        if self._target_state is None:
            return

        uav = self._uav_state
        target = self._target_state
        mode = self._intercept_mode
        path = self._planned_path or []

        lon, lat, alt, radius = self._builder.build_guide_point(path, uav, target, mode)

        msg = UavGuidePoint()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "wgs84"
        msg.target_longitude = lon
        msg.target_latitude = lat
        msg.target_altitude = alt
        msg.target_radius = radius
        self._pub.publish(msg)


def main():
    try:
        node = UavGuidePointNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
