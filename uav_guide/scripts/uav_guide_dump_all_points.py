#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印当前规划路径中的全部转弯圆心与 finalshot（测试用）。"""

from __future__ import annotations

import rospy
import rospkg
import yaml

from nav_msgs.msg import Odometry
from uav_guide.field_extractor import extract_state_5d
from uav_guide.geodesy import Geodesy
from uav_guide.guide_point_builder import GuidePointBuilder
from uav_guide.msg import UavPlannedPath
from uav_guide.path_segmenter import arc_center_xy, segment_path
from uav_guide.state_adapter import StateAdapter


def main():
    rospy.init_node("uav_guide_dump_all_points")
    pkg = rospkg.RosPack().get_path("uav_guide")
    with open(pkg + "/config/uav_guide.yaml", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    lat = rospy.get_param("/geodesy/origin_latitude_deg", cfg["geodesy"]["origin_latitude_deg"])
    lon = rospy.get_param("/geodesy/origin_longitude_deg", cfg["geodesy"]["origin_longitude_deg"])
    alt0 = rospy.get_param("/geodesy/origin_altitude_m", 0.0)
    builder = GuidePointBuilder(cfg, Geodesy(lat, lon, alt0))
    fields = StateAdapter._default_odometry_fields()
    mode = rospy.get_param("/uav_guide/intercept_mode", "head-on")

    rospy.loginfo("等待 /uav/planned_path ...")
    path_msg = rospy.wait_for_message("/uav/planned_path", UavPlannedPath, timeout=60)
    target_msg = rospy.wait_for_message("/target/state", Odometry, timeout=10)
    target = extract_state_5d(fields, target_msg)
    target_alt = builder._target_altitude_wgs84(target)

    print("\n========== 全部引导点 ==========")
    if path_msg.success and path_msg.path:
        arc_idx = 0
        for seg in segment_path(path_msg.path):
            if seg.kind != "arc":
                continue
            p0 = seg.points[0]
            cx, cy = arc_center_xy(
                float(p0.x), float(p0.y), float(p0.yaw), seg.radius_m, seg.turn_direction
            )
            geo = builder._geodesy.to_geodetic(cx, cy, float(p0.z))
            r = seg.radius_m if seg.turn_direction == "R" else -seg.radius_m
            print(
                f"[转弯 {arc_idx}] lon={geo.longitude_deg:.8f} lat={geo.latitude_deg:.8f} "
                f"alt={target_alt:.2f} radius={r:.1f}"
            )
            arc_idx += 1
    else:
        print("[无有效路径]")

    lon, lat, alt, r = builder.build_offset_guide(target, mode)
    print(f"[offset_guide_750m] lon={lon:.8f} lat={lat:.8f} alt={alt:.2f} radius={r:.1f}")
    print("================================\n")


if __name__ == "__main__":
    main()
