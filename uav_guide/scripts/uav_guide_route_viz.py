#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV 引导路径可视化：根据 /uav/state、/target/state、/uav/planned_path 绘制 map 平面航路。

用法（先启动 uav_guide.launch 或 uav_guide_test.launch）：
  rosrun uav_guide uav_guide_route_viz.py

默认保存到 /tmp/uav_guide_route.png（避免 Tk 与 rospy 冲突）。
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional, Sequence, Tuple

import rospy
import rospkg
import yaml
from nav_msgs.msg import Odometry

from uav_guide.field_extractor import extract_state_5d
from uav_guide.geodesy import Geodesy
from uav_guide.guide_point_builder import GuidePointBuilder, compute_offset_goal_xy
from uav_guide.msg import UavGuidePoint, UavPlannedPath
from uav_guide.path_segmenter import (
    ArcSegment,
    LineSegment,
    arc_center_xy,
    closest_path_index,
    segment_path,
)
from uav_guide.state_adapter import StateAdapter


def yaw_arrow(x: float, y: float, yaw: float, length: float = 200.0):
    return x, y, length * math.cos(yaw), length * math.sin(yaw)


class UavGuideRouteViz:
    def __init__(self):
        rospy.init_node("uav_guide_route_viz", anonymous=True)
        cfg = rospy.get_param("~", {})

        pkg = rospkg.RosPack().get_path("uav_guide")
        with open(pkg + "/config/uav_guide.yaml", encoding="utf-8") as f:
            self._cfg = yaml.safe_load(f)
        self._cfg.update(cfg)

        self._save_path = str(self._cfg.get("save_path", "/tmp/uav_guide_route.png"))
        self._gui = bool(self._cfg.get("gui", False))
        self._update_hz = float(self._cfg.get("update_hz", 1.0))

        lat = rospy.get_param("/geodesy/origin_latitude_deg", self._cfg["geodesy"]["origin_latitude_deg"])
        lon = rospy.get_param("/geodesy/origin_longitude_deg", self._cfg["geodesy"]["origin_longitude_deg"])
        alt0 = rospy.get_param("/geodesy/origin_altitude_m", 0.0)
        self._geodesy = Geodesy(lat, lon, alt0)
        self._builder = GuidePointBuilder(self._cfg, self._geodesy)
        self._fields = StateAdapter._default_odometry_fields()

        self._lock = threading.Lock()
        self._path_msg: Optional[UavPlannedPath] = None
        self._guide_msg: Optional[UavGuidePoint] = None
        self._uav: Optional[List[float]] = None
        self._target: Optional[List[float]] = None
        self._gui_shown = False

        import matplotlib

        if not self._gui:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # noqa: E402
        from matplotlib.patches import Circle  # noqa: E402

        self._plt = plt
        self._Circle = Circle
        self._fig, self._ax = plt.subplots(figsize=(12, 10))

        path_topic = self._cfg.get("output", {}).get("planned_path_topic", "/uav/planned_path")
        guide_topic = self._cfg.get("guide_point", {}).get("topic", "/uav/guide_point")
        uav_topic = self._cfg.get("inputs", {}).get("uav_topic", "/uav/state")
        target_topic = self._cfg.get("inputs", {}).get("target_topic", "/target/state")

        rospy.Subscriber(path_topic, UavPlannedPath, self._path_cb, queue_size=1)
        rospy.Subscriber(guide_topic, UavGuidePoint, self._guide_cb, queue_size=1)
        rospy.Subscriber(uav_topic, Odometry, self._uav_cb, queue_size=1)
        rospy.Subscriber(target_topic, Odometry, self._target_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / max(self._update_hz, 0.1)), self._timer_cb)

        rospy.loginfo("[uav_guide_route_viz] save -> %s (gui=%s)", self._save_path, self._gui)

    def _path_cb(self, msg: UavPlannedPath) -> None:
        if msg.success and msg.path:
            with self._lock:
                self._path_msg = msg

    def _guide_cb(self, msg: UavGuidePoint) -> None:
        with self._lock:
            self._guide_msg = msg

    def _uav_cb(self, msg: Odometry) -> None:
        try:
            self._uav = extract_state_5d(self._fields, msg)
        except Exception:
            pass

    def _target_cb(self, msg: Odometry) -> None:
        try:
            self._target = extract_state_5d(self._fields, msg)
        except Exception:
            pass

    def _timer_cb(self, _event) -> None:
        with self._lock:
            path_msg = self._path_msg
            guide_msg = self._guide_msg
        if self._uav is None or self._target is None:
            return
        if path_msg is None and guide_msg is None:
            return
        if self._gui and self._gui_shown:
            return
        self._draw(path_msg, guide_msg)
        if self._gui and not self._gui_shown:
            self._gui_shown = True
            self._plt.show(block=False)

    def _draw(self, path_msg: Optional[UavPlannedPath], guide_msg: Optional[UavGuidePoint]) -> None:
        ax = self._ax
        ax.clear()

        mode = rospy.get_param("/uav_guide/intercept_mode", "head-on")
        tracking_mode = bool(rospy.get_param("/uav_guide/tracking_mode", False))
        uav = self._uav
        target = self._target
        assert uav is not None and target is not None

        phase = "TRACKING" if tracking_mode else "APPROACH"
        ax.set_title("UAV Guide Route — %s / %s" % (phase, mode))
        ax.set_xlabel("X / East (m)")
        ax.set_ylabel("Y / North (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)

        # UAV / Target
        ax.plot(uav[0], uav[1], "^", color="#2ca02c", markersize=14, label="UAV", zorder=6)
        x, y, dx, dy = yaw_arrow(uav[0], uav[1], uav[3], 250)
        ax.arrow(x, y, dx, dy, head_width=100, head_length=80, fc="#2ca02c", ec="#1a6b1a",
                 length_includes_head=True, zorder=6)

        ax.plot(target[0], target[1], "*", color="#d62728", markersize=18, label="Target", zorder=6)
        x, y, dx, dy = yaw_arrow(target[0], target[1], target[3], 250)
        ax.arrow(x, y, dx, dy, head_width=100, head_length=80, fc="#d62728", ec="#8b0000",
                 length_includes_head=True, zorder=6)

        # 750m 引导点参考
        fs_x, fs_y = compute_offset_goal_xy(target, mode, self._builder._offset_m)
        ax.plot(fs_x, fs_y, "P", color="#9467bd", markersize=12, label="750m guide", zorder=5)
        ax.plot([target[0], fs_x], [target[1], fs_y], ":", color="#9467bd", alpha=0.6)

        path_points = list(path_msg.path) if path_msg and path_msg.path else []
        active_map: Optional[Tuple[float, float]] = None
        active_label = "guide"

        if path_points:
            xs = [p.x for p in path_points]
            ys = [p.y for p in path_points]
            ax.plot(xs, ys, "-", color="#1f77b4", linewidth=2, label="Planned path (%d)" % len(path_points))

            path_ptr = closest_path_index(uav, path_points)
            ax.plot(xs[path_ptr], ys[path_ptr], "o", color="#17becf", markersize=8, label="Path index")

            segments = segment_path(
                path_points,
                curvature_eps=self._builder._curvature_eps,
                default_radius_m=self._builder._default_radius_m,
                yaw_eps=self._builder._yaw_eps,
            )

            arc_n = 0
            for seg in segments:
                if isinstance(seg, LineSegment):
                    st, en = seg.points[0], seg.points[-1]
                    ax.plot([st.x, en.x], [st.y, en.y], "--", color="#bcbd22", alpha=0.5, linewidth=1)
                elif isinstance(seg, ArcSegment):
                    arc_n += 1
                    p0 = seg.points[0]
                    cx, cy = arc_center_xy(float(p0.x), float(p0.y), float(p0.yaw), seg.radius_m, seg.turn_direction)
                    r_sign = seg.radius_m if seg.turn_direction == "R" else -seg.radius_m
                    ax.add_patch(
                        self._Circle((cx, cy), seg.radius_m, fill=False, linewidth=1.5,
                                     edgecolor="#ff7f0e", alpha=0.7)
                    )
                    ax.plot(cx, cy, "x", color="#ff7f0e", markersize=10, markeredgewidth=2)
                    ax.annotate("A%d R=%+.0f" % (arc_n, r_sign), (cx, cy),
                                xytext=(6, 6), textcoords="offset points", fontsize=8, color="#ff7f0e")

            arcs_remaining = [
                seg for seg in segments if isinstance(seg, ArcSegment) and seg.end_index >= path_ptr
            ]
            if not arcs_remaining:
                active_map = (fs_x, fs_y)
                active_label = "750m (no arc remaining)"
            else:
                far_arc = max(arcs_remaining, key=lambda seg: seg.start_index)
                p0 = far_arc.points[0]
                active_map = arc_center_xy(
                    float(p0.x), float(p0.y), float(p0.yaw), far_arc.radius_m, far_arc.turn_direction
                )
                active_label = "far arc center"
        elif self._builder.uses_offset_guide(path_points, uav):
            active_map = (fs_x, fs_y)
            active_label = "750m (no arc remaining)"

        if guide_msg is not None:
            gx, gy, _ = self._geodesy.to_local(
                guide_msg.target_latitude, guide_msg.target_longitude, guide_msg.target_altitude
            )
            ax.plot(gx, gy, "D", color="#e377c2", markersize=10,
                    label="/uav/guide_point (R=%.0f)" % guide_msg.target_radius, zorder=7)
            ax.plot([uav[0], gx], [uav[1], gy], "-.", color="#e377c2", alpha=0.7, linewidth=1)

        if active_map is not None:
            ax.plot(active_map[0], active_map[1], "s", color="#000000", markersize=9,
                    label="Active: " + active_label, zorder=7)

        ax.legend(loc="upper left", fontsize=8)
        self._fig.tight_layout()
        self._fig.savefig(self._save_path, dpi=150)
        rospy.loginfo_throttle(3.0, "[uav_guide_route_viz] updated %s", self._save_path)

    def spin(self) -> None:
        rospy.spin()


def main():
    try:
        UavGuideRouteViz().spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
