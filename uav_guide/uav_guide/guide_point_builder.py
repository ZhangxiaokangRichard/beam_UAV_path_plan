#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""引导点构建：按路径几何选最远弧圆心或 750m 偏移点（与 APPROACH/TRACKING 模式无关）。"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Sequence, Tuple

from uav_guide.geodesy import Geodesy
from uav_guide.path_segmenter import (
    ArcSegment,
    arc_center_xy,
    closest_path_index,
    segment_path,
)


def compute_offset_goal_xy(target: Sequence[float], intercept_mode: str, offset_m: float) -> Tuple[float, float]:
    """head-on：目标身后；tail：目标前方。"""
    psi = target[3]
    sign = -1.0 if intercept_mode == "head-on" else 1.0
    x = target[0] + sign * offset_m * math.cos(psi)
    y = target[1] + sign * offset_m * math.sin(psi)
    return x, y


def _signed_radius(turn_direction: str, radius_m: float) -> float:
    return radius_m if turn_direction == "R" else -radius_m


class GuidePointBuilder:
    def __init__(self, config: Dict[str, Any], geodesy: Geodesy):
        seg_cfg = config.get("segmentation", {})
        self._curvature_eps = float(seg_cfg.get("curvature_eps", 1e-4))
        self._yaw_eps = float(seg_cfg.get("yaw_segment_eps", 0.03))
        self._default_radius_m = float(seg_cfg.get("default_turn_radius_m", 500.0))

        fs_cfg = config.get("finalshot", {})
        self._offset_m = float(fs_cfg.get("offset_m", 750.0))
        self._offset_radius_m = float(fs_cfg.get("radius_m", 500.0))

        self._geodesy = geodesy

    def build_guide_point(
        self,
        path_points: Sequence[Any],
        uav_state: Sequence[float],
        target_state: Sequence[float],
        intercept_mode: str,
    ) -> Tuple[float, float, float, float]:
        """
        返回 (longitude, latitude, altitude, radius)。
        有剩余弧段 → 最远圆心；无弧/无路径 → 750m 偏移点。
        与 tracking_mode 无关（APPROACH 末段也可发 750m）。
        """
        if uav_state is None:
            return self.build_offset_guide(target_state, intercept_mode)

        return self.build_distant_arc_from_path(path_points, uav_state, target_state, intercept_mode)

    def uses_offset_guide(
        self,
        path_points: Sequence[Any],
        uav_state: Sequence[float],
    ) -> bool:
        """当前几何下是否会输出 750m 偏移引导点。"""
        if uav_state is None or not path_points:
            return True
        segments = segment_path(
            path_points,
            curvature_eps=self._curvature_eps,
            default_radius_m=self._default_radius_m,
            yaw_eps=self._yaw_eps,
        )
        path_ptr = closest_path_index(uav_state, path_points)
        arcs_remaining = [
            seg for seg in segments if isinstance(seg, ArcSegment) and seg.end_index >= path_ptr
        ]
        return not arcs_remaining

    def build_distant_arc_from_path(
        self,
        path_points: Sequence[Any],
        uav_state: Sequence[float],
        target_state: Sequence[float],
        intercept_mode: str,
    ) -> Tuple[float, float, float, float]:
        """
        路径上尚未飞过的弧段中，沿航路最远的一个圆心。
        无路径、全直线或无剩余弧段 → 750m 偏移引导点。
        """
        alt = self._target_altitude_wgs84(target_state)

        if not path_points:
            return self._build_offset_guide(target_state, intercept_mode, alt)

        segments = segment_path(
            path_points,
            curvature_eps=self._curvature_eps,
            default_radius_m=self._default_radius_m,
            yaw_eps=self._yaw_eps,
        )
        path_ptr = closest_path_index(uav_state, path_points)

        arcs_remaining = [
            seg for seg in segments if isinstance(seg, ArcSegment) and seg.end_index >= path_ptr
        ]
        if not arcs_remaining:
            return self._build_offset_guide(target_state, intercept_mode, alt)

        far_arc = max(arcs_remaining, key=lambda seg: seg.start_index)
        return self._build_turn_center(far_arc, alt)

    def build_offset_guide(
        self,
        target_state: Sequence[float],
        intercept_mode: str,
    ) -> Tuple[float, float, float, float]:
        """750m 偏移引导点（head-on 身后 / tail 前方）。"""
        alt = self._target_altitude_wgs84(target_state)
        return self._build_offset_guide(target_state, intercept_mode, alt)

    def _target_altitude_wgs84(self, target_state: Sequence[float]) -> float:
        geo = self._geodesy.to_geodetic(target_state[0], target_state[1], target_state[2])
        return geo.altitude_m

    def _build_offset_guide(
        self,
        target_state: Sequence[float],
        intercept_mode: str,
        alt_wgs84: float,
    ) -> Tuple[float, float, float, float]:
        x, y = compute_offset_goal_xy(target_state, intercept_mode, self._offset_m)
        geo = self._geodesy.to_geodetic(x, y, target_state[2])
        return geo.longitude_deg, geo.latitude_deg, alt_wgs84, self._offset_radius_m

    def _build_turn_center(
        self,
        arc: ArcSegment,
        alt_wgs84: float,
    ) -> Tuple[float, float, float, float]:
        p0 = arc.points[0]
        cx, cy = arc_center_xy(float(p0.x), float(p0.y), float(p0.yaw), arc.radius_m, arc.turn_direction)
        geo = self._geodesy.to_geodetic(cx, cy, float(p0.z))
        radius = _signed_radius(arc.turn_direction, arc.radius_m)
        return geo.longitude_deg, geo.latitude_deg, alt_wgs84, radius
