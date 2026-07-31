#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PathPoint 序列 → 直线 / 圆弧段。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, List, Sequence, Tuple

from uav_guide.field_extractor import wrap_angle


@dataclass
class LineSegment:
    points: List[Any]
    start_index: int
    end_index: int

    @property
    def kind(self) -> str:
        return "line"


@dataclass
class ArcSegment:
    points: List[Any]
    turn_direction: str
    radius_m: float
    start_index: int
    end_index: int

    @property
    def kind(self) -> str:
        return "arc"


def _point_mode(index: int, points: Sequence[Any], curvature_eps: float, yaw_eps: float) -> str:
    p = points[index]
    k = abs(float(getattr(p, "curvature", 0.0)))
    if k > curvature_eps:
        return "arc"
    if index > 0:
        dyaw = abs(wrap_angle(float(p.yaw) - float(points[index - 1].yaw)))
        if dyaw > yaw_eps:
            return "arc"
    if index + 1 < len(points):
        dyaw = abs(wrap_angle(float(points[index + 1].yaw) - float(p.yaw)))
        if dyaw > yaw_eps:
            return "arc"
    return "line"


def _infer_turn_direction(points: Sequence[Any]) -> str:
    if len(points) < 2:
        return "R"
    dyaw = wrap_angle(float(points[-1].yaw) - float(points[0].yaw))
    return "L" if dyaw > 0 else "R"


def _mean_radius(points: Sequence[Any], default_radius: float, curvature_eps: float) -> float:
    vals = []
    for p in points:
        k = float(getattr(p, "curvature", 0.0))
        if abs(k) > curvature_eps:
            vals.append(1.0 / k)
    if not vals:
        return default_radius
    return abs(sum(vals) / len(vals))


def segment_path(
    path_points: Sequence[Any],
    curvature_eps: float = 1e-4,
    default_radius_m: float = 500.0,
    yaw_eps: float = 0.03,
) -> List[Any]:
    if not path_points:
        return []

    segments = []
    i = 0
    n = len(path_points)
    while i < n:
        mode = _point_mode(i, path_points, curvature_eps, yaw_eps)
        j = i + 1
        while j < n and _point_mode(j, path_points, curvature_eps, yaw_eps) == mode:
            j += 1
        chunk = list(path_points[i:j])
        if mode == "line":
            segments.append(LineSegment(points=chunk, start_index=i, end_index=j - 1))
        else:
            turn = _infer_turn_direction(chunk)
            radius = _mean_radius(chunk, default_radius_m, curvature_eps)
            segments.append(
                ArcSegment(
                    points=chunk,
                    turn_direction=turn,
                    radius_m=radius,
                    start_index=i,
                    end_index=j - 1,
                )
            )
        i = j
    return segments


def arc_center_xy(x: float, y: float, yaw: float, radius_m: float, turn_direction: str) -> Tuple[float, float]:
    if turn_direction == "L":
        cx = x - radius_m * math.sin(yaw)
        cy = y + radius_m * math.cos(yaw)
    else:
        cx = x + radius_m * math.sin(yaw)
        cy = y - radius_m * math.cos(yaw)
    return cx, cy


def closest_path_index(uav_xy: Sequence[float], path_points: Sequence[Any]) -> int:
    best_i = 0
    best_d = float("inf")
    for i, p in enumerate(path_points):
        dx = float(p.x) - uav_xy[0]
        dy = float(p.y) - uav_xy[1]
        d = math.hypot(dx, dy)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def segment_at_index(segments: Sequence[Any], path_index: int):
    for seg in segments:
        if seg.start_index <= path_index <= seg.end_index:
            return seg
    return segments[-1] if segments else None


def next_arc_segment(segments: Sequence[Any], after_index: int):
    for seg in segments:
        if seg.kind == "arc" and seg.start_index > after_index:
            return seg
    for seg in segments:
        if seg.kind == "arc" and seg.start_index >= after_index:
            return seg
    return None


def arc_segment_at_index(segments: Sequence[Any], path_index: int):
    seg = segment_at_index(segments, path_index)
    if seg is not None and seg.kind == "arc":
        return seg
    return next_arc_segment(segments, path_index)


def is_on_final_straight(segments: Sequence[Any], path_index: int) -> bool:
    """末段直线：当前在直线段且之后无弧段。"""
    seg = segment_at_index(segments, path_index)
    if seg is None or seg.kind != "line":
        return False
    return next_arc_segment(segments, path_index) is None
