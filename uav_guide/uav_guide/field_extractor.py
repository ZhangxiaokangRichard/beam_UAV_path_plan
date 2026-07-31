#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 ROS 消息对象按点路径提取字段值。"""

from __future__ import annotations

import math
from typing import Any, Dict, Union


def wrap_angle(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def get_by_path(obj: Any, path: str) -> Any:
    if not path:
        raise ValueError("empty field path")
    cur = obj
    for part in path.split("."):
        cur = getattr(cur, part)
    return cur


def parse_yaw(spec: Union[float, int, str, Dict[str, Any]], msg: Any) -> float:
    if isinstance(spec, (float, int)):
        return float(spec)
    if isinstance(spec, str):
        return float(get_by_path(msg, spec))
    if not isinstance(spec, dict):
        raise TypeError("unsupported yaw spec: %r" % (spec,))

    source = spec.get("source", "field")
    if source == "field":
        return float(get_by_path(msg, spec["path"]))
    if source == "degrees":
        return math.radians(float(get_by_path(msg, spec["path"])))
    if source == "quaternion":
        qw = float(get_by_path(msg, spec["qw"]))
        qx = float(get_by_path(msg, spec.get("qx", "0")))
        qy = float(get_by_path(msg, spec.get("qy", "0")))
        qz = float(get_by_path(msg, spec["qz"]))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    raise ValueError("unknown yaw source: %s" % source)


def parse_scalar(spec: Union[float, int, str, Dict[str, Any]], msg: Any) -> float:
    if isinstance(spec, (float, int)):
        return float(spec)
    if isinstance(spec, str):
        return float(get_by_path(msg, spec))
    if isinstance(spec, dict):
        source = spec.get("source", "field")
        if source == "field":
            return float(get_by_path(msg, spec["path"]))
        if source == "degrees":
            return math.radians(float(get_by_path(msg, spec["path"])))
    raise TypeError("unsupported scalar spec: %r" % (spec,))


def extract_state_5d(fields_cfg: Dict[str, Any], msg: Any) -> list:
    """提取 [x, y, z, yaw, pitch]。"""
    x = parse_scalar(fields_cfg["x"], msg)
    y = parse_scalar(fields_cfg["y"], msg)
    z = parse_scalar(fields_cfg.get("z", 0.0), msg)
    yaw = parse_yaw(fields_cfg.get("yaw", 0.0), msg)
    pitch = parse_scalar(fields_cfg.get("pitch", 0.0), msg)
    return [x, y, z, yaw, pitch]
