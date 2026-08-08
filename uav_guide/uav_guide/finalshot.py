#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""末段拦截偏移点计算（finalshot）：按拦截模式（head-on / tail）给出目标前方/后方的 map 系偏移点。

仅保留被 plan_orchestrator 引用的 compute_offset_goal_xy；
原 guide_point 节点（MQTT fly_point 服务）与路径几何引导点（GuidePointBuilder/path_segmenter）已移除。
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple


def compute_offset_goal_xy(target: Sequence[float], intercept_mode: str, offset_m: float, offset_m_t: float) -> Tuple[float, float]:
    """head-on：目标身后；tail：目标前方。"""
    psi = target[3]
    if intercept_mode == "head-on":
        x = target[0] - (offset_m + 0.4 * offset_m_t) * math.cos(psi)
        y = target[1] - (offset_m + 0.4 * offset_m_t) * math.sin(psi)
    else:
        x = target[0] + (offset_m + offset_m_t) * math.cos(psi)
        y = target[1] + (offset_m + offset_m_t) * math.sin(psi)
    return x, y
