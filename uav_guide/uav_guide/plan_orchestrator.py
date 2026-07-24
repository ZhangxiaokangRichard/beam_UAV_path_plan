#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""规划调度：虚拟目标（APPROACH）/ TRACKING（直飞目标本体）。"""

from __future__ import annotations

import math
from typing import Any, Callable, Dict, List, Optional

import rospy
from beam_dubins.srv import PlanPath, PlanPathRequest

from uav_guide.field_extractor import wrap_angle
from uav_guide.geodesy import Geodesy
from uav_guide.guide_point_builder import compute_offset_goal_xy
from uav_guide.intercept_mode_manager import normalize_intercept_mode


def dist2d(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def normalize_yaw(yaw: float) -> float:
    return wrap_angle(yaw)


def expected_intercept_yaw(target_state: List[float], intercept_mode: str) -> float:
    psi = target_state[3]
    if normalize_intercept_mode(intercept_mode) == "head-on":
        return normalize_yaw(psi + math.pi)
    return psi


class PlanOrchestrator:
    def __init__(self, config: Dict[str, Any], on_planned: Callable[[Any], None]):
        self._config = config
        self._on_planned = on_planned

        planning = config.get("planning", {})
        self.plan_service = planning.get("service", "/beam_dubins/plan_path")

        target_cfg = config.get("target", {})
        self.approach_L_tail = float(target_cfg.get("approach_distance_tail", 1000.0))
        self.approach_L_headon = float(target_cfg.get("approach_distance_headon", 1000.0))
        self.tracking_entry_dist = float(target_cfg.get("tracking_entry_dist", 2000.0))
        self.tracking_window = float(target_cfg.get("tracking_window", 100.0))
        self.divergence_threshold = int(target_cfg.get("divergence_threshold", 3))
        self.velocity_align_rad = math.radians(float(target_cfg.get("velocity_align_deg", 20.0)))
        self.min_closure_m = float(target_cfg.get("min_closure_m", 5.0))
        self.min_closure_m_tail = float(target_cfg.get("min_closure_m_tail", 1.5))
        self.entry_hold_count = int(target_cfg.get("entry_hold_count", 2))

        fs_cfg = config.get("finalshot", {})
        self._guide_offset_m = float(fs_cfg.get("offset_m", 750.0))
        self._guide_radius_m = float(fs_cfg.get("radius_m", 500.0))
        self._geo_cfg = config.get("geodesy", {})

        space = config.get("space", {})
        self.space_lx = float(space.get("lx", 10000.0))
        self.space_ly = float(space.get("ly", 5000.0))
        self.space_lz = float(space.get("lz", 1000.0))
        self.boundary_margin = float(space.get("boundary_margin", 50.0))

        self.tracking_mode = False
        self.tracking_distance_accum = 0.0
        self.last_tracking_dist = float("inf")
        self.consecutive_diverging = 0
        self._entry_hold = 0
        self._last_d_to_target: Optional[float] = None
        self._last_uav = None
        self._last_path: List[Any] = []

        self._client = rospy.ServiceProxy(self.plan_service, PlanPath)
        rospy.loginfo("[plan_orchestrator] waiting for %s ...", self.plan_service)
        self._client.wait_for_service(timeout=rospy.Duration(30.0))
        rospy.set_param("/uav_guide/tracking_mode", False)

    def _geodesy(self) -> Geodesy:
        lat = rospy.get_param(
            "/geodesy/origin_latitude_deg",
            float(self._geo_cfg.get("origin_latitude_deg", 0.0)),
        )
        lon = rospy.get_param(
            "/geodesy/origin_longitude_deg",
            float(self._geo_cfg.get("origin_longitude_deg", 0.0)),
        )
        alt0 = rospy.get_param(
            "/geodesy/origin_altitude_m",
            float(self._geo_cfg.get("origin_altitude_m", 0.0)),
        )
        return Geodesy(lat, lon, alt0)

    def _tracking_guide_wgs84(
        self,
        target_state: List[float],
        intercept_mode: str,
    ) -> tuple:
        gx, gy = compute_offset_goal_xy(target_state, intercept_mode, self._guide_offset_m)
        geo = self._geodesy().to_geodetic(gx, gy, target_state[2])
        target_geo = self._geodesy().to_geodetic(target_state[0], target_state[1], target_state[2])
        return gx, gy, geo.longitude_deg, geo.latitude_deg, target_geo.altitude_m

    def set_last_path(self, path_points) -> None:
        self._last_path = list(path_points) if path_points else []

    def reset_on_mode_change(self, new_mode: str) -> None:
        was_tracking = self.tracking_mode
        self.tracking_mode = False
        self.tracking_distance_accum = 0.0
        self.last_tracking_dist = float("inf")
        self.consecutive_diverging = 0
        self._entry_hold = 0
        self._last_d_to_target = None
        self._last_uav = None
        self._last_path = []
        rospy.set_param("/uav_guide/tracking_mode", False)
        rospy.loginfo(
            "[plan_orchestrator] intercept mode -> %s, reset planning state%s",
            normalize_intercept_mode(new_mode),
            " (was TRACKING)" if was_tracking else "",
        )

    def plan_on_update(
        self,
        uav_state: List[float],
        target_state: List[float],
        intercept_mode: str,
    ) -> bool:
        if uav_state is None or target_state is None:
            return False

        intercept_mode = normalize_intercept_mode(intercept_mode)
        d_to_target = dist2d(uav_state, target_state)

        if self._last_uav is not None:
            advance = dist2d(uav_state, self._last_uav)
            self._evaluate_tracking_trend(target_state, uav_state, advance)
        self._last_uav = list(uav_state)

        self._evaluate_tracking_entry(
            uav_state, target_state, intercept_mode, d_to_target
        )
        self._last_d_to_target = d_to_target

        return self._request_plan(uav_state, target_state, intercept_mode)

    def _compute_virtual_goal(
        self,
        uav_state: List[float],
        target_state: List[float],
        intercept_mode: str,
    ) -> List[float]:
        approach_L = (
            self.approach_L_headon
            if normalize_intercept_mode(intercept_mode) == "head-on"
            else self.approach_L_tail
        )

        psi = target_state[3]
        sign = 1.0 if normalize_intercept_mode(intercept_mode) == "head-on" else -1.0
        vx = target_state[0] + sign * approach_L * math.cos(psi)
        vy = target_state[1] + sign * approach_L * math.sin(psi)
        vz = target_state[2]
        vyaw = expected_intercept_yaw(target_state, intercept_mode)
        return [vx, vy, vz, vyaw, target_state[4]]

    def _request_plan(
        self,
        uav_state: List[float],
        target_state: List[float],
        intercept_mode: str,
    ) -> bool:
        if self.tracking_mode:
            plan_goal = list(target_state)
            plan_goal[3] = expected_intercept_yaw(target_state, intercept_mode)
        else:
            plan_goal = self._compute_virtual_goal(uav_state, target_state, intercept_mode)

        req = PlanPathRequest()
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = self._config.get("planning", {}).get("frame_id", "map")
        req.start = list(uav_state[:5])
        req.goal = list(plan_goal[:5])
        req.space_lx = self.space_lx
        req.space_ly = self.space_ly
        req.space_lz = self.space_lz
        req.boundary_margin = self.boundary_margin
        req.obstacles = []
        req.time_limit_ms = float(self._config.get("planning", {}).get("time_limit_ms", 0.0))
        req.use_3d = bool(self._config.get("planning", {}).get("use_3d", False))

        try:
            resp = self._client(req)
        except rospy.ServiceException as exc:
            rospy.logerr("[plan_orchestrator] plan service failed: %s", exc)
            return False

        phase = "TRACKING" if self.tracking_mode else "APPROACH"
        if resp.success:
            self.set_last_path(resp.path)
            rospy.loginfo(
                "[plan_orchestrator] plan ok cost=%.1f pts=%d phase=%s intercept=%s",
                resp.cost,
                len(resp.path),
                phase,
                intercept_mode,
            )
        else:
            rospy.logwarn(
                "[plan_orchestrator] plan failed phase=%s: %s",
                phase,
                resp.status_message,
            )

        self._on_planned(resp)
        return resp.success

    def _is_velocity_aligned(
        self,
        uav_state: List[float],
        target_state: List[float],
        intercept_mode: str,
    ) -> float:
        psi_ref = expected_intercept_yaw(target_state, intercept_mode)
        align_err = abs(wrap_angle(uav_state[3] - psi_ref))
        return align_err

    def _is_closing(self, d_to_target: float) -> float:
        if self._last_d_to_target is None:
            return 0.0
        return self._last_d_to_target - d_to_target

    def _min_closure_for_mode(self, intercept_mode: str) -> float:
        if normalize_intercept_mode(intercept_mode) == "tail":
            return self.min_closure_m_tail
        return self.min_closure_m

    def _evaluate_tracking_entry(
        self,
        uav_state: List[float],
        target_state: List[float],
        intercept_mode: str,
        d_to_target: float,
    ) -> None:
        if self.tracking_mode:
            return

        intercept_mode = normalize_intercept_mode(intercept_mode)
        min_closure = self._min_closure_for_mode(intercept_mode)

        if d_to_target >= self.tracking_entry_dist:
            self._entry_hold = 0
            return

        align_err = self._is_velocity_aligned(uav_state, target_state, intercept_mode)
        closure = self._is_closing(d_to_target)
        aligned = align_err <= self.velocity_align_rad
        closing = closure >= min_closure
        psi_ref = expected_intercept_yaw(target_state, intercept_mode)

        rospy.loginfo_throttle(
            2.0,
            "[plan_orchestrator] tracking gate (%s): d=%.0fm align=%.1fdeg(<=%.1f) "
            "uav_yaw=%.1fdeg tgt_yaw=%.1fdeg ref_yaw=%.1fdeg closure=%.1fm(>=%.1f) hold=%d/%d",
            intercept_mode,
            d_to_target,
            math.degrees(align_err),
            math.degrees(self.velocity_align_rad),
            math.degrees(uav_state[3]),
            math.degrees(target_state[3]),
            math.degrees(psi_ref),
            closure,
            min_closure,
            self._entry_hold,
            self.entry_hold_count,
        )

        if aligned and closing:
            self._entry_hold += 1
        else:
            self._entry_hold = 0

        if self._entry_hold < self.entry_hold_count:
            return

        self._enter_tracking(
            target_state,
            d_to_target,
            intercept_mode,
            align_err,
            closure,
        )

    def _enter_tracking(
        self,
        target_state: List[float],
        d_to_target: float,
        intercept_mode: str,
        align_err_rad: float,
        closure_m: float,
    ) -> None:
        self.tracking_mode = True
        self.tracking_distance_accum = 0.0
        self.consecutive_diverging = 0
        self.last_tracking_dist = d_to_target
        self._entry_hold = 0
        rospy.set_param("/uav_guide/tracking_mode", True)

        gx, gy, lon, lat, alt = self._tracking_guide_wgs84(target_state, intercept_mode)
        rospy.loginfo(
            "[plan_orchestrator] >>> 进入 TRACKING 模式 (%s): "
            "共线+接近满足，规划终点切换为目标本体（引导点仍按路径几何，无弧时为750m） | "
            "d=%.0fm align=%.1fdeg closure=%.0fm | "
            "750m参考 map=(%.1f, %.1f) lon=%.8f lat=%.8f alt=%.1f radius=+%.0f",
            intercept_mode,
            d_to_target,
            math.degrees(align_err_rad),
            closure_m,
            gx,
            gy,
            lon,
            lat,
            alt,
            self._guide_radius_m,
        )

    def _exit_tracking(self, d_to_target: float, delta: float, streak: int) -> None:
        self.tracking_mode = False
        self.tracking_distance_accum = 0.0
        self.consecutive_diverging = 0
        self._entry_hold = 0
        self._last_path = []
        rospy.set_param("/uav_guide/tracking_mode", False)
        rospy.logwarn(
            "[plan_orchestrator] MODE TRACKING -> APPROACH (miss) d=%.0fm delta=+%.0fm streak=%d/%d",
            d_to_target,
            delta,
            streak,
            self.divergence_threshold,
        )

    def _evaluate_tracking_trend(
        self,
        target_state: List[float],
        uav_state: List[float],
        advance_dist: float,
    ) -> None:
        if not self.tracking_mode:
            return

        self.tracking_distance_accum += advance_dist
        if self.tracking_distance_accum < self.tracking_window:
            return

        current_dist = dist2d(uav_state, target_state)
        delta = current_dist - self.last_tracking_dist
        if delta > 0:
            self.consecutive_diverging += 1
            rospy.logwarn(
                "[plan_orchestrator] tracking diverging +%.0fm (%d/%d)",
                delta,
                self.consecutive_diverging,
                self.divergence_threshold,
            )
        else:
            self.consecutive_diverging = 0

        self.last_tracking_dist = current_dist
        self.tracking_distance_accum = 0.0

        if self.consecutive_diverging >= self.divergence_threshold:
            streak = self.consecutive_diverging
            self._exit_tracking(current_dist, delta, streak)
