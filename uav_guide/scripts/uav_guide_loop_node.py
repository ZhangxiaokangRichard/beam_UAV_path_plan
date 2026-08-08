#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV 引导主循环：每次 state 更新 → 规划 → 发布 /uav/planned_path；
规划成功后调用 /uav_dynamic/solve 得到运动指令并发布 /uav/cmd。
"""

from __future__ import annotations

import rospy
from uav_dynamic.srv import UavDynamicsSolve, UavDynamicsSolveRequest

from uav_guide.intercept_mode_manager import InterceptModeManager
from uav_guide.msg import UavCmd, UavPlannedPath
from uav_guide.plan_orchestrator import PlanOrchestrator
from uav_guide.state_adapter import StateAdapter


class UavGuideLoopNode:
    def __init__(self):
        rospy.init_node("uav_guide_loop")
        self._config = rospy.get_param("~", {})

        output = self._config.get("output", {})
        out_topic = output.get("planned_path_topic", "/uav/planned_path")
        self._pub = rospy.Publisher(out_topic, UavPlannedPath, queue_size=1, latch=True)

        # 对接 uav_dynamic：规划成功后解算运动指令并发布
        self._cmd_topic = output.get("dynamics_cmd_topic", "/uav/cmd")
        self._cmd_pub = rospy.Publisher(self._cmd_topic, UavCmd, queue_size=1, latch=True)
        self._speed_mps = float(self._config.get("dynamics", {}).get("speed_mps", 50.0))
        self._solve_client = rospy.ServiceProxy("/uav_dynamic/solve", UavDynamicsSolve)
        self._last_path = []

        self._orchestrator = PlanOrchestrator(self._config, self._publish_plan)
        self._adapter = None

        def on_mode_change(_old: str, new: str) -> None:
            self._orchestrator.reset_on_mode_change(new)
            if self._adapter is not None:
                uav, target, mode, ready = self._adapter.get_snapshot()
                if ready:
                    self._orchestrator.plan_on_update(uav, target, mode)

        self._mode_manager = InterceptModeManager(self._config, on_mode_change=on_mode_change)
        self._adapter = StateAdapter(self._config, self._on_state_update, self._mode_manager)

        rospy.loginfo(
            "[uav_guide_loop] ready, plan on every state update | "
            "cmd_topic=%s speed=%.1fm/s",
            self._cmd_topic,
            self._speed_mps,
        )

    def _on_state_update(self) -> None:
        uav, target, mode, ready = self._adapter.get_snapshot()
        if not ready:
            return
        ok = self._orchestrator.plan_on_update(uav, target, mode)
        if ok:
            self._solve_dynamics(uav, target)

    def _solve_dynamics(self, uav, target) -> None:
        """把最近一次成功规划路径 + uav/target 高度发给 uav_dynamic，发布 /uav/cmd。"""
        if not self._last_path:
            return
        try:
            req = UavDynamicsSolveRequest()
            req.path = list(self._last_path)
            req.uav_height = float(uav[2])
            req.target_height = float(target[2])
            req.speed_mps = self._speed_mps
            resp = self._solve_client(req)

            msg = UavCmd()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.success = bool(resp.success)
            msg.heading_rad = float(resp.heading_rad)
            msg.roll_rad = float(resp.roll_rad)
            msg.pitch_rad = float(resp.pitch_rad)
            msg.climb_mps = float(resp.climb_mps)
            self._cmd_pub.publish(msg)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "[uav_guide_loop] uav_dynamic solve failed: %s", exc)

    def _publish_plan(self, resp) -> None:
        msg = UavPlannedPath()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._config.get("planning", {}).get("frame_id", "map")
        msg.success = bool(resp.success)
        msg.cost = float(resp.cost)
        msg.status_message = resp.status_message
        msg.path = resp.path
        self._pub.publish(msg)
        if resp.success:
            self._last_path = list(resp.path)


def main():
    try:
        UavGuideLoopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
