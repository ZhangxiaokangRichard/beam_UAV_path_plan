#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UAV 引导主循环：每次 state 更新 → 规划 → 发布 /uav/planned_path。
"""

from __future__ import annotations

import rospy

from uav_guide.intercept_mode_manager import InterceptModeManager
from uav_guide.msg import UavPlannedPath
from uav_guide.plan_orchestrator import PlanOrchestrator
from uav_guide.state_adapter import StateAdapter


class UavGuideLoopNode:
    def __init__(self):
        rospy.init_node("uav_guide_loop")
        self._config = rospy.get_param("~", {})

        out_topic = self._config.get("output", {}).get("planned_path_topic", "/uav/planned_path")
        self._pub = rospy.Publisher(out_topic, UavPlannedPath, queue_size=1, latch=True)

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

        rospy.loginfo("[uav_guide_loop] ready, plan on every state update")

    def _on_state_update(self) -> None:
        uav, target, mode, ready = self._adapter.get_snapshot()
        if not ready:
            return
        self._orchestrator.plan_on_update(uav, target, mode)

    def _publish_plan(self, resp) -> None:
        msg = UavPlannedPath()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._config.get("planning", {}).get("frame_id", "map")
        msg.success = bool(resp.success)
        msg.cost = float(resp.cost)
        msg.status_message = resp.status_message
        msg.path = resp.path
        self._pub.publish(msg)


def main():
    try:
        UavGuideLoopNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
