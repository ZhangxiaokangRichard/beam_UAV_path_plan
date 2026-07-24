#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 /uav/state 与 /target/state，提取 5D 状态。"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

import rospy
from nav_msgs.msg import Odometry

from uav_guide.field_extractor import extract_state_5d
from uav_guide.intercept_mode_manager import InterceptModeManager


class StateAdapter:
    """订阅本机与目标 Odometry，每次更新触发回调。"""

    def __init__(
        self,
        config: Dict[str, Any],
        on_update: Callable[[], None],
        mode_manager: InterceptModeManager,
    ):
        self._config = config
        self._on_update = on_update
        self._mode_manager = mode_manager
        self._lock = threading.Lock()

        self.uav_state = None  # type: Optional[list]
        self.target_state = None  # type: Optional[list]

        self._uav_ready = False
        self._target_ready = False
        self._subs = []
        self._setup_subscribers()

    @property
    def states_ready(self) -> bool:
        return self._uav_ready and self._target_ready

    def _setup_subscribers(self) -> None:
        inputs = self._config.get("inputs", {})
        uav_topic = inputs.get("uav_topic", "/uav/state")
        target_topic = inputs.get("target_topic", "/target/state")
        fields = inputs.get(
            "odometry_fields",
            self._default_odometry_fields(),
        )

        self._subs.append(
            rospy.Subscriber(uav_topic, Odometry, self._make_cb("uav", fields), queue_size=10)
        )
        self._subs.append(
            rospy.Subscriber(
                target_topic, Odometry, self._make_cb("target", fields), queue_size=10
            )
        )
        rospy.loginfo("[state_adapter] uav=%s target=%s", uav_topic, target_topic)

    @staticmethod
    def _default_odometry_fields() -> Dict[str, Any]:
        quat_yaw = {
            "source": "quaternion",
            "qw": "pose.pose.orientation.w",
            "qx": "pose.pose.orientation.x",
            "qy": "pose.pose.orientation.y",
            "qz": "pose.pose.orientation.z",
        }
        return {
            "x": "pose.pose.position.x",
            "y": "pose.pose.position.y",
            "z": "pose.pose.position.z",
            "yaw": quat_yaw,
            "pitch": 0.0,
        }

    def _make_cb(self, label: str, fields: Dict[str, Any]):
        def cb(msg, _label=label, _fields=fields):
            try:
                state = extract_state_5d(_fields, msg)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "[state_adapter] %s extract failed: %s", _label, exc)
                return

            trigger = False
            with self._lock:
                if _label == "uav":
                    self.uav_state = state
                    self._uav_ready = True
                else:
                    self.target_state = state
                    self._target_ready = True
                trigger = self._uav_ready and self._target_ready

            if trigger:
                self._on_update()

        return cb

    def get_snapshot(self):
        with self._lock:
            return (
                list(self.uav_state) if self.uav_state else None,
                list(self.target_state) if self.target_state else None,
                self._mode_manager.get_mode(),
                self._uav_ready and self._target_ready,
            )
