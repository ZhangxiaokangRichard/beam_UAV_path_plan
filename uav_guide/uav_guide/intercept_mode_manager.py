#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拦截模式：yaml 默认 + 命令话题 latch 热切换。"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

import rospy
from std_msgs.msg import String


def parse_intercept_mode(raw: str) -> Optional[str]:
    mode = str(raw).strip().lower()
    if mode in ("head-on", "headon", "head_on"):
        return "head-on"
    if mode == "tail":
        return "tail"
    return None


def normalize_intercept_mode(intercept_mode: str) -> str:
    parsed = parse_intercept_mode(intercept_mode)
    return parsed if parsed is not None else "head-on"


class InterceptModeManager:
    """订阅命令话题 latch 模式；发布 latched 状态话题供其它节点同步。"""

    def __init__(
        self,
        config: Dict[str, Any],
        on_mode_change: Optional[Callable[[str, str], None]] = None,
    ):
        self._lock = threading.Lock()
        self._on_mode_change = on_mode_change
        default = normalize_intercept_mode(config.get("intercept_mode", "head-on"))
        self._active_mode = default

        status_topic = str(
            config.get("intercept_mode_status_topic", "/uav_guide/intercept_mode")
        ).strip()
        self._status_pub = rospy.Publisher(status_topic, String, queue_size=1, latch=True)
        self._publish_status()

        cmd_topic = str(config.get("intercept_mode_topic", "")).strip()
        if cmd_topic:
            rospy.Subscriber(cmd_topic, String, self._cmd_cb, queue_size=10)
            rospy.loginfo(
                "[intercept_mode] default=%s cmd=%s status=%s",
                default,
                cmd_topic,
                status_topic,
            )
        else:
            rospy.loginfo(
                "[intercept_mode] default=%s (no cmd topic, yaml only)",
                default,
            )

    def get_mode(self) -> str:
        with self._lock:
            return self._active_mode

    def _cmd_cb(self, msg: String) -> None:
        parsed = parse_intercept_mode(msg.data)
        if parsed is None:
            rospy.logwarn_throttle(
                5.0,
                "[intercept_mode] ignore invalid mode: %r (use head-on or tail)",
                msg.data,
            )
            return

        with self._lock:
            if parsed == self._active_mode:
                return
            old_mode = self._active_mode
            self._active_mode = parsed

        self._publish_status()
        rospy.loginfo("[intercept_mode] %s -> %s", old_mode, parsed)

        if self._on_mode_change is not None:
            self._on_mode_change(old_mode, parsed)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self.get_mode()
        self._status_pub.publish(msg)
