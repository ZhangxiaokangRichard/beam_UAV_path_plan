#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT 命令桥（Linux 虚拟机运行，独立于 beam_UAV_path_plan）。

订阅 beam_dubins/guide/cmd，执行 roslaunch / rostopic / rosrun；
发布 beam_dubins/guide/status 供 Windows 控制台显示。
"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict

import yaml

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("缺少 paho-mqtt，请执行: pip3 install -r requirements.txt") from exc


CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"


def load_config() -> Dict[str, Any]:
    with CONFIG_PATH.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def pgrep(pattern: str) -> bool:
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


class GuideMqttBridge:
    def __init__(self, cfg: Dict[str, Any]):
        self._cfg = cfg
        self._lock = threading.Lock()
        self._strike_paused = False
        self._last_error = ""
        self._cached_intercept_mode = "unknown"
        self._cached_tracking_mode = False

        ros_cfg = cfg["ros"]
        self._setup = ros_cfg["setup_shell"].strip()
        self._launch_cmd = (
            f"{self._setup} && exec roslaunch "
            f"{ros_cfg['launch_package']} {ros_cfg['launch_file']}"
        )
        self._waypoint_cmd = f"{self._setup} && exec rosrun ros_mqtt_bridge mqtt_waypoint_bridge"
        self._launch_match = ros_cfg["launch_match"]
        self._waypoint_match = ros_cfg["waypoint_match"]

        tmux = cfg["tmux"]
        self._guide_session = tmux["guide_session"]
        self._waypoint_session = tmux["waypoint_session"]

        mqtt_cfg = cfg["mqtt"]
        self._status_topic = mqtt_cfg["status_topic"]
        self._status_hz = float(cfg.get("status", {}).get("publish_hz", 1.0))

        client_id = f"guide_mqtt_bridge_{os.getpid()}"
        try:
            from paho.mqtt.enums import CallbackAPIVersion

            self._client = mqtt.Client(
                CallbackAPIVersion.VERSION2, client_id=client_id, protocol=mqtt.MQTTv311
            )
        except (ImportError, AttributeError):
            self._client = mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv311)
        self._client.reconnect_delay_set(min_delay=1, max_delay=10)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self._log_dir = Path(__file__).resolve().parent / "logs"

    def _shell(self, cmd: str, timeout: float = 15.0) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["bash", "-lc", cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )

    def _has_tmux(self) -> bool:
        return shutil.which("tmux") is not None

    def _run_bg(self, name: str, cmd: str) -> None:
        if self._has_tmux():
            self._run_bg_tmux(name, cmd)
        else:
            self._run_bg_nohup(name, cmd)

    def _run_bg_nohup(self, name: str, cmd: str) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        log_file = self._log_dir / f"{name}.log"
        wrapped = (
            f"nohup bash -lc {shlex.quote(cmd)} "
            f">> {shlex.quote(str(log_file))} 2>&1 & disown"
        )
        proc = self._shell(wrapped)
        if proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or proc.stdout.strip() or "nohup start failed")
        print(f"[guide_mqtt_bridge] started {name} via nohup, log={log_file}")

    def _run_bg_tmux(self, session: str, cmd: str) -> None:
        check = self._shell(f"tmux has-session -t {shlex.quote(session)} 2>/dev/null")
        if check.returncode == 0:
            return
        start_cmd = (
            f"tmux new-session -d -s {shlex.quote(session)} "
            f"bash -lc {shlex.quote(cmd)}"
        )
        proc = self._shell(start_cmd)
        if proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or proc.stdout.strip() or "tmux start failed")

    def _stop_bg(self, session: str, match: str) -> None:
        if self._has_tmux():
            self._shell(f"tmux kill-session -t {shlex.quote(session)} 2>/dev/null || true")
        self._shell(f"pkill -f {shlex.quote(match)} || true")

    def _ensure_launch(self) -> None:
        if pgrep(self._launch_match):
            return
        self._run_bg(self._guide_session, self._launch_cmd)
        for _ in range(30):
            if pgrep(self._launch_match):
                return
            time.sleep(1.0)
        raise RuntimeError("uav_guide.launch 启动超时")

    def _start_waypoint_bridge(self) -> None:
        if pgrep(self._waypoint_match):
            return
        self._run_bg(self._waypoint_session, self._waypoint_cmd)
        for _ in range(15):
            if pgrep(self._waypoint_match):
                return
            time.sleep(0.5)
        raise RuntimeError("mqtt_waypoint_bridge 启动超时")

    def _stop_waypoint_bridge(self) -> None:
        self._stop_bg(self._waypoint_session, self._waypoint_match)

    def _set_intercept_mode(self, mode: str) -> None:
        mode = mode.strip().lower()
        if mode in ("headon", "head_on"):
            mode = "head-on"
        if mode not in ("head-on", "tail"):
            raise ValueError(f"invalid mode: {mode}")
        cmd = (
            f"{self._setup} && "
            f"rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "
            f"\"data: '{mode}'\""
        )
        proc = self._shell(cmd, timeout=10.0)
        if proc.returncode != 0:
            raise RuntimeError(proc.stderr.strip() or "rostopic pub failed")

    def _read_intercept_mode(self) -> str:
        cmd = f"{self._setup} && rostopic echo -n 1 /uav_guide/intercept_mode"
        proc = self._shell(cmd, timeout=8.0)
        if proc.returncode != 0:
            return "unknown"
        for line in proc.stdout.splitlines():
            line = line.strip()
            if line.startswith("data:"):
                return line.split(":", 1)[1].strip().strip('"').strip("'") or "unknown"
        return "unknown"

    def _read_tracking_mode(self) -> bool:
        cmd = f"{self._setup} && rosparam get /uav_guide/tracking_mode"
        proc = self._shell(cmd, timeout=5.0)
        if proc.returncode != 0:
            return False
        return proc.stdout.strip().lower() in ("true", "1")

    def _refresh_ros_status_cache(self) -> None:
        self._cached_intercept_mode = self._read_intercept_mode()
        self._cached_tracking_mode = self._read_tracking_mode()

    def _build_status(self) -> Dict[str, Any]:
        launch_running = pgrep(self._launch_match)
        waypoint_running = pgrep(self._waypoint_match)

        if not launch_running:
            strike_state = "idle"
        elif waypoint_running:
            strike_state = "striking"
        elif self._strike_paused:
            strike_state = "stopped"
        else:
            strike_state = "planning"

        return {
            "schema": "beam_dubins.guide_status.v1",
            "timestamp": int(time.time()),
            "intercept_mode": self._cached_intercept_mode,
            "strike_state": strike_state,
            "guide_launch_running": launch_running,
            "waypoint_bridge_running": waypoint_running,
            "tracking_mode": self._cached_tracking_mode,
            "last_error": self._last_error,
        }

    def _status_cache_loop(self) -> None:
        while True:
            try:
                if pgrep(self._launch_match):
                    self._refresh_ros_status_cache()
            except Exception as exc:
                print(f"[guide_mqtt_bridge] status cache error: {exc}")
            time.sleep(2.0)

    def _publish_status_loop(self) -> None:
        interval = 1.0 / max(self._status_hz, 0.1)
        while True:
            try:
                payload = json.dumps(self._build_status(), ensure_ascii=False)
                self._client.publish(self._status_topic, payload, qos=0, retain=False)
            except Exception as exc:
                print(f"[guide_mqtt_bridge] status publish error: {exc}")
            time.sleep(interval)

    def _handle_cmd(self, payload: str) -> None:
        try:
            msg = json.loads(payload)
        except json.JSONDecodeError as exc:
            self._last_error = f"invalid json: {exc}"
            return

        method = str(msg.get("method", "")).strip()
        data = msg.get("data") or {}

        try:
            with self._lock:
                if method == "session_start":
                    self._ensure_launch()
                    self._strike_paused = False
                elif method == "strike_start":
                    self._ensure_launch()
                    self._start_waypoint_bridge()
                    self._strike_paused = False
                elif method == "strike_stop":
                    self._stop_waypoint_bridge()
                    self._strike_paused = True
                elif method == "set_intercept_mode":
                    mode = str(data.get("mode", ""))
                    self._ensure_launch()
                    self._set_intercept_mode(mode)
                else:
                    self._last_error = f"unknown method: {method}"
                    return
                self._last_error = ""
                print(f"[guide_mqtt_bridge] ok: {method}")
                if method == "set_intercept_mode":
                    self._refresh_ros_status_cache()
        except Exception as exc:
            self._last_error = str(exc)
            print(f"[guide_mqtt_bridge] cmd failed ({method}): {exc}")

    def _on_connect(self, client, userdata, flags, rc, *args):
        code = rc.value if hasattr(rc, "value") else rc
        if code != 0:
            print(f"[guide_mqtt_bridge] MQTT connect failed rc={code}")
            return
        topic = self._cfg["mqtt"]["cmd_topic"]
        client.subscribe(topic, qos=0)
        print(f"[guide_mqtt_bridge] connected, subscribed {topic}")

    def _on_message(self, client, userdata, msg):
        text = msg.payload.decode("utf-8", errors="replace")
        print(f"[guide_mqtt_bridge] cmd <= {text}")
        threading.Thread(target=self._handle_cmd, args=(text,), daemon=True).start()

    def _on_disconnect(self, client, userdata, rc, *args):
        code = rc.value if hasattr(rc, "value") else rc
        if code != 0:
            print(f"[guide_mqtt_bridge] MQTT disconnected rc={code}, will retry")

    def run(self) -> None:
        mqtt_cfg = self._cfg["mqtt"]
        threading.Thread(target=self._status_cache_loop, daemon=True).start()
        threading.Thread(target=self._publish_status_loop, daemon=True).start()
        self._client.connect(
            mqtt_cfg["host"],
            int(mqtt_cfg["port"]),
            keepalive=int(mqtt_cfg.get("keepalive", 30)),
        )
        print(
            f"[guide_mqtt_bridge] broker {mqtt_cfg['host']}:{mqtt_cfg['port']} "
            f"status -> {self._status_topic}"
        )
        self._client.loop_forever()


def main() -> None:
    cfg = load_config()
    bridge = GuideMqttBridge(cfg)
    bridge.run()


if __name__ == "__main__":
    main()
