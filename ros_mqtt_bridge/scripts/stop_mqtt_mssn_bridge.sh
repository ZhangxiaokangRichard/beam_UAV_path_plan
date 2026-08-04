#!/usr/bin/env bash
# 停止 mqtt_mssn_bridge 与 roscore 的 tmux 会话（供 systemd ExecStop 使用）
set -eo pipefail

echo "[stop] stopping tmux sessions..."
tmux kill-session -t mqtt_mssn_bridge 2>/dev/null || true
tmux kill-session -t roscore 2>/dev/null || true
sleep 2

# 兜底：tmux kill-session 杀不掉 roslaunch 自建的子进程树，按命令行补杀
pkill -f '[m]qtt_mssn_bridge.launch' 2>/dev/null || true      # roslaunch
pkill -f '[l]ib/ros_mqtt_bridge/mqtt_mssn_bridge' 2>/dev/null || true  # 节点
pkill -f '[r]osmaster' 2>/dev/null || true                    # roscore
echo "[stop] done."
