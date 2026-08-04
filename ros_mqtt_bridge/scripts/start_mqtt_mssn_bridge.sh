#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# 启动 roscore + mqtt_mssn_bridge（tmux 会话，适合无窗口环境）
#
#   tmux attach -t roscore            查看 roscore
#   tmux attach -t mqtt_mssn_bridge   查看 mqtt_mssn_bridge
#   tmux kill-session -t <name>       手动停止某会话
# ═══════════════════════════════════════════════════════════════
set -eo pipefail

# ── ROS 环境 ──
export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_PYTHON_VERSION=3

source /opt/ros/noetic/setup.bash
source /home/aos-dev/catkin_ws/devel/setup.bash

command -v tmux >/dev/null 2>&1 || { echo "[start] tmux not installed"; exit 1; }

# 清理可能残留的同名 tmux 会话
tmux kill-session -t roscore 2>/dev/null || true
tmux kill-session -t mqtt_mssn_bridge 2>/dev/null || true

# 1) roscore（tmux 会话）
echo "[start] starting roscore in tmux session 'roscore' ..."
tmux new-session -d -s roscore "source /opt/ros/noetic/setup.bash; roscore"
sleep 5
if ! pgrep -f "rosmaster" >/dev/null 2>&1; then
    echo "[FATAL] roscore failed to start"
    exit 1
fi
echo "[start] roscore OK"

# 2) mqtt_mssn_bridge（tmux 会话）
echo "[start] launching mqtt_mssn_bridge in tmux session 'mqtt_mssn_bridge' ..."
tmux new-session -d -s mqtt_mssn_bridge \
    "source /opt/ros/noetic/setup.bash; source /home/aos-dev/catkin_ws/devel/setup.bash; roslaunch ros_mqtt_bridge mqtt_mssn_bridge.launch"

echo "[start] done. tmux sessions: roscore, mqtt_mssn_bridge"
