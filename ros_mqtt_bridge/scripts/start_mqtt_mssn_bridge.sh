#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# 开机自启：启动 roscore + mqtt_mssn_bridge（xterm 终端显示日志）
#
# 用法：
#   1) 依赖 xterm：  sudo apt install -y xterm
#   2) 赋予执行权限： chmod +x start_mqtt_mssn_bridge.sh
#   3) 手动运行：     ./start_mqtt_mssn_bridge.sh
#      开机自启：在「启动应用程序」中添加本脚本，或用 systemd 服务。
# ═══════════════════════════════════════════════════════════════

set -euo pipefail

export ROS_DISTRO=noetic
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"

command -v xterm >/dev/null 2>&1 || {
    echo "[start] xterm not found, run: sudo apt install -y xterm"
    exit 1
}

# 1) roscore（若未运行则启动）
if ! pgrep -f "rosmaster" >/dev/null 2>&1; then
    echo "[start] starting roscore in xterm ..."
    xterm -T roscore -geometry 110x28 -e \
        "source /opt/ros/noetic/setup.bash; roscore" &
    sleep 4
else
    echo "[start] roscore already running"
fi

# 2) mqtt_mssn_bridge（xterm 终端，可查看节点日志；respawn 保证自动拉起）
echo "[start] launching mqtt_mssn_bridge in xterm ..."
xterm -T mqtt_mssn_bridge -geometry 140x36 -e \
    "source /opt/ros/noetic/setup.bash; source $HOME/catkin_ws/devel/setup.bash; exec roslaunch ros_mqtt_bridge mqtt_mssn_bridge.launch" &

echo "[start] done."
