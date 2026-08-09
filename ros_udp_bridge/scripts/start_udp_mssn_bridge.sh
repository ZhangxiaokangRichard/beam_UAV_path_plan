#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# 启动 roscore + udp_mssn_bridge（tmux 会话，适合无窗口/容器环境）
#
#   tmux attach -t roscore             查看 roscore
#   tmux attach -t udp_mssn_bridge     查看 udp_mssn_bridge
#   tmux kill-session -t <name>        手动停止某会话
#
# ⚠ 环境说明：本脚本不依赖 ~/.bashrc（systemd / Docker 等非登录环境
#   不会加载它）。ROS 与工作空间环境在脚本内以及每个 tmux 会话的
#   内层命令中都显式 source；路径可用环境变量覆盖，便于 Docker 打包：
#     ROS_DISTRO   ROS_SETUP   WS_ROOT   WS_SETUP
# ═══════════════════════════════════════════════════════════════
set -eo pipefail

# ── 可配置路径（优先取环境变量，便于 systemd / Docker 注入） ──
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export WS_ROOT="${WS_ROOT:-/home/aos-dev/catkin_ws}"
export WS_SETUP="${WS_SETUP:-${WS_ROOT}/devel/setup.bash}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"
export ROS_PYTHON_VERSION=3

# ── 显式载入 ROS + 工作空间环境（不依赖 ~/.bashrc） ──
if [ ! -f "$ROS_SETUP" ]; then
    echo "[FATAL] 未找到系统 ROS 环境：$ROS_SETUP（请检查 ROS_DISTRO / ROS_SETUP）" >&2
    exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
else
    echo "[warn] 未找到工作空间环境：$WS_SETUP（仅载入系统 ROS 环境）" >&2
fi

command -v tmux >/dev/null 2>&1 || { echo "[start] tmux not installed"; exit 1; }

# 清理可能残留的同名 tmux 会话
tmux kill-session -t roscore 2>/dev/null || true
tmux kill-session -t udp_mssn_bridge 2>/dev/null || true

# 1) roscore（tmux 会话；内层同样显式 source）
#    若 roscore 已在运行（如容器 entrypoint 已拉起），跳过避免端口冲突
if ! pgrep -f "rosmaster" >/dev/null 2>&1; then
    echo "[start] starting roscore in tmux session 'roscore' ..."
    tmux new-session -d -s roscore "source '$ROS_SETUP'; roscore"
    sleep 5
    if ! pgrep -f "rosmaster" >/dev/null 2>&1; then
        echo "[FATAL] roscore failed to start"
        exit 1
    fi
    echo "[start] roscore OK"
else
    echo "[start] roscore already running, skip"
fi

# 2) udp_mssn_bridge（tmux 会话）——入口：tmux 运行 udp_mssn_bridge.launch
#    （该 launch 内的 uav_guide / uav_bridge 两个窗口由节点按
#     udp_mssn_bridge.yaml mssn/setup_shell 再显式 source 后启动）
echo "[start] launching udp_mssn_bridge in tmux session 'udp_mssn_bridge' ..."
tmux new-session -d -s udp_mssn_bridge \
    "source '$ROS_SETUP' && source '$WS_SETUP' && roslaunch ros_udp_bridge udp_mssn_bridge.launch"

echo "[start] done. tmux sessions: roscore, udp_mssn_bridge"
echo "[start] UDP 收包: 0.0.0.0:11301（udp_receiver，由 udp_mssn_bridge 内部 tmux 拉起 uav_bridge.launch 提供）"
echo "[start] 注意: 11302 指令接收已随极简版移除；确保宿主机无其他进程占用 11301/11311"
