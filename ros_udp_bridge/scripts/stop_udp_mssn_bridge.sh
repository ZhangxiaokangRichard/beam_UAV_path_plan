#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
# 停止整套服务：udp_mssn_bridge / uav_guide / uav_bridge / roscore
# 的 tmux 会话与残留进程（供 systemd ExecStop 使用）。
#
# ⚠ 环境说明：不依赖 ~/.bashrc；pkill / tmux 均为系统命令，无需 ROS 环境。
# ═══════════════════════════════════════════════════════════════
set -eo pipefail

echo "[stop] stopping tmux sessions..."
# udp_mssn_bridge.launch 内部会拉起 uav_guide / uav_bridge 两个窗口
# 以及 start 脚本的 roscore 会话，一并清理
for s in udp_mssn_bridge uav_guide uav_bridge roscore; do
    tmux kill-session -t "$s" 2>/dev/null || true
done
sleep 2

# 兜底：tmux kill-session 可能杀不掉 roslaunch 自建的子进程树，按命令行补杀
# ── roslaunch 进程 ──
pkill -f '[u]dp_mssn_bridge.launch' 2>/dev/null || true
pkill -f '[u]av_guide.launch'       2>/dev/null || true
pkill -f '[u]av_bridge.launch'      2>/dev/null || true
# ── ros_udp_bridge 节点 ──
pkill -f '[r]os_udp_bridge/udp_mssn_bridge' 2>/dev/null || true
pkill -f '[r]os_udp_bridge/udp_receiver'    2>/dev/null || true
pkill -f '[r]os_udp_bridge/udp_sender'      2>/dev/null || true
# ── uav_guide / 规划 / 运动解算节点 ──
pkill -f '[u]av_guide_loop_node'    2>/dev/null || true
pkill -f '[p]lanner_server'         2>/dev/null || true
pkill -f '[u]av_dynamics_server'    2>/dev/null || true
# ── roscore ──
pkill -f '[r]osmaster'              2>/dev/null || true
echo "[stop] done."
