#pragma once

#include <string>
#include <vector>

namespace ros_mqtt_bridge {

/**
 * @brief 受管进程规格（纯数据 + 命令行构造，**零 ROS 依赖**，可离线单测）
 *
 * ROS2 参数不支持「结构体数组」，因此 yaml 里用**单行字符串**描述一个受管进程：
 *
 *     "<command>|<pattern>|<session>"
 *
 *   command : 完整命令行，例如 `ros2 launch uav_guide uav_guide_launch.py`
 *   pattern : /proc/<pid>/cmdline 的子串匹配串，用于兜底存活探测与残留清理
 *             （tmux 会话名可能被人工改动，pattern 是最后一道保险）
 *   session : tmux 会话名，**唯一**，也是该进程的调试/日志入口
 *             （留空 → 使用调用方给的 fallback_session）
 *
 * 例：
 *     "ros2 launch uav_guide uav_guide_launch.py|uav_guide_launch.py|aoa_nav"
 */
struct ProcSpec {
    std::string command;
    std::string pattern;
    std::string session;
    bool valid = false;
};

/** 解析 "<command>|<pattern>|<session>"；command 为空 → valid=false。
 *  session 为空时使用 fallback_session。pattern 为空时回退为 session。 */
ProcSpec parseProcSpec(const std::string& text, const std::string& fallback_session);

/**
 * 生成被 tmux 执行的包装脚本内容。**用 `exec`** 让 bash 被目标进程替换，
 * 这样 tmux pane 里跑的就是 ros2 本体（Ctrl-C / kill 语义直通），
 * 且 cmdline 含启动文件名 → /proc pattern 匹配成立。
 *
 *   cd <workspace_dir> && <setup_shell> && exec <command> [2>&1 | tee -a <log_path>]
 *
 * @param log_path 非空时用 `tee -a` 同时落盘（保留「不 attach 也能 tail 日志」），
 *                 为空时仅输出到 tmux pane。
 */
std::string buildLaunchScript(const std::string& workspace_dir,
                              const std::string& setup_shell,
                              const std::string& command,
                              const std::string& log_path = "");

/** tmux new-session 参数（不含 argv[0]）：-d -s <sess> -c <ws> "bash <script>" */
std::vector<std::string> tmuxNewSessionArgs(const std::string& session,
                                            const std::string& workspace_dir,
                                            const std::string& script_path);

/** tmux kill-session 参数（不含 argv[0]） */
std::vector<std::string> tmuxKillSessionArgs(const std::string& session);

/** tmux has-session 参数（不含 argv[0]） */
std::vector<std::string> tmuxHasSessionArgs(const std::string& session);

/** tmux list-panes 参数：输出 #{pane_dead}，配合 remain-on-exit 判断真实存活 */
std::vector<std::string> tmuxListPanesArgs(const std::string& session);

/** tmux set-option remain-on-exit on 参数（崩溃后保留 pane 便于看报错） */
std::vector<std::string> tmuxRemainOnExitArgs(const std::string& session);

/**
 * 解析 `tmux list-panes -F '#{pane_dead}'` 的输出。
 * 只要存在 **dead 标志为 0** 的 pane → 视为存活；输出为空 / 全为 1 → 不存活。
 */
bool paneOutputAlive(const std::string& list_panes_stdout);

}  // namespace ros_mqtt_bridge
