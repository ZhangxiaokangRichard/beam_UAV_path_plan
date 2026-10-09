/**
 * @file proc_spec.cpp
 * @brief 受管进程规格解析 + tmux 命令行构造（零 ROS 依赖，见 header 注释）
 */

#include "ros_mqtt_bridge/proc_spec.h"

#include <sstream>

namespace ros_mqtt_bridge {

namespace {

std::string trim(const std::string& s)
{
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

/// 按 '|' 切分（最多 3 段；段内不允许出现 '|'）
std::vector<std::string> splitPipe(const std::string& s)
{
    std::vector<std::string> out;
    std::string cur;
    for (const char c : s) {
        if (c == '|') {
            out.push_back(trim(cur));
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(trim(cur));
    return out;
}

}  // namespace

ProcSpec parseProcSpec(const std::string& text, const std::string& fallback_session)
{
    ProcSpec spec;
    const auto parts = splitPipe(text);
    if (parts.empty()) return spec;

    spec.command = parts[0];
    if (spec.command.empty()) return spec;

    spec.pattern = parts.size() > 1 ? parts[1] : std::string();
    spec.session = parts.size() > 2 ? parts[2] : std::string();
    if (spec.session.empty()) spec.session = fallback_session;
    if (spec.pattern.empty()) spec.pattern = spec.session;
    if (spec.session.empty()) return spec;   // 既没给 session 也没有 fallback

    spec.valid = true;
    return spec;
}

std::string buildLaunchScript(const std::string& workspace_dir,
                              const std::string& setup_shell,
                              const std::string& command,
                              const std::string& log_path)
{
    std::ostringstream o;
    o << "#!/bin/bash\n";
    o << "# 由 mqtt_mssn_bridge 生成：tmux 会话的启动脚本（可人工复现）\n";
    o << "set -e\n";
    if (!workspace_dir.empty()) o << "cd " << workspace_dir << "\n";
    if (!setup_shell.empty()) o << setup_shell << "\n";
    if (log_path.empty()) {
        o << "exec " << command << "\n";
    } else {
        // exec + tee：既在 tmux pane 里可见，又落盘（tail -f 无需 attach）
        o << "exec " << command << " 2>&1 | tee -a " << log_path << "\n";
    }
    return o.str();
}

std::vector<std::string> tmuxNewSessionArgs(const std::string& session,
                                            const std::string& workspace_dir,
                                            const std::string& script_path)
{
    std::vector<std::string> args{"new-session", "-d", "-s", session};
    if (!workspace_dir.empty()) {
        args.push_back("-c");
        args.push_back(workspace_dir);
    }
    // tmux 用 /bin/sh -c 执行该字符串；脚本路径由本进程生成，不含空格以外的怪字符，
    // 仍加引号以防路径含空格。
    args.push_back("bash '" + script_path + "'");
    return args;
}

std::vector<std::string> tmuxKillSessionArgs(const std::string& session)
{
    return {"kill-session", "-t", session};
}

std::vector<std::string> tmuxHasSessionArgs(const std::string& session)
{
    return {"has-session", "-t", session};
}

std::vector<std::string> tmuxListPanesArgs(const std::string& session)
{
    return {"list-panes", "-t", session, "-F", "#{pane_dead}"};
}

std::vector<std::string> tmuxRemainOnExitArgs(const std::string& session)
{
    return {"set-option", "-t", session, "remain-on-exit", "on"};
}

bool paneOutputAlive(const std::string& list_panes_stdout)
{
    std::istringstream iss(list_panes_stdout);
    std::string line;
    while (std::getline(iss, line)) {
        const std::string t = trim(line);
        if (t.empty()) continue;
        if (t != "1") return true;   // pane_dead == 0 → 进程仍在
    }
    return false;   // 无输出 / 全部 pane_dead == 1 → 不存活
}

}  // namespace ros_mqtt_bridge
