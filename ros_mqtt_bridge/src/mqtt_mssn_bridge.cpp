/**
 * @file mqtt_mssn_bridge.cpp
 * @brief 任务/模式桥：MQTT `aoa/ai_guide/*` ↔ ROS，兼管导航栈与控制桥的启停
 *
 * 职责（2026-10-08 修订）：
 *   1) 模式翻译（**纯翻译器**）：MQTT `aoa/ai_guide/nav/mode`（auto|setpoint|guidance）
 *      → ROS 镜像 `aoa/uav/nav_mode`（std_msgs/String，**纯模式名**，latched）。
 *      本节点**不再**因模式变化启停任何进程。
 *   2) 进程管理（**tmux 会话**）：作为导航算法的 MQTT 入口，为 docker 封装提供
 *      可 attach / 可人工接管 / 崩溃后 pane 保留（remain-on-exit）的进程管理。
 *
 * 入站（MQTT → 本节点）：
 *   `aoa/ai_guide/nav/mode`   "guidance" 或 {"mode":"guidance"}  → 转 ROS `aoa/uav/nav_mode`
 *   `aoa/ai_guide/intercept/mode`  "head-on"|"tail"（或 {"mode":...}）
 *                                 → 转 ROS `aoa/uav/intercept_mode_cmd`（拦截模式：迎头/尾追）
 *   `aoa/ai_guide/nav/cmd`    {"start":<bool>}                    → 启停 **nav 组**
 *                                （默认 2 个：uav_guide_launch.py + mqtt_control_bridge.launch.py）
 *   `aoa/ai_guide/guide/cmd`  {"start":<bool>}                    → 启停 **guide 组**
 *                                （默认 1 个：mqtt_bridge.launch.py = mqtt_target_bridge）
 * 出站（本节点 → MQTT）：
 *   `aoa/ai_guide/nav/status`   {"running":<bool>,"mode":"...","intercept":"...","procs":[...]}
 *   `aoa/ai_guide/guide/status` {"running":<bool>,"mode":"...","tracking":...,"procs":[...]}
 *
 * 进程管理（tmux）：
 *   启动 → 生成包装脚本 `<log_dir>/<session>.sh`，再 `tmux new-session -d -s <session>`
 *          （脚本内 `exec <cmd>`，故 pane 里跑的就是 ros2 本体；pane 输出同时 tee 落盘）
 *   停止 → `tmux kill-session` + 等待 /proc pattern 清空 + 残留 SIGTERM/SIGKILL 兜底
 *   存活 → `tmux has-session` + `list-panes -F '#{pane_dead}'`（配合 remain-on-exit
 *          区分「会话残留但进程已崩」），并用 /proc pattern 兜底
 *   人工调试：`tmux attach -t <session>`；崩溃后 pane 保留可回溯报错
 *   tmux 为**强依赖**：`mssn.use_tmux=true` + 启用进程管理时若 tmux 缺失即 FATAL。
 *   回退路径：`mssn.use_tmux=false` → fork + setsid + 日志重定向（无 tmux 环境用）。
 * 安全：`mssn.enable_process_control=false` 时只做模式转发，不启停任何进程（离线测试用）。
 */

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mode_gate.h"
#include "ros_mqtt_bridge/mqtt_client.h"
#include "ros_mqtt_bridge/proc_spec.h"

namespace {

// ── 默认受管进程（yaml `mssn.{nav,guide}_procs` 可覆盖）──
// 格式："<command>|<pattern>|<session>"（见 proc_spec.h）
const std::vector<std::string> kDefaultNavProcs = {
    "ros2 launch uav_guide uav_guide_launch.py|uav_guide_launch.py|aoa_nav",
    "ros2 launch ros_mqtt_bridge mqtt_control_bridge.launch.py|mqtt_control_bridge.launch.py|aoa_control",
};
const std::vector<std::string> kDefaultGuideProcs = {
    "ros2 launch ros_mqtt_bridge mqtt_bridge.launch.py|mqtt_bridge.launch.py|aoa_bridge",
};

// ── 参数助手 ──
std::string param_string(const rclcpp::Node& node, const std::string& name,
                         const std::string& fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return v.as_string();
    return fallback;
}

std::int64_t param_int(const rclcpp::Node& node, const std::string& name, std::int64_t fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return v.as_int();
    return fallback;
}

double param_double(const rclcpp::Node& node, const std::string& name, double fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.as_double();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(v.as_int());
    return fallback;
}

bool param_bool(const rclcpp::Node& node, const std::string& name, bool fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) return v.as_bool();
    return fallback;
}

std::vector<std::string> param_string_array(const rclcpp::Node& node, const std::string& name,
                                            const std::vector<std::string>& fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) return v.as_string_array();
    return fallback;
}

/** 容错提取模式：纯模式名（"guidance"）或 JSON {"mode":...} / {"data":{"mode":...}} */
std::string extractModeTolerant(const std::string& payload)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    auto b = std::find_if(payload.begin(), payload.end(), not_space);
    auto e = std::find_if(payload.rbegin(), payload.rend(), not_space).base();
    if (b >= e) return {};
    const std::string trimmed(b, e);
    if (trimmed.front() != '{') return trimmed;

    std::string err;
    std::string mode;
    if (ros_mqtt_bridge::decodeStateString(trimmed, "mode", mode, err) && !mode.empty()) return mode;
    const auto key = trimmed.find("\"mode\"");
    if (key == std::string::npos) return {};
    const auto colon = trimmed.find(':', key);
    if (colon == std::string::npos) return {};
    const auto q1 = trimmed.find('"', colon);
    if (q1 == std::string::npos) return {};
    const auto q2 = trimmed.find('"', q1 + 1);
    if (q2 == std::string::npos) return {};
    return trimmed.substr(q1 + 1, q2 - q1 - 1);
}

// ── /proc 进程扫描（不依赖 shell，避免 pgrep/pkill 自我匹配）────────────

/// 读取 /proc/<pid>/cmdline（NUL 替换为空格）
bool readCmdline(int pid, std::string& out)
{
    std::ifstream f("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
    if (!f) return false;
    std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (raw.empty()) return false;
    for (auto& c : raw) {
        if (c == '\0') c = ' ';
    }
    out = raw;
    return true;
}

/// 读取 /proc/<pid>/stat 的 pgrp(第5字段) 与 session(第6字段)
bool readPgrpSession(int pid, int& pgrp, int& session)
{
    std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
    if (!f) return false;
    std::string line;
    std::getline(f, line);
    const auto rp = line.rfind(')');   // comm 可能含空格/括号
    if (rp == std::string::npos) return false;
    std::istringstream ss(line.substr(rp + 1));
    std::string state;
    long ppid = 0, pg = 0, sid = 0;
    ss >> state >> ppid >> pg >> sid;
    pgrp = static_cast<int>(pg);
    session = static_cast<int>(sid);
    return true;
}

/// 进程是否存活（僵尸进程视为已退出）
bool processAlive(int pid)
{
    if (pid <= 0) return false;
    if (::kill(pid, 0) != 0) return false;
    std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
    if (!f) return false;
    std::string line;
    std::getline(f, line);
    const auto rp = line.rfind(')');
    if (rp == std::string::npos || rp + 2 >= line.size()) return true;
    return line[rp + 2] != 'Z';
}

/**
 * 扫描 /proc，返回 cmdline 包含 pattern 的进程 pid。
 * **排除自身、同进程组、同会话的进程**：本节点的命令行可能因参数覆盖而含有被管理
 * 进程的名字（例如 `-p mssn.guide_run_command:="... mqtt_control_bridge"`），
 * 若不排除会自我匹配（误判 running）甚至自杀（pkill 命中自身）。
 * 由 setsid() 启动的子进程位于独立会话，因此仍能被正确发现。
 */
std::vector<int> matchPids(const std::string& pattern)
{
    std::vector<int> out;
    if (pattern.empty()) return out;
    DIR* dir = ::opendir("/proc");
    if (dir == nullptr) return out;
    const int self = static_cast<int>(::getpid());
    const int self_pgrp = static_cast<int>(::getpgrp());
    const int self_sid = static_cast<int>(::getsid(0));
    while (dirent* ent = ::readdir(dir)) {
        const char* name = ent->d_name;
        if (!std::isdigit(static_cast<unsigned char>(name[0]))) continue;
        const int pid = std::atoi(name);
        if (pid == self) continue;
        int pgrp = 0, sid = 0;
        if (readPgrpSession(pid, pgrp, sid) && (pgrp == self_pgrp || sid == self_sid)) continue;
        std::string cmd;
        if (!readCmdline(pid, cmd)) continue;
        if (cmd.find(pattern) != std::string::npos) out.push_back(pid);
    }
    ::closedir(dir);
    return out;
}

// ── 外部命令执行 / 文件写入（tmux 会话管理用）────────────────────────

/** 以 argv 形式执行外部命令（不经 shell，避免注入）；返回退出码，-1 = 无法执行。
 *  @param out 非空时捕获 stdout（先读管道再 waitpid，避免管道写满死锁）。 */
int runCmd(const std::vector<std::string>& argv, std::string* out = nullptr)
{
    int pipefd[2] = {-1, -1};
    if (out != nullptr && ::pipe(pipefd) != 0) return -1;

    const pid_t pid = ::fork();
    if (pid < 0) {
        if (out != nullptr) {
            ::close(pipefd[0]);
            ::close(pipefd[1]);
        }
        return -1;
    }
    if (pid == 0) {
        if (out != nullptr) {
            ::close(pipefd[0]);
            ::dup2(pipefd[1], STDOUT_FILENO);
            ::close(pipefd[1]);
        } else {
            if (::freopen("/dev/null", "w", stdout) == nullptr) { /* 忽略 */ }
            if (::freopen("/dev/null", "w", stderr) == nullptr) { /* 忽略 */ }
        }
        std::vector<char*> cargv;
        cargv.reserve(argv.size() + 1);
        for (const auto& a : argv) cargv.push_back(const_cast<char*>(a.c_str()));
        cargv.push_back(nullptr);
        ::execvp(cargv[0], cargv.data());
        ::_exit(127);
    }

    if (out != nullptr) {
        ::close(pipefd[1]);
        char buf[256];
        ssize_t n = 0;
        while ((n = ::read(pipefd[0], buf, sizeof(buf))) > 0)
            out->append(buf, static_cast<std::size_t>(n));
        ::close(pipefd[0]);
    }

    int status = 0;
    while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) { /* 重试 */ }
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

/// 写文本文件并置可执行位（tmux 会话启动脚本）
bool writeFile(const std::string& path, const std::string& content)
{
    std::ofstream f(path, std::ios::trunc);
    if (!f) return false;
    f << content;
    f.close();
    if (::chmod(path.c_str(), 0755) != 0) { /* 尽力而为：非致命 */ }
    return true;
}

/// 去掉尾部换行（用于 tmux -V 输出）
std::string trimEol(const std::string& s)
{
    const auto e = s.find_last_not_of(" \t\r\n");
    return e == std::string::npos ? std::string() : s.substr(0, e + 1);
}

}  // namespace

namespace {

/**
 * 单个受管进程。
 *   use_tmux = true  → tmux 会话（默认；tmux 缺失时由调用方在启动阶段 FATAL）
 *   use_tmux = false → fork + setsid + 日志重定向（无 tmux 环境的回退路径）
 */
class ManagedProcess {
public:
    void configure(ros_mqtt_bridge::ProcSpec spec, std::string name, std::string log_dir,
                   std::string setup_shell, std::string workspace_dir, double stop_timeout_s,
                   bool use_tmux, bool remain_on_exit, std::string tmux_bin)
    {
        spec_ = std::move(spec);
        name_ = std::move(name);
        log_dir_ = std::move(log_dir);
        setup_shell_ = std::move(setup_shell);
        workspace_dir_ = std::move(workspace_dir);
        stop_timeout_s_ = stop_timeout_s;
        use_tmux_ = use_tmux;
        remain_on_exit_ = remain_on_exit;
        tmux_bin_ = std::move(tmux_bin);
        script_path_ = log_dir_ + "/" + spec_.session + ".sh";
        log_path_ = log_dir_ + "/" + spec_.session + ".log";
    }

    const std::string& name() const { return name_; }
    const std::string& session() const { return spec_.session; }
    const std::string& command() const { return spec_.command; }
    const std::string& logPath() const { return log_path_; }

    bool start(rclcpp::Logger logger)
    {
        if (running()) {
            RCLCPP_INFO(logger, "[%s] already running (session=%s), skip start", name_.c_str(),
                        spec_.session.c_str());
            return true;
        }
        // tmux + remain-on-exit：进程崩溃后会话仍在（pane 保留供回溯）→ 先清掉再重建
        if (use_tmux_ && tmuxSessionExists()) {
            RCLCPP_INFO(logger, "[%s] 清理残留会话 '%s'（其中进程已退出）", name_.c_str(),
                        spec_.session.c_str());
            runCmd(tmuxArgs(ros_mqtt_bridge::tmuxKillSessionArgs(spec_.session)));
        }
        pid_ = -1;   // 已退出但未回收 → 清空并交给下面的 /proc 扫描
        const auto leftovers = matchPids(spec_.pattern);
        if (!leftovers.empty()) {
            std::ostringstream oss;
            for (std::size_t i = 0; i < leftovers.size() && i < 5; ++i)
                oss << (i ? "," : "") << leftovers[i];
            RCLCPP_WARN(logger,
                        "[%s] %zu untracked instance(s) already running (pids=%s), skip start",
                        name_.c_str(), leftovers.size(), oss.str().c_str());
            return true;
        }

        if (use_tmux_) {
            if (!writeFile(script_path_, ros_mqtt_bridge::buildLaunchScript(
                                             workspace_dir_, setup_shell_, spec_.command, log_path_))) {
                RCLCPP_ERROR(logger, "[%s] 无法写入启动脚本 %s", name_.c_str(), script_path_.c_str());
                return false;
            }
            std::string out;
            const int rc = runCmd(tmuxArgs(ros_mqtt_bridge::tmuxNewSessionArgs(
                                     spec_.session, workspace_dir_, script_path_)), &out);
            if (rc != 0) {
                RCLCPP_ERROR(logger, "[%s] tmux new-session 失败 (rc=%d): %s", name_.c_str(), rc,
                             out.c_str());
                return false;
            }
            if (remain_on_exit_) {
                runCmd(tmuxArgs(ros_mqtt_bridge::tmuxRemainOnExitArgs(spec_.session)));
            }
            RCLCPP_INFO(logger, "[%s] started tmux session '%s': %s", name_.c_str(),
                        spec_.session.c_str(), spec_.command.c_str());
            RCLCPP_INFO(logger, "[%s] 调试: tmux attach -t %s | 日志: tail -f %s", name_.c_str(),
                        spec_.session.c_str(), log_path_.c_str());
            return true;
        }

        // ── 回退路径：fork + setsid + 日志重定向 ──
        const std::string full =
            "cd " + workspace_dir_ + " && " + setup_shell_ + " && exec " + spec_.command;

        const pid_t pid = ::fork();
        if (pid < 0) {
            RCLCPP_ERROR(logger, "[%s] fork failed: %s", name_.c_str(), std::strerror(errno));
            return false;
        }
        if (pid == 0) {
            // 子进程：独立会话，避免被父进程的 Ctrl-C 连带杀死
            ::setsid();
            if (!log_path_.empty()) {
                if (::freopen(log_path_.c_str(), "a", stdout) == nullptr) { /* 日志失败不致命 */ }
                if (::freopen(log_path_.c_str(), "a", stderr) == nullptr) { /* 同上 */ }
            }
            ::execl("/bin/bash", "bash", "-c", full.c_str(), static_cast<char*>(nullptr));
            ::_exit(127);
        }
        pid_ = pid;
        RCLCPP_INFO(logger, "[%s] started pid=%d cmd='%s' log=%s (无 tmux 回退路径)", name_.c_str(),
                    pid_, spec_.command.c_str(), log_path_.c_str());
        return true;
    }

    bool stop(rclcpp::Logger logger)
    {
        bool did_something = false;

        // 1) tmux 会话：kill-session（SIGHUP 到 pane 进程组），随后等 /proc 清空
        if (use_tmux_) {
            if (tmuxSessionExists()) {
                did_something = true;
                RCLCPP_INFO(logger, "[%s] tmux kill-session -t %s ...", name_.c_str(),
                            spec_.session.c_str());
                runCmd(tmuxArgs(ros_mqtt_bridge::tmuxKillSessionArgs(spec_.session)));
                waitPatternClear();
                RCLCPP_INFO(logger, "[%s] session '%s' cleared", name_.c_str(), spec_.session.c_str());
            }
            // 2) 未见会话也继续走 /proc 兜底（人工启动 / 会话被外部删除）
        } else if (pid_ > 0) {
            did_something = true;
            RCLCPP_INFO(logger, "[%s] stopping pgid=%d ...", name_.c_str(), pid_);
            ::killpg(pid_, SIGTERM);

            // 关键：一边回收组长（否则 killpg(pid,0) 对僵尸组长恒为成功，
            //      会导致每次都误走 SIGKILL 分支），一边等进程组清空
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(static_cast<int>(stop_timeout_s_ * 1000));
            int status = 0;
            bool leader_reaped = false;
            while (std::chrono::steady_clock::now() < deadline) {
                if (!leader_reaped && ::waitpid(pid_, &status, WNOHANG) == pid_) leader_reaped = true;
                if (leader_reaped && ::killpg(pid_, 0) != 0) break;   // 组内已无成员
                ::usleep(100 * 1000);
            }
            if (::killpg(pid_, 0) == 0) {
                RCLCPP_WARN(logger, "[%s] SIGTERM timeout (%.0fs) → SIGKILL", name_.c_str(),
                            stop_timeout_s_);
                ::killpg(pid_, SIGKILL);
            }
            // 清僵尸
            for (int i = 0; i < 20; ++i) {
                if (!leader_reaped && ::waitpid(pid_, &status, WNOHANG) == pid_) leader_reaped = true;
                if (leader_reaped) break;
                ::usleep(50 * 1000);
            }
            RCLCPP_INFO(logger, "[%s] group cleared", name_.c_str());
            pid_ = -1;
        }

        // 3) 扫除遗留实例（脱离进程组的子进程 / 上一轮桥未清理干净 / 人工启动）
        //    matchPids 已排除自身、同进程组、同会话，不会误杀本节点或调用终端
        auto leftovers = matchPids(spec_.pattern);
        if (!leftovers.empty()) {
            did_something = true;
            for (int p : leftovers) {
                ::kill(p, SIGTERM);
                ::killpg(p, SIGTERM);   // 组内其它成员（p 通常不是组长，失败无妨）
            }
            ::usleep(1000 * 1000);
            int forced = 0;
            for (int p : leftovers) {
                if (::kill(p, 0) == 0) {
                    ::kill(p, SIGKILL);
                    ++forced;
                }
            }
            if (forced > 0) RCLCPP_WARN(logger, "[%s] force-killed %d leftover process(es)",
                                        name_.c_str(), forced);
        }

        RCLCPP_INFO(logger, "[%s] stopped (acted=%d)", name_.c_str(), static_cast<int>(did_something));
        return true;
    }

    /** 存活判定：
     *   tmux 模式 → has-session + list-panes '#{pane_dead}'
     *              （remain-on-exit 下「会话在、进程死」不算存活），
     *              会话缺失/已死时用 /proc pattern 兜底（人工启动的同名进程）；
     *   回退模式 → 已跟踪 pid 或 /proc pattern。 */
    bool running() const
    {
        if (use_tmux_) {
            std::string panes;
            if (tmuxSessionAlive(panes)) return true;
            return !matchPids(spec_.pattern).empty();
        }
        if (processAlive(pid_)) return true;
        return !matchPids(spec_.pattern).empty();
    }

    int pid() const { return pid_; }

private:
    /// 在 tmux 子命令前插入可执行文件
    std::vector<std::string> tmuxArgs(std::vector<std::string> args) const
    {
        args.insert(args.begin(), tmux_bin_);
        return args;
    }

    bool tmuxSessionExists() const
    {
        return runCmd(tmuxArgs(ros_mqtt_bridge::tmuxHasSessionArgs(spec_.session))) == 0;
    }

    /// 会话存在且至少一个 pane 的进程仍活着
    bool tmuxSessionAlive(std::string& pane_out) const
    {
        if (!tmuxSessionExists()) return false;
        pane_out.clear();
        if (runCmd(tmuxArgs(ros_mqtt_bridge::tmuxListPanesArgs(spec_.session)), &pane_out) != 0)
            return false;
        return ros_mqtt_bridge::paneOutputAlive(pane_out);
    }

    /// 等待 pattern 对应进程退出（kill-session 后可能有短暂残留）
    void waitPatternClear() const
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(static_cast<int>(stop_timeout_s_ * 1000));
        while (std::chrono::steady_clock::now() < deadline) {
            if (matchPids(spec_.pattern).empty()) return;
            ::usleep(100 * 1000);
        }
    }

    ros_mqtt_bridge::ProcSpec spec_;
    std::string name_, log_dir_, setup_shell_, workspace_dir_;
    std::string script_path_, log_path_, tmux_bin_;
    double stop_timeout_s_ = 5.0;
    bool use_tmux_ = true;
    bool remain_on_exit_ = true;
    pid_t pid_ = -1;
};

// ─────────────────────────────────────────────────────────────

class MssnBridge : public rclcpp::Node {
public:
    MssnBridge()
        : rclcpp::Node("mqtt_mssn_bridge",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        const std::string host = param_string(*this, "mqtt.host", "192.168.70.62");
        const int port = static_cast<int>(param_int(*this, "mqtt.port", 1883));
        const int qos = static_cast<int>(param_int(*this, "mqtt.qos", 0));
        const int keepalive = static_cast<int>(param_int(*this, "mqtt.keepalive_s", 30));

        const std::string mode_cmd_topic =
            param_string(*this, "mqtt.mode_cmd_topic", "aoa/ai_guide/nav/mode");
        const std::string nav_cmd_topic =
            param_string(*this, "mqtt.nav_cmd_topic", "aoa/ai_guide/nav/cmd");
        const std::string guide_cmd_topic =
            param_string(*this, "mqtt.guide_cmd_topic", "aoa/ai_guide/guide/cmd");
        const std::string intercept_mode_topic =
            param_string(*this, "mqtt.intercept_mode_topic", "aoa/ai_guide/intercept/mode");
        nav_status_topic_ = param_string(*this, "mqtt.nav_status_topic", "aoa/ai_guide/nav/status");
        guide_status_topic_ =
            param_string(*this, "mqtt.guide_status_topic", "aoa/ai_guide/guide/status");

        mode_state_topic_ = param_string(*this, "topics.mode_state", "aoa/uav/nav_mode");
        tracking_topic_ = param_string(*this, "topics.tracking_mode", "aoa/uav/tracking_mode");
        intercept_cmd_topic_ =
            param_string(*this, "topics.intercept_mode_cmd", "aoa/uav/intercept_mode_cmd");
        const bool intercept_latched = param_bool(*this, "intercept.latch", true);

        // ── 进程管理配置（tmux 会话 + 进程组）──
        enable_process_control_ = param_bool(*this, "mssn.enable_process_control", true);
        const std::string workspace_dir = param_string(*this, "mssn.workspace_dir", ".");
        const std::string setup_shell = param_string(
            *this, "mssn.setup_shell",
            "source /opt/ros/humble/setup.bash && source install/setup.bash");
        const std::string log_dir = param_string(*this, "mssn.log_dir", "/tmp/ros_mqtt_bridge_logs");
        const double stop_timeout_s = param_double(*this, "mssn.stop_timeout_s", 5.0);
        const double status_poll_hz = param_double(*this, "mssn.status_poll_hz", 1.0);
        use_tmux_ = param_bool(*this, "mssn.use_tmux", true);
        const bool remain_on_exit = param_bool(*this, "mssn.tmux_remain_on_exit", true);
        const std::string tmux_bin = param_string(*this, "mssn.tmux_bin", "tmux");
        const std::string session_prefix = param_string(*this, "mssn.tmux_session_prefix", "aoa");

        ::mkdir(log_dir.c_str(), 0755);

        // tmux 是**强依赖**：启用进程管理且 use_tmux 时缺失即 FATAL（不做静默回退）
        if (enable_process_control_ && use_tmux_) {
            std::string ver;
            if (runCmd({tmux_bin, "-V"}, &ver) != 0) {
                RCLCPP_FATAL(get_logger(),
                             "[mqtt_mssn_bridge] tmux 不可用（'%s -V' 执行失败）。"
                             "请先安装：sudo apt install -y tmux；或显式关闭 tmux："
                             "-p mssn.use_tmux:=false（回退 fork+setsid）",
                             tmux_bin.c_str());
                throw std::runtime_error("tmux not available");
            }
            RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] tmux: %s", trimEol(ver).c_str());
        }

        // 构建受管进程组（yaml 单行规格 "command|pattern|session"，见 proc_spec.h）
        const auto build_group = [&](const char* label, const std::vector<std::string>& raw,
                                    std::vector<ManagedProcess>& out) {
            int idx = 0;
            for (const auto& text : raw) {
                ++idx;
                const auto spec = ros_mqtt_bridge::parseProcSpec(
                    text, session_prefix + "_" + label + std::to_string(idx));
                if (!spec.valid) {
                    RCLCPP_WARN(get_logger(),
                                "[mqtt_mssn_bridge] 忽略非法 %s 进程规格 '%s'"
                                "（格式 command|pattern|session）",
                                label, text.c_str());
                    continue;
                }
                ManagedProcess proc;
                proc.configure(spec, std::string(label) + ":" + spec.session, log_dir, setup_shell,
                               workspace_dir, stop_timeout_s, use_tmux_, remain_on_exit, tmux_bin);
                out.push_back(std::move(proc));
            }
        };
        build_group("nav", param_string_array(*this, "mssn.nav_procs", kDefaultNavProcs),
                    nav_procs_);
        build_group("guide", param_string_array(*this, "mssn.guide_procs", kDefaultGuideProcs),
                    guide_procs_);

        // ── ROS 镜像话题：模式（transient_local，缓存最新值给后启动的控制桥）──
        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();
        pub_mode_state_ = create_publisher<std_msgs::msg::String>(mode_state_topic_, latched);
        // 拦截模式命令（迎头/尾追）：默认同样 latched，
        // 使先发命令、后启导航栈（nav/cmd）的时序也能生效
        pub_intercept_cmd_ = create_publisher<std_msgs::msg::String>(
            intercept_cmd_topic_, intercept_latched ? latched : rclcpp::QoS(rclcpp::KeepLast(10)));
        sub_tracking_ = create_subscription<std_msgs::msg::Bool>(
            tracking_topic_, latched, [this](std_msgs::msg::Bool::SharedPtr msg) {
                tracking_ = msg->data;
            });
        publishModeState();   // 启动即广播初始 auto
        publishInterceptMode();   // 若有初始值（latched 场景）

        // ── MQTT 客户端（入站 + 状态出站）──
        using MH = ros_mqtt_bridge::MqttClient::MessageHandler;
        mqtt_ = std::make_unique<ros_mqtt_bridge::MqttClient>(
            "bridge_mssn_" + std::to_string(::getpid()), host, port, keepalive, qos,
            MH([this](const std::string& topic, const std::string& payload) {
                onMqttMessage(topic, payload);
            }));
        mqtt_->addSubscription(mode_cmd_topic, qos);
        mqtt_->addSubscription(intercept_mode_topic, qos);
        mqtt_->addSubscription(nav_cmd_topic, qos);
        mqtt_->addSubscription(guide_cmd_topic, qos);
        if (!mqtt_->start()) {
            RCLCPP_FATAL(get_logger(), "[mqtt_mssn_bridge] MQTT client start failed");
            throw std::runtime_error("mqtt start failed");
        }

        const int period_ms = std::max(100, static_cast<int>(1000.0 / std::max(0.1, status_poll_hz)));
        timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() { publishStatus(); });

        RCLCPP_INFO(get_logger(),
                    "[mqtt_mssn_bridge] ready: broker=%s:%d\n"
                    "  in : %s | %s | %s | %s\n"
                    "  out: %s | %s\n"
                    "  ros mirror: %s (%s, 初始 auto；模式仅翻译，不启停进程)\n"
                    "  ros intercept: %s (latch=%d)\n"
                    "  process_control=%d use_tmux=%d workspace='%s'",
                    host.c_str(), port, mode_cmd_topic.c_str(), intercept_mode_topic.c_str(),
                    nav_cmd_topic.c_str(), guide_cmd_topic.c_str(), nav_status_topic_.c_str(),
                    guide_status_topic_.c_str(), mode_state_topic_.c_str(),
                    ros_mqtt_bridge::navModeName(mode_), intercept_cmd_topic_.c_str(),
                    static_cast<int>(intercept_latched),
                    static_cast<int>(enable_process_control_), static_cast<int>(use_tmux_),
                    workspace_dir.c_str());
        for (const std::vector<ManagedProcess>* group : {&nav_procs_, &guide_procs_}) {
            for (const auto& p : *group) {
                RCLCPP_INFO(get_logger(), "  [%s] session='%s' cmd='%s'", p.name().c_str(),
                            p.session().c_str(), p.command().c_str());
            }
        }
    }

private:
    // ── MQTT 入站分发 ──
    void onMqttMessage(const std::string& topic, const std::string& payload)
    {
        if (topic.find("intercept") != std::string::npos) {
            handleInterceptMode(payload);
        } else if (topic.find("nav/mode") != std::string::npos) {
            handleMode(payload);
        } else if (topic.find("/nav/cmd") != std::string::npos) {
            handleGroupSwitch(payload, "nav", nav_procs_);
        } else if (topic.find("/guide/cmd") != std::string::npos) {
            handleGroupSwitch(payload, "guide", guide_procs_);
        } else {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] unexpected topic: %s", topic.c_str());
        }
    }

    /** 拦截模式入口（迎头 / 尾追）= **纯翻译器**：
     *  MQTT `aoa/ai_guide/intercept/mode` → ROS `aoa/uav/intercept_mode_cmd`。
     *  取值：`head-on`（兼容 headon / head_on）| `tail`，大小写不敏感；
     *  非法值 → 告警、**不发布**（下游 uav_guide_loop_node 要求严格取值）。
     *  发前统一规范化为 `head-on` / `tail`，避免下游歧义。 */
    void handleInterceptMode(const std::string& payload)
    {
        const std::string mode = ros_mqtt_bridge::extractInterceptMode(payload);
        if (mode.empty()) {
            RCLCPP_WARN(get_logger(),
                        "[mqtt_mssn_bridge] invalid intercept mode payload: %s "
                        "(期望 head-on | tail)",
                        payload.c_str());
            return;
        }
        const bool changed = (mode != intercept_mode_);
        intercept_mode_ = mode;
        publishInterceptMode();
        if (changed) {
            RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] intercept mode -> %s (mirrored to %s)",
                        mode.c_str(), intercept_cmd_topic_.c_str());
        } else {
            RCLCPP_DEBUG(get_logger(), "[mqtt_mssn_bridge] intercept mode unchanged: %s",
                         mode.c_str());
        }
    }

    void publishInterceptMode()
    {
        if (intercept_mode_.empty()) return;   // 未收到过命令 → 不发（下游用 yaml 默认值）
        std_msgs::msg::String msg;
        msg.data = intercept_mode_;   // 规范形式：head-on | tail
        pub_intercept_cmd_->publish(msg);
    }

    /** 模式入口 = **纯翻译器**：只把 MQTT mode 转成 ROS `aoa/uav/nav_mode`，
     *  不启停任何进程（进程完全由 nav/cmd、guide/cmd 控制）。 */
    void handleMode(const std::string& payload)
    {
        const std::string raw = extractModeTolerant(payload);
        ros_mqtt_bridge::NavMode parsed;
        if (raw.empty() || !ros_mqtt_bridge::parseNavMode(raw, parsed)) {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] invalid nav_mode payload: %s "
                                     "(期望 auto | setpoint | guidance)", payload.c_str());
            return;
        }
        const auto previous = mode_;
        mode_ = parsed;
        publishModeState();
        RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] mode %s -> %s (mirrored to %s)",
                    ros_mqtt_bridge::navModeName(previous), ros_mqtt_bridge::navModeName(mode_),
                    mode_state_topic_.c_str());
    }

    /** 进程组启停：start=true 按序启动；start=false **逆序**停止
     *  （先停消费者 mqtt_control_bridge，再停生产者导航栈，避免停机期间的无效指令）。 */
    void handleGroupSwitch(const std::string& payload, const char* label,
                           std::vector<ManagedProcess>& procs)
    {
        bool start = false;
        std::string err;
        if (!ros_mqtt_bridge::decodeStateBool(payload, "start", start, err)) {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] bad %s cmd payload: %s (%s)", label,
                        payload.c_str(), err.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] %s cmd: %s（%zu 个受管进程）", label,
                    start ? "start" : "stop", procs.size());
        if (!enable_process_control_) {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] process control disabled, %s ignored",
                        label);
            return;
        }
        if (start) {
            for (auto& p : procs) p.start(get_logger());
        } else {
            for (auto it = procs.rbegin(); it != procs.rend(); ++it) it->stop(get_logger());
        }
    }

    void publishModeState()
    {
        std_msgs::msg::String msg;
        msg.data = ros_mqtt_bridge::navModeName(mode_);   // 纯模式名（std_msgs/String）
        pub_mode_state_->publish(msg);
    }

    void publishStatus()
    {
        const bool nav_running = groupRunning(nav_procs_);
        const bool guide_running = groupRunning(guide_procs_);
        if (nav_running != last_nav_running_ || guide_running != last_guide_running_) {
            RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] status nav=%d guide=%d", 
                        static_cast<int>(nav_running), static_cast<int>(guide_running));
            last_nav_running_ = nav_running;
            last_guide_running_ = guide_running;
        }
        std::ostringstream nav_json;
        nav_json << "{\"running\":" << (nav_running ? "true" : "false")
                 << ",\"mode\":\"" << ros_mqtt_bridge::navModeName(mode_) << "\""
                 << ",\"intercept\":\"" << intercept_mode_ << "\""
                 << ",\"procs\":" << groupProcsJson(nav_procs_) << "}";
        publishRaw(nav_status_topic_, nav_json.str(), "nav/status");

        std::ostringstream guid_json;
        guid_json << "{\"running\":" << (guide_running ? "true" : "false")
                  << ",\"mode\":\"" << ros_mqtt_bridge::navModeName(mode_) << "\""
                  << ",\"tracking\":" << (tracking_ ? "true" : "false")
                  << ",\"procs\":" << groupProcsJson(guide_procs_) << "}";
        publishRaw(guide_status_topic_, guid_json.str(), "guide/status");
    }

    /// 组内全部进程都在跑才算 running（空组 = false）
    static bool groupRunning(const std::vector<ManagedProcess>& procs)
    {
        if (procs.empty()) return false;
        for (const auto& p : procs) {
            if (!p.running()) return false;
        }
        return true;
    }

    static std::string groupProcsJson(const std::vector<ManagedProcess>& procs)
    {
        std::ostringstream o;
        o << "[";
        for (std::size_t i = 0; i < procs.size(); ++i) {
            if (i != 0) o << ",";
            o << "{\"name\":\"" << procs[i].session() << "\",\"running\":"
              << (procs[i].running() ? "true" : "false") << "}";
        }
        o << "]";
        return o.str();
    }

    void publishRaw(const std::string& topic, const std::string& payload, const char* what)
    {
        if (mqtt_->publish(topic, payload)) {
            RCLCPP_DEBUG(get_logger(), "[mqtt_mssn_bridge] %s → %s: %s", what, topic.c_str(),
                         payload.c_str());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_mssn_bridge] publish %s failed (broker down?)", what);
        }
    }

    // ── 成员 ──
    std::unique_ptr<ros_mqtt_bridge::MqttClient> mqtt_;
    std::vector<ManagedProcess> nav_procs_, guide_procs_;

    std::string mode_state_topic_, tracking_topic_, nav_status_topic_, guide_status_topic_;
    std::string intercept_cmd_topic_;
    std::string intercept_mode_;   // 最近一次有效拦截模式（空 = 未收到过）
    ros_mqtt_bridge::NavMode mode_ = ros_mqtt_bridge::NavMode::Auto;
    bool enable_process_control_ = true;
    bool use_tmux_ = true;
    bool tracking_ = false;
    bool last_nav_running_ = false;
    bool last_guide_running_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_intercept_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tracking_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MssnBridge>());
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("mqtt_mssn_bridge"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
