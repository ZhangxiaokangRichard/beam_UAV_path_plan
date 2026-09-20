/**
 * @file mqtt_mssn_bridge.cpp
 * @brief 任务/模式桥：MQTT `aoa/ai_guide/*` ↔ ROS，兼管导航栈与控制桥的启停
 *
 * 职责：
 *   1) 模式状态入口：MQTT `aoa/ai_guide/nav/mode`（auto | setpoint | guidance）
 *      → ROS 镜像 `aoa/uav/nav_mode`（std_msgs/String，**纯模式名**，latched）
 *      `mqtt_control_bridge` 订阅该话题得到**当前模式状态**（非边沿），并按映射表
 *      把制导消息翻译为 MQTT 指令。
 *   2) `uav_guide_launch.py`（导航栈：planner + 3 个 guide 节点）的启动与停止
 *   3) `mqtt_control_bridge`（出站控制节点）的启动与停止
 *
 * 入站（MQTT → 本节点）：
 *   `aoa/ai_guide/nav/mode`   "guidance" 或 {"mode":"guidance"}   → 转 ROS `aoa/uav/nav_mode`
 *   `aoa/ai_guide/nav/cmd`    {"start":<bool>}                     → 启停导航栈
 *   `aoa/ai_guide/guide/cmd`  {"start":<bool>}                     → 启停控制桥
 * 出站（本节点 → MQTT）：
 *   `aoa/ai_guide/nav/status`   {"running":<bool>}
 *   `aoa/ai_guide/guide/status` {"running":<bool>,"mode":"..."}
 *
 * 进程管理：fork + setsid + `/bin/bash -c "<setup> && exec <cmd>"`，日志落盘；
 * 停止：killpg(SIGTERM) → 超时 SIGKILL，并用 /proc 扫描排除遗留实例。
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

namespace {

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

}  // namespace

namespace {

/// 单个受管进程
class ManagedProcess {
public:
    void configure(std::string name, std::string command, std::string pattern, std::string log_path,
                   std::string setup_shell, std::string workspace_dir, double stop_timeout_s)
    {
        name_ = std::move(name);
        command_ = std::move(command);
        pattern_ = std::move(pattern);
        log_path_ = std::move(log_path);
        setup_shell_ = std::move(setup_shell);
        workspace_dir_ = std::move(workspace_dir);
        stop_timeout_s_ = stop_timeout_s;
    }

    bool start(rclcpp::Logger logger)
    {
        if (processAlive(pid_)) {
            RCLCPP_INFO(logger, "[%s] already running (tracked pid=%d), skip start", name_.c_str(),
                        pid_);
            return true;
        }
        pid_ = -1;   // 已退出但未回收 → 清空并交给下面的 /proc 扫描
        const auto leftovers = matchPids(pattern_);
        if (!leftovers.empty()) {
            std::ostringstream oss;
            for (std::size_t i = 0; i < leftovers.size() && i < 5; ++i)
                oss << (i ? "," : "") << leftovers[i];
            RCLCPP_WARN(logger,
                        "[%s] %zu untracked instance(s) already running (pids=%s), skip start",
                        name_.c_str(), leftovers.size(), oss.str().c_str());
            return true;
        }
        const std::string full = "cd " + workspace_dir_ + " && " + setup_shell_ + " && exec " + command_;

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
        RCLCPP_INFO(logger, "[%s] started pid=%d cmd='%s' log=%s", name_.c_str(), pid_,
                    command_.c_str(), log_path_.c_str());
        return true;
    }

    bool stop(rclcpp::Logger logger)
    {
        bool did_something = false;

        // 1) 已跟踪的子进程：先对整个进程组 SIGTERM，超时再 SIGKILL
        if (pid_ > 0) {
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

        // 2) 扫除遗留实例（脱离进程组的子进程 / 上一轮桥未清理干净）
        //    ⚠ matchPids 已排除自身、同进程组、同会话，不会误杀本节点或调用终端
        auto leftovers = matchPids(pattern_);
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

    bool running(rclcpp::Logger logger) const
    {
        (void)logger;
        if (processAlive(pid_)) return true;
        return !matchPids(pattern_).empty();
    }

    int pid() const { return pid_; }

private:
    std::string name_, command_, pattern_, log_path_, setup_shell_, workspace_dir_;
    double stop_timeout_s_ = 5.0;
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
        nav_status_topic_ = param_string(*this, "mqtt.nav_status_topic", "aoa/ai_guide/nav/status");
        guide_status_topic_ =
            param_string(*this, "mqtt.guide_status_topic", "aoa/ai_guide/guide/status");

        mode_state_topic_ = param_string(*this, "topics.mode_state", "aoa/uav/nav_mode");
        tracking_topic_ = param_string(*this, "topics.tracking_mode", "aoa/uav/tracking_mode");

        // ── 进程管理配置 ──
        enable_process_control_ = param_bool(*this, "mssn.enable_process_control", true);
        const std::string workspace_dir = param_string(*this, "mssn.workspace_dir", ".");
        const std::string setup_shell =
            param_string(*this, "mssn.setup_shell", "source /opt/ros/humble/setup.bash && source install/setup.bash");
        const std::string log_dir = param_string(*this, "mssn.log_dir", "/tmp/ros_mqtt_bridge_logs");
        const double stop_timeout_s = param_double(*this, "mssn.stop_timeout_s", 5.0);
        const double status_poll_hz = param_double(*this, "mssn.status_poll_hz", 1.0);
        start_guide_on_mode_ = param_bool(*this, "mssn.start_guide_on_mode", true);
        stop_guide_on_auto_ = param_bool(*this, "mssn.stop_guide_on_auto", false);

        ::mkdir(log_dir.c_str(), 0755);
        const auto now_s = []() {
            return std::to_string(static_cast<long long>(::time(nullptr)));
        };
        nav_.configure("nav", param_string(*this, "mssn.nav_launch_command",
                                           "ros2 launch uav_guide uav_guide_launch.py"),
                       param_string(*this, "mssn.nav_pgrep_pattern", "uav_guide_launch.py"),
                       log_dir + "/nav_" + now_s() + ".log", setup_shell, workspace_dir, stop_timeout_s);
        guide_.configure("guide", param_string(*this, "mssn.guide_run_command",
                                               "ros2 run ros_mqtt_bridge mqtt_control_bridge"),
                         param_string(*this, "mssn.guide_pgrep_pattern", "mqtt_control_bridge"),
                         log_dir + "/guide_" + now_s() + ".log", setup_shell, workspace_dir,
                         stop_timeout_s);

        // ── ROS 镜像话题：模式（transient_local，缓存最新值给后启动的控制桥）──
        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();
        pub_mode_state_ = create_publisher<std_msgs::msg::String>(mode_state_topic_, latched);
        sub_tracking_ = create_subscription<std_msgs::msg::Bool>(
            tracking_topic_, latched, [this](std_msgs::msg::Bool::SharedPtr msg) {
                tracking_ = msg->data;
            });
        publishModeState();   // 启动即广播初始 auto

        // ── MQTT 客户端（入站 + 状态出站）──
        using MH = ros_mqtt_bridge::MqttClient::MessageHandler;
        mqtt_ = std::make_unique<ros_mqtt_bridge::MqttClient>(
            "bridge_mssn_" + std::to_string(::getpid()), host, port, keepalive, qos,
            MH([this](const std::string& topic, const std::string& payload) {
                onMqttMessage(topic, payload);
            }));
        mqtt_->addSubscription(mode_cmd_topic, qos);
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
                    "  in : %s | %s | %s\n"
                    "  out: %s | %s\n"
                    "  ros mirror: %s (%s, 初始 auto)\n"
                    "  process_control=%d workspace='%s'",
                    host.c_str(), port, mode_cmd_topic.c_str(), nav_cmd_topic.c_str(),
                    guide_cmd_topic.c_str(), nav_status_topic_.c_str(), guide_status_topic_.c_str(),
                    mode_state_topic_.c_str(), ros_mqtt_bridge::navModeName(mode_),
                    static_cast<int>(enable_process_control_), workspace_dir.c_str());
    }

private:
    // ── MQTT 入站分发 ──
    void onMqttMessage(const std::string& topic, const std::string& payload)
    {
        if (topic.find("nav/mode") != std::string::npos) {
            handleMode(payload);
        } else if (topic.find("/nav/cmd") != std::string::npos) {
            handleSwitch(payload, "start", nav_, "nav");
        } else if (topic.find("/guide/cmd") != std::string::npos) {
            handleSwitch(payload, "start", guide_, "guide");
        } else {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] unexpected topic: %s", topic.c_str());
        }
    }

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

        if (!enable_process_control_) return;
        if (mode_ != ros_mqtt_bridge::NavMode::Auto && start_guide_on_mode_) {
            // 非 auto：确保出站控制桥在线（导航栈由显式 nav/cmd 控制）
            guide_.start(get_logger());
        } else if (mode_ == ros_mqtt_bridge::NavMode::Auto && stop_guide_on_auto_) {
            guide_.stop(get_logger());
        }
    }

    void handleSwitch(const std::string& payload, const char* key, ManagedProcess& proc,
                      const char* label)
    {
        bool start = false;
        std::string err;
        if (!ros_mqtt_bridge::decodeStateBool(payload, key, start, err)) {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] bad %s cmd payload: %s (%s)", label,
                        payload.c_str(), err.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] %s cmd: %s", label, start ? "start" : "stop");
        if (!enable_process_control_) {
            RCLCPP_WARN(get_logger(), "[mqtt_mssn_bridge] process control disabled, %s ignored",
                        label);
            return;
        }
        if (start) {
            proc.start(get_logger());
        } else {
            proc.stop(get_logger());
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
        const bool nav_running = nav_.running(get_logger());
        const bool guide_running = guide_.running(get_logger());
        if (nav_running != last_nav_running_ || guide_running != last_guide_running_) {
            RCLCPP_INFO(get_logger(), "[mqtt_mssn_bridge] status nav=%d guide=%d", 
                        static_cast<int>(nav_running), static_cast<int>(guide_running));
            last_nav_running_ = nav_running;
            last_guide_running_ = guide_running;
        }
        publishRaw(nav_status_topic_, ros_mqtt_bridge::encodeStateBool("running", nav_running),
                   "nav/status");
        std::ostringstream oss;
        oss << "{\"running\":" << (guide_running ? "true" : "false")
            << ",\"mode\":\"" << ros_mqtt_bridge::navModeName(mode_) << "\""
            << ",\"tracking\":" << (tracking_ ? "true" : "false") << "}";
        publishRaw(guide_status_topic_, oss.str(), "guide/status");
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
    ManagedProcess nav_, guide_;

    std::string mode_state_topic_, tracking_topic_, nav_status_topic_, guide_status_topic_;
    ros_mqtt_bridge::NavMode mode_ = ros_mqtt_bridge::NavMode::Auto;
    bool enable_process_control_ = true;
    bool start_guide_on_mode_ = true;
    bool stop_guide_on_auto_ = false;
    bool tracking_ = false;
    bool last_nav_running_ = false;
    bool last_guide_running_ = false;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_state_;
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
