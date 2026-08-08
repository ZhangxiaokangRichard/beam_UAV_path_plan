/**
 * @file udp_mssn_bridge.cpp
 * @brief 服务入口节点（移植自 ros_mqtt_bridge::mqtt_mssn_bridge，通信改为本地 UDP）。
 *
 * 功能：
 *   - 监听本地 cmd.local_port（默认 11302）收 UDP 指令 JSON，一包一 JSON：
 *       {"type":"nav",  "start":true}   启动/停止 uav_guide.launch
 *       {"type":"guide","start":true}   启动/停止可配置辅助进程（默认无）
 *       {"type":"mode", "mode":"head-on"} | {"type":"mode","mode":"tail"}
 *       切换迎头/尾追拦截模式（发布 /uav_guide/intercept_mode_cmd）
 *   - 订阅 /uav_guide/intercept_mode（模式状态）、/uav/state、/target/state（位置），
 *     读取 /uav_guide/tracking_mode（tracking 状态）
 *   - 定时（status_poll_hz）通过 UDP 向 cmd.remote_host:remote_port 回传状态 JSON
 *     （{"type":"status", ...}，默认不开启）
 *
 * 进程管理：pgrep/pkill + xterm（有 DISPLAY）/ tmux（无头）/ 后台日志兜底。
 * JSON 编解码复用 ros_udp_bridge/udp_codec（sanitizeJsonBytes + property_tree）。
 * 自包含实现，不依赖 ros_mqtt_bridge（解耦）。
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atomic>
#include <condition_variable>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <queue>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "ros_udp_bridge/udp_codec.h"
#include "ros_udp_bridge/udp_socket.h"

namespace {

// ---------------------------------------------------------------------------
// 基础工具（与 mqtt_mssn_bridge 一致）
// ---------------------------------------------------------------------------

std::string trim(const std::string& s)
{
    const char* ws = " \t\r\n";
    const size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::string expandHome(const std::string& path)
{
    if (path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home) return std::string(home) + path.substr(1);
    }
    return path;
}

/// 首字符括号化，避免 pgrep/pkill -f 自匹配。
std::string bracketFirst(const std::string& pattern)
{
    if (pattern.empty()) return pattern;
    return "[" + pattern.substr(0, 1) + "]" + pattern.substr(1);
}

bool pgrep(const std::string& pattern)
{
    const std::string cmd = "pgrep -f '" + bracketFirst(pattern) + "' >/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    return WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
}

void pkillMatch(const std::string& pattern)
{
    const std::string cmd = "pkill -f '" + bracketFirst(pattern) + "' >/dev/null 2>&1 || true";
    std::system(cmd.c_str());
}

pid_t runDetached(const std::string& shell_cmd, const std::string& log_file)
{
    const pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        setsid();
        const int fd = open(log_file.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        }
        execl("/bin/bash", "bash", "-c", shell_cmd.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    return pid;
}

bool tmuxAvailable()
{
    const int rc = std::system("command -v tmux >/dev/null 2>&1");
    return WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
}

std::string collapseToLine(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) out += (c == '\n') ? ';' : c;
    return out;
}

std::string sanitizeSessionName(const std::string& s)
{
    std::string out;
    out.reserve(s.size());
    for (char c : s) out += (c == '.' || c == ':') ? '_' : c;
    return out;
}

void tmuxKillSession(const std::string& session)
{
    const std::string cmd = "tmux kill-session -t '" + session + "' >/dev/null 2>&1 || true";
    std::system(cmd.c_str());
}

void tmuxNewSession(const std::string& session, const std::string& shell_cmd)
{
    tmuxKillSession(session);
    const std::string line = collapseToLine(shell_cmd);
    const std::string cmd = "tmux new-session -d -s '" + session + "' \"bash -lc '" +
                            line + "'\" >/dev/null 2>&1";
    std::system(cmd.c_str());
}

/// 打开终端运行命令：有 DISPLAY 用 xterm，无头用 tmux，兜底后台日志。
pid_t launchInTerminal(const std::string& title, const std::string& shell_cmd,
                       const std::string& log_file, std::string& session_out)
{
    session_out.clear();
    const char* display = std::getenv("DISPLAY");
    if (display && display[0] != '\0') {
        const pid_t pid = fork();
        if (pid < 0) return -1;
        if (pid == 0) {
            setsid();
            execlp("xterm", "xterm", "-hold", "-T", title.c_str(),
                   "-geometry", "140x36", "-e", "bash", "-c", shell_cmd.c_str(),
                   static_cast<char*>(nullptr));
            _exit(127);
        }
        return pid;
    }
    if (tmuxAvailable()) {
        session_out = sanitizeSessionName(title);
        tmuxNewSession(session_out, shell_cmd);
        ROS_INFO("[udp_mssn_bridge] 无 DISPLAY，以 tmux 会话 '%s' 启动"
                 "（查看: tmux attach -t %s）", session_out.c_str(), session_out.c_str());
        return 0;
    }
    ROS_WARN("[udp_mssn_bridge] 无 DISPLAY 且无 tmux，回退为后台日志方式启动 %s",
             title.c_str());
    return runDetached(shell_cmd, log_file);
}

std::string buildLaunchCmd(const std::string& setup_shell, const std::string& exec_part)
{
    const std::string shell = trim(setup_shell);
    if (shell.empty()) return exec_part;
    return shell + " && " + exec_part;
}

void stopProcessGroup(pid_t pid)
{
    if (pid <= 0) return;
    if (kill(-pid, SIGTERM) != 0) return;
    for (int i = 0; i < 30; ++i) {
        if (kill(pid, 0) != 0) return;
        usleep(100000);
    }
    kill(-pid, SIGKILL);
}

}  // namespace

// ---------------------------------------------------------------------------
// 配置
// ---------------------------------------------------------------------------

struct Config {
    // UDP 接收（本地指令入口）
    std::string local_host = "0.0.0.0";
    int local_port = 11302;
    int recv_buf = 1024;
    int recv_timeout_ms = 100;

    // UDP 状态回传（可选）
    std::string remote_host = "127.0.0.1";
    int remote_port = 11303;
    bool report_enable = false;
    double status_hz = 2.0;

    // 进程管理
    std::string setup_shell;
    std::string launch_pkg = "uav_guide";
    std::string launch_file = "uav_guide.launch";
    std::string launch_match = "uav_guide.launch";
    std::string guide_pkg;         // 辅助进程（guide 命令），默认空=不启用
    std::string guide_exec;
    std::string guide_match;
    std::string log_dir = "~/.ros/udp_mssn_bridge/logs";
    int launch_timeout_s = 30;
    int guide_timeout_s = 15;
    bool ensure_nav_on_mode = true;
};

Config loadConfig(ros::NodeHandle& nh)
{
    Config c;
    nh.param("cmd/local_host", c.local_host, c.local_host);
    nh.param("cmd/local_port", c.local_port, c.local_port);
    nh.param("cmd/recv_buf_size", c.recv_buf, c.recv_buf);
    nh.param("cmd/recv_timeout_ms", c.recv_timeout_ms, c.recv_timeout_ms);

    nh.param("cmd/remote_host", c.remote_host, c.remote_host);
    nh.param("cmd/remote_port", c.remote_port, c.remote_port);
    nh.param("cmd/report_enable", c.report_enable, c.report_enable);
    nh.param("cmd/status_poll_hz", c.status_hz, c.status_hz);

    nh.param("mssn/setup_shell", c.setup_shell,
             std::string("source /opt/ros/noetic/setup.bash\n"
                         "source ~/catkin_ws/devel/setup.bash"));
    nh.param("mssn/launch_package", c.launch_pkg, c.launch_pkg);
    nh.param("mssn/launch_file", c.launch_file, c.launch_file);
    nh.param("mssn/launch_match", c.launch_match, c.launch_match);
    nh.param("mssn/guide_package", c.guide_pkg, c.guide_pkg);
    nh.param("mssn/guide_executable", c.guide_exec, c.guide_exec);
    nh.param("mssn/guide_match", c.guide_match, c.guide_match);
    nh.param("mssn/log_dir", c.log_dir, c.log_dir);
    nh.param("mssn/launch_startup_timeout_s", c.launch_timeout_s, c.launch_timeout_s);
    nh.param("mssn/guide_startup_timeout_s", c.guide_timeout_s, c.guide_timeout_s);
    nh.param("mssn/ensure_nav_on_mode", c.ensure_nav_on_mode, c.ensure_nav_on_mode);

    c.log_dir = expandHome(c.log_dir);
    return c;
}

// ---------------------------------------------------------------------------
// 节点主体
// ---------------------------------------------------------------------------

class UdpMssnBridge {
public:
    UdpMssnBridge(ros::NodeHandle& nh, const Config& cfg)
        : nh_(nh), cfg_(cfg)
    {
        const std::string mkdir = "mkdir -p '" + cfg_.log_dir + "' >/dev/null 2>&1 || true";
        std::system(mkdir.c_str());

        // ROS 侧：订阅模式状态、发布模式切换命令
        mode_sub_ = nh_.subscribe("/uav_guide/intercept_mode", 1,
                                  &UdpMssnBridge::onInterceptMode, this);
        mode_cmd_pub_ = nh_.advertise<std_msgs::String>("/uav_guide/intercept_mode_cmd", 1);

        // ROS 侧：订阅本机/目标位姿（用于状态上报）
        uav_sub_ = nh_.subscribe("/uav/state", 1, &UdpMssnBridge::onUavState, this);
        target_sub_ = nh_.subscribe("/target/state", 1, &UdpMssnBridge::onTargetState, this);

        // UDP 接收线程
        recv_thread_ = std::thread(&UdpMssnBridge::recvLoop, this);
        // 命令处理 / 状态上报线程
        worker_ = std::thread(&UdpMssnBridge::workerLoop, this);
        if (cfg_.report_enable) poller_ = std::thread(&UdpMssnBridge::pollerLoop, this);

        ROS_INFO("[udp_mssn_bridge] ready, listen udp %s:%d report=%s(%s:%d)",
                 cfg_.local_host.c_str(), cfg_.local_port,
                 cfg_.report_enable ? "on" : "off",
                 cfg_.remote_host.c_str(), cfg_.remote_port);
    }

    ~UdpMssnBridge() { shutdown(); }

    void shutdown()
    {
        stop_ = true;
        cv_.notify_all();
        if (recv_thread_.joinable()) recv_thread_.join();
        if (worker_.joinable()) worker_.join();
        if (poller_.joinable()) poller_.join();
    }

private:
    struct CmdMsg { std::string payload; std::string from_ip; };

    // ---- UDP 接收线程 ----

    void recvLoop()
    {
        ros_udp_bridge::UdpSocket sock(cfg_.recv_buf, cfg_.recv_timeout_ms);
        if (!sock.bind(cfg_.local_host, static_cast<std::uint16_t>(cfg_.local_port))) {
            ROS_FATAL("[udp_mssn_bridge] bind %s:%d failed",
                      cfg_.local_host.c_str(), cfg_.local_port);
            return;
        }
        ROS_INFO("[udp_mssn_bridge] listening %s:%d",
                 cfg_.local_host.c_str(), cfg_.local_port);
        std::string data, ip;
        std::uint16_t port = 0;
        while (!stop_) {
            if (!sock.recv(data, ip, port)) continue;
            if (data.empty()) continue;
            {
                std::lock_guard<std::mutex> lock(q_mutex_);
                q_.push({data, ip});
            }
            cv_.notify_one();
        }
    }

    // ---- 命令处理（worker 线程，串行） ----

    void workerLoop()
    {
        std::unique_lock<std::mutex> lock(q_mutex_);
        while (!stop_) {
            cv_.wait(lock, [this] { return stop_ || !q_.empty(); });
            while (!q_.empty()) {
                const CmdMsg msg = q_.front();
                q_.pop();
                lock.unlock();
                handleCmd(msg);
                lock.lock();
            }
        }
    }

    void handleCmd(const CmdMsg& msg)
    {
        ros_udp_bridge::UdpCmdMsg cmd;
        std::string err;
        if (!ros_udp_bridge::parseUdpCmd(msg.payload, cmd, err)) {
            ROS_WARN("[udp_mssn_bridge] bad cmd from %s: %s", msg.from_ip.c_str(), err.c_str());
            return;
        }
        if (cmd.type == "nav") {
            ROS_INFO("[udp_mssn_bridge] cmd nav=%s", cmd.start ? "start" : "stop");
            if (cmd.start) ensureLaunch();
            else stopLaunch();
        } else if (cmd.type == "guide") {
            ROS_INFO("[udp_mssn_bridge] cmd guide=%s", cmd.start ? "start" : "stop");
            if (cmd.start) startGuide();
            else stopGuide();
        } else if (cmd.type == "mode") {
            ROS_INFO("[udp_mssn_bridge] cmd set intercept mode -> %s", cmd.mode.c_str());
            if (cfg_.ensure_nav_on_mode) ensureLaunch();
            std_msgs::String out;
            out.data = cmd.mode;
            mode_cmd_pub_.publish(out);
        }
        publishStatus();
    }

    // ---- 进程管理 ----

    void ensureLaunch()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pgrep(cfg_.launch_match)) {
            ROS_INFO("[udp_mssn_bridge] uav_guide.launch already running");
            return;
        }
        const std::string cmd = buildLaunchCmd(
            cfg_.setup_shell,
            "exec roslaunch " + cfg_.launch_pkg + " " + cfg_.launch_file);
        const std::string log = cfg_.log_dir + "/uav_guide_launch.log";
        ROS_INFO("[udp_mssn_bridge] starting roslaunch %s %s", cfg_.launch_pkg.c_str(),
                 cfg_.launch_file.c_str());
        nav_pid_ = launchInTerminal("uav_guide.launch", cmd, log, nav_session_);
        for (int i = 0; i < cfg_.launch_timeout_s * 2; ++i) {
            if (pgrep(cfg_.launch_match)) {
                ROS_INFO("[udp_mssn_bridge] uav_guide.launch started");
                return;
            }
            usleep(500000);
        }
        ROS_ERROR("[udp_mssn_bridge] uav_guide.launch start timeout, check %s", log.c_str());
    }

    void startGuide()
    {
        if (cfg_.guide_pkg.empty() || cfg_.guide_exec.empty()) {
            ROS_WARN("[udp_mssn_bridge] guide 辅助进程未配置（mssn/guide_package, mssn/guide_executable），忽略");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!cfg_.guide_match.empty() && pgrep(cfg_.guide_match)) {
            ROS_INFO("[udp_mssn_bridge] guide 辅助进程 already running");
            return;
        }
        const std::string cmd = buildLaunchCmd(
            cfg_.setup_shell,
            "exec rosrun " + cfg_.guide_pkg + " " + cfg_.guide_exec);
        const std::string log = cfg_.log_dir + "/guide.log";
        ROS_INFO("[udp_mssn_bridge] starting guide 辅助进程 %s/%s",
                 cfg_.guide_pkg.c_str(), cfg_.guide_exec.c_str());
        guide_pid_ = launchInTerminal("guide_aux", cmd, log, guide_session_);
        for (int i = 0; i < cfg_.guide_timeout_s * 2; ++i) {
            if (!cfg_.guide_match.empty() && pgrep(cfg_.guide_match)) {
                ROS_INFO("[udp_mssn_bridge] guide 辅助进程 started");
                return;
            }
            usleep(500000);
        }
    }

    void stopGuide()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!guide_session_.empty()) { tmuxKillSession(guide_session_); guide_session_.clear(); }
        else stopProcessGroup(guide_pid_);
        if (!cfg_.guide_match.empty()) pkillMatch(cfg_.guide_match);
        guide_pid_ = -1;
        ROS_INFO("[udp_mssn_bridge] guide 辅助进程 stopped");
    }

    void stopLaunch()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!nav_session_.empty()) { tmuxKillSession(nav_session_); nav_session_.clear(); }
        else stopProcessGroup(nav_pid_);
        pkillMatch(cfg_.launch_match);
        nav_pid_ = -1;
        ROS_INFO("[udp_mssn_bridge] uav_guide.launch stopped");
    }

    // ---- 状态上报 ----

    void publishStatus()
    {
        bool nav = false, guide = false, tracking = false;
        std::string mode;
        double ux = 0, uy = 0, uz = 0, tx = 0, ty = 0, tz = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            nav = pgrep(cfg_.launch_match);
            guide = (!cfg_.guide_match.empty()) && pgrep(cfg_.guide_match);
            mode = intercept_mode_;
            ux = uav_x_; uy = uav_y_; uz = uav_z_;
            tx = tgt_x_; ty = tgt_y_; tz = tgt_z_;
        }
        ros::param::get("/uav_guide/tracking_mode", tracking);

        const std::string json = ros_udp_bridge::encodeStatusReport(
            nav, guide, tracking, mode, ux, uy, uz, tx, ty, tz);
        ROS_DEBUG("[udp_mssn_bridge] status: %s", json.c_str());
        if (!cfg_.report_enable) return;
        std::lock_guard<std::mutex> lock(send_mutex_);
        send_sock_.send(cfg_.remote_host, static_cast<std::uint16_t>(cfg_.remote_port), json);
    }

    void pollerLoop()
    {
        const double hz = cfg_.status_hz > 0.1 ? cfg_.status_hz : 2.0;
        ros::Rate rate(hz);
        while (ros::ok() && !stop_) {
            publishStatus();
            rate.sleep();
        }
    }

    void onInterceptMode(const std_msgs::String::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        intercept_mode_ = msg->data;
    }

    void onUavState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        uav_x_ = msg->pose.pose.position.x;
        uav_y_ = msg->pose.pose.position.y;
        uav_z_ = msg->pose.pose.position.z;
    }

    void onTargetState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tgt_x_ = msg->pose.pose.position.x;
        tgt_y_ = msg->pose.pose.position.y;
        tgt_z_ = msg->pose.pose.position.z;
    }

    // ---- 成员 ----

    ros::NodeHandle& nh_;
    const Config& cfg_;
    ros::Subscriber mode_sub_;
    ros::Publisher mode_cmd_pub_;
    ros::Subscriber uav_sub_;
    ros::Subscriber target_sub_;

    std::mutex mutex_;
    pid_t nav_pid_ = -1;
    pid_t guide_pid_ = -1;
    std::string nav_session_;
    std::string guide_session_;
    std::string intercept_mode_ = "unknown";
    double uav_x_ = 0, uav_y_ = 0, uav_z_ = 0;
    double tgt_x_ = 0, tgt_y_ = 0, tgt_z_ = 0;

    std::mutex q_mutex_;
    std::queue<CmdMsg> q_;
    std::condition_variable cv_;
    std::thread recv_thread_;
    std::thread worker_;
    std::thread poller_;
    std::atomic<bool> stop_{false};

    std::mutex send_mutex_;
    ros_udp_bridge::UdpSocket send_sock_{4096, 100};
};

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_mssn_bridge");
    ros::NodeHandle nh;
    const Config cfg = loadConfig(nh);

    UdpMssnBridge bridge(nh, cfg);

    ros::spin();
    bridge.shutdown();
    return 0;
}
