/**
 * @file mqtt_mssn_bridge.cpp
 * @brief 导航任务远程控制桥：MQTT 命令 → ROS 进程/话题管理，状态回报。
 *
 * 融合原 guide_mqtt_bridge（Python）能力到 ros_mqtt_bridge：
 *   - 订阅 aoa/ai_guide/ 命令话题，管理 uav_guide.launch 与 mqtt_waypoint_bridge
 *     进程的启停（xterm 弹窗运行，无图形会话时后台日志）。
 *   - 通过 ROS Publisher 直接切换拦截模式（等效 rostopic pub -1）。
 *   - 订阅 /uav_guide/intercept_mode、读取 /uav_guide/tracking_mode；
 *     订阅 /uav/state、/target/state、/uav/planned_path 上报状态与路径。
 *   - 以 status_poll_hz（默认 2Hz）发布 aoa/ai_guide/ 状态话题（JSON）。
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <uav_guide/UavPlannedPath.h>

#include <array>
#include <atomic>
#include <cctype>
#include <condition_variable>
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
#include <vector>

#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mqtt_client.h"

namespace {

// ---------------------------------------------------------------------------
// 基础工具
// ---------------------------------------------------------------------------

std::string trim(const std::string& s)
{
    const char* ws = " \t\r\n";
    const size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
}

std::string toLower(std::string s)
{
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string normalizeMode(const std::string& payload)
{
    const std::string t = toLower(trim(payload));
    if (t == "head-on" || t == "headon" || t == "head_on") return "head-on";
    if (t == "tail") return "tail";
    return "";
}

std::string expandHome(const std::string& path)
{
    if (path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home) return std::string(home) + path.substr(1);
    }
    return path;
}

// ---------------------------------------------------------------------------
// 进程管理
// ---------------------------------------------------------------------------

/// 将首字符括号化（如 uav_guide.launch → [u]av_guide.launch），
/// 避免 pgrep/pkill -f 匹配到自身 sh -c 调用进程的 cmdline（自匹配）。
std::string bracketFirst(const std::string& pattern)
{
    if (pattern.empty()) return pattern;
    return "[" + pattern.substr(0, 1) + "]" + pattern.substr(1);
}

/// 按命令行模式匹配进程是否存在（对应原 Python 的 pgrep）。
bool pgrep(const std::string& pattern)
{
    const std::string cmd = "pgrep -f '" + bracketFirst(pattern) + "' >/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    return WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
}

/// 按命令行模式强杀残留进程（兜底，避免孤儿节点）。
void pkillMatch(const std::string& pattern)
{
    const std::string cmd = "pkill -f '" + bracketFirst(pattern) + "' >/dev/null 2>&1 || true";
    std::system(cmd.c_str());
}

/// fork + setsid 分离启动子进程，stdout/stderr 追加到 log_file。返回子进程 PID。
pid_t runDetached(const std::string& shell_cmd, const std::string& log_file)
{
    const pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        setsid();  // 新会话，便于 killpg 整组停止
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

/// 打开 xterm 终端运行命令（用户可实时观察 launch / rosrun 输出）。
/// 无 DISPLAY（无图形会话）时回退为后台日志方式。返回进程 PID。
pid_t launchInTerminal(const std::string& title, const std::string& shell_cmd,
                       const std::string& log_file)
{
    const char* display = std::getenv("DISPLAY");
    if (display && display[0] != '\0') {
        const pid_t pid = fork();
        if (pid < 0) return -1;
        if (pid == 0) {
            setsid();
            // -hold：进程退出后保留窗口，便于查看启动错误；停止时由进程组信号一并关闭
            execlp("xterm", "xterm", "-hold", "-T", title.c_str(),
                   "-geometry", "140x36", "-e", "bash", "-c", shell_cmd.c_str(),
                   static_cast<char*>(nullptr));
            _exit(127);
        }
        return pid;
    }
    ROS_WARN("[mqtt_mssn_bridge] DISPLAY 未设置，回退为后台日志方式启动 %s",
             title.c_str());
    return runDetached(shell_cmd, log_file);
}

/// 拼接启动命令：去除 setup_shell 首尾空白（YAML 块标量自带尾随换行，
/// 直接拼 " && exec" 会产生 "行首 &&" 的 bash 语法错误）。
std::string buildLaunchCmd(const std::string& setup_shell, const std::string& exec_part)
{
    const std::string shell = trim(setup_shell);
    if (shell.empty()) return exec_part;
    return shell + " && " + exec_part;
}

/// 停止某进程组：先 SIGTERM 优雅退出，超时后 SIGKILL。
void stopProcessGroup(pid_t pid)
{
    if (pid <= 0) return;
    if (kill(-pid, SIGTERM) != 0) return;
    for (int i = 0; i < 30; ++i) {  // 最多等 3s
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
    std::string host;
    int port = 1883;
    int qos = 0;
    int keepalive_s = 30;

    std::string nav_cmd_topic;
    std::string guide_cmd_topic;
    std::string mode_cmd_topic;
    std::string nav_status_topic;
    std::string guide_status_topic;
    std::string agg_status_topic;
    std::string planedpath_topic;

    std::string setup_shell;
    std::string launch_pkg;
    std::string launch_file;
    std::string launch_match;
    std::string waypoint_match;
    std::string log_dir;
    int launch_timeout_s = 30;
    int waypoint_timeout_s = 15;
    double status_hz = 2.0;
    bool ensure_nav_on_mode = true;
};

Config loadConfig(ros::NodeHandle& nh)
{
    Config c;
    nh.param("mqtt/host", c.host, std::string("192.168.70.62"));
    nh.param("mqtt/port", c.port, 1883);
    nh.param("mqtt/qos", c.qos, 0);
    nh.param("mqtt/keepalive_s", c.keepalive_s, 30);

    nh.param("mqtt/nav_cmd_topic", c.nav_cmd_topic, std::string("aoa/ai_guide/nav/cmd"));
    nh.param("mqtt/guide_cmd_topic", c.guide_cmd_topic, std::string("aoa/ai_guide/guide/cmd"));
    nh.param("mqtt/mode_cmd_topic", c.mode_cmd_topic, std::string("aoa/ai_guide/guide/mode_cmd"));
    nh.param("mqtt/nav_status_topic", c.nav_status_topic, std::string("aoa/ai_guide/nav/status"));
    nh.param("mqtt/guide_status_topic", c.guide_status_topic, std::string("aoa/ai_guide/guide/status"));
    nh.param("mqtt/agg_status_topic", c.agg_status_topic, std::string("aoa/ai_guide/status"));
    nh.param("mqtt/planedpath_topic", c.planedpath_topic,
             std::string("aoa/ai_guide/guide/planedpath"));

    nh.param("mssn/setup_shell", c.setup_shell,
             std::string("source /opt/ros/noetic/setup.bash\n"
                         "source ~/catkin_ws/devel/setup.bash"));
    nh.param("mssn/launch_package", c.launch_pkg, std::string("uav_guide"));
    nh.param("mssn/launch_file", c.launch_file, std::string("uav_guide.launch"));
    nh.param("mssn/launch_match", c.launch_match, std::string("uav_guide.launch"));
    nh.param("mssn/waypoint_match", c.waypoint_match, std::string("mqtt_waypoint_bridge"));
    nh.param("mssn/log_dir", c.log_dir, std::string("~/.ros/mqtt_mssn_bridge/logs"));
    nh.param("mssn/launch_startup_timeout_s", c.launch_timeout_s, 30);
    nh.param("mssn/waypoint_startup_timeout_s", c.waypoint_timeout_s, 15);
    nh.param("mssn/status_poll_hz", c.status_hz, 2.0);
    nh.param("mssn/ensure_nav_on_mode", c.ensure_nav_on_mode, true);

    c.log_dir = expandHome(c.log_dir);
    return c;
}

// ---------------------------------------------------------------------------
// 节点主体
// ---------------------------------------------------------------------------

class MssnBridge {
public:
    MssnBridge(ros::NodeHandle& nh, const Config& cfg)
        : nh_(nh), cfg_(cfg)
    {
        const std::string mkdir = "mkdir -p '" + cfg_.log_dir + "' >/dev/null 2>&1 || true";
        std::system(mkdir.c_str());

        // ROS 侧：订阅拦截模式状态、发布模式切换命令
        mode_sub_ = nh_.subscribe("/uav_guide/intercept_mode", 1,
                                  &MssnBridge::onInterceptMode, this);
        mode_cmd_pub_ = nh_.advertise<std_msgs::String>("/uav_guide/intercept_mode_cmd", 1);

        // ROS 侧：订阅本机/目标位姿与规划路径，用于状态与路径上报
        uav_sub_ = nh_.subscribe("/uav/state", 1, &MssnBridge::onUavState, this);
        target_sub_ = nh_.subscribe("/target/state", 1, &MssnBridge::onTargetState, this);
        planned_sub_ = nh_.subscribe("/uav/planned_path", 1, &MssnBridge::onPlannedPath, this);

        // 命令队列 / 状态轮询线程
        worker_ = std::thread(&MssnBridge::workerLoop, this);
        poller_ = std::thread(&MssnBridge::pollerLoop, this);

        ROS_INFO("[mqtt_mssn_bridge] ready, broker=%s:%d status_hz=%.1f",
                 cfg_.host.c_str(), cfg_.port, cfg_.status_hz);
    }

    ~MssnBridge() { shutdown(); }

    void attachMqtt(ros_mqtt_bridge::MqttClient* mqtt)
    {
        mqtt_ = mqtt;
        mqtt_->addSubscription(cfg_.nav_cmd_topic);
        mqtt_->addSubscription(cfg_.guide_cmd_topic);
        mqtt_->addSubscription(cfg_.mode_cmd_topic);
    }

    /// MQTT 网络线程回调：入队，不阻塞网络线程。
    void onMqttMessage(const std::string& topic, const std::string& payload)
    {
        {
            std::lock_guard<std::mutex> lock(q_mutex_);
            q_.push({topic, payload});
        }
        cv_.notify_one();
    }

    /// 停止工作线程（幂等）。须在 MQTT 客户端析构前调用。
    void shutdown()
    {
        stop_ = true;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (poller_.joinable()) poller_.join();
    }

private:
    struct CmdMsg { std::string topic; std::string payload; };

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
        if (msg.topic == cfg_.nav_cmd_topic) {
            bool start = false;
            std::string err;
            if (!ros_mqtt_bridge::decodeStateBool(msg.payload, "start", start, err)) {
                ROS_WARN("[mqtt_mssn_bridge] bad nav cmd JSON: %s", err.c_str());
                return;
            }
            ROS_INFO("[mqtt_mssn_bridge] cmd nav=%s", start ? "start" : "stop");
            if (start) ensureLaunch();
            else stopLaunch();
        } else if (msg.topic == cfg_.guide_cmd_topic) {
            bool start = false;
            std::string err;
            if (!ros_mqtt_bridge::decodeStateBool(msg.payload, "start", start, err)) {
                ROS_WARN("[mqtt_mssn_bridge] bad guide cmd JSON: %s", err.c_str());
                return;
            }
            ROS_INFO("[mqtt_mssn_bridge] cmd guide=%s", start ? "start" : "stop");
            if (start) {
                ensureLaunch();   // 引导动作依赖导航服务
                startWaypoint();
            } else {
                stopWaypoint();
            }
        } else if (msg.topic == cfg_.mode_cmd_topic) {
            std::string raw;
            std::string err;
            if (!ros_mqtt_bridge::decodeStateString(msg.payload, "mode", raw, err)) {
                ROS_WARN("[mqtt_mssn_bridge] bad mode cmd JSON: %s", err.c_str());
                return;
            }
            const std::string mode = normalizeMode(raw);
            if (mode.empty()) {
                ROS_WARN("[mqtt_mssn_bridge] invalid intercept mode: '%s'",
                         raw.c_str());
                return;
            }
            ROS_INFO("[mqtt_mssn_bridge] cmd set intercept mode -> %s", mode.c_str());
            if (cfg_.ensure_nav_on_mode) ensureLaunch();
            std_msgs::String out;
            out.data = mode;
            mode_cmd_pub_.publish(out);
        } else {
            ROS_WARN("[mqtt_mssn_bridge] unknown cmd topic: %s", msg.topic.c_str());
            return;
        }
        publishStatus();
    }

    // ---- 进程管理 ----

    void ensureLaunch()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pgrep(cfg_.launch_match)) {
            ROS_INFO("[mqtt_mssn_bridge] uav_guide.launch already running");
            return;
        }
        const std::string cmd = buildLaunchCmd(
            cfg_.setup_shell,
            "exec roslaunch " + cfg_.launch_pkg + " " + cfg_.launch_file);
        const std::string log = cfg_.log_dir + "/uav_guide_launch.log";
        ROS_INFO("[mqtt_mssn_bridge] starting roslaunch %s %s (new xterm)",
                 cfg_.launch_pkg.c_str(), cfg_.launch_file.c_str());
        nav_pid_ = launchInTerminal("uav_guide.launch", cmd, log);
        for (int i = 0; i < cfg_.launch_timeout_s * 2; ++i) {
            if (pgrep(cfg_.launch_match)) {
                ROS_INFO("[mqtt_mssn_bridge] uav_guide.launch started");
                return;
            }
            usleep(500000);
        }
        ROS_ERROR("[mqtt_mssn_bridge] uav_guide.launch start timeout, "
                  "check the xterm window or %s", log.c_str());
    }

    void startWaypoint()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pgrep(cfg_.waypoint_match)) {
            ROS_INFO("[mqtt_mssn_bridge] mqtt_waypoint_bridge already running");
            return;
        }
        const std::string cmd = buildLaunchCmd(
            cfg_.setup_shell,
            "exec rosrun ros_mqtt_bridge mqtt_waypoint_bridge");
        const std::string log = cfg_.log_dir + "/mqtt_waypoint_bridge.log";
        ROS_INFO("[mqtt_mssn_bridge] starting mqtt_waypoint_bridge (new xterm)");
        waypoint_pid_ = launchInTerminal("mqtt_waypoint_bridge", cmd, log);
        for (int i = 0; i < cfg_.waypoint_timeout_s * 2; ++i) {
            if (pgrep(cfg_.waypoint_match)) {
                ROS_INFO("[mqtt_mssn_bridge] mqtt_waypoint_bridge started");
                return;
            }
            usleep(500000);
        }
        ROS_ERROR("[mqtt_mssn_bridge] mqtt_waypoint_bridge start timeout, "
                  "check the xterm window or %s", log.c_str());
    }

    void stopWaypoint()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopProcessGroup(waypoint_pid_);
        pkillMatch(cfg_.waypoint_match);
        waypoint_pid_ = -1;
        ROS_INFO("[mqtt_mssn_bridge] mqtt_waypoint_bridge stopped");
    }

    void stopLaunch()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 引导动作依赖导航，先停引导
        stopProcessGroup(waypoint_pid_);
        pkillMatch(cfg_.waypoint_match);
        waypoint_pid_ = -1;

        stopProcessGroup(nav_pid_);
        pkillMatch(cfg_.launch_match);
        nav_pid_ = -1;
        ROS_INFO("[mqtt_mssn_bridge] uav_guide.launch stopped");
    }

    // ---- 状态回报 ----

    void publishStatus()
    {
        bool nav = false, guide = false, tracking = false;
        std::string mode;
        ros_mqtt_bridge::UavStateJson uav, tgt;
        std::vector<std::array<double, 6>> path;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            nav = pgrep(cfg_.launch_match);
            guide = pgrep(cfg_.waypoint_match);
            mode = intercept_mode_;
            uav = uav_state_;
            tgt = target_state_;
            path = planned_path_;
        }
        ros::param::get("/uav_guide/tracking_mode", tracking);

        if (!mqtt_) return;
        // aoa/ai_guide/nav/status：导航服务运行态
        mqtt_->publish(cfg_.nav_status_topic, ros_mqtt_bridge::encodeStateBool("running", nav));
        // aoa/ai_guide/guide/status：引导运行态 + 本机/目标位姿
        mqtt_->publish(cfg_.guide_status_topic,
                       "{\"running\":" + std::string(guide ? "true" : "false") + ","
                       + ros_mqtt_bridge::encodeUavStateField("uav_state", uav) + ","
                       + ros_mqtt_bridge::encodeUavStateField("target_state", tgt) + "}");
        // aoa/ai_guide/status：拦截模式 + TRACKING 状态（聚合）
        mqtt_->publish(cfg_.agg_status_topic,
                       "{\"mode\":\"" + (mode.empty() ? std::string("unknown") : mode)
                       + "\",\"tracking\":" + std::string(tracking ? "true" : "false") + "}");
        // aoa/ai_guide/guide/planedpath：6 维规划路径列表
        mqtt_->publish(cfg_.planedpath_topic,
                       "{" + ros_mqtt_bridge::encodePlannedPathField("planedPath", path) + "}");
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
        uav_state_ = toUavStateJson(*msg);
    }

    void onTargetState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_state_ = toUavStateJson(*msg);
    }

    void onPlannedPath(const uav_guide::UavPlannedPath::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        planned_path_.clear();
        planned_path_.reserve(msg->path.size());
        for (const auto& p : msg->path) {
            planned_path_.push_back({p.x, p.y, p.z, p.yaw, p.pitch, p.curvature});
        }
    }

    static ros_mqtt_bridge::UavStateJson toUavStateJson(const nav_msgs::Odometry& odom)
    {
        ros_mqtt_bridge::UavStateJson s;
        s.px = odom.pose.pose.position.x;
        s.py = odom.pose.pose.position.y;
        s.pz = odom.pose.pose.position.z;
        s.ox = odom.pose.pose.orientation.x;
        s.oy = odom.pose.pose.orientation.y;
        s.oz = odom.pose.pose.orientation.z;
        s.ow = odom.pose.pose.orientation.w;
        s.vx = odom.twist.twist.linear.x;
        s.vy = odom.twist.twist.linear.y;
        s.vz = odom.twist.twist.linear.z;
        return s;
    }

    // ---- 成员 ----

    ros::NodeHandle& nh_;
    const Config& cfg_;
    ros_mqtt_bridge::MqttClient* mqtt_ = nullptr;
    ros::Subscriber mode_sub_;
    ros::Publisher mode_cmd_pub_;
    ros::Subscriber uav_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber planned_sub_;

    std::mutex mutex_;
    pid_t nav_pid_ = -1;
    pid_t waypoint_pid_ = -1;
    std::string intercept_mode_ = "unknown";
    ros_mqtt_bridge::UavStateJson uav_state_;
    ros_mqtt_bridge::UavStateJson target_state_;
    std::vector<std::array<double, 6>> planned_path_;

    std::mutex q_mutex_;
    std::queue<CmdMsg> q_;
    std::condition_variable cv_;
    std::thread worker_;
    std::thread poller_;
    std::atomic<bool> stop_{false};
};

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mqtt_mssn_bridge");
    ros::NodeHandle nh("~");
    const Config cfg = loadConfig(nh);

    MssnBridge bridge(nh, cfg);

    ros_mqtt_bridge::MqttClient mqtt(
        "bridge_mssn_" + std::to_string(::getpid()), cfg.host, cfg.port, cfg.keepalive_s,
        cfg.qos, [&bridge](const std::string& topic, const std::string& payload) {
            bridge.onMqttMessage(topic, payload);
        });
    bridge.attachMqtt(&mqtt);

    if (!mqtt.start()) {
        ROS_FATAL("[mqtt_mssn_bridge] MQTT client start failed");
        return 1;
    }

    ROS_INFO("[mqtt_mssn_bridge] connected to %s:%d", cfg.host.c_str(), cfg.port);

    ros::spin();
    bridge.shutdown();
    return 0;
}
