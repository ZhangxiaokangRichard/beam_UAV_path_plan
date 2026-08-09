/**
 * @file udp_mssn_bridge.cpp
 * @brief 任务判断 / 命中上报节点（极简版）。
 *
 * 职责（极度简化的工作流程）：
 *  1. 订阅 /uav/state 与 /target/state，以 check_hz（默认 10Hz）计算二者欧氏距离；
 *     距离 < launch_dist_m（默认 7km）→ 发布 /uav/launch_cmd = true（否则 false）；
 *     未到阈值前持续打印监控日志。
 *  2. 订阅 /target/crashed，为 true 时按命中协议通过 UDP socket 发送
 *     含 target_key（参数配置）的 JSON 数据包（encodeHitEvent）。
 *  3. 启动时用 tmux 依次拉起 uav_guide.launch（主循环）与 uav_bridge.launch
 *     （udp_receiver / udp_sender）两个独立窗口。
 *
 * 已移除：11302 指令接收（nav/guide/mode）、复杂进程管理（pgrep/xterm）、状态回传。
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ros_udp_bridge/udp_codec.h"
#include "ros_udp_bridge/udp_socket.h"

namespace {

// ---- 精简 tmux 窗口启动工具 ----

std::string trim(const std::string& s)
{
    const char* ws = " \t\r\n";
    const size_t b = s.find_first_not_of(ws);
    if (b == std::string::npos) return "";
    const size_t e = s.find_last_not_of(ws);
    return s.substr(b, e - b + 1);
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

/// 用 tmux 启动一个独立窗口。
void launchTmuxWindow(const std::string& title, const std::string& shell_cmd)
{
    tmuxNewSession(sanitizeSessionName(title), shell_cmd);
}

}  // namespace

// ---------------------------------------------------------------------------
// 配置
// ---------------------------------------------------------------------------

struct Config {
    // 话题
    std::string uav_state_topic = "/uav/state";
    std::string target_state_topic = "/target/state";
    std::string launch_cmd_topic = "/uav/launch_cmd";
    std::string crashed_topic = "/target/crashed";

    // 发射判断
    double check_hz = 10.0;          // 距离判断循环频率（Hz）
    double launch_dist_m = 7000.0;   // uav-target 欧氏距离阈值（m）

    // 命中协议发送
    std::string remote_host = "127.0.0.1";
    int remote_port = 9100;
    std::int64_t target_key = 0;

    // tmux 窗口启动
    std::string setup_shell;
    std::string uav_guide_launch = "roslaunch uav_guide uav_guide.launch";
    std::string uav_bridge_launch = "roslaunch ros_udp_bridge uav_bridge.launch";
};

Config loadConfig(ros::NodeHandle& nh)
{
    Config c;
    std::string target_key_s = "30064967681";

    nh.param("cmd/uav_state_topic", c.uav_state_topic, c.uav_state_topic);
    nh.param("cmd/target_state_topic", c.target_state_topic, c.target_state_topic);
    nh.param("cmd/launch_cmd_topic", c.launch_cmd_topic, c.launch_cmd_topic);
    nh.param("cmd/crashed_topic", c.crashed_topic, c.crashed_topic);

    nh.param("mission/check_hz", c.check_hz, c.check_hz);
    nh.param("mission/launch_dist_m", c.launch_dist_m, c.launch_dist_m);

    nh.param("cmd/remote_host", c.remote_host, c.remote_host);
    nh.param("cmd/remote_port", c.remote_port, c.remote_port);
    nh.param("cmd/target_key", target_key_s, target_key_s);
    c.target_key = std::stoll(target_key_s);

    nh.param("mssn/setup_shell", c.setup_shell, c.setup_shell);
    nh.param("mssn/uav_guide_launch", c.uav_guide_launch, c.uav_guide_launch);
    nh.param("mssn/uav_bridge_launch", c.uav_bridge_launch, c.uav_bridge_launch);
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
        uav_sub_ = nh_.subscribe(cfg_.uav_state_topic, 1,
                                 &UdpMssnBridge::onUavState, this);
        target_sub_ = nh_.subscribe(cfg_.target_state_topic, 1,
                                    &UdpMssnBridge::onTargetState, this);
        crashed_sub_ = nh_.subscribe(cfg_.crashed_topic, 1,
                                     &UdpMssnBridge::onCrashed, this);
        launch_pub_ = nh_.advertise<std_msgs::Bool>(cfg_.launch_cmd_topic, 1, true);

        // 先用 tmux 拉起主循环与 bridge 两个独立窗口
        ensureWindows();

        socket_ = std::make_unique<ros_udp_bridge::UdpSocket>(1024, 100);
        timer_ = nh_.createTimer(
            ros::Duration(1.0 / std::max(cfg_.check_hz, 0.1)),
            &UdpMssnBridge::onTimer, this);
        ROS_INFO("[udp_mssn_bridge] ready: dist<%.0fm -> %s | hit event -> %s:%d "
                 "target_key=%lld",
                 cfg_.launch_dist_m, cfg_.launch_cmd_topic.c_str(),
                 cfg_.remote_host.c_str(), cfg_.remote_port,
                 static_cast<long long>(cfg_.target_key));
    }

private:
    // ---- 订阅回调 ----

    void onUavState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        uav_ = {{msg->pose.pose.position.x, msg->pose.pose.position.y,
                 msg->pose.pose.position.z}, true};
    }

    void onTargetState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tgt_ = {{msg->pose.pose.position.x, msg->pose.pose.position.y,
                 msg->pose.pose.position.z}, true};
    }

    void onCrashed(const std_msgs::Bool::ConstPtr& msg)
    {
        const bool hit = msg->data;
        if (hit && !hit_sent_) {
            hit_sent_ = true;
            sendHitEvent();
            ROS_INFO("[udp_mssn_bridge] ★ 命中（/target/crashed=true）→ 发送 hit 事件 "
                     "target_key=%lld -> %s:%d",
                     static_cast<long long>(cfg_.target_key),
                     cfg_.remote_host.c_str(), cfg_.remote_port);
        } else if (!hit) {
            hit_sent_ = false;
        }
    }

    // ---- 距离判断（10Hz） ----

    void onTimer(const ros::TimerEvent&)
    {
        bool has_uav = false, has_tgt = false;
        double ux = 0, uy = 0, uz = 0, tx = 0, ty = 0, tz = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            has_uav = uav_.second;
            has_tgt = tgt_.second;
            if (has_uav) { ux = uav_.first[0]; uy = uav_.first[1]; uz = uav_.first[2]; }
            if (has_tgt) { tx = tgt_.first[0]; ty = tgt_.first[1]; tz = tgt_.first[2]; }
        }
        if (!has_uav || !has_tgt) {
            ROS_WARN_THROTTLE(1.0, "[udp_mssn_bridge] 等待 uav/target 状态...");
            return;
        }
        const double dist = std::hypot(ux - tx, std::hypot(uy - ty, uz - tz));
        const bool allow = dist < cfg_.launch_dist_m;
        if (allow != launch_cmd_) {
            launch_cmd_ = allow;
            std_msgs::Bool m;
            m.data = allow;
            launch_pub_.publish(m);
            ROS_INFO("[udp_mssn_bridge] /uav/launch_cmd -> %s (dist=%.1fkm)",
                     allow ? "true" : "false", dist / 1000.0);
        }
        if (allow) {
            ROS_INFO_THROTTLE(2.0, "[udp_mssn_bridge] 允许发射：dist=%.1fkm < %.1fkm",
                              dist / 1000.0, cfg_.launch_dist_m / 1000.0);
        } else {
            ROS_WARN_THROTTLE(2.0, "[udp_mssn_bridge] 监控：dist=%.1fkm，未到发射阈值 %.1fkm",
                              dist / 1000.0, cfg_.launch_dist_m / 1000.0);
        }
    }

    // ---- 命中协议：UDP 发送含 target_key 的 JSON 数据包 ----

    void sendHitEvent()
    {
        const double t = ros::Time::now().toSec();
        const std::string json = ros_udp_bridge::encodeHitEvent(cfg_.target_key, t);
        socket_->send(cfg_.remote_host, static_cast<std::uint16_t>(cfg_.remote_port), json);
        ROS_DEBUG("[udp_mssn_bridge] sent %zuB %s", json.size(), json.c_str());
    }

    // ---- tmux 启动两个窗口 ----

    void ensureWindows()
    {
        const std::string shell = trim(cfg_.setup_shell);
        const std::string guide = shell.empty() ? cfg_.uav_guide_launch
                                                : shell + " && " + cfg_.uav_guide_launch;
        const std::string bridge = shell.empty() ? cfg_.uav_bridge_launch
                                                 : shell + " && " + cfg_.uav_bridge_launch;
        ROS_INFO("[udp_mssn_bridge] tmux 启动窗口 uav_guide.launch ...");
        launchTmuxWindow("uav_guide", guide);
        ROS_INFO("[udp_mssn_bridge] tmux 启动窗口 uav_bridge.launch ...");
        launchTmuxWindow("uav_bridge", bridge);
    }

    // ---- 成员 ----

    ros::NodeHandle& nh_;
    const Config& cfg_;
    ros::Subscriber uav_sub_;
    ros::Subscriber target_sub_;
    ros::Subscriber crashed_sub_;
    ros::Publisher launch_pub_;
    ros::Timer timer_;
    std::unique_ptr<ros_udp_bridge::UdpSocket> socket_;

    std::mutex mutex_;
    std::pair<std::array<double, 3>, bool> uav_{{{0, 0, 0}}, false};
    std::pair<std::array<double, 3>, bool> tgt_{{{0, 0, 0}}, false};
    bool launch_cmd_ = false;
    bool hit_sent_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_mssn_bridge");
    ros::NodeHandle nh;
    const Config cfg = loadConfig(nh);
    UdpMssnBridge bridge(nh, cfg);
    ros::spin();
    return 0;
}
