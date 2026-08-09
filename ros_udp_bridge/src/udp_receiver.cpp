/**
 * @file udp_receiver.cpp
 * @brief UDP 位姿接收节点：
 *   - 从 udp.local_host:udp.local_port 收 UDP JSON（一包一 JSON）
 *   - 按 key 过滤：uav_key（首次初始化起点）/ target_key（差分解算 target 状态）
 *   - 发布：/target/state（40Hz）、/uav/init_pos（1Hz，首包后不变）、
 *           /uav/init_heading（1Hz）、/uav/relaunch（位置重置信号）
 *
 * target 状态解算（不经过滤波算法）：
 *   - 位置：经纬高 → 局部 ENU 坐标（geodesy 直接解算）
 *   - 姿态：UDP pitch/roll/heading 角度字段 → 四元数
 *   - 线速度：位置差分（dx/dt, dy/dt）
 *   - 姿态角速度：角度差分（droll/dt, dpitch/dt, dyaw/dt）
 *
 * 自包含实现，不依赖 ros_mqtt_bridge / uav_guide（解耦）。
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/tf.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ros_udp_bridge/geodesy.h"
#include "ros_udp_bridge/udp_codec.h"
#include "ros_udp_bridge/udp_socket.h"

namespace {
constexpr double kPi = 3.14159265358979323846;

/// 航空航向（正北 0° 顺时针，度）→ ENU yaw（0=东，弧度）。
double headingDegToYawRad(double heading_deg)
{
    return (90.0 - heading_deg) * kPi / 180.0;
}
}  // namespace

class UdpReceiver {
public:
    explicit UdpReceiver(ros::NodeHandle& nh) : nh_(nh)
    {
        nh_.param("udp/local_host", local_host_, std::string("0.0.0.0"));
        nh_.param("udp/local_port", local_port_, 11301);
        nh_.param("udp/recv_buf_size", recv_buf_, 4096);
        nh_.param("udp/recv_timeout_ms", recv_timeout_ms_, 100);

        // key 超过 XML-RPC int 上限（2^31-1），配置以字符串存储，读取后转 int64
        std::string uav_key_s, target_key_s;
        nh_.param("bridge/uav_key", uav_key_s, std::string("425201827841"));
        nh_.param("bridge/target_key", target_key_s, std::string("30064967681"));
        uav_key_ = std::stoll(uav_key_s);
        target_key_ = std::stoll(target_key_s);
        nh_.param("init/heading_deg", init_heading_deg_, 0.0);
        // warmup 兜底起飞点（启动 warmup 内未收到 uav_key 位姿时使用；非 ENU 解算锚点）
        nh_.param("init/default_latitude_deg", default_lat_, 30.301803);
        nh_.param("init/default_longitude_deg", default_lon_, 104.152629);
        nh_.param("init/default_altitude_m", default_alt_, 1.0);
        nh_.param("init/warmup_s", warmup_s_, 3.0);

        nh_.param("target/publish_hz", target_hz_, 40.0);
        nh_.param("init_pos/publish_hz", init_pos_hz_, 1.0);
        nh_.param("uav_state/publish_hz", uav_state_hz_, 20.0);

        // target 差分解算防护（非滤波）：抑制坐标噪声解算出虚假速度
        nh_.param("filter/cog_min_dist_m", cog_min_dist_, 0.5);
        nh_.param("filter/vel_deadband_mps", vel_deadband_, 0.5);

        double olat = 0.0, olon = 0.0, oalt = 0.0;
        nh_.param("geodesy/origin_latitude_deg", olat, 0.0);
        nh_.param("geodesy/origin_longitude_deg", olon, 0.0);
        nh_.param("geodesy/origin_altitude_m", oalt, 0.0);
        if (std::abs(olat) < 1e-12 && std::abs(olon) < 1e-12) {
            ROS_FATAL("[udp_receiver] geodesy origin not configured");
            ros::shutdown();
            return;
        }
        geodesy_ = std::make_unique<ros_udp_bridge::Geodesy>(olat, olon, oalt);

        target_pub_ = nh_.advertise<nav_msgs::Odometry>("/target/state", 10);
        init_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav/init_pos", 1, true);   // latch 初始起点
        init_heading_pub_ = nh_.advertise<std_msgs::Float64>("/uav/init_heading", 1, true);     // latch 初始朝向
        relaunch_pub_ = nh_.advertise<std_msgs::Bool>("/uav/relaunch", 1);   // 位置重置信号
        raw_pub_ = nh_.advertise<std_msgs::String>("/target/raw", 200);      // 原始 JSON 转发（诊断）
        nh_.param("relaunch/gap_s", relaunch_gap_s_, 5.0);

        socket_ = std::make_unique<ros_udp_bridge::UdpSocket>(recv_buf_, recv_timeout_ms_);
        if (!socket_->bind(local_host_, local_port_)) {
            ROS_FATAL("[udp_receiver] bind %s:%d failed", local_host_.c_str(), local_port_);
            ros::shutdown();
            return;
        }
        ROS_INFO("[udp_receiver] bound %s:%d, uav_key=%lld target_key=%lld",
                 local_host_.c_str(), local_port_,
                 static_cast<long long>(uav_key_), static_cast<long long>(target_key_));

        recv_thread_ = std::thread(&UdpReceiver::recvLoop, this);

        // relaunch 默认 false：启动即发布 false，收到 uav_key 新发射时 true，低频定时器保持默认 false
        publishRelaunch(false);
        relaunch_timer_ = nh_.createTimer(ros::Duration(1.0),
                                          &UdpReceiver::publishRelaunchDefault, this);

        // warmup：启动 warmup_s 内收包，若未收到 uav_key 位姿则用 config 默认起飞点（oneshot）
        warmup_timer_ = nh_.createTimer(ros::Duration(warmup_s_),
                                        &UdpReceiver::warmupCheck, this, true);

        target_timer_ = nh_.createTimer(ros::Duration(1.0 / target_hz_),
                                        &UdpReceiver::publishTarget, this);
        init_pos_timer_ = nh_.createTimer(ros::Duration(1.0 / init_pos_hz_),
                                          &UdpReceiver::publishInitPos, this);
    }

    ~UdpReceiver()
    {
        stop_ = true;
        if (recv_thread_.joinable()) recv_thread_.join();
    }

private:
    void recvLoop()
    {
        std::string data, ip;
        std::uint16_t port = 0;
        while (ros::ok() && !stop_) {
            if (!socket_->recv(data, ip, port)) continue;
            ros_udp_bridge::UdpPoseMsg msg;
            std::string err;
            if (!ros_udp_bridge::parseUdpMsg(data, msg, err)) {
                ROS_WARN_THROTTLE(5.0, "[udp_receiver] parse fail: %s", err.c_str());
                continue;
            }
            if (msg.key == uav_key_) {
                handleUav(msg);
            } else if (msg.key == target_key_) {
                publishRaw(data);   // 转发原始 JSON（供诊断工具 /target/raw + /target/state 同刻对比）
                handleTarget(msg);
            } else {
                ROS_DEBUG("[udp_receiver] ignore key=%lld type=%s",
                          static_cast<long long>(msg.key), msg.type.c_str());
            }
        }
    }

    // 分支 A：uav 起始初始化 + 重新发射（位置重置）检测
    void handleUav(const ros_udp_bridge::UdpPoseMsg& msg)
    {
        std::lock_guard<std::mutex> lock(mu_);
        const double now = ros::Time::now().toSec();
        if (!uav_initialized_) {
            // 首次收到 uav UDP → 初始化起点
            const auto pos = geodesy_->toLocal({msg.lat, msg.lon, msg.alt});
            uav_init_x_ = pos.x;
            uav_init_y_ = pos.y;
            uav_init_z_ = pos.z;
            // 朝向：优先报文 heading（有效时），否则 config（默认 0° 正北）
            const double heading_deg = msg.has_heading ? msg.heading_deg : init_heading_deg_;
            uav_init_heading_deg_ = heading_deg;
            uav_init_yaw_ = headingDegToYawRad(heading_deg);
            uav_initialized_ = true;
            uav_last_msg_time_ = now;
            publishRelaunch(true);   // 新发射 → 位置重置
            ROS_INFO("[udp_receiver] uav init: key=%lld pos=(%.2f,%.2f,%.2f) heading=%.2fdeg yaw=%.3frad -> relaunch",
                     static_cast<long long>(msg.key), pos.x, pos.y, pos.z, heading_deg, uav_init_yaw_);
        } else {
            // 已初始化：若距上次 uav 消息超过 gap_s → 视为新发射（位置重置）
            if (relaunch_gap_s_ > 0.0 && (now - uav_last_msg_time_) > relaunch_gap_s_) {
                publishRelaunch(true);
                ROS_INFO("[udp_receiver] uav 断流 %.1fs 后恢复 -> relaunch", relaunch_gap_s_);
            }
            uav_last_msg_time_ = now;
        }
    }

    void publishRelaunch(bool val)
    {
        std_msgs::Bool m;
        m.data = val;
        relaunch_pub_.publish(m);
    }

    // relaunch 默认 false：低频复位（收到 uav_key 新发射时为 true，之后恢复默认）
    void publishRelaunchDefault(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        publishRelaunch(false);
    }

    // warmup 3s 检查：未收到 uav_key 位姿 → 用 config 默认起飞点初始化 /uav/init_pos、/uav/init_heading
    void warmupCheck(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (uav_initialized_) {
            ROS_INFO("[udp_receiver] warmup %.0fs 收到 uav_key=%lld 位姿 → 起飞点=实际 (%.2f,%.2f,%.2f) heading=%.1fdeg",
                     warmup_s_, static_cast<long long>(uav_key_),
                     uav_init_x_, uav_init_y_, uav_init_z_, uav_init_heading_deg_);
            return;
        }
        // 用 config 默认起飞点（经纬高 → ENU；该点非 ENU 解算锚点，仅为起飞位置），朝向默认正北
        const auto pos = geodesy_->toLocal({default_lat_, default_lon_, default_alt_});
        uav_init_x_ = pos.x;
        uav_init_y_ = pos.y;
        uav_init_z_ = pos.z;
        uav_init_heading_deg_ = init_heading_deg_;   // 默认正北（config init.heading_deg）
        uav_init_yaw_ = headingDegToYawRad(init_heading_deg_);
        uav_initialized_ = true;
        ROS_INFO("[udp_receiver] warmup %.0fs 未收到 uav_key=%lld 位姿 → 用 config 默认起飞点 "
                 "lat=%.7f lon=%.7f alt=%.1f heading=正北(%.1fdeg) → /uav/init_pos",
                 warmup_s_, static_cast<long long>(uav_key_),
                 default_lat_, default_lon_, default_alt_, init_heading_deg_);
    }

    // 转发 target 原始 JSON（std_msgs/String），供 udp_test 等诊断工具与 /target/state 同刻对比
    void publishRaw(const std::string& raw)
    {
        std_msgs::String m;
        m.data = raw;
        raw_pub_.publish(m);
    }

    // 分支 B：target 更新（位置直解 + 差分速度/姿态角速度，不经过滤波算法）
    void handleTarget(const ros_udp_bridge::UdpPoseMsg& msg)
    {
        std::lock_guard<std::mutex> lock(mu_);
        // 位置：经纬高 → 局部 ENU 坐标（geodesy 直接解算）
        const auto pos = geodesy_->toLocal({msg.lat, msg.lon, msg.alt});
        // 姿态：UDP 角度字段直接转四元数（heading→ENU yaw）
        const double yaw   = headingDegToYawRad(msg.heading_deg);
        const double pitch = msg.pitch_deg * kPi / 180.0;
        const double roll  = msg.roll_deg  * kPi / 180.0;

        const auto ts = static_cast<unsigned long long>(msg.t * 1000.0);

        // ── 差分线速度 / 差分姿态角速度（无卡尔曼） ──
        double vx = 0.0, vy = 0.0;   // 线速度 (m/s)，ENU 分量
        double roll_rate = 0.0, pitch_rate = 0.0, yaw_rate = 0.0;  // 姿态角速度 (rad/s)
        if (target_has_prev_) {
            const double dx = pos.x - target_prev_x_;
            const double dy = pos.y - target_prev_y_;
            if (ts > target_prev_ts_) {
                const double dt = static_cast<double>(ts - target_prev_ts_) * 0.001;
                if (dt > 0.01) {   // 适配 40~100Hz 高频
                    // 线速度：位置差分（COG）；位置差 < cog_min_dist_ 视为坐标噪声 → 0
                    if (std::hypot(dx, dy) >= cog_min_dist_) {
                        vx = dx / dt;
                        vy = dy / dt;
                    }
                    // 姿态角速度：角度差分（yaw 需 ±π 归一化）
                    double dyaw = yaw - target_prev_yaw_;
                    while (dyaw >  kPi) dyaw -= 2.0 * kPi;
                    while (dyaw < -kPi) dyaw += 2.0 * kPi;
                    roll_rate  = (roll  - target_prev_roll_)  / dt;
                    pitch_rate = (pitch - target_prev_pitch_) / dt;
                    yaw_rate   = dyaw / dt;
                }
            }
        }
        target_prev_x_ = pos.x; target_prev_y_ = pos.y;
        target_prev_yaw_ = yaw; target_prev_pitch_ = pitch; target_prev_roll_ = roll;
        target_prev_ts_ = ts;
        target_has_prev_ = true;

        // 速度死区：低于阈值输出 0，避免坐标噪声/数值下溢产生虚假微小速度
        if (std::abs(vx) < vel_deadband_) vx = 0.0;
        if (std::abs(vy) < vel_deadband_) vy = 0.0;

        target_odom_.header.stamp = ros::Time::now();
        target_odom_.header.frame_id = "map";
        target_odom_.child_frame_id = std::to_string(target_key_);
        target_odom_.pose.pose.position.x = pos.x;
        target_odom_.pose.pose.position.y = pos.y;
        target_odom_.pose.pose.position.z = pos.z;
        const tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        target_odom_.pose.pose.orientation.x = q.x();
        target_odom_.pose.pose.orientation.y = q.y();
        target_odom_.pose.pose.orientation.z = q.z();
        target_odom_.pose.pose.orientation.w = q.w();
        target_odom_.twist.twist.linear.x = vx;
        target_odom_.twist.twist.linear.y = vy;
        target_odom_.twist.twist.linear.z = 0.0;
        target_odom_.twist.twist.angular.x = roll_rate;    // 绕 X 轴角速度（横滚率 p）
        target_odom_.twist.twist.angular.y = pitch_rate;   // 绕 Y 轴角速度（俯仰率 q）
        target_odom_.twist.twist.angular.z = yaw_rate;     // 绕 Z 轴角速度（偏航率 r）
        target_has_update_ = true;
    }

    void publishTarget(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (target_has_update_) target_pub_.publish(target_odom_);
    }

    void publishInitPos(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!uav_initialized_) return;
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";
        p.pose.position.x = uav_init_x_;
        p.pose.position.y = uav_init_y_;
        p.pose.position.z = uav_init_z_;
        const tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, uav_init_yaw_);
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();
        init_pos_pub_.publish(p);

        std_msgs::Float64 h;
        h.data = uav_init_heading_deg_;   // 初始发射朝向（角度制）
        init_heading_pub_.publish(h);
    }

    ros::NodeHandle& nh_;

    std::string local_host_;
    int local_port_ = 11301;
    int recv_buf_ = 4096;
    int recv_timeout_ms_ = 100;
    std::int64_t uav_key_ = 0;
    std::int64_t target_key_ = 0;
    double init_heading_deg_ = 0.0;
    double target_hz_ = 40.0;
    double init_pos_hz_ = 1.0;
    double uav_state_hz_ = 20.0;

    std::unique_ptr<ros_udp_bridge::Geodesy> geodesy_;
    std::unique_ptr<ros_udp_bridge::UdpSocket> socket_;

    ros::Publisher target_pub_;
    ros::Publisher init_pos_pub_;
    ros::Publisher init_heading_pub_;
    ros::Publisher relaunch_pub_;      // /uav/relaunch (std_msgs/Bool) 位置重置信号
    ros::Publisher raw_pub_;           // /target/raw (std_msgs/String) 原始 JSON 转发（诊断）
    double relaunch_gap_s_ = 5.0;      // 断流超过该时长再收到 uav 消息 → 视为新发射
    double uav_last_msg_time_ = 0.0;
    ros::Timer target_timer_;
    ros::Timer init_pos_timer_;
    ros::Timer relaunch_timer_;        // relaunch 默认 false 复位（1Hz）
    ros::Timer warmup_timer_;          // warmup 3s 检查（oneshot）
    double warmup_s_ = 3.0;            // warmup 时长（秒）
    double default_lat_ = 30.301803;   // config 默认起飞点（非 ENU 解算锚点）
    double default_lon_ = 104.152629;
    double default_alt_ = 1.0;

    std::mutex mu_;
    std::atomic<bool> stop_{false};
    std::thread recv_thread_;

    // uav 初始化缓存
    bool uav_initialized_ = false;
    double uav_init_x_ = 0.0, uav_init_y_ = 0.0, uav_init_z_ = 0.0;
    double uav_init_heading_deg_ = 0.0;
    double uav_init_yaw_ = 0.0;

    // target 状态与差分解算
    bool target_has_update_ = false;
    nav_msgs::Odometry target_odom_;
    bool target_has_prev_ = false;
    double target_prev_x_ = 0.0, target_prev_y_ = 0.0;
    double target_prev_yaw_ = 0.0, target_prev_pitch_ = 0.0, target_prev_roll_ = 0.0;
    unsigned long long target_prev_ts_ = 0;
    double cog_min_dist_ = 0.5;      // 位置差低于该值视为坐标噪声，不解算速度
    double vel_deadband_ = 0.5;      // 速度输出死区 (m/s)
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nh;   // 全局命名空间读取（与 launch 全局 rosparam load 匹配）
    UdpReceiver receiver(nh);
    ros::spin();
    return 0;
}
