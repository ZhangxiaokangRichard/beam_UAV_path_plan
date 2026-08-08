/**
 * @file udp_receiver.cpp
 * @brief UDP 位姿接收节点（阶段 1-3）：
 *   - 从 udp.local_host:udp.local_port 收 UDP JSON（一包一 JSON）
 *   - 按 key 过滤：uav_key（首次初始化起点）/ target_key（40Hz 卡尔曼滤波）
 *   - 发布：/target/state（40Hz）、/uav/init_pos（1Hz，首包后不变）、
 *           /uav/init_heading（1Hz）、/uav/state（起点，20Hz）
 *
 * 自包含实现，不依赖 ros_mqtt_bridge / uav_guide（解耦）。
 */

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ros_udp_bridge/filter.h"
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

        nh_.param("target/publish_hz", target_hz_, 40.0);
        nh_.param("init_pos/publish_hz", init_pos_hz_, 1.0);
        nh_.param("uav_state/publish_hz", uav_state_hz_, 20.0);

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
        init_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/uav/init_pos", 1, true);   // latch
        init_heading_pub_ = nh_.advertise<std_msgs::Float64>("/uav/init_heading", 1, true);     // latch
        uav_state_pub_ = nh_.advertise<nav_msgs::Odometry>("/uav/state", 10);

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

        target_timer_ = nh_.createTimer(ros::Duration(1.0 / target_hz_),
                                        &UdpReceiver::publishTarget, this);
        uav_state_timer_ = nh_.createTimer(ros::Duration(1.0 / uav_state_hz_),
                                           &UdpReceiver::publishUavState, this);
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
                handleTarget(msg);
            } else {
                ROS_DEBUG("[udp_receiver] ignore key=%lld type=%s",
                          static_cast<long long>(msg.key), msg.type.c_str());
            }
        }
    }

    // 分支 A：uav 起始初始化（仅首次）
    void handleUav(const ros_udp_bridge::UdpPoseMsg& msg)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (uav_initialized_) return;
        const auto pos = geodesy_->toLocal({msg.lat, msg.lon, msg.alt});
        uav_init_x_ = pos.x;
        uav_init_y_ = pos.y;
        uav_init_z_ = pos.z;
        // 朝向：优先报文 heading（有效时），否则 config（默认 0° 正北）
        const double heading_deg = msg.has_heading ? msg.heading_deg : init_heading_deg_;
        uav_init_heading_deg_ = heading_deg;
        uav_init_yaw_ = headingDegToYawRad(heading_deg);
        uav_initialized_ = true;
        ROS_INFO("[udp_receiver] uav init: key=%lld pos=(%.2f,%.2f,%.2f) heading=%.2fdeg yaw=%.3frad",
                 static_cast<long long>(msg.key), pos.x, pos.y, pos.z, heading_deg, uav_init_yaw_);
    }

    // 分支 B：target 更新（卡尔曼滤波）
    void handleTarget(const ros_udp_bridge::UdpPoseMsg& msg)
    {
        std::lock_guard<std::mutex> lock(mu_);
        const auto pos = geodesy_->toLocal({msg.lat, msg.lon, msg.alt});
        const double yaw = headingDegToYawRad(msg.heading_deg);
        const double pitch = msg.pitch_deg * kPi / 180.0;
        const double roll = msg.roll_deg * kPi / 180.0;

        const double vx_ins = msg.velocity * std::cos(yaw);
        const double vy_ins = msg.velocity * std::sin(yaw);

        const auto ts = static_cast<unsigned long long>(msg.t * 1000.0);
        bool cog_ok = false;
        double vx_cog = 0.0, vy_cog = 0.0, yr_meas = 0.0, pr_meas = 0.0;

        if (target_has_prev_) {
            const double dx = pos.x - target_prev_x_;
            const double dy = pos.y - target_prev_y_;
            if (std::hypot(dx, dy) >= 1.0 && ts > target_prev_ts_) {
                const double dt = static_cast<double>(ts - target_prev_ts_) * 0.001;
                if (dt > 0.1) {
                    vx_cog = dx / dt; vy_cog = dy / dt; cog_ok = true;
                    double dyaw = yaw - target_prev_yaw_;
                    double dp = pitch - target_prev_pitch_;
                    while (dyaw > kPi) dyaw -= 2.0 * kPi;
                    while (dyaw < -kPi) dyaw += 2.0 * kPi;
                    yr_meas = dyaw / dt;
                    pr_meas = dp / dt;
                }
                target_prev_x_ = pos.x; target_prev_y_ = pos.y;
                target_prev_yaw_ = yaw; target_prev_pitch_ = pitch;
                target_prev_ts_ = ts;
            }
        } else {
            target_prev_x_ = pos.x; target_prev_y_ = pos.y;
            target_prev_yaw_ = yaw; target_prev_pitch_ = pitch;
            target_prev_ts_ = ts;
            target_has_prev_ = true;
        }

        if (!target_kf_init_) {
            target_vel_.reset(vx_ins, vy_ins);
            target_yr_.reset(0.0);
            target_pr_.reset(0.0);
            target_kf_init_ = true;
        } else {
            target_vel_.predict();
            target_yr_.predict();
            target_pr_.predict();
            target_vel_.update(vx_ins, vy_ins);
            if (cog_ok) target_vel_.update(vx_cog, vy_cog);
            if (ts > target_prev_ts_ || !target_has_prev_) {
                target_yr_.update(yr_meas);
                target_pr_.update(pr_meas);
            }
        }

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
        target_odom_.twist.twist.linear.x = target_vel_.vx();
        target_odom_.twist.twist.linear.y = target_vel_.vy();
        target_odom_.twist.twist.linear.z = 0.0;
        target_odom_.twist.twist.angular.x = target_pr_.value();
        target_odom_.twist.twist.angular.y = 0.0;
        target_odom_.twist.twist.angular.z = target_yr_.value();
        target_has_update_ = true;
    }

    void publishTarget(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (target_has_update_) target_pub_.publish(target_odom_);
    }

    void publishUavState(const ros::TimerEvent&)
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!uav_initialized_) return;
        nav_msgs::Odometry o;
        o.header.stamp = ros::Time::now();
        o.header.frame_id = "map";
        o.child_frame_id = std::to_string(uav_key_);
        o.pose.pose.position.x = uav_init_x_;
        o.pose.pose.position.y = uav_init_y_;
        o.pose.pose.position.z = uav_init_z_;
        const tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, uav_init_yaw_);
        o.pose.pose.orientation.x = q.x();
        o.pose.pose.orientation.y = q.y();
        o.pose.pose.orientation.z = q.z();
        o.pose.pose.orientation.w = q.w();
        uav_state_pub_.publish(o);
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
    ros::Publisher uav_state_pub_;
    ros::Timer target_timer_;
    ros::Timer uav_state_timer_;
    ros::Timer init_pos_timer_;

    std::mutex mu_;
    std::atomic<bool> stop_{false};
    std::thread recv_thread_;

    // uav 初始化缓存
    bool uav_initialized_ = false;
    double uav_init_x_ = 0.0, uav_init_y_ = 0.0, uav_init_z_ = 0.0;
    double uav_init_heading_deg_ = 0.0;
    double uav_init_yaw_ = 0.0;

    // target 状态与卡尔曼
    bool target_has_update_ = false;
    nav_msgs::Odometry target_odom_;
    bool target_kf_init_ = false;
    bool target_has_prev_ = false;
    double target_prev_x_ = 0.0, target_prev_y_ = 0.0;
    double target_prev_yaw_ = 0.0, target_prev_pitch_ = 0.0;
    unsigned long long target_prev_ts_ = 0;
    ros_udp_bridge::VelocityFilter target_vel_{0.5, 4.0};
    ros_udp_bridge::ScalarKalmanFilter target_yr_{0.01, 1.0};
    ros_udp_bridge::ScalarKalmanFilter target_pr_{0.01, 1.0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_receiver");
    ros::NodeHandle nh;   // 全局命名空间读取（与 launch 全局 rosparam load 匹配）
    UdpReceiver receiver(nh);
    ros::spin();
    return 0;
}
