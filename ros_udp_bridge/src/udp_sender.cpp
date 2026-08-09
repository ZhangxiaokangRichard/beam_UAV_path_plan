/**
 * @file udp_sender.cpp
 * @brief UDP 发送节点（阶段 6）：订阅 /uav/state → 组装 red_interceptor JSON
 *        （含 targetKey）→ UDP 发送到 udp.remote_host:udp.remote_port。
 */

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "ros_udp_bridge/geodesy.h"
#include "ros_udp_bridge/udp_codec.h"
#include "ros_udp_bridge/udp_socket.h"

class UdpSender {
public:
    explicit UdpSender(ros::NodeHandle& nh) : nh_(nh)
    {
        nh_.param("udp/remote_host", remote_host_, std::string("10.119.228.21"));
        nh_.param("udp/remote_port", remote_port_, 9100);
        std::string uav_key_s, target_key_s;
        nh_.param("bridge/uav_key", uav_key_s, std::string("425201827841"));
        nh_.param("bridge/target_key", target_key_s, std::string("30064967681"));
        nh_.param("bridge/uav_name", uav_name_, std::string("IM-01"));
        nh_.param("bridge/uav_state_topic", uav_state_topic_, std::string("uav/uav_state"));
        uav_key_ = std::stoll(uav_key_s);
        target_key_ = std::stoll(target_key_s);

        double olat = 0.0, olon = 0.0, oalt = 0.0;
        nh_.param("geodesy/origin_latitude_deg", olat, 0.0);
        nh_.param("geodesy/origin_longitude_deg", olon, 0.0);
        nh_.param("geodesy/origin_altitude_m", oalt, 0.0);
        geodesy_ = std::make_unique<ros_udp_bridge::Geodesy>(olat, olon, oalt);

        sub_ = nh_.subscribe(uav_state_topic_, 10, &UdpSender::onUavState, this);
        socket_ = std::make_unique<ros_udp_bridge::UdpSocket>(4096, 100);
        ROS_INFO("[udp_sender] %s -> %s:%d uav_key=%lld target_key=%lld",
                 uav_state_topic_.c_str(), remote_host_.c_str(), remote_port_,
                 static_cast<long long>(uav_key_), static_cast<long long>(target_key_));
    }

private:
    void onUavState(const nav_msgs::Odometry::ConstPtr& msg)
    {
        const auto g = geodesy_->toGeodetic(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z);
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // ENU yaw(0=东) -> 航空 heading(0=北, CW, 度)
        const double heading_deg = 90.0 - yaw * 180.0 / M_PI;
        const double pitch_deg = pitch * 180.0 / M_PI;
        const double roll_deg = roll * 180.0 / M_PI;
        const double t = ros::Time::now().toSec();

        const std::string json = ros_udp_bridge::encodeRedInterceptor(
            uav_key_, uav_name_, t,
            g.latitude_deg, g.longitude_deg, g.altitude_m,
            heading_deg, pitch_deg, roll_deg, target_key_, first_frame_);
        first_frame_ = false;

        socket_->send(remote_host_, remote_port_, json);
        ROS_DEBUG("[udp_sender] sent %zuB key=%lld",
                  json.size(), static_cast<long long>(uav_key_));
    }

    ros::NodeHandle& nh_;
    std::string remote_host_;
    int remote_port_ = 9100;
    std::int64_t uav_key_ = 0;
    std::int64_t target_key_ = 0;
    std::string uav_name_;
    std::string uav_state_topic_;   // 订阅的 uav 状态话题（loop 发布的 uav/uav_state）
    std::unique_ptr<ros_udp_bridge::Geodesy> geodesy_;
    std::unique_ptr<ros_udp_bridge::UdpSocket> socket_;
    ros::Subscriber sub_;
    bool first_frame_ = true;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_sender");
    ros::NodeHandle nh;   // 全局命名空间读取
    UdpSender sender(nh);
    ros::spin();
    return 0;
}
