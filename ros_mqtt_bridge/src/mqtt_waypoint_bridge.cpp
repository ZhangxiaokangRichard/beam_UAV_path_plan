/**
 * @file mqtt_waypoint_bridge.cpp
 * @brief ROS /uav/guide_point → MQTT fly_point 命令转发节点。
 *
 * 从规划器接收 UavGuidePoint 消息，封装为 fly_point JSON，
 * 发布到 aoa/uav_control/${uav_sn}/event_services。
 * uav_sn 从 /uav/state 的 child_frame_id 动态获取。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <uav_guide/UavGuidePoint.h>

#include <atomic>
#include <sstream>
#include <string>

#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mqtt_client.h"

namespace {
uav_guide::UavGuidePoint g_guide_point;
bool g_has_guide = false;
std::string g_uav_sn;
}

void guideCallback(const uav_guide::UavGuidePoint::ConstPtr& msg)
{
    g_guide_point = *msg;
    g_has_guide = true;
}

void ownStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 从 /uav/state 的 child_frame_id 提取 uav_sn（由 mqtt_ros_bridge 填入）
    if (!msg->child_frame_id.empty())
        g_uav_sn = msg->child_frame_id;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mqtt_waypoint_bridge");
    ros::NodeHandle nh;

    std::string host, prefix;
    int port, qos, keepalive;
    nh.param<std::string>("/mqtt/host",        host,     "192.168.70.62");
    nh.param<std::string>("/mqtt/topic_prefix", prefix,   "beam_dubins");
    nh.param<int>        ("/mqtt/port",         port,     1883);
    nh.param<int>        ("/mqtt/qos",          qos,      0);
    nh.param<int>        ("/mqtt/keepalive_s",  keepalive, 30);

    ros::Subscriber guide_sub = nh.subscribe<uav_guide::UavGuidePoint>(
        "/uav/guide_point", 1, guideCallback);
    ros::Subscriber own_sub = nh.subscribe<nav_msgs::Odometry>(
        "/uav/state", 1, ownStateCallback);

    ros_mqtt_bridge::MqttClient mqtt("bridge_guide_point", host, port,
                                     keepalive, qos, {});
    if (!mqtt.start()) {
        ROS_FATAL("[mqtt_waypoint_bridge] MQTT client start failed");
        return 1;
    }

    ros::Rate rate(5.0);
    // 仅当 guide_point 有更新时才发送
    ros::Time last_guide_stamp;
    while (ros::ok()) {
        ros::spinOnce();

        if (!g_has_guide) {
            rate.sleep();
            continue;
        }

        if (g_uav_sn.empty()) {
            ROS_WARN_THROTTLE(5.0, "[mqtt_waypoint_bridge] waiting for /uav/state to get uav_sn");
            rate.sleep();
            continue;
        }

        // 仅在 guide_point 时间戳更新时发送（避免重复发送同一指令）
        if (g_guide_point.header.stamp == last_guide_stamp) {
            rate.sleep();
            continue;
        }
        last_guide_stamp = g_guide_point.header.stamp;

        if (!mqtt.connected()) {
            rate.sleep();
            continue;
        }

        const std::string topic = "aoa/uav_control/" + g_uav_sn + "/event_services";
        const std::string payload = ros_mqtt_bridge::encodeFlyPoint(
            g_guide_point.target_longitude,
            g_guide_point.target_latitude,
            g_guide_point.target_altitude,
            g_guide_point.target_radius);

        if (mqtt.publish(topic, payload)) {
            ROS_INFO("[mqtt_waypoint_bridge] fly_point sent to %s: "
                     "lon=%.7f lat=%.7f alt=%.1f r=%.1f",
                     g_uav_sn.c_str(),
                     g_guide_point.target_longitude,
                     g_guide_point.target_latitude,
                     g_guide_point.target_altitude,
                     g_guide_point.target_radius);
        } else {
            ROS_WARN_THROTTLE(2.0, "[mqtt_waypoint_bridge] MQTT publish failed");
        }

        rate.sleep();
    }
    return 0;
}
