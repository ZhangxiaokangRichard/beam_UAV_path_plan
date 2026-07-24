/**
 * @file mqtt_target_bridge.cpp
 * @brief MQTT → ROS 双 UAV 位姿桥接节点 (mqtt_ros_bridge)。
 *
 * 从 aoa/uav_center/fst_info 自动发现 UAV 列表，按字符特征（"SIL"/"NOCON"）
 * 分别路由到 /target/state 和 /uav/state。姿态直接取自 MQTT，线速度与角速度
 * 经卡尔曼滤波器平滑。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "ros_mqtt_bridge/geodesy.h"
#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mqtt_client.h"
#include "ros_mqtt_bridge/velocity_filter.h"

namespace {

struct PendingMessage { std::string topic; std::string payload; };
std::mutex g_mutex;
std::queue<PendingMessage> g_queue;

// ---- 单架 UAV 的追踪状态 ----
struct UavTracker {
    std::string uav_sn;
    std::string role;
    ros::Publisher pub;
    tf::TransformBroadcaster* tf_br = nullptr;
    bool publish_tf = true;

    std::string last_mid;
    unsigned long long last_ts = 0;
    ros::Time last_received;
    bool has_state = false;

    double prev_x = 0.0, prev_y = 0.0;
    double prev_yaw = 0.0, prev_pitch = 0.0;
    unsigned long long last_new_ts = 0;
    bool has_prev_local = false;

    ros_mqtt_bridge::VelocityFilter vel_kf{0.5, 4.0};
    ros_mqtt_bridge::ScalarKalmanFilter yaw_rate_kf{0.01, 1.0};
    ros_mqtt_bridge::ScalarKalmanFilter pitch_rate_kf{0.01, 1.0};
    bool kf_initialized = false;
    bool first_publish = true;

    explicit UavTracker(const std::string& sn, const std::string& r)
        : uav_sn(sn), role(r) {}
};

std::string pickByFilter(const std::vector<std::string>& list,
                         const std::string& filter) {
    if (filter.empty()) return list.empty() ? "" : list.front();
    for (const auto& sn : list)
        if (sn.find(filter) != std::string::npos) return sn;
    return "";
}

void processUavInfo(UavTracker& trk,
                    const ros_mqtt_bridge::TargetState& state,
                    ros_mqtt_bridge::Geodesy& geodesy,
                    double timeout_ms)
{
    (void)timeout_ms;
    auto pos = geodesy.toLocal(state.geodetic);
    const double x = pos.x, y = pos.y, z = pos.z;
    const double yaw   = state.local.yaw;
    const double pitch = state.local.pitch;
    const double roll  = state.local.roll;

    const double vx_ins = state.ground_speed_mps * std::cos(yaw);
    const double vy_ins = state.ground_speed_mps * std::sin(yaw);

    const auto msg_ts = std::stoull(state.timestamp);
    bool cog_ok = false;
    double vx_cog = 0.0, vy_cog = 0.0;
    double yr_meas = 0.0, pr_meas = 0.0;

    if (trk.has_prev_local) {
        const double dx = x - trk.prev_x;
        const double dy = y - trk.prev_y;
        if (std::hypot(dx, dy) >= 1.0 && msg_ts > trk.last_new_ts) {
            const double dt = static_cast<double>(msg_ts - trk.last_new_ts) * 0.001;
            if (dt > 0.1) {
                vx_cog = dx / dt; vy_cog = dy / dt; cog_ok = true;
                double dyaw = yaw - trk.prev_yaw;
                double dp   = pitch - trk.prev_pitch;
                while (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
                while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                yr_meas = dyaw / dt;
                pr_meas = dp   / dt;
            }
            trk.prev_x = x; trk.prev_y = y;
            trk.prev_yaw = yaw; trk.prev_pitch = pitch;
            trk.last_new_ts = msg_ts;
        }
    } else {
        trk.prev_x = x; trk.prev_y = y;
        trk.prev_yaw = yaw; trk.prev_pitch = pitch;
        trk.last_new_ts = msg_ts;
        trk.has_prev_local = true;
    }

    if (!trk.kf_initialized) {
        trk.vel_kf.reset(vx_ins, vy_ins);
        trk.yaw_rate_kf.reset(0.0);
        trk.pitch_rate_kf.reset(0.0);
        trk.kf_initialized = true;
    } else {
        trk.vel_kf.predict();
        trk.yaw_rate_kf.predict();
        trk.pitch_rate_kf.predict();
        trk.vel_kf.update(vx_ins, vy_ins);
        if (cog_ok) trk.vel_kf.update(vx_cog, vy_cog);
        if (msg_ts > trk.last_new_ts || !trk.has_prev_local) {
            trk.yaw_rate_kf.update(yr_meas);
            trk.pitch_rate_kf.update(pr_meas);
        }
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = trk.uav_sn;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    const auto q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x  = trk.vel_kf.vx();
    odom.twist.twist.linear.y  = trk.vel_kf.vy();
    odom.twist.twist.linear.z  = state.up_mps;
    odom.twist.twist.angular.x = trk.pitch_rate_kf.value();
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = trk.yaw_rate_kf.value();
    trk.pub.publish(odom);

    if (trk.publish_tf && trk.tf_br) {
        tf::Transform tf;
        tf.setOrigin(tf::Vector3(x, y, z));
        tf.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
        trk.tf_br->sendTransform(
            tf::StampedTransform(tf, odom.header.stamp, "map", trk.uav_sn));
    }

    trk.last_mid = state.mid;
    trk.last_ts  = msg_ts;
    trk.has_state = true;
    trk.last_received = odom.header.stamp;

    if (trk.first_publish) {
        ROS_INFO("[mqtt_ros_bridge] %s first pub: sn=%s pos=(%.1f,%.1f,%.1f) "
                 "rpy=(%.1f,%.1f,%.1f)deg v=(%.1f,%.1f)m/s",
                 trk.role.c_str(), trk.uav_sn.c_str(),
                 x, y, z,
                 roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                 trk.vel_kf.vx(), trk.vel_kf.vy());
        trk.first_publish = false;
    }
}

}  // anonymous namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mqtt_ros_bridge");
    ros::NodeHandle nh;

    std::string host, prefix, uav_info_topic;
    std::string target_list_topic, target_filter;
    std::string own_list_topic,    own_filter;
    int port, qos, keepalive;
    double origin_lat, origin_lon, origin_alt, timeout_ms;
    bool publish_tf;
    nh.param<std::string>("/mqtt/host",          host,          "192.168.70.62");
    nh.param<std::string>("/mqtt/topic_prefix",   prefix,        "beam_dubins");
    nh.param<std::string>("/mqtt/uav_info_topic", uav_info_topic,"aoa/uav_center/+/uav_info");
    nh.param<int>        ("/mqtt/port",           port,          1883);
    nh.param<int>        ("/mqtt/qos",            qos,           0);
    nh.param<int>        ("/mqtt/keepalive_s",    keepalive,     30);
    nh.param<double>     ("/geodesy/origin_latitude_deg",  origin_lat, 0.0);
    nh.param<double>     ("/geodesy/origin_longitude_deg", origin_lon, 0.0);
    nh.param<double>     ("/geodesy/origin_altitude_m",    origin_alt, 0.0);
    nh.param<double>     ("/target/timeout_ms",    timeout_ms,   5000.0);
    nh.param<bool>       ("/target/publish_tf",    publish_tf,   true);
    nh.param<std::string>("/bridge/target_uav_topic",  target_list_topic, "uav_caster/uavs_information");
    nh.param<std::string>("/bridge/target_uav_filter", target_filter,     "SIL");
    nh.param<std::string>("/bridge/own_uav_topic",     own_list_topic,    "aoa/uav_center/fst_info");
    nh.param<std::string>("/bridge/own_uav_filter",    own_filter,        "NOCON");

    if (std::abs(origin_lat) < 1e-12 && std::abs(origin_lon) < 1e-12) {
        ROS_FATAL("[mqtt_ros_bridge] geodesy origin not configured");
        return 1;
    }

    ros_mqtt_bridge::Geodesy geodesy(origin_lat, origin_lon, origin_alt);
    tf::TransformBroadcaster tf_br;

    // MQTT 1: target UAV 列表 (uav_caster/uavs_information)
    std::vector<std::string> target_uav_list;
    std::mutex target_list_mutex;
    ros_mqtt_bridge::MqttClient mqtt_target_list("bridge_target_list", host, port, keepalive, qos,
        [&](const std::string&, const std::string& payload) {
            std::vector<std::string> list;
            std::string err;
            if (ros_mqtt_bridge::decodeUavsInfo(payload, list, err)) {
                std::lock_guard<std::mutex> lock(target_list_mutex);
                target_uav_list = std::move(list);
            }
        });

    // MQTT 2: own UAV 列表 (aoa/uav_center/fst_info)
    std::vector<std::string> own_uav_list;
    std::mutex own_list_mutex;
    ros_mqtt_bridge::MqttClient mqtt_own_list("bridge_own_list", host, port, keepalive, qos,
        [&](const std::string&, const std::string& payload) {
            std::vector<std::string> list;
            std::string err;
            if (ros_mqtt_bridge::decodeFstInfo(payload, list, err)) {
                std::lock_guard<std::mutex> lock(own_list_mutex);
                own_uav_list = std::move(list);
            }
        });

    // MQTT 3: uav_info 通配
    ros_mqtt_bridge::MqttClient mqtt_uav("bridge_uav", host, port, keepalive, qos,
        [](const std::string& topic, const std::string& payload) {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_queue.push({topic, payload});
        });

    if (!mqtt_target_list.start(target_list_topic))
        ROS_WARN("[mqtt_ros_bridge] target UAV list topic unavailable");
    if (!mqtt_own_list.start(own_list_topic))
        ROS_WARN("[mqtt_ros_bridge] own UAV list topic unavailable");
    if (!mqtt_uav.start(uav_info_topic)) {
        ROS_FATAL("[mqtt_ros_bridge] cannot subscribe uav_info");
        return 1;
    }

    ros::Rate rate(10.0);
    std::string cur_target_sn, cur_own_sn;
    UavTracker* target_trk = nullptr;
    UavTracker* own_trk    = nullptr;
    ros::Publisher target_pub, own_pub;

    while (ros::ok()) {
        // 从各自来源刷新 UAV 选择
        {
            std::string nt, no;
            {
                std::lock_guard<std::mutex> lock(target_list_mutex);
                if (!target_uav_list.empty()) {
                    nt = pickByFilter(target_uav_list, target_filter);
                    if (nt.empty()) {
                        nt = target_uav_list.front();
                        ROS_WARN_THROTTLE(30.0, "[mqtt_ros_bridge] target: no '%s' in uavs_information, fallback %s",
                                         target_filter.c_str(), nt.c_str());
                    }
                }
            }
            {
                std::lock_guard<std::mutex> lock(own_list_mutex);
                if (!own_uav_list.empty()) {
                    no = pickByFilter(own_uav_list, own_filter);
                    if (no.empty()) {
                        no = own_uav_list.front();
                        ROS_WARN_THROTTLE(30.0, "[mqtt_ros_bridge] own: no '%s' in fst_info, fallback %s",
                                         own_filter.c_str(), no.c_str());
                    }
                }
            }

            if (!nt.empty() && nt != cur_target_sn) {
                delete target_trk;
                target_trk = new UavTracker(nt, "target");
                target_trk->tf_br = &tf_br;
                target_trk->publish_tf = publish_tf;
                target_pub = nh.advertise<nav_msgs::Odometry>("/target/state", 10);
                target_trk->pub = target_pub;
                cur_target_sn = nt;
                ROS_INFO("[mqtt_ros_bridge] target  UAV: %s", nt.c_str());
            }
            if (!no.empty() && no != cur_own_sn) {
                delete own_trk;
                own_trk = new UavTracker(no, "own");
                own_trk->tf_br = &tf_br;
                own_trk->publish_tf = publish_tf;
                own_pub = nh.advertise<nav_msgs::Odometry>("/uav/state", 10);
                own_trk->pub = own_pub;
                cur_own_sn = no;
                ROS_INFO("[mqtt_ros_bridge] ownship UAV: %s", no.c_str());
            }
        }

        PendingMessage pending;
        bool has_msg = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            if (!g_queue.empty()) { pending = g_queue.front(); g_queue.pop(); has_msg = true; }
        }

        if (has_msg) {
            ros_mqtt_bridge::TargetState state;
            std::string error;
            if (ros_mqtt_bridge::decodeUavInfo(pending.payload, state, error)) {
                UavTracker* trk = nullptr;
                if (target_trk && state.uav_sn == target_trk->uav_sn)
                    trk = target_trk;
                else if (own_trk && state.uav_sn == own_trk->uav_sn)
                    trk = own_trk;
                if (trk) {
                    if (!(trk->has_state && state.mid == trk->last_mid &&
                          state.timestamp == std::to_string(trk->last_ts))) {
                        processUavInfo(*trk, state, geodesy, timeout_ms);
                    }
                }
            }
        }

        auto check = [&](UavTracker* t) {
            if (t && t->has_state &&
                (ros::Time::now() - t->last_received).toSec() * 1000.0 > timeout_ms) {
                t->has_state = false;
                ROS_WARN_THROTTLE(5.0, "[mqtt_ros_bridge] %s timeout: %s",
                                  t->role.c_str(), t->uav_sn.c_str());
            }
        };
        check(target_trk);
        check(own_trk);

        ros::spinOnce();
        rate.sleep();
    }

    delete target_trk;
    delete own_trk;
    return 0;
}
