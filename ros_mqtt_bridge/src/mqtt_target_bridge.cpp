/**
 * @file mqtt_target_bridge.cpp
 * @brief MQTT → ROS2 位姿桥（P2）：aoa/uav_center/+/uav_info → aoa/uav/state、aoa/target/state
 *
 * 与 1.0.1 的 mqtt_target_bridge 等价：
 *   - **MQTT 订阅主题、uav_sn 过滤与路由、去重、速度卡尔曼**全部保持不变
 *   - 新增/变化：ROS 话题加 aoa/ 前缀；TF 改用 tf2_ros；主循环改 wall timer
 *   - 本节点全权负责：uav_sn 辨识、局部 ENU 坐标（geodesy）、tf2 接口
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ros_mqtt_bridge/geodesy.h"
#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mqtt_client.h"
#include "ros_mqtt_bridge/velocity_filter.h"

namespace {

struct PendingMessage {
    std::string topic;
    std::string payload;
};

/// 读参数（YAML int/double 混写均可）
double param_double(const rclcpp::Node& node, const std::string& name, double fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.as_double();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(v.as_int());
    return fallback;
}

std::int64_t param_int(const rclcpp::Node& node, const std::string& name, std::int64_t fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return v.as_int();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return static_cast<std::int64_t>(v.as_double());
    return fallback;
}

std::string param_string(const rclcpp::Node& node, const std::string& name,
                         const std::string& fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return v.as_string();
    return fallback;
}

bool param_bool(const rclcpp::Node& node, const std::string& name, bool fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) return v.as_bool();
    return fallback;
}

/// 单架 UAV 的追踪状态（算法与 1.0.1 一致）
struct UavTracker {
    std::string uav_sn;
    std::string role;
    rclcpp::Publisher<nav_msgs::msg::Odometry>* pub = nullptr;

    std::string last_mid;
    unsigned long long last_ts = 0;
    rclcpp::Time last_received;
    bool has_state = false;

    double prev_x = 0.0, prev_y = 0.0, prev_yaw = 0.0, prev_pitch = 0.0;
    unsigned long long last_new_ts = 0;
    bool has_prev_local = false;

    ros_mqtt_bridge::VelocityFilter vel_kf{0.5, 4.0};
    ros_mqtt_bridge::ScalarKalmanFilter yaw_rate_kf{0.01, 1.0};
    ros_mqtt_bridge::ScalarKalmanFilter pitch_rate_kf{0.01, 1.0};
    bool kf_initialized = false;
    bool first_publish = true;

    UavTracker(const std::string& sn, const std::string& r) : uav_sn(sn), role(r) {}
};

std::string pickByFilter(const std::vector<std::string>& list, const std::string& filter)
{
    if (filter.empty()) return list.empty() ? "" : list.front();
    for (const auto& sn : list)
        if (sn.find(filter) != std::string::npos) return sn;
    return "";
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("mqtt_target_bridge", options);

    // ── 参数（与 1.0.1 同名，仅去掉 / 前缀）──
    const std::string host = param_string(*node, "mqtt.host", "192.168.70.62");
    const int port = static_cast<int>(param_int(*node, "mqtt.port", 1883));
    const int qos = static_cast<int>(param_int(*node, "mqtt.qos", 0));
    const int keepalive = static_cast<int>(param_int(*node, "mqtt.keepalive_s", 30));
    const std::string uav_info_topic =
        param_string(*node, "mqtt.uav_info_topic", "aoa/uav_center/+/uav_info");

    const double origin_lat = param_double(*node, "geodesy.origin_latitude_deg", 0.0);
    const double origin_lon = param_double(*node, "geodesy.origin_longitude_deg", 0.0);
    const double origin_alt = param_double(*node, "geodesy.origin_altitude_m", 0.0);

    const bool publish_tf = param_bool(*node, "target.publish_tf", true);
    const double timeout_ms = param_double(*node, "target.timeout_ms", 5000.0);

    const std::string target_list_topic =
        param_string(*node, "bridge.target_uav_topic", "uav_caster/uavs_information");
    const std::string target_filter = param_string(*node, "bridge.target_uav_filter", "SIL");
    const std::string own_list_topic =
        param_string(*node, "bridge.own_uav_topic", "uav_caster/uavs_information");
    const std::string own_filter = param_string(*node, "bridge.own_uav_filter", "NOCON");

    const std::string target_host = param_string(*node, "target.mqtt_host", host);
    const std::string own_host = param_string(*node, "own.mqtt_host", host);
    const int target_port = static_cast<int>(param_int(*node, "target.mqtt_port", port));
    const int own_port = static_cast<int>(param_int(*node, "own.mqtt_port", port));

    const std::string uav_state_topic = param_string(*node, "topics.uav_state", "aoa/uav/state");
    const std::string target_state_topic =
        param_string(*node, "topics.target_state", "aoa/target/state");
    const std::string world_frame = param_string(*node, "tf.world_frame", "map");

    if (std::abs(origin_lat) < 1e-12 && std::abs(origin_lon) < 1e-12) {
        RCLCPP_FATAL(node->get_logger(),
                     "[mqtt_target_bridge] geodesy origin not configured "
                     "(geodesy.origin_latitude_deg / longitude_deg)");
        rclcpp::shutdown();
        return 1;
    }

    ros_mqtt_bridge::Geodesy geodesy(origin_lat, origin_lon, origin_alt);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    // 发布器（话题固定，uav_sn 变化只影响 TF child_frame 与消息 child_frame_id）
    auto pub_uav = node->create_publisher<nav_msgs::msg::Odometry>(uav_state_topic, 10);
    auto pub_target = node->create_publisher<nav_msgs::msg::Odometry>(target_state_topic, 10);

    // ── 队列 ──
    std::mutex target_uav_mutex;
    std::queue<PendingMessage> target_uav_queue;
    std::mutex own_uav_mutex;
    std::queue<PendingMessage> own_uav_queue;

    std::vector<std::string> target_uav_list;
    std::mutex target_list_mutex;
    std::vector<std::string> own_uav_list;
    std::mutex own_list_mutex;

    // ── MQTT 客户端（4 个，与 1.0.1 一致：target/own 列表 + target/own uav_info）──
    ros_mqtt_bridge::MqttClient mqtt_target_list(
        "bridge_target_list", target_host, target_port, keepalive, qos,
        [&](const std::string&, const std::string& payload) {
            std::vector<std::string> list;
            std::string err;
            if (ros_mqtt_bridge::decodeUavsInfo(payload, list, err)) {
                std::lock_guard<std::mutex> lock(target_list_mutex);
                target_uav_list = std::move(list);
            }
        });

    ros_mqtt_bridge::MqttClient mqtt_own_list(
        "bridge_own_list", own_host, own_port, keepalive, qos,
        [&](const std::string&, const std::string& payload) {
            std::vector<std::string> list;
            std::string err;
            if (ros_mqtt_bridge::decodeUavsInfo(payload, list, err)) {
                std::lock_guard<std::mutex> lock(own_list_mutex);
                own_uav_list = std::move(list);
            }
        });

    ros_mqtt_bridge::MqttClient mqtt_target_uav(
        "bridge_target_uav", target_host, target_port, keepalive, qos,
        [&](const std::string& topic, const std::string& payload) {
            std::lock_guard<std::mutex> lock(target_uav_mutex);
            target_uav_queue.push({topic, payload});
        });

    ros_mqtt_bridge::MqttClient mqtt_own_uav(
        "bridge_own_uav", own_host, own_port, keepalive, qos,
        [&](const std::string& topic, const std::string& payload) {
            std::lock_guard<std::mutex> lock(own_uav_mutex);
            own_uav_queue.push({topic, payload});
        });

    if (!mqtt_target_list.start(target_list_topic))
        RCLCPP_WARN(node->get_logger(), "[mqtt_target_bridge] target UAV list topic unavailable");
    if (!mqtt_own_list.start(own_list_topic))
        RCLCPP_WARN(node->get_logger(), "[mqtt_target_bridge] own UAV list topic unavailable");
    if (!mqtt_target_uav.start(uav_info_topic) || !mqtt_own_uav.start(uav_info_topic)) {
        RCLCPP_FATAL(node->get_logger(), "[mqtt_target_bridge] cannot subscribe uav_info");
        rclcpp::shutdown();
        return 1;
    }

    // ── 追踪器 ──
    auto target_trk = std::make_unique<UavTracker>("", "target");
    auto own_trk = std::make_unique<UavTracker>("", "own");
    target_trk->pub = &*pub_target;
    own_trk->pub = &*pub_uav;

    auto processUavInfo = [&](UavTracker& trk, const ros_mqtt_bridge::TargetState& state) {
        const auto pos = geodesy.toLocal(state.geodetic);
        const double x = pos.x, y = pos.y, z = pos.z;
        const double yaw = state.local.yaw, pitch = state.local.pitch, roll = state.local.roll;

        const double vx_ins = state.ground_speed_mps * std::cos(yaw);
        const double vy_ins = state.ground_speed_mps * std::sin(yaw);

        const auto msg_ts = std::stoull(state.timestamp);
        bool cog_ok = false;
        double vx_cog = 0.0, vy_cog = 0.0, yr_meas = 0.0, pr_meas = 0.0;

        if (trk.has_prev_local) {
            const double dx = x - trk.prev_x;
            const double dy = y - trk.prev_y;
            if (std::hypot(dx, dy) >= 1.0 && msg_ts > trk.last_new_ts) {
                const double dt = static_cast<double>(msg_ts - trk.last_new_ts) * 0.001;
                if (dt > 0.1) {
                    vx_cog = dx / dt;
                    vy_cog = dy / dt;
                    cog_ok = true;
                    const double dyaw = yaw - trk.prev_yaw;
                    const double dp = pitch - trk.prev_pitch;
                    yr_meas = dyaw / dt;
                    pr_meas = dp / dt;
                }
                trk.prev_x = x;
                trk.prev_y = y;
                trk.prev_yaw = yaw;
                trk.prev_pitch = pitch;
                trk.last_new_ts = msg_ts;
            }
        } else {
            trk.prev_x = x;
            trk.prev_y = y;
            trk.prev_yaw = yaw;
            trk.prev_pitch = pitch;
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

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = node->now();
        odom.header.frame_id = world_frame;
        odom.child_frame_id = trk.uav_sn;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom.twist.twist.linear.x = trk.vel_kf.vx();
        odom.twist.twist.linear.y = trk.vel_kf.vy();
        odom.twist.twist.linear.z = state.up_mps;
        odom.twist.twist.angular.x = trk.pitch_rate_kf.value();
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = trk.yaw_rate_kf.value();
        trk.pub->publish(odom);

        if (publish_tf && !trk.uav_sn.empty()) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = odom.header.stamp;
            ts.header.frame_id = world_frame;
            ts.child_frame_id = trk.uav_sn;
            ts.transform.translation.x = x;
            ts.transform.translation.y = y;
            ts.transform.translation.z = z;
            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(ts);
        }

        trk.last_mid = state.mid;
        trk.last_ts = msg_ts;
        trk.has_state = true;
        trk.last_received = odom.header.stamp;

        if (trk.first_publish) {
            RCLCPP_INFO(node->get_logger(),
                        "[mqtt_target_bridge] %s first pub: sn=%s pos=(%.1f,%.1f,%.1f) "
                        "rpy=(%.1f,%.1f,%.1f)deg v=(%.1f,%.1f)m/s",
                        trk.role.c_str(), trk.uav_sn.c_str(), x, y, z,
                        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI,
                        trk.vel_kf.vx(), trk.vel_kf.vy());
            trk.first_publish = false;
        }
    };

    // ── 主循环（10 Hz，等价 1.0.1 的 ros::Rate(10)）──
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
        // 1) 刷新 UAV 选择
        {
            std::string nt;
            std::string no;
            {
                std::lock_guard<std::mutex> lock(target_list_mutex);
                if (!target_uav_list.empty()) {
                    nt = pickByFilter(target_uav_list, target_filter);
                    if (nt.empty()) {
                        nt = target_uav_list.front();
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 30000,
                                             "[mqtt_target_bridge] target: no '%s' in list, fallback %s",
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
                        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 30000,
                                             "[mqtt_target_bridge] own: no '%s' in list, fallback %s",
                                             own_filter.c_str(), no.c_str());
                    }
                }
            }

            if (!nt.empty() && nt != target_trk->uav_sn) {
                target_trk = std::make_unique<UavTracker>(nt, "target");
                target_trk->pub = &*pub_target;
                RCLCPP_INFO(node->get_logger(), "[mqtt_target_bridge] target  UAV: %s", nt.c_str());
            }
            if (!no.empty() && no != own_trk->uav_sn) {
                own_trk = std::make_unique<UavTracker>(no, "own");
                own_trk->pub = &*pub_uav;
                RCLCPP_INFO(node->get_logger(), "[mqtt_target_bridge] ownship UAV: %s", no.c_str());
            }
        }

        // 2) 消费队列
        PendingMessage pending;
        bool has_msg = false;
        {
            std::lock_guard<std::mutex> lock(target_uav_mutex);
            if (!target_uav_queue.empty()) {
                pending = target_uav_queue.front();
                target_uav_queue.pop();
                has_msg = true;
            }
        }
        if (has_msg && !target_trk->uav_sn.empty()) {
            ros_mqtt_bridge::TargetState state;
            std::string error;
            if (ros_mqtt_bridge::decodeUavInfo(pending.payload, state, error) &&
                state.uav_sn == target_trk->uav_sn &&
                !(target_trk->has_state && state.mid == target_trk->last_mid &&
                  state.timestamp == std::to_string(target_trk->last_ts))) {
                processUavInfo(*target_trk, state);
            }
        }

        has_msg = false;
        {
            std::lock_guard<std::mutex> lock(own_uav_mutex);
            if (!own_uav_queue.empty()) {
                pending = own_uav_queue.front();
                own_uav_queue.pop();
                has_msg = true;
            }
        }
        if (has_msg && !own_trk->uav_sn.empty()) {
            ros_mqtt_bridge::TargetState state;
            std::string error;
            if (ros_mqtt_bridge::decodeUavInfo(pending.payload, state, error) &&
                state.uav_sn == own_trk->uav_sn &&
                !(own_trk->has_state && state.mid == own_trk->last_mid &&
                  state.timestamp == std::to_string(own_trk->last_ts))) {
                processUavInfo(*own_trk, state);
            }
        }

        // 3) 超时检查
        for (UavTracker* trk : {target_trk.get(), own_trk.get()}) {
            if (trk->has_state &&
                (node->now() - trk->last_received).seconds() * 1000.0 > timeout_ms) {
                trk->has_state = false;
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                     "[mqtt_target_bridge] %s timeout: %s", trk->role.c_str(),
                                     trk->uav_sn.c_str());
            }
        }
    });

    RCLCPP_INFO(node->get_logger(),
                "[mqtt_target_bridge] ready: broker=%s:%d (target %s:%d / own %s:%d) "
                "uav_info=%s → %s / %s (frame=%s, publish_tf=%d)",
                host.c_str(), port, target_host.c_str(), target_port, own_host.c_str(), own_port,
                uav_info_topic.c_str(), uav_state_topic.c_str(), target_state_topic.c_str(),
                world_frame.c_str(), static_cast<int>(publish_tf));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
