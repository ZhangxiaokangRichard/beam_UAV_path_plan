/**
 * @file mqtt_control_bridge.cpp
 * @brief MQTT 出站控制桥（P6）：ROS 制导 → MQTT（**按模式截断**）
 *
 * 出站主题：`aoa/uav_control/${uav_sn}/event_services`（QoS0，不 retain）
 *   模式 auto     : 不发任何指令（初始态 / 失效安全态）
 *   模式 setpoint : fly_point（← aoa/uav/setpoint）
 *   模式 cruise   : set_cruise_mode（跃迁边沿 1 次）+ set_fly_head + set_fly_height（← aoa/uav/guidance）
 *
 * 模式来源：订阅 ROS 镜像 `aoa/uav/set_cruise_mode`（由 mqtt_mssn_bridge 从 MQTT
 * `aoa/ai_guide/set_cruise_mode` 转入）。本节点**不直连 MQTT 入站**，便于离线单测。
 *
 * 安全开关：
 *   mqtt.dry_run            = true  → 只打印不发布
 *   mqtt.event_services_template    → 可将出站重定向到测试话题（默认协议话题）
 *   cmd.require_mode        = false → 直通模式（等价 P6 前行为：始终按 setpoint 发 fly_point）
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unistd.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mode_gate.h"
#include "ros_mqtt_bridge/mqtt_client.h"
#include "uav_guide/msg/guidance.hpp"
#include "uav_guide/msg/setpoint.hpp"

namespace {

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

/// 展开 ${uav_sn}
std::string expandTopic(const std::string& tmpl, const std::string& sn)
{
    const std::string key = "${uav_sn}";
    std::string out = tmpl;
    const auto pos = out.find(key);
    if (pos != std::string::npos) out.replace(pos, key.size(), sn);
    return out;
}

/// 解析模式 JSON：主格式 {"mode":"..."}；容错 {"data":{"mode":"..."}}
std::string extractMode(const std::string& payload)
{
    try {
        std::stringstream stream(payload);
        boost::property_tree::ptree tree;
        boost::property_tree::read_json(stream, tree);
        if (auto m = tree.get_optional<std::string>("mode")) return *m;
        if (auto m = tree.get_optional<std::string>("data.mode")) return *m;
    } catch (const std::exception&) {
        // 非法 JSON → 由调用方告警
    }
    return {};
}

}  // namespace

namespace {

class ControlBridge : public rclcpp::Node {
public:
    ControlBridge()
        : rclcpp::Node("mqtt_control_bridge",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        const std::string host = param_string(*this, "mqtt.host", "192.168.70.62");
        const int port = static_cast<int>(param_int(*this, "mqtt.port", 1883));
        const int qos = static_cast<int>(param_int(*this, "mqtt.qos", 0));
        const int keepalive = static_cast<int>(param_int(*this, "mqtt.keepalive_s", 30));

        const std::string event_tmpl = param_string(*this, "mqtt.event_services_template",
                                                    "aoa/uav_control/${uav_sn}/event_services");
        dry_run_ = param_bool(*this, "mqtt.dry_run", false);

        mode_state_topic_ = param_string(*this, "topics.mode_state", "aoa/uav/set_cruise_mode");
        setpoint_topic_ = param_string(*this, "topics.setpoint", "aoa/uav/setpoint");
        guidance_topic_ = param_string(*this, "topics.guidance", "aoa/uav/guidance");
        uav_state_topic_ = param_string(*this, "topics.uav_state", "aoa/uav/state");
        tracking_topic_ = param_string(*this, "topics.tracking_mode", "aoa/uav/tracking_mode");

        height_type_ = static_cast<int>(param_int(*this, "cmd.height_type", 0));
        min_interval_s_ = param_double(*this, "cmd.min_interval_s", 1.0);
        heading_eps_deg_ = param_double(*this, "cmd.heading_eps_deg", 1.0);
        height_eps_m_ = param_double(*this, "cmd.height_eps_m", 0.5);
        resend_cruise_mode_s_ = param_double(*this, "cmd.resend_cruise_mode_s", 0.0);
        require_mode_ = param_bool(*this, "cmd.require_mode", true);
        mode_timeout_s_ = param_double(*this, "cmd.mode_timeout_s", 0.0);
        set_fly_speed_enabled_ = param_bool(*this, "cmd.set_fly_speed_enabled", false);

        // ── MQTT 客户端（出站）──
        mqtt_ = std::make_unique<ros_mqtt_bridge::MqttClient>(
            "bridge_control_" + std::to_string(::getpid()), host, port, keepalive, qos,
            ros_mqtt_bridge::MqttClient::MessageHandler{});
        if (!mqtt_->start()) {
            RCLCPP_FATAL(get_logger(), "[mqtt_control_bridge] MQTT client start failed");
            throw std::runtime_error("mqtt start failed");
        }

        // ── ROS 订阅 ──
        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();

        sub_mode_ = create_subscription<std_msgs::msg::String>(
            mode_state_topic_, latched,
            [this](std_msgs::msg::String::SharedPtr msg) { on_mode_state(msg->data); });
        sub_setpoint_ = create_subscription<uav_guide::msg::Setpoint>(
            setpoint_topic_, 1,
            [this](uav_guide::msg::Setpoint::SharedPtr msg) { on_setpoint(*msg); });
        sub_guidance_ = create_subscription<uav_guide::msg::Guidance>(
            guidance_topic_, 1, [this](uav_guide::msg::Guidance::SharedPtr msg) { on_guidance(*msg); });
        sub_uav_state_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_state_topic_, 10, [this, event_tmpl](nav_msgs::msg::Odometry::SharedPtr msg) {
                if (!msg->child_frame_id.empty() && msg->child_frame_id != uav_sn_) {
                    uav_sn_ = msg->child_frame_id;
                    RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] uav_sn = %s → event topic %s",
                                uav_sn_.c_str(),
                                expandTopic(event_tmpl, uav_sn_).c_str());
                }
            });
        sub_tracking_ = create_subscription<std_msgs::msg::Bool>(
            tracking_topic_, latched, [this](std_msgs::msg::Bool::SharedPtr msg) {
                tracking_ = msg->data;
            });

        // 1 Hz：模式看门狗 + 可选周期重发
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { tick(); });

        last_mode_rx_ = now();
        RCLCPP_INFO(get_logger(),
                    "[mqtt_control_bridge] ready: broker=%s:%d event_topic=%s dry_run=%d "
                    "require_mode=%d height_type=%d (min_interval=%.1fs eps=%.1f°/%.1fm)",
                    host.c_str(), port, event_tmpl.c_str(), static_cast<int>(dry_run_),
                    static_cast<int>(require_mode_), height_type_, min_interval_s_,
                    heading_eps_deg_, height_eps_m_);
    }

private:
    // ── 发布 ──
    void publishRaw(const std::string& payload, const char* what)
    {
        if (uav_sn_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] drop %s: uav_sn unknown (waiting for %s)",
                                 what, uav_state_topic_.c_str());
            return;
        }
        const std::string topic = expandTopic(event_template(), uav_sn_);
        if (dry_run_) {
            RCLCPP_INFO(get_logger(), "[mqtt_control_bridge][dry-run] %s → %s: %s", what,
                        topic.c_str(), payload.c_str());
            return;
        }
        if (!mqtt_->publish(topic, payload)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_control_bridge] publish %s failed (broker down? topic=%s)",
                                 what, topic.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] %s → %s", what, topic.c_str());
    }

    std::string event_template() const
    {
        return param_string(*this, "mqtt.event_services_template",
                            "aoa/uav_control/${uav_sn}/event_services");
    }

    // ── 模式 ──
    void on_mode_state(const std::string& payload)
    {
        const std::string raw = extractMode(payload);
        if (raw.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] bad mode payload: %s", payload.c_str());
            return;
        }
        ros_mqtt_bridge::CruiseMode parsed;
        if (!ros_mqtt_bridge::parseCruiseMode(raw, parsed)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] invalid mode: '%s' (auto|setpoint|cruise)",
                                 raw.c_str());
            return;
        }

        last_mode_rx_ = now();
        if (parsed == mode_) return;

        const auto previous = mode_;
        mode_ = parsed;
        RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] mode %s -> %s",
                    ros_mqtt_bridge::cruiseModeName(previous),
                    ros_mqtt_bridge::cruiseModeName(mode_));

        if (ros_mqtt_bridge::needsCruiseModeCommand(previous, mode_)) {
            const std::string mid = mid_gen_.next();
            publishRaw(ros_mqtt_bridge::encodeSetCruiseMode(mid), "set_cruise_mode");
            last_cruise_mode_tx_ = now();
        }
    }

    // ── 指点通道 ──
    void on_setpoint(const uav_guide::msg::Setpoint& msg)
    {
        const bool enabled = !require_mode_ ||
                             ros_mqtt_bridge::isChannelEnabled(mode_, ros_mqtt_bridge::OutChannel::Setpoint);
        if (!enabled) {
            RCLCPP_DEBUG(get_logger(), "[mqtt_control_bridge] drop fly_point (mode=%s)",
                         ros_mqtt_bridge::cruiseModeName(mode_));
            return;
        }
        // 与 1.0.1 一致：时间戳更新才下发
        if (has_setpoint_stamp_ && msg.header.stamp == last_setpoint_stamp_) return;
        last_setpoint_stamp_ = msg.header.stamp;
        has_setpoint_stamp_ = true;

        const std::string payload = ros_mqtt_bridge::encodeFlyPoint(
            msg.target_longitude, msg.target_latitude, msg.target_altitude, msg.target_radius);
        publishRaw(payload, "fly_point");
    }

    // ── 巡航通道 ──
    void on_guidance(const uav_guide::msg::Guidance& msg)
    {
        if (!ros_mqtt_bridge::isChannelEnabled(mode_, ros_mqtt_bridge::OutChannel::Guidance)) {
            RCLCPP_DEBUG(get_logger(), "[mqtt_control_bridge] drop guidance (mode=%s)",
                         ros_mqtt_bridge::cruiseModeName(mode_));
            return;
        }
        publish_guidance(msg, /*force=*/false);
    }

    void publish_guidance(const uav_guide::msg::Guidance& msg, bool force)
    {
        const rclcpp::Time stamp = now();
        const bool changed = !has_guidance_ ||
                             std::abs(msg.heads - last_heads_) >= heading_eps_deg_ ||
                             std::abs(msg.fly_height - last_fly_height_) >= height_eps_m_;
        const bool interval_ok =
            !has_guidance_ || (stamp - last_guidance_tx_).seconds() >= min_interval_s_;
        if (!force && !(changed && interval_ok)) return;

        const std::string mid = mid_gen_.next();
        publishRaw(ros_mqtt_bridge::encodeSetFlyHead(msg.heads, mid), "set_fly_head");
        publishRaw(ros_mqtt_bridge::encodeSetFlyHeight(msg.fly_height, height_type_, mid_gen_.next()),
                   "set_fly_height");
        if (set_fly_speed_enabled_) {
            // 预留（协议 4.4.23，本轮不实现编码）
            RCLCPP_DEBUG(get_logger(), "[mqtt_control_bridge] fly_speed=%.1f reserved", msg.fly_speed);
        }

        last_heads_ = msg.heads;
        last_fly_height_ = msg.fly_height;
        last_guidance_tx_ = stamp;
        has_guidance_ = true;
    }

    // ── 1 Hz：看门狗 + 可选重发 ──
    void tick()
    {
        // 模式超时 → 回落 auto（失效安全：不发优于误发）
        if (mode_timeout_s_ > 0.0 && mode_ != ros_mqtt_bridge::CruiseMode::Auto &&
            (now() - last_mode_rx_).seconds() > mode_timeout_s_) {
            RCLCPP_WARN(get_logger(),
                        "[mqtt_control_bridge] mode timeout (%.0fs) → fallback to auto (no output)",
                        mode_timeout_s_);
            mode_ = ros_mqtt_bridge::CruiseMode::Auto;
            has_guidance_ = false;
            has_setpoint_stamp_ = false;
        }

        if (resend_cruise_mode_s_ > 0.0 && mode_ == ros_mqtt_bridge::CruiseMode::Cruise &&
            (now() - last_cruise_mode_tx_).seconds() >= resend_cruise_mode_s_) {
            publishRaw(ros_mqtt_bridge::encodeSetCruiseMode(mid_gen_.next()),
                       "set_cruise_mode(resend)");
            last_cruise_mode_tx_ = now();
        }
    }

    // ── 成员 ──
    std::unique_ptr<ros_mqtt_bridge::MqttClient> mqtt_;
    ros_mqtt_bridge::MidGenerator mid_gen_;

    std::string mode_state_topic_, setpoint_topic_, guidance_topic_, uav_state_topic_, tracking_topic_;
    std::string uav_sn_;

    ros_mqtt_bridge::CruiseMode mode_ = ros_mqtt_bridge::CruiseMode::Auto;
    bool dry_run_ = false;
    bool require_mode_ = true;
    bool set_fly_speed_enabled_ = false;
    bool tracking_ = false;

    int height_type_ = 0;
    double min_interval_s_ = 1.0;
    double heading_eps_deg_ = 1.0;
    double height_eps_m_ = 0.5;
    double resend_cruise_mode_s_ = 0.0;
    double mode_timeout_s_ = 0.0;

    bool has_setpoint_stamp_ = false;
    rclcpp::Time last_setpoint_stamp_;
    bool has_guidance_ = false;
    double last_heads_ = 0.0;
    double last_fly_height_ = 0.0;
    rclcpp::Time last_guidance_tx_;
    rclcpp::Time last_mode_rx_;
    rclcpp::Time last_cruise_mode_tx_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
    rclcpp::Subscription<uav_guide::msg::Setpoint>::SharedPtr sub_setpoint_;
    rclcpp::Subscription<uav_guide::msg::Guidance>::SharedPtr sub_guidance_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_state_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_tracking_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ControlBridge>());
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("mqtt_control_bridge"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
