/**
 * @file mqtt_control_bridge.cpp
 * @brief MQTT 出站控制桥：ROS 制导 → MQTT 的**纯翻译器**
 *
 * 出站主题：`aoa/uav_control/${uav_sn}/event_services`（QoS0，不 retain）
 *
 * 映射表（
 * mode 由 `aoa/uav/nav_mode` 得到，由 mqtt_mssn_bridge 从 MQTT `aoa/ai_guide/nav/mode` 转入）：
 *   auto     : 全 false（失效安全：不出网）
 *   setpoint : aoa/uav/setpoint ─1:1─► fly_point                                  (5 Hz)
 *   guidance : aoa/uav/guidance ─1:1─► set_cruise_mode + set_fly_head + set_fly_height (各 5 Hz)
 *
 * **不做节流、不做阈值去抖、不做时间戳去重**：出站频率 == 上游 ROS 话题频率
 * （两个制导节点均为 5 Hz 定时器）。模式是**状态**而非边沿：启动时靠 latched 的
 * `aoa/uav/nav_mode` 立即得知当前模式，无需等模式变化事件。
 *
 * 安全开关：
 *   mqtt.dry_run            = true  → 只打印不发布（联调用）
 *   mqtt.event_services_template    → 可将出站重定向到测试话题（默认协议话题）
 */

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <memory>
#include <string>
#include <unistd.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_mqtt_bridge/json_codec.h"
#include "ros_mqtt_bridge/mode_gate.h"
#include "ros_mqtt_bridge/mqtt_client.h"
#include "uav_guide/msg/guidance.hpp"
#include "uav_guide/msg/setpoint.hpp"

namespace {

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

/** 解析 nav_mode 载荷：纯模式名（"guidance"）或 JSON {"mode":...} / {"data":{"mode":...}} */
std::string extractNavMode(const std::string& payload)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    auto b = std::find_if(payload.begin(), payload.end(), not_space);
    auto e = std::find_if(payload.rbegin(), payload.rend(), not_space).base();
    if (b >= e) return {};
    const std::string trimmed(b, e);

    if (trimmed.front() != '{') return trimmed;   // 纯字符串形式
    try {
        std::stringstream stream(trimmed);
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

        event_tmpl_ = param_string(*this, "mqtt.event_services_template",
                                   "aoa/uav_control/${uav_sn}/event_services");
        dry_run_ = param_bool(*this, "mqtt.dry_run", false);
        // 排查开关：true = 每次进入 guidance 只发 1 条 set_cruise_mode 
        // 默认 false = 与 head/height 同频 5 Hz。
        cruise_mode_once_ = param_bool(*this, "cmd.cruise_mode_once_per_mode", true);

        const std::string nav_mode_topic =
            param_string(*this, "topics.nav_mode", "aoa/uav/nav_mode");
        setpoint_topic_ = param_string(*this, "topics.setpoint", "aoa/uav/setpoint");
        guidance_topic_ = param_string(*this, "topics.guidance", "aoa/uav/guidance");
        uav_state_topic_ = param_string(*this, "topics.uav_state", "aoa/uav/state");
        height_type_ = static_cast<int>(param_int(*this, "cmd.height_type", 0));

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

        sub_nav_mode_ = create_subscription<std_msgs::msg::String>(
            nav_mode_topic, latched,
            [this](std_msgs::msg::String::SharedPtr msg) { on_nav_mode(msg->data); });
        sub_setpoint_ = create_subscription<uav_guide::msg::Setpoint>(
            setpoint_topic_, 1,
            [this](uav_guide::msg::Setpoint::SharedPtr msg) { on_setpoint(*msg); });
        sub_guidance_ = create_subscription<uav_guide::msg::Guidance>(
            guidance_topic_, 1, [this](uav_guide::msg::Guidance::SharedPtr msg) { on_guidance(*msg); });
        sub_uav_state_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_state_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                if (!msg->child_frame_id.empty() && msg->child_frame_id != uav_sn_) {
                    uav_sn_ = msg->child_frame_id;
                    RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] uav_sn = %s → event topic %s",
                                uav_sn_.c_str(), expandTopic(event_tmpl_, uav_sn_).c_str());
                }
            });

        RCLCPP_INFO(get_logger(),
                    "[mqtt_control_bridge] ready（纯翻译器，无节流）: broker=%s:%d event_topic=%s "
                    "dry_run=%d nav_mode('%s')=%s height_type=%d cruise_mode_once=%d\n"
                    "  映射: auto→不出网 | setpoint→fly_point | "
                    "guidance→set_cruise_mode+set_fly_head+set_fly_height（与上游同频 5 Hz）",
                    host.c_str(), port, event_tmpl_.c_str(), static_cast<int>(dry_run_),
                    nav_mode_topic.c_str(), ros_mqtt_bridge::navModeName(mode_), height_type_,
                    static_cast<int>(cruise_mode_once_));
    }

private:
    // ── 发布（纯透传：无节流/无去抖）──
    void publishRaw(const std::string& payload, const char* what)
    {
        if (uav_sn_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] drop %s: uav_sn unknown (waiting for %s)",
                                 what, uav_state_topic_.c_str());
            return;
        }
        const std::string topic = expandTopic(event_tmpl_, uav_sn_);
        if (dry_run_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_control_bridge][dry-run] %s → %s: %s", what,
                                 topic.c_str(), payload.c_str());
            return;
        }
        if (!mqtt_->publish(topic, payload)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_control_bridge] publish %s failed (broker down? topic=%s)",
                                 what, topic.c_str());
            return;
        }
        RCLCPP_DEBUG(get_logger(), "[mqtt_control_bridge] %s → %s", what, topic.c_str());
    }

    // ── 模式（状态，非边沿）──
    void on_nav_mode(const std::string& payload)
    {
        const std::string raw = extractNavMode(payload);
        ros_mqtt_bridge::NavMode parsed;
        if (raw.empty() || !ros_mqtt_bridge::parseNavMode(raw, parsed)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] invalid nav_mode: '%s' "
                                 "(期望 auto | setpoint | guidance)",
                                 payload.c_str());
            return;
        }
        if (parsed == mode_) return;
        mode_ = parsed;
        cruise_mode_sent_ = false;   // 新模式进入：允许重新发一条 set_cruise_mode
        const ros_mqtt_bridge::OutboundPlan p = ros_mqtt_bridge::outboundPlan(mode_);
        RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] nav_mode = %s （出网: %s）",
                    ros_mqtt_bridge::navModeName(mode_),
                    p.fly_point        ? "fly_point"
                    : p.set_fly_head   ? "set_cruise_mode + set_fly_head + set_fly_height"
                                       : "无（auto）");
    }

    // ── 指点通道（setpoint 模式）：1 条 setpoint → 1 条 fly_point ──
    void on_setpoint(const uav_guide::msg::Setpoint& msg)
    {
        if (!ros_mqtt_bridge::forwardsSetpoint(mode_)) return;   // auto / guidance：丢弃
        publishRaw(ros_mqtt_bridge::encodeFlyPoint(msg.target_longitude, msg.target_latitude,
                                                   msg.target_altitude, msg.target_radius),
                   "fly_point");
    }

    // ── 制导通道（guidance 模式）：1 条 guidance → set_cruise_mode + head + height ──
    void on_guidance(const uav_guide::msg::Guidance& msg)
    {
        const ros_mqtt_bridge::OutboundPlan p = ros_mqtt_bridge::outboundPlan(mode_);
        if (!p.set_cruise_mode && !p.set_fly_head && !p.set_fly_height) return;

        if (p.set_cruise_mode &&
            ros_mqtt_bridge::shouldSendCruiseMode(cruise_mode_once_, cruise_mode_sent_)) {
            publishRaw(ros_mqtt_bridge::encodeSetCruiseMode(mid_gen_.next()), "set_cruise_mode");
            cruise_mode_sent_ = true;
        }
        if (p.set_fly_head) {
            publishRaw(ros_mqtt_bridge::encodeSetFlyHead(msg.heads, mid_gen_.next()), "set_fly_head");
        }
        if (p.set_fly_height) {
            publishRaw(ros_mqtt_bridge::encodeSetFlyHeight(msg.fly_height, height_type_,
                                                           mid_gen_.next()),
                       "set_fly_height");
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "[mqtt_control_bridge] guidance 透传（%s）: heads=%.1f height=%.1f "
                             "→ 3 条/帧 ≈ 15 msg/s",
                             ros_mqtt_bridge::navModeName(mode_), msg.heads, msg.fly_height);
    }

    // ── 成员 ──
    std::unique_ptr<ros_mqtt_bridge::MqttClient> mqtt_;
    ros_mqtt_bridge::MidGenerator mid_gen_;

    std::string event_tmpl_, setpoint_topic_, guidance_topic_, uav_state_topic_;
    std::string uav_sn_;

    ros_mqtt_bridge::NavMode mode_ = ros_mqtt_bridge::NavMode::Auto;   // 默认 auto = 不出网
    bool dry_run_ = false;
    bool cruise_mode_once_ = false;   // set_cruise_mode 是否改为“每次模式进入 1 条”
    bool cruise_mode_sent_ = false;   // 本次 guidance 进入是否已发过 set_cruise_mode
    int height_type_ = 0;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_nav_mode_;
    rclcpp::Subscription<uav_guide::msg::Setpoint>::SharedPtr sub_setpoint_;
    rclcpp::Subscription<uav_guide::msg::Guidance>::SharedPtr sub_guidance_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_state_;
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
