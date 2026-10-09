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
 *   guidance : aoa/uav/guidance ─1:1─► set_cruise_mode + set_fly_head              (各 5 Hz)
 *                                    └► set_fly_height **闭环重发门控**
 *
 * **不做节流、不做阈值去抖、不做时间戳去重**：出站频率 == 上游 ROS 话题频率
 * （两个制导节点均为 5 Hz 定时器）。模式是**状态**而非边沿：启动时靠 latched 的
 * `aoa/uav/nav_mode` 立即得知当前模式，无需等模式变化事件。
 *
 * **例外（唯一有状态的一路）：`set_fly_height` —— 闭环重发门控**
 * 现场问题（2026-10-09）：高度若 5 Hz 持续下发会使飞控「持续初始化」；但自举式死区门控
 * 又会在指令稳定时**永不重发**（→ 高度静差），目标快速变化时又发得过快。
 * 现方案：每发一条后监测 `aoa/uav/state` 的高度变化率 dz/dt：
 *   - `|cmd − 实测| <= height_reach_eps_m`（默认 0.5 m） → 已到位，静默
 *   - `|dz/dt| < height_rate_eps_mps`（默认 0.25 m/s）且未到位 → **补发一条**
 *   - 仍在爬/降（速度 ≥ 阈值） → 不打断，等它停下再说
 *   - 距上次下发 < `height_resend_delay_s`（默认 1 s） → 不补发（等飞机反应）
 * 实测高度取 `aoa/uav/state.pose.pose.position.z`（ENU，m），可用 `cmd.height_offset_m`
 * 修正与飞控高度基准的固定偏差。
 *
 * 安全开关：
 *   mqtt.dry_run            = true  → 只打印不发布（联调用）
 *   mqtt.event_services_template    → 可将出站重定向到测试话题（默认协议话题）
 */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
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

double param_double(const rclcpp::Node& node, const std::string& name, double fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto v = node.get_parameter(name);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.as_double();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(v.as_int());
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
        // ── 高度闭环重发门控参数（阈值全部来自 yaml）──
        height_reach_eps_m_ = param_double(*this, "cmd.height_reach_eps_m", 0.5);
        height_rate_eps_mps_ = param_double(*this, "cmd.height_rate_eps_mps", 0.25);
        height_resend_delay_s_ = param_double(*this, "cmd.height_resend_delay_s", 1.0);
        height_offset_m_ = param_double(*this, "cmd.height_offset_m", 0.0);
        height_state_timeout_s_ = param_double(*this, "cmd.height_state_timeout_s", 2.0);
        height_rate_window_s_ = param_double(*this, "cmd.height_rate_window_s", 1.0);

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

                // ── 高度闭环反馈：pose.z + **滑动窗差分**变化率 ──
                // 不用 EMA：一阶平滑从 20 m/s 衰减到阈值以下要 ~3 s，会把“已停下”
                // 漏判；滑动窗差分（窗长可调）在 1 个窗内确定性给出结果。
                const double z = msg->pose.pose.position.z;
                const rclcpp::Time now = this->now();
                measured_z_ = z;
                has_measured_z_ = true;
                last_z_time_ = now;

                z_hist_.emplace_back(now, z);
                const double win = std::max(0.2, height_rate_window_s_);
                while (!z_hist_.empty() && (now - z_hist_.front().first).seconds() > 3.0 * win) {
                    z_hist_.pop_front();
                }
                has_z_rate_ = false;
                for (auto it = z_hist_.rbegin(); it != z_hist_.rend(); ++it) {
                    const double dt = (now - it->first).seconds();
                    if (dt >= win) {
                        z_rate_mps_ = (z - it->second) / dt;
                        has_z_rate_ = true;
                        break;
                    }
                }
            });

        const std::string height_gate =
            "闭环重发: |cmd−实测|>" + std::to_string(height_reach_eps_m_) +
            "m 且 |dz/dt|<" + std::to_string(height_rate_eps_mps_) +
            "m/s → 补发（最小间隔 " + std::to_string(height_resend_delay_s_) + "s）";
        RCLCPP_INFO(get_logger(),
                    "[mqtt_control_bridge] ready（纯翻译器，无节流）: broker=%s:%d event_topic=%s "
                    "dry_run=%d nav_mode('%s')=%s height_type=%d offset=%.1fm "
                    "cruise_mode_once=%d\n"
                    "  映射: auto→不出网 | setpoint→fly_point | "
                    "guidance→set_cruise_mode+set_fly_head（5 Hz）+set_fly_height（%s）",
                    host.c_str(), port, event_tmpl_.c_str(), static_cast<int>(dry_run_),
                    nav_mode_topic.c_str(), ros_mqtt_bridge::navModeName(mode_), height_type_,
                    height_offset_m_, static_cast<int>(cruise_mode_once_), height_gate.c_str());
    }

private:
    // ── 发布（纯透传：无节流/无去抖）──
    /// @return 是否已成功处理（发布成功 / dry_run 打印）
    bool publishRaw(const std::string& payload, const char* what)
    {
        if (uav_sn_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[mqtt_control_bridge] drop %s: uav_sn unknown (waiting for %s)",
                                 what, uav_state_topic_.c_str());
            return false;
        }
        const std::string topic = expandTopic(event_tmpl_, uav_sn_);
        if (dry_run_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_control_bridge][dry-run] %s → %s: %s", what,
                                 topic.c_str(), payload.c_str());
            return true;
        }
        if (!mqtt_->publish(topic, payload)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[mqtt_control_bridge] publish %s failed (broker down? topic=%s)",
                                 what, topic.c_str());
            return false;
        }
        RCLCPP_DEBUG(get_logger(), "[mqtt_control_bridge] %s → %s", what, topic.c_str());
        return true;
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
        height_sent_ = false;        // 新模式进入：高度必须重建闭环（必发一条）
        height_resend_streak_ = 0;   // 重置“未到位连续补发”计数
        const ros_mqtt_bridge::OutboundPlan p = ros_mqtt_bridge::outboundPlan(mode_);
        RCLCPP_INFO(get_logger(), "[mqtt_control_bridge] nav_mode = %s （出网: %s）",
                    ros_mqtt_bridge::navModeName(mode_),
                    p.fly_point        ? "fly_point"
                    : p.set_fly_head   ? "set_cruise_mode + set_fly_head + set_fly_height(门控)"
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
            // 只有**真正发出**才算“本模式已发”：否则前几帧 uav_sn 未知（publishRaw 丢弃）
            // 会把状态置位，导致 set_cruise_mode 永久丢失（飞控不被切到巡航模式）。
            if (publishRaw(ros_mqtt_bridge::encodeSetCruiseMode(mid_gen_.next()), "set_cruise_mode")) {
                cruise_mode_sent_ = true;
            }
        }
        if (p.set_fly_head) {
            publishRaw(ros_mqtt_bridge::encodeSetFlyHead(msg.heads, mid_gen_.next()), "set_fly_head");
        }
        // 高度：**闭环重发门控**（发一条 → 看飞机停没停、到没到位 → 未到位再补发）
        if (p.set_fly_height) {
            const rclcpp::Time now = this->now();
            const bool fresh =
                has_measured_z_ && (now - last_z_time_).seconds() <= height_state_timeout_s_;
            const double measured = measured_z_ + height_offset_m_;
            const double elapsed_s = (now - last_height_cmd_time_).seconds();

            bool send = false;
            if (!height_sent_) {
                send = true;                     // 首条：建立闭环
            } else if (!fresh) {
                // 无有效高度反馈 → 不做闭环判定（避免盲目补发）
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "[mqtt_control_bridge] set_fly_height: aoa/uav/state 高度反馈"
                                     "超时（>%.1fs），暫停补发",
                                     height_state_timeout_s_);
            } else {
                send = ros_mqtt_bridge::shouldResendFlyHeight(
                    true, elapsed_s, msg.fly_height, measured,
                    has_z_rate_ ? z_rate_mps_ : 0.0,   // 历史不足 → 按“已停下”处理（有 delay 兜底）
                    height_reach_eps_m_, height_rate_eps_mps_, height_resend_delay_s_);
            }

            if (send) {
                if (publishRaw(ros_mqtt_bridge::encodeSetFlyHeight(msg.fly_height, height_type_,
                                                                  mid_gen_.next()),
                               "set_fly_height")) {
                    last_height_cmd_ = msg.fly_height;
                    last_height_cmd_time_ = now;
                    height_sent_ = true;
                    const double err = std::fabs(msg.fly_height - measured);
                    RCLCPP_INFO(get_logger(),
                                "[mqtt_control_bridge] set_fly_height 下发: %.1f m（实测 %.1f m，"
                                "差 %.1f m，dz/dt %.2f m/s，height_type=%d offset=%.1f m）",
                                msg.fly_height, measured, err, z_rate_mps_, height_type_,
                                height_offset_m_);
                    if (err > height_reach_eps_m_) {
                        if (++height_resend_streak_ == 5) {
                            RCLCPP_WARN(get_logger(),
                                        "[mqtt_control_bridge] 已连续 5 次下发高度但误差仍 %.1f m"
                                        "（> %.1f m）：疑似高度基准不一致，请校准 "
                                        "cmd.height_offset_m（当前 %.1f m）",
                                        err, height_reach_eps_m_, height_offset_m_);
                        }
                    } else {
                        height_resend_streak_ = 0;
                    }
                }
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                                     "[mqtt_control_bridge] set_fly_height 静默: 指令 %.1f m / "
                                     "实测 %.1f m（差 %.1f m, dz/dt %.2f m/s, 距上次 %.1f s）",
                                     msg.fly_height, measured, std::fabs(msg.fly_height - measured),
                                     z_rate_mps_, elapsed_s);
            }
        }

        RCLCPP_DEBUG(get_logger(),
                     "[mqtt_control_bridge] guidance 透传（%s）: heads=%.1f height=%.1f",
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
    // ── 高度闭环反馈（来自 aoa/uav/state 的 pose.z）──
    double measured_z_ = 0.0;         // 实测高度（ENU z, m）
    double z_rate_mps_ = 0.0;         // 滑动窗差分的高度变化率（m/s）
    bool has_measured_z_ = false;
    bool has_z_rate_ = false;         // 历史样本足够、已算出变化率
    std::deque<std::pair<rclcpp::Time, double>> z_hist_;   // (时刻, z) 环形历史
    rclcpp::Time last_z_time_{0, 0, RCL_ROS_TIME};
    // ── 高度闭环重发门控状态 ──
    double height_reach_eps_m_ = 0.5;      // 到位判据（m）
    double height_rate_eps_mps_ = 0.25;    // 静止判据（m/s）
    double height_rate_window_s_ = 1.0;    // 变化率计算窗长（s）
    double height_resend_delay_s_ = 1.0;   // 最小重发间隔（s）
    double height_offset_m_ = 0.0;         // 实测高度基准修正
    double height_state_timeout_s_ = 2.0;  // 状态反馈超时（s）
    double last_height_cmd_ = 0.0;         // 上次已下发的高度指令
    rclcpp::Time last_height_cmd_time_{0, 0, RCL_ROS_TIME};
    bool height_sent_ = false;             // 本次模式进入后是否已下发过
    int height_resend_streak_ = 0;         // 连续未到位补发计数（基准异常告警用）

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
