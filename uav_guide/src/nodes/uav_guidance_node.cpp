/**
 * @file uav_guidance_node.cpp
 * @brief guidance 模式制导节点：aoa/uav/planed_path → aoa/uav/guidance
 *
 * heads 产生方式由 `guidance.heads_source` 决定：
 *   - `sample_yaw` / `bearing_to_sample`（Q12/Q16 裁决）：自本机沿 planed_path 起，
 *     取**首个曲率为 0 的点**，heads 取该采样点在 ENU 下的航向（默认 sample_yaw）；
 *   - `l1`：**L1 路径跟踪**（前视 L1 点 + 曲率饱和 + 前馈补偿），设计见 doc/L1_tracking_plan.md。
 * fly_height 由 `guidance.height_source` 决定（sample_point_z | target_z）。
 *
 * 发布节奏：**严格按 `guidance.publish_rate_hz`（现场 5 Hz）单一定时器周期发布**；
 *          不再做“路径到达变化触发”补发（否则会与定时器叠加成 5~9 Hz 的非均匀流，
 *          而 mqtt_control_bridge 对该话题做 1:1 透传，抖动会直接传到飞控）。
 *
 * 位姿来源：`pose_source` = `tf2`（查 map→uav）| `topic`（订阅 aoa/uav/state，现场推荐）；
 *           tf2 失败时自动回退 topic 并限流告警。
 *
 * **不间断发布**：不判断模式、不暂停（是否出网由 mqtt_control_bridge 依 `aoa/uav/nav_mode`
 * 决定）；不订阅 `aoa/uav/nav_mode`。
 */

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "uav_guide/geo.hpp"
#include "uav_guide/guidance.hpp"
#include "uav_guide/msg/guidance.hpp"
#include "uav_guide/msg/uav_planned_path.hpp"
#include "uav_guide/ros_utils.hpp"
#include "uav_guide/state.hpp"
#include "uav_guide/types.hpp"

namespace {

/// 保护持续周期上限（@5 Hz = 1 s）：超过则降级为“朝参考点直飞”
constexpr int kHoldLimit = 5;

class GuidanceNode : public rclcpp::Node {
public:
    GuidanceNode()
        : rclcpp::Node("uav_guidance_node",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_)
    {
        topic_ = uav_guide::ros_utils::get_string(*this, "guidance.topic", "aoa/uav/guidance");
        const double rate_hz =
            uav_guide::ros_utils::get_double(*this, "guidance.publish_rate_hz", 5.0);
        pose_source_ = uav_guide::ros_utils::get_string(*this, "guidance.pose_source", "tf2");
        map_frame_ = uav_guide::ros_utils::get_string(*this, "guidance.map_frame", "map");
        uav_frame_ = uav_guide::ros_utils::get_string(*this, "guidance.uav_frame", "aoa/uav_base_link");
        frame_id_ = map_frame_;

        uav_guide::ros_utils::read_guidance_config(*this, cfg_);

        pub_ = create_publisher<uav_guide::msg::Guidance>(topic_, rclcpp::QoS(1));

        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();

        sub_plan_ = create_subscription<uav_guide::msg::UavPlannedPath>(
            uav_guide::ros_utils::get_string(*this, "output.planed_path_topic", "aoa/uav/planed_path"),
            latched, [this](uav_guide::msg::UavPlannedPath::SharedPtr msg) {
                if (msg->success && !msg->path.empty()) {
                    path_ = uav_guide::ros_utils::to_path_points(msg->path);   // 仅缓存，发布交给定时器
                }
            });
        sub_uav_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_guide::ros_utils::get_string(*this, "inputs.uav_topic", "aoa/uav/state"), 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                uav_from_topic_ = uav_guide::ros_utils::to_state5(*msg);
            });
        sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_guide::ros_utils::get_string(*this, "inputs.target_topic", "aoa/target/state"), 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                target_ = uav_guide::ros_utils::to_state5(*msg);
            });

        const double period_s = (rate_hz > 0.1) ? (1.0 / rate_hz) : 1.0;
        timer_ = create_wall_timer(std::chrono::duration<double>(period_s),
                                   [this]() { publish_guidance(); });

        if (cfg_.heads_source == uav_guide::HeadsSource::L1) {
            RCLCPP_INFO(get_logger(),
                        "[uav_guide_guidance] publishing %s at %.1f Hz (heads_source=l1 "
                        "L1=%.0fm R_c=%.0fm T_lead=%.2fs speed=%.1fm/s deadband=%.2f° guard=%.0f° "
                        "height_source=%s pose_source=%s)",
                        topic_.c_str(), rate_hz, cfg_.l1_distance_m, cfg_.turn_radius_m,
                        cfg_.lead_time_s, cfg_.speed_mps, cfg_.beta_deadband_deg,
                        cfg_.beta_guard_deg,
                        cfg_.height_source == uav_guide::HeightSource::TargetZ ? "target_z"
                                                                              : "sample_point_z",
                        pose_source_.c_str());
        } else {
            RCLCPP_INFO(get_logger(),
                        "[uav_guide_guidance] publishing %s at %.1f Hz (sample_rule=%s "
                        "heads_source=%s height_source=%s pose_source=%s fly_speed=%.1f)",
                        topic_.c_str(), rate_hz,
                        cfg_.sample_rule == uav_guide::SampleRule::FirstZeroCurvature
                            ? "first_zero_curvature" : "first_nonzero_curvature",
                        cfg_.heads_source == uav_guide::HeadsSource::SampleYaw
                            ? "sample_yaw" : "bearing_to_sample",
                        cfg_.height_source == uav_guide::HeightSource::SamplePointZ
                            ? "sample_point_z" : "target_z",
                        pose_source_.c_str(), cfg_.fly_speed_mps);
        }
    }

private:
    /// 解析本机位姿：优先 tf2（map→uav），失败回退 aoa/uav/state
    uav_guide::State5 resolve_uav_state()
    {
        if (pose_source_ != "tf2") return uav_from_topic_;

        try {
            const auto tf = tf_buffer_.lookupTransform(map_frame_, uav_frame_, tf2::TimePointZero);
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;
            return uav_guide::state::from_odometry(t.x, t.y, t.z, q.x, q.y, q.z, q.w);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_guidance] tf2 %s→%s failed (%s), fallback to topic",
                                 map_frame_.c_str(), uav_frame_.c_str(), ex.what());
            return uav_from_topic_;
        }
    }

    void publish_guidance()
    {
        if (path_.empty() || !target_.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_guidance] waiting for %s / target state ...",
                                 "aoa/uav/planed_path");
            return;
        }
        publish(uav_guide::guidance::build(path_, resolve_uav_state(), target_, cfg_));
    }

    void publish(const uav_guide::GuidanceOutput& out)
    {
        const bool l1_mode = (cfg_.heads_source == uav_guide::HeadsSource::L1);
        uav_guide::GuidanceOutput cmd = out;   // 允许改写 heads（保持上一指令 / 保护降级）

        if (out.hold_previous) {
            if (out.l1_eff_m <= 0.0) {
                // 位姿/路径不可用：本周期不发布（宁可静默，也不发无效航向）
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "[uav_guide_guidance] L1: 位姿/路径不可用 → 本周期不发布");
                return;
            }
            ++hold_count_;
            if (has_last_heads_ && hold_count_ < kHoldLimit) {
                cmd.heads_deg = last_heads_deg_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[uav_guide_guidance] L1 保护: β=%+.1f° (|β|>%.0f°) → "
                                     "保持上一指令 heads=%.1f°（第 %d 周期）",
                                     out.beta_deg, cfg_.beta_guard_deg, cmd.heads_deg, hold_count_);
            } else {
                // 持续越界 → 降级为“朝参考点直飞”（cmd.heads_deg 已为 LOS 航向）
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "[uav_guide_guidance] L1 保护持续 %d 周期 → 降级为直飞参考点 "
                                     "heads=%.1f°",
                                     hold_count_, cmd.heads_deg);
            }
        } else {
            hold_count_ = 0;
        }

        if (out.sample_fallback && !l1_mode) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_guidance] sample point fallback "
                                 "(no zero-curvature point in path; using end point)");
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id_;
        pub_->publish(uav_guide::ros_utils::make_guidance_msg(cmd, header));
        last_heads_deg_ = cmd.heads_deg;
        has_last_heads_  = true;

        if (l1_mode) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[uav_guide_guidance] L1: heads=%.1f° β=%+.1f° κ=%+.5f%s "
                                 "L1=%.0fm%s fly_height=%.1f",
                                 cmd.heads_deg, out.beta_deg, out.kappa_cmd,
                                 out.l1_saturated ? "(饱和)" : "", out.l1_eff_m,
                                 out.near_end ? "(near_end)" : "", cmd.fly_height);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[uav_guide_guidance] heads=%.1f° fly_height=%.1f fly_speed=%.1f "
                                 "(sample#%zu%s)",
                                 cmd.heads_deg, cmd.fly_height, cmd.fly_speed_mps, out.sample_index,
                                 out.sample_fallback ? ", fallback" : "");
        }
    }

    std::string topic_, pose_source_, map_frame_, uav_frame_, frame_id_;

    uav_guide::GuidanceConfig cfg_;
    std::vector<uav_guide::PathPoint> path_;
    uav_guide::State5 target_;
    uav_guide::State5 uav_from_topic_;

    // L1 保护状态：保持上一指令 / 连续越界降级
    bool   has_last_heads_ = false;
    double last_heads_deg_ = 0.0;
    int    hold_count_     = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<uav_guide::msg::Guidance>::SharedPtr pub_;
    rclcpp::Subscription<uav_guide::msg::UavPlannedPath>::SharedPtr sub_plan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidanceNode>());
    rclcpp::shutdown();
    return 0;
}
