/**
 * @file uav_guidance_node.cpp
 * @brief 巡航（cruise）制导节点（R3②，新增）：aoa/uav/planed_path → aoa/uav/guidance
 *
 * 采样点（Q12/Q16 裁决）：自本机沿 planed_path 起，取**首个曲率为 0 的点**；
 * heads 取该采样点在 ENU 下的航向（默认 sample_yaw），fly_height 取采样点高度。
 *
 * 位姿来源：`pose_source` = `tf2`（默认，查 map→uav）| `topic`（订阅 aoa/uav/state）；
 *           tf2 失败时自动回退 topic 并限流告警。
 *
 * **不间断发布**：不判断模式、不暂停（是否出网由 mqtt_control_bridge 依 MQTT 模式决定）；
 * 不订阅 aoa/uav/set_cruise_mode。
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
            uav_guide::ros_utils::get_double(*this, "guidance.publish_rate_hz", 1.0);
        heads_eps_deg_ = uav_guide::ros_utils::get_double(*this, "guidance.heads_eps_deg", 1.0);
        height_eps_m_ = uav_guide::ros_utils::get_double(*this, "guidance.height_eps_m", 0.5);
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
                    path_ = uav_guide::ros_utils::to_path_points(msg->path);
                    publish_if_changed();
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

        RCLCPP_INFO(get_logger(),
                    "[uav_guide_guidance] publishing %s at %.1f Hz (sample_rule=%s heads_source=%s "
                    "height_source=%s pose_source=%s fly_speed=%.1f)",
                    topic_.c_str(), rate_hz,
                    cfg_.sample_rule == uav_guide::SampleRule::FirstZeroCurvature
                        ? "first_zero_curvature" : "first_nonzero_curvature",
                    cfg_.heads_source == uav_guide::HeadsSource::SampleYaw ? "sample_yaw"
                                                                          : "bearing_to_sample",
                    cfg_.height_source == uav_guide::HeightSource::SamplePointZ ? "sample_point_z"
                                                                                : "target_z",
                    pose_source_.c_str(), cfg_.fly_speed_mps);
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

    void publish_if_changed()
    {
        if (path_.empty() || !target_.valid) return;
        const auto out = uav_guide::guidance::build(path_, resolve_uav_state(), target_, cfg_);
        const double dh = std::abs(out.heads_deg - last_heads_deg_);
        const double dz = std::abs(out.fly_height - last_fly_height_);
        const bool changed = !has_published_ || dh >= heads_eps_deg_ || dz >= height_eps_m_;
        if (changed) publish(out);
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
        if (out.sample_fallback) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_guidance] sample point fallback "
                                 "(no zero-curvature point in path; using end point)");
        }

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id_;
        pub_->publish(uav_guide::ros_utils::make_guidance_msg(out, header));

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "[uav_guide_guidance] heads=%.1f° fly_height=%.1f fly_speed=%.1f "
                             "(sample#%zu%s)",
                             out.heads_deg, out.fly_height, out.fly_speed_mps, out.sample_index,
                             out.sample_fallback ? ", fallback" : "");

        last_heads_deg_ = out.heads_deg;
        last_fly_height_ = out.fly_height;
        has_published_ = true;
    }

    std::string topic_, pose_source_, map_frame_, uav_frame_, frame_id_;
    double heads_eps_deg_ = 1.0;
    double height_eps_m_ = 0.5;

    uav_guide::GuidanceConfig cfg_;
    std::vector<uav_guide::PathPoint> path_;
    uav_guide::State5 target_;
    uav_guide::State5 uav_from_topic_;

    double last_heads_deg_ = 0.0;
    double last_fly_height_ = 0.0;
    bool has_published_ = false;

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
