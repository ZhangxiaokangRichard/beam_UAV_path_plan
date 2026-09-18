/**
 * @file uav_guide_point_node.cpp
 * @brief 指点（setpoint）制导节点（R3①）：aoa/uav/planed_path → aoa/uav/setpoint
 *
 * 设计原理与 1.0.1 的 uav_guide_point_node 完全一致（几何不变），仅：
 *   - 改为 C++17（算法在 uav_guide::setpoint，零 ROS）
 *   - 话题名统一 aoa/ 前缀
 *   - **不间断发布**：不判断模式、不暂停（模式互斥下沉到 mqtt_control_bridge）
 *   - 不订阅 aoa/uav/set_cruise_mode（该话题对节点仅为 debug 状态流）
 */

#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "uav_guide/mode.hpp"
#include "uav_guide/msg/setpoint.hpp"
#include "uav_guide/msg/uav_planned_path.hpp"
#include "uav_guide/ros_utils.hpp"
#include "uav_guide/setpoint.hpp"
#include "uav_guide/types.hpp"

namespace {

class GuidePointNode : public rclcpp::Node {
public:
    GuidePointNode()
        : rclcpp::Node("uav_guide_point_node",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        topic_ = uav_guide::ros_utils::get_string(*this, "guide_point.topic", "aoa/uav/setpoint");
        const double rate_hz =
            uav_guide::ros_utils::get_double(*this, "guide_point.publish_rate_hz", 5.0);
        const std::string frame_id =
            uav_guide::ros_utils::get_string(*this, "guide_point.frame_id", "wgs84");

        uav_guide::ros_utils::read_setpoint_config(*this, cfg_);
        origin_ = uav_guide::ros_utils::read_origin(*this);
        mode_ = uav_guide::mode::normalize(
            uav_guide::ros_utils::get_string(*this, "intercept_mode", "head-on"));

        pub_ = create_publisher<uav_guide::msg::Setpoint>(topic_, rclcpp::QoS(1));

        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();

        sub_plan_ = create_subscription<uav_guide::msg::UavPlannedPath>(
            uav_guide::ros_utils::get_string(*this, "output.planed_path_topic", "aoa/uav/planed_path"),
            latched, [this](uav_guide::msg::UavPlannedPath::SharedPtr msg) {
                if (msg->success && !msg->path.empty()) {
                    path_ = uav_guide::ros_utils::to_path_points(msg->path);
                }
            });
        sub_uav_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_guide::ros_utils::get_string(*this, "inputs.uav_topic", "aoa/uav/state"), 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                uav_ = uav_guide::ros_utils::to_state5(*msg);
            });
        sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_guide::ros_utils::get_string(*this, "inputs.target_topic", "aoa/target/state"), 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                target_ = uav_guide::ros_utils::to_state5(*msg);
            });
        sub_mode_ = create_subscription<std_msgs::msg::String>(
            uav_guide::ros_utils::get_string(*this, "intercept_mode_status_topic",
                                             "aoa/uav/intercept_mode"),
            rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::String::SharedPtr msg) {
                mode_ = uav_guide::mode::normalize(msg->data);
            });

        const double period_s = (rate_hz > 0.1) ? (1.0 / rate_hz) : 0.2;
        timer_ = create_wall_timer(std::chrono::duration<double>(period_s),
                                   [this]() { publish_setpoint(); });
        frame_id_ = frame_id;

        if (!origin_.valid()) {
            RCLCPP_ERROR(get_logger(),
                         "[uav_guide_point] geodesy origin not configured "
                         "(geodesy.origin_latitude_deg/longitude_deg) — 输出经纬度将无效");
        }
        RCLCPP_INFO(get_logger(), "[uav_guide_point] publishing %s at %.1f Hz (segmentation.curvature_eps=%.5f, "
                                  "finalshot offset_m=%.1f offset_m_t=%.1f radius=%.1f)",
                    topic_.c_str(), rate_hz, cfg_.curvature_eps, cfg_.offset_m, cfg_.offset_m_t,
                    cfg_.offset_radius_m);
    }

private:
    void publish_setpoint()
    {
        if (!target_.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_point] waiting for target state ...");
            return;
        }
        const auto out = uav_guide::setpoint::build(path_, uav_, target_, mode_, origin_, cfg_);

        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = frame_id_;
        pub_->publish(uav_guide::ros_utils::make_setpoint_msg(out, header));
    }

    std::string topic_;
    std::string frame_id_ = "wgs84";
    uav_guide::SetpointConfig cfg_;
    uav_guide::GeoOrigin origin_;
    uav_guide::InterceptMode mode_ = uav_guide::InterceptMode::HeadOn;

    std::vector<uav_guide::PathPoint> path_;
    uav_guide::State5 uav_;
    uav_guide::State5 target_;

    rclcpp::Publisher<uav_guide::msg::Setpoint>::SharedPtr pub_;
    rclcpp::Subscription<uav_guide::msg::UavPlannedPath>::SharedPtr sub_plan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuidePointNode>());
    rclcpp::shutdown();
    return 0;
}
