/**
 * @file uav_guide_loop_node.cpp
 * @brief 规划主循环节点（R3①）：state 更新即规划 → aoa/uav/planed_path
 *
 * 职责边界（全权分布式）：**只专注规划**，不做模式判断、不产生制导输出。
 * 内部使用 uav_guide::orchestrator 的 APPROACH / TRACKING 状态机决定规划终点。
 * 观测话题：aoa/uav/tracking_mode、aoa/uav/intercept_mode（transient_local，1 Hz 心跳）。
 */

#include <chrono>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "beam_dubins/srv/plan_path.hpp"
#include "uav_guide/mode.hpp"
#include "uav_guide/msg/uav_planned_path.hpp"
#include "uav_guide/orchestrator.hpp"
#include "uav_guide/ros_utils.hpp"
#include "uav_guide/types.hpp"

namespace {

class GuideLoopNode : public rclcpp::Node {
public:
    GuideLoopNode()
        : rclcpp::Node("uav_guide_loop_node",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // ── 参数 ──
        uav_topic_ = uav_guide::ros_utils::get_string(*this, "inputs.uav_topic", "aoa/uav/state");
        target_topic_ =
            uav_guide::ros_utils::get_string(*this, "inputs.target_topic", "aoa/target/state");
        mode_cmd_topic_ = uav_guide::ros_utils::get_string(*this, "intercept_mode_topic",
                                                           "aoa/uav/intercept_mode_cmd");
        mode_status_topic_ = uav_guide::ros_utils::get_string(*this, "intercept_mode_status_topic",
                                                              "aoa/uav/intercept_mode");
        planed_path_topic_ = uav_guide::ros_utils::get_string(*this, "output.planed_path_topic",
                                                              "aoa/uav/planed_path");
        tracking_topic_ = uav_guide::ros_utils::get_string(*this, "output.tracking_mode_topic",
                                                           "aoa/uav/tracking_mode");
        service_name_ =
            uav_guide::ros_utils::get_string(*this, "planning.service", "aoa/beam_dubins/plan_path");

        uav_guide::ros_utils::read_orchestrator_config(*this, orch_cfg_);
        uav_guide::ros_utils::read_plan_params(*this, plan_params_);
        sm_ = std::make_unique<uav_guide::orchestrator::StateMachine>(orch_cfg_);
        mode_ = uav_guide::mode::normalize(
            uav_guide::ros_utils::get_string(*this, "intercept_mode", "head-on"));

        // ── QoS：等效 ROS1 latch ──
        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();

        pub_plan_ = create_publisher<uav_guide::msg::UavPlannedPath>(planed_path_topic_, latched);
        pub_mode_ = create_publisher<std_msgs::msg::String>(mode_status_topic_, latched);
        pub_tracking_ = create_publisher<std_msgs::msg::Bool>(tracking_topic_, latched);

        sub_uav_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                uav_ = uav_guide::ros_utils::to_state5(*msg);
                on_state_update();
            });
        sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
            target_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                target_ = uav_guide::ros_utils::to_state5(*msg);
                on_state_update();
            });
        sub_mode_cmd_ = create_subscription<std_msgs::msg::String>(
            mode_cmd_topic_, 10, [this](std_msgs::msg::String::SharedPtr msg) {
                on_mode_command(msg->data);
            });

        client_ = create_client<beam_dubins::srv::PlanPath>(service_name_);

        // 观测话题 1 Hz 心跳（迟加入者也能看到）
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish_tracking(); });

        publish_mode_status();
        RCLCPP_INFO(get_logger(),
                    "[uav_guide_loop] ready: plan on state update; planed_path=%s service=%s "
                    "mode=%s (entry=%.0fm align=%.0f° closure=%.1f/%.1fm)",
                    planed_path_topic_.c_str(), service_name_.c_str(),
                    uav_guide::mode::to_string(mode_), orch_cfg_.tracking_entry_dist,
                    orch_cfg_.velocity_align_deg, orch_cfg_.min_closure_m,
                    orch_cfg_.min_closure_m_tail);
    }

private:
    void on_state_update()
    {
        if (!uav_.valid || !target_.valid) return;
        if (pending_) return;   // 上一帧规划尚未返回，跳过本帧（避免请求堆积）

        const auto decision = sm_->update(uav_, target_, mode_);

        if (decision.tracking_changed) {
            publish_tracking();
            RCLCPP_INFO(get_logger(),
                        "[uav_guide_loop] phase -> %s (d=%.0fm closure=%.0fm align=%.1f°)",
                        decision.phase == uav_guide::Phase::Tracking ? "TRACKING" : "APPROACH",
                        decision.distance, decision.closure, decision.align_err_deg);
        } else {
            RCLCPP_DEBUG(get_logger(),
                         "[uav_guide_loop] tracking gate (%s): d=%.0fm align=%.1f° closure=%.1fm",
                         uav_guide::mode::to_string(mode_), decision.distance,
                         decision.align_err_deg, decision.closure);
        }

        if (!client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_loop] waiting for %s ...", service_name_.c_str());
            return;
        }

        auto req = std::make_shared<beam_dubins::srv::PlanPath::Request>();
        req->header.stamp = now();
        req->header.frame_id = plan_params_.frame_id;
        req->start = {uav_.x, uav_.y, uav_.z, uav_.yaw, uav_.pitch};
        req->goal = {decision.goal.x, decision.goal.y, decision.goal.z, decision.goal.yaw,
                     decision.goal.pitch};
        req->space_lx = plan_params_.space.lx;
        req->space_ly = plan_params_.space.ly;
        req->space_lz = plan_params_.space.lz;
        req->boundary_margin = plan_params_.space.boundary_margin;
        req->obstacles.clear();
        req->time_limit_ms = static_cast<float>(plan_params_.time_limit_ms);
        req->use_3d = plan_params_.use_3d;

        pending_ = true;
        const uav_guide::Phase phase = decision.phase;
        client_->async_send_request(
            req, [this, phase](rclcpp::Client<beam_dubins::srv::PlanPath>::SharedFuture future) {
                pending_ = false;
                publish_plan(*future.get(), phase);
            });
    }

    void publish_plan(const beam_dubins::srv::PlanPath::Response& resp, uav_guide::Phase phase)
    {
        uav_guide::msg::UavPlannedPath msg;
        msg.header.stamp = now();
        msg.header.frame_id = plan_params_.frame_id;
        msg.success = resp.success;
        msg.cost = resp.cost;
        msg.status_message = resp.status_message;
        msg.path = resp.path;
        pub_plan_->publish(msg);

        const char* phase_str = (phase == uav_guide::Phase::Tracking) ? "TRACKING" : "APPROACH";
        if (resp.success) {
            RCLCPP_INFO(get_logger(), "[uav_guide_loop] plan ok cost=%.1f pts=%zu phase=%s intercept=%s",
                        resp.cost, resp.path.size(), phase_str,
                        uav_guide::mode::to_string(mode_));
        } else {
            RCLCPP_WARN(get_logger(), "[uav_guide_loop] plan failed phase=%s: %s", phase_str,
                        resp.status_message.c_str());
        }
    }

    void on_mode_command(const std::string& raw)
    {
        const auto parsed = uav_guide::mode::parse(raw);
        if (!parsed.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "[uav_guide_loop] invalid intercept mode: '%s' (use head-on|tail)",
                                 raw.c_str());
            return;
        }
        if (*parsed == mode_) return;

        const auto old = mode_;
        mode_ = *parsed;
        sm_->reset(mode_);   // 清缓存、退出 TRACKING
        publish_mode_status();
        publish_tracking();
        RCLCPP_INFO(get_logger(), "[uav_guide_loop] intercept mode -> %s (from %s), planning state reset",
                    uav_guide::mode::to_string(mode_), uav_guide::mode::to_string(old));

        if (uav_.valid && target_.valid) on_state_update();   // 立即重规划
    }

    void publish_mode_status()
    {
        std_msgs::msg::String msg;
        msg.data = uav_guide::mode::to_string(mode_);
        pub_mode_->publish(msg);
    }

    void publish_tracking()
    {
        std_msgs::msg::Bool msg;
        msg.data = sm_->tracking();
        pub_tracking_->publish(msg);
    }

    // ── 成员 ──
    std::string uav_topic_, target_topic_, mode_cmd_topic_, mode_status_topic_;
    std::string planed_path_topic_, tracking_topic_, service_name_;

    uav_guide::OrchestratorConfig orch_cfg_;
    uav_guide::PlanParams plan_params_;
    std::unique_ptr<uav_guide::orchestrator::StateMachine> sm_;
    uav_guide::InterceptMode mode_ = uav_guide::InterceptMode::HeadOn;

    uav_guide::State5 uav_;
    uav_guide::State5 target_;
    bool pending_ = false;

    rclcpp::Publisher<uav_guide::msg::UavPlannedPath>::SharedPtr pub_plan_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_tracking_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_cmd_;
    rclcpp::Client<beam_dubins::srv::PlanPath>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GuideLoopNode>());
    rclcpp::shutdown();
    return 0;
}
