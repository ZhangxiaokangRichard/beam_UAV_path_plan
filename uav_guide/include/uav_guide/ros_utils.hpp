/**
 * @file ros_utils.hpp
 * @brief 节点侧复用层：ROS ↔ 算法层结构体适配（**唯一允许依赖 ROS 的 uav_guide 部分**）
 *
 * 三个节点共用：参数→POD 配置、Odometry→State5、beam_dubins/PathPoint→算法层 PathPoint、
 * 算法输出→自定义消息。算法层（geo/path/setpoint/guidance/orchestrator）保持零 ROS。
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "beam_dubins/msg/path_point.hpp"
#include "uav_guide/msg/guidance.hpp"
#include "uav_guide/msg/setpoint.hpp"
#include "uav_guide/types.hpp"

namespace uav_guide::ros_utils {

// ── 参数读取（YAML 中 int/double 混写均可）───────────────────
double get_double(const rclcpp::Node& node, const std::string& name, double fallback);
std::int64_t get_int(const rclcpp::Node& node, const std::string& name, std::int64_t fallback);
std::string get_string(const rclcpp::Node& node, const std::string& name,
                       const std::string& fallback);
bool get_bool(const rclcpp::Node& node, const std::string& name, bool fallback);

// ── 配置装配 ─────────────────────────────────────────────────
/// 读取地理原点（geodesy.origin_latitude_deg / longitude_deg / altitude_m）
GeoOrigin read_origin(const rclcpp::Node& node);
void read_setpoint_config(const rclcpp::Node& node, SetpointConfig& cfg);
void read_guidance_config(const rclcpp::Node& node, GuidanceConfig& cfg);
void read_orchestrator_config(const rclcpp::Node& node, OrchestratorConfig& cfg);
void read_space_config(const rclcpp::Node& node, SpaceConfig& cfg);
void read_plan_params(const rclcpp::Node& node, PlanParams& params);

// ── 状态/路径转换 ────────────────────────────────────────────
State5 to_state5(const nav_msgs::msg::Odometry& odom);
std::vector<PathPoint> to_path_points(const std::vector<beam_dubins::msg::PathPoint>& msg_path);

// ── 输出→消息 ────────────────────────────────────────────────
msg::Setpoint make_setpoint_msg(const SetpointOutput& out,
                                const std_msgs::msg::Header& header);
msg::Guidance make_guidance_msg(const GuidanceOutput& out,
                                const std_msgs::msg::Header& header);

}  // namespace uav_guide::ros_utils
