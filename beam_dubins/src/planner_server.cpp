/**
 * @file planner_server.cpp
 * @brief Beam Dubins 路径规划 ROS2 (Humble) Service 封装节点
 *
 * 与 1.0.1 的 ROS1 版本职责一一对应：
 *   1. 加载参数（同名扁平/点分层级参数）→ BeamConfig
 *   2. 提供规划服务（2.0 服务名：aoa/beam_dubins/plan_path）
 *   3. 服务回调：ROS 消息 ↔ C++ 结构体 → beam_dubins::plan_path() → 回填响应
 *
 * 分层约定：算法核心（beam_dubins_core）零 ROS 依赖；**本文件是全包唯一的 ROS 包装层**。
 */

#include <rclcpp/rclcpp.hpp>

#include <clocale>
#include <cstdint>
#include <string>
#include <vector>

#include "beam_dubins/beam_search.h"
#include "beam_dubins/msg/obstacle.hpp"
#include "beam_dubins/msg/path_point.hpp"
#include "beam_dubins/srv/plan_path.hpp"
#include "beam_dubins/types.h"

namespace {

// ═══════════════════════════════════════════════════════════════
// 1. 参数读取辅助（YAML 中 int/double 混写都能正确读取）
// ═══════════════════════════════════════════════════════════════

double param_double(const rclcpp::Node& node, const std::string& name, double fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return value.as_double();
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(value.as_int());
    return fallback;
}

std::int64_t param_int(const rclcpp::Node& node, const std::string& name, std::int64_t fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return value.as_int();
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return static_cast<std::int64_t>(value.as_double());
    return fallback;
}

std::string param_string(const rclcpp::Node& node, const std::string& name,
                         const std::string& fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        return value.as_string();
    return fallback;
}

// ═══════════════════════════════════════════════════════════════
// 2. 参数 → BeamConfig（参数名与 1.0.1 一致，仅去掉 /beam_dubins 前缀）
// ═══════════════════════════════════════════════════════════════

beam_dubins::BeamConfig load_config(const rclcpp::Node& node)
{
    beam_dubins::BeamConfig cfg;

    // ── 搜索参数 ──
    cfg.beam_width = static_cast<int>(param_int(node, "beam_width", cfg.beam_width));
    cfg.beam_width_max = static_cast<int>(param_int(node, "beam_width_max", cfg.beam_width_max));
    cfg.max_depth = static_cast<int>(param_int(node, "max_depth", cfg.max_depth));
    cfg.max_extend_length = param_double(node, "max_extend_length", cfg.max_extend_length);
    cfg.min_extend_length = param_double(node, "min_extend_length", cfg.min_extend_length);
    cfg.goal_tolerance_xy = param_double(node, "goal_tolerance_xy", cfg.goal_tolerance_xy);
    cfg.goal_tolerance_z = param_double(node, "goal_tolerance_z", cfg.goal_tolerance_z);
    cfg.dubins_shot_interval =
        static_cast<int>(param_int(node, "dubins_shot_interval", cfg.dubins_shot_interval));
    cfg.min_diversity_separation =
        param_double(node, "min_diversity_separation", cfg.min_diversity_separation);

    // ── 评分器权重 ──
    cfg.w_h = param_double(node, "scorer.w_h", cfg.w_h);
    cfg.w_c = param_double(node, "scorer.w_c", cfg.w_c);
    cfg.w_p = param_double(node, "scorer.w_p", cfg.w_p);

    // ── UAV 运动学约束 ──
    cfg.R_min = param_double(node, "uav.min_turn_radius_m", cfg.R_min);
    cfg.r_body = param_double(node, "uav.collision_radius_m", cfg.r_body);
    const double pitch_deg = param_double(node, "uav.max_pitch_angle_deg", 5.0);
    cfg.gamma_max_rad = pitch_deg * M_PI / 180.0;

    return cfg;
}

// ═══════════════════════════════════════════════════════════════
// 3. 消息转换：ROS msg ↔ C++ 结构体
// ═══════════════════════════════════════════════════════════════

beam_dubins::PlanRequest ros_to_request(
    const beam_dubins::srv::PlanPath::Request& ros_req,
    const rclcpp::Logger& logger)
{
    beam_dubins::PlanRequest req;

    // ── 起点 / 终点（5D 定长数组，rosidl 保证长度为 5）──
    req.start = {ros_req.start[0], ros_req.start[1], ros_req.start[2],
                 ros_req.start[3], ros_req.start[4]};
    req.goal = {ros_req.goal[0], ros_req.goal[1], ros_req.goal[2],
                ros_req.goal[3], ros_req.goal[4]};

    // ── 空间边界与选项 ──
    req.space_lx = ros_req.space_lx;
    req.space_ly = ros_req.space_ly;
    req.space_lz = ros_req.space_lz;
    req.boundary_margin = ros_req.boundary_margin;
    req.time_limit_ms = ros_req.time_limit_ms;
    req.use_3d = ros_req.use_3d;

    // ── 障碍物列表 ──
    req.obstacles.reserve(ros_req.obstacles.size());
    for (const auto& obs_msg : ros_req.obstacles) {
        beam_dubins::AABB aabb;
        aabb.min = {obs_msg.aabb_min[0], obs_msg.aabb_min[1], obs_msg.aabb_min[2]};
        aabb.max = {obs_msg.aabb_max[0], obs_msg.aabb_max[1], obs_msg.aabb_max[2]};
        req.obstacles.push_back(aabb);
    }

    if (ros_req.obstacles.empty()) {
        RCLCPP_DEBUG(logger, "PlanPath request: no obstacles (obstacle-free planning)");
    }
    return req;
}

void response_to_ros(const beam_dubins::PlanResponse& resp,
                     beam_dubins::srv::PlanPath::Response& ros_resp)
{
    ros_resp.success = resp.success;
    ros_resp.cost = resp.cost;
    ros_resp.nodes_explored = resp.nodes_explored;
    ros_resp.depth_reached = resp.depth_reached;
    ros_resp.planning_time_ms = resp.planning_time_ms;
    ros_resp.status_message = resp.status_message;

    ros_resp.path.clear();
    ros_resp.path.reserve(resp.path.size());
    for (const auto& pp : resp.path) {
        beam_dubins::msg::PathPoint ros_pp;
        ros_pp.x = pp.x;
        ros_pp.y = pp.y;
        ros_pp.z = pp.z;
        ros_pp.yaw = pp.yaw;
        ros_pp.pitch = pp.pitch;
        ros_pp.curvature = pp.curvature;
        ros_resp.path.push_back(ros_pp);
    }
}

// ═══════════════════════════════════════════════════════════════
// 4. 服务回调
// ═══════════════════════════════════════════════════════════════

void handle_plan_path(const std::shared_ptr<beam_dubins::srv::PlanPath::Request> req,
                      std::shared_ptr<beam_dubins::srv::PlanPath::Response> res,
                      const beam_dubins::BeamConfig& cfg,
                      const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger,
                "[planner_server] planning request received: "
                "start=(%.0f,%.0f,%.0f, ψ=%.1f°) goal=(%.0f,%.0f,%.0f, ψ=%.1f°) obstacles=%zu",
                req->start[0], req->start[1], req->start[2], req->start[3] * 180.0 / M_PI,
                req->goal[0], req->goal[1], req->goal[2], req->goal[3] * 180.0 / M_PI,
                req->obstacles.size());

    for (std::size_t i = 0; i < req->obstacles.size() && i < 3; ++i) {
        const auto& o = req->obstacles[i];
        RCLCPP_INFO(logger,
                    "[planner_server]   obstacle#%zu: min=(%.0f,%.0f,%.0f) max=(%.0f,%.0f,%.0f) "
                    "size=(%.0f×%.0f×%.0f)m",
                    i,
                    o.aabb_min[0], o.aabb_min[1], o.aabb_min[2],
                    o.aabb_max[0], o.aabb_max[1], o.aabb_max[2],
                    o.aabb_max[0] - o.aabb_min[0],
                    o.aabb_max[1] - o.aabb_min[1],
                    o.aabb_max[2] - o.aabb_min[2]);
    }

    const auto plan_req = ros_to_request(*req, logger);
    const auto plan_resp = beam_dubins::plan_path(plan_req, cfg);
    response_to_ros(plan_resp, *res);

    if (plan_resp.success) {
        RCLCPP_INFO(logger,
                    "[planner_server] planning succeeded: cost=%.1fm, path_points=%zu, "
                    "nodes=%d, depth=%d, time=%.1fms",
                    plan_resp.cost, plan_resp.path.size(), plan_resp.nodes_explored,
                    plan_resp.depth_reached, plan_resp.planning_time_ms);
    } else {
        RCLCPP_WARN(logger,
                    "[planner_server] planning failed: %s, nodes=%d, depth=%d, time=%.1fms",
                    plan_resp.status_message.c_str(), plan_resp.nodes_explored,
                    plan_resp.depth_reached, plan_resp.planning_time_ms);
    }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════
// 5. 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    std::setlocale(LC_ALL, "");  // 中文日志

    rclcpp::init(argc, argv);

    // 允许 YAML 中的参数键自动声明（无需逐个 declare_parameter）
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("planner_server", options);

    const beam_dubins::BeamConfig cfg = load_config(*node);
    const std::string service_name =
        param_string(*node, "service_name", "aoa/beam_dubins/plan_path");

    RCLCPP_INFO(node->get_logger(), "[planner_server] configuration loaded:");
    RCLCPP_INFO(node->get_logger(), "  beam_width=%d, beam_width_max=%d, max_depth=%d",
                cfg.beam_width, cfg.beam_width_max, cfg.max_depth);
    RCLCPP_INFO(node->get_logger(), "  max_extend=%.1fm, min_extend=%.1fm",
                cfg.max_extend_length, cfg.min_extend_length);
    RCLCPP_INFO(node->get_logger(), "  goal_tolerance: xy=%.1fm, z=%.1fm",
                cfg.goal_tolerance_xy, cfg.goal_tolerance_z);
    RCLCPP_INFO(node->get_logger(), "  R_min=%.1fm, gamma_max=%.1f°, r_body=%.1fm",
                cfg.R_min, cfg.gamma_max_rad * 180.0 / M_PI, cfg.r_body);
    RCLCPP_INFO(node->get_logger(), "  scorer: w_h=%.2f, w_c=%.2f, w_p=%.2f",
                cfg.w_h, cfg.w_c, cfg.w_p);

    const rclcpp::Logger logger = node->get_logger();
    auto service = node->create_service<beam_dubins::srv::PlanPath>(
        service_name,
        [cfg, logger](std::shared_ptr<beam_dubins::srv::PlanPath::Request> req,
                      std::shared_ptr<beam_dubins::srv::PlanPath::Response> res) {
            handle_plan_path(req, res, cfg, logger);
        });

    RCLCPP_INFO(node->get_logger(), "[planner_server] service ready: %s", service_name.c_str());
    RCLCPP_INFO(node->get_logger(), "[planner_server] waiting for planning requests...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
