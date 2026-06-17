/**
 * @file planner_server.cpp
 * @brief Beam Dubins 路径规划 ROS Service 封装节点
 *
 * Phase 2a: 将 Phase 1 的纯 C++ 算法库包装为 ROS Service。
 *
 * 功能：
 *  1. 加载 rosparam 参数（/beam_dubins/ 命名空间）
 *  2. 创建 ServiceServer（/beam_dubins/plan_path）
 *  3. 服务回调中：ROS 消息 ↔ C++ 结构体转换 → 调用 plan_path() → 返回结果
 *
 * 设计要点：
 *  - 服务节点无状态：每次调用独立实例化搜索器
 *  - 参数在启动时加载，运行时不变（Phase 5 可加 dynamic_reconfigure）
 *  - 记录规划耗时并与响应一起返回
 */

#include <ros/ros.h>

#include <chrono>
#include <string>
#include <vector>

#include "beam_dubins/Obstacle.h"
#include "beam_dubins/PathPoint.h"
#include "beam_dubins/PlanPath.h"
#include "beam_dubins/beam_search.h"
#include "beam_dubins/types.h"

// ═══════════════════════════════════════════════════════════════
// 1. 参数加载
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 从 ROS 参数服务器加载 BeamConfig
 *
 * 参数位于 /beam_dubins/ 命名空间下。
 * 使用默认值回退机制：参数缺失时使用 BeamConfig 的构造默认值。
 */
static beam_dubins::BeamConfig load_config(ros::NodeHandle& nh)
{
    beam_dubins::BeamConfig cfg;

    // 参数位于 /beam_dubins/ 命名空间下，通过传入的 nh 句柄访问

    // ── 搜索参数 ─────────────────────────────────────────────
    nh.param("beam_width",               cfg.beam_width,               cfg.beam_width);
    nh.param("beam_width_max",           cfg.beam_width_max,           cfg.beam_width_max);
    nh.param("max_depth",                cfg.max_depth,                cfg.max_depth);
    nh.param("max_extend_length",        cfg.max_extend_length,        cfg.max_extend_length);
    nh.param("min_extend_length",        cfg.min_extend_length,        cfg.min_extend_length);
    nh.param("goal_tolerance_xy",        cfg.goal_tolerance_xy,        cfg.goal_tolerance_xy);
    nh.param("goal_tolerance_z",         cfg.goal_tolerance_z,         cfg.goal_tolerance_z);
    nh.param("dubins_shot_interval",    cfg.dubins_shot_interval,    cfg.dubins_shot_interval);
    nh.param("min_diversity_separation", cfg.min_diversity_separation, cfg.min_diversity_separation);

    // ── 评分器权重（嵌套子命名空间 scorer/）─────────────────
    nh.param("scorer/w_h", cfg.w_h, cfg.w_h);
    nh.param("scorer/w_c", cfg.w_c, cfg.w_c);
    nh.param("scorer/w_p", cfg.w_p, cfg.w_p);

    // ── UAV 运动学约束 ──────────────────────────────────────
    nh.param("uav/min_turn_radius_m",  cfg.R_min,         cfg.R_min);
    nh.param("uav/collision_radius_m", cfg.r_body,        cfg.r_body);

    // gamma_max: 从角度(°) → 弧度(rad)
    double pitch_deg = 5.0;
    nh.param("uav/max_pitch_angle_deg", pitch_deg, pitch_deg);
    cfg.gamma_max_rad = pitch_deg * M_PI / 180.0;

    return cfg;
}

// ═══════════════════════════════════════════════════════════════
// 2. 消息转换：ROS msg ↔ C++ 结构体
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 将 ROS PlanPath 请求转换为 C++ PlanRequest
 *
 * 提取 start, goal, 空间边界, 障碍物列表等字段。
 * 使用 use_3d 标志位控制 3D 模式（Phase 4+ 启用）。
 */
static beam_dubins::PlanRequest ros_to_request(
    const beam_dubins::PlanPath::Request& ros_req)
{
    beam_dubins::PlanRequest req;

    // ── 起点（5D 状态数组）───────────────────────────────────
    if (ros_req.start.size() >= 5) {
        req.start = {ros_req.start[0], ros_req.start[1], ros_req.start[2],
                     ros_req.start[3], ros_req.start[4]};
    } else {
        ROS_WARN("PlanPath request: start array has < 5 elements, padding zeros");
        req.start = {0, 0, 0, 0, 0};
    }

    // ── 终点 ─────────────────────────────────────────────────
    if (ros_req.goal.size() >= 5) {
        req.goal = {ros_req.goal[0], ros_req.goal[1], ros_req.goal[2],
                    ros_req.goal[3], ros_req.goal[4]};
    } else {
        ROS_WARN("PlanPath request: goal array has < 5 elements, padding zeros");
        req.goal = {0, 0, 0, 0, 0};
    }

    // ── 空间边界 ─────────────────────────────────────────────
    req.space_lx        = ros_req.space_lx;
    req.space_ly        = ros_req.space_ly;
    req.space_lz        = ros_req.space_lz;
    req.boundary_margin = ros_req.boundary_margin;
    req.time_limit_ms   = ros_req.time_limit_ms;
    req.use_3d          = ros_req.use_3d;

    // ── 障碍物列表 ───────────────────────────────────────────
    req.obstacles.reserve(ros_req.obstacles.size());
    for (const auto& obs_msg : ros_req.obstacles) {
        beam_dubins::AABB aabb;
        if (obs_msg.aabb_min.size() >= 3 && obs_msg.aabb_max.size() >= 3) {
            aabb.min = {obs_msg.aabb_min[0], obs_msg.aabb_min[1], obs_msg.aabb_min[2]};
            aabb.max = {obs_msg.aabb_max[0], obs_msg.aabb_max[1], obs_msg.aabb_max[2]};
        }
        req.obstacles.push_back(aabb);
    }

    return req;
}

/**
 * @brief 将 C++ PlanResponse 转换为 ROS PlanPath 响应
 *
 * 填充 path、cost、状态消息等字段。
 * PathPoint 消息额外携带 curvature 字段（用于速度自适应）。
 */
static void response_to_ros(const beam_dubins::PlanResponse& resp,
                            beam_dubins::PlanPath::Response& ros_resp)
{
    ros_resp.success          = resp.success;
    ros_resp.cost             = resp.cost;
    ros_resp.nodes_explored   = resp.nodes_explored;
    ros_resp.depth_reached    = resp.depth_reached;
    ros_resp.planning_time_ms = resp.planning_time_ms;
    ros_resp.status_message   = resp.status_message;

    // ── 路径点序列 ───────────────────────────────────────────
    ros_resp.path.reserve(resp.path.size());
    for (const auto& pp : resp.path) {
        beam_dubins::PathPoint ros_pp;
        ros_pp.x         = pp.x;
        ros_pp.y         = pp.y;
        ros_pp.z         = pp.z;
        ros_pp.yaw       = pp.yaw;
        ros_pp.pitch     = pp.pitch;
        ros_pp.curvature = pp.curvature;
        ros_resp.path.push_back(ros_pp);
    }
}

// ═══════════════════════════════════════════════════════════════
// 3. 服务回调
// ═══════════════════════════════════════════════════════════════

/**
 * @brief /beam_dubins/plan_path 服务回调
 *
 * 处理流程：
 *  1. 消息反序列化（ROS → C++）
 *  2. 调用 plan_path()（纯 C++ 算法）
 *  3. 结果序列化（C++ → ROS）
 *
 * 该函数为同步阻塞执行，规划期间节点不响应其他服务请求。
 */
static bool handle_plan_path(
    beam_dubins::PlanPath::Request&  req,
    beam_dubins::PlanPath::Response& res,
    const beam_dubins::BeamConfig&   cfg)
{
    // ── 日志：记录请求摘要 ───────────────────────────────────
    ROS_INFO("[planner_server] 收到规划请求: "
             "start=(%.0f,%.0f,%.0f, ψ=%.1f°) "
             "goal=(%.0f,%.0f,%.0f, ψ=%.1f°) "
             "obstacles=%zu",
             req.start[0], req.start[1], req.start[2],
             req.start[3] * 180.0 / M_PI,
             req.goal[0], req.goal[1], req.goal[2],
             req.goal[3] * 180.0 / M_PI,
             req.obstacles.size());

    // ── 1. 消息 → C++ 结构体 ─────────────────────────────────
    auto plan_req = ros_to_request(req);

    // ── 2. 调用核心算法 ──────────────────────────────────────
    auto plan_resp = beam_dubins::plan_path(plan_req, cfg);

    // ── 3. C++ 结构体 → 消息 ─────────────────────────────────
    response_to_ros(plan_resp, res);

    // ── 日志：记录响应摘要 ───────────────────────────────────
    if (plan_resp.success) {
        ROS_INFO("[planner_server] 规划成功: cost=%.1fm, "
                 "path_points=%zu, nodes=%d, depth=%d, time=%.1fms",
                 plan_resp.cost, plan_resp.path.size(),
                 plan_resp.nodes_explored, plan_resp.depth_reached,
                 plan_resp.planning_time_ms);
    } else {
        ROS_WARN("[planner_server] 规划失败: %s, "
                 "nodes=%d, depth=%d, time=%.1fms",
                 plan_resp.status_message.c_str(),
                 plan_resp.nodes_explored, plan_resp.depth_reached,
                 plan_resp.planning_time_ms);
    }

    return true;  // 始终返回 true（Service 语义：成功响应）
}

// ═══════════════════════════════════════════════════════════════
// 4. 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{   
    // 中文支持
    setlocale(LC_ALL, "");  // 设置系统区域为默认环境
    // ── ROS 初始化 ────────────────────────────────────────────
    ros::init(argc, argv, "planner_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ── 加载参数配置 ──────────────────────────────────────────
    // 参数在 /beam_dubins/ 命名空间下
    // launch 文件中通过 <rosparam ns="beam_dubins"> 加载
    ros::NodeHandle beam_nh("beam_dubins");  // 全局命名空间
    beam_dubins::BeamConfig cfg = load_config(beam_nh);

    ROS_INFO("[planner_server] 参数加载完成:");
    ROS_INFO("  beam_width=%d, beam_width_max=%d, max_depth=%d",
             cfg.beam_width, cfg.beam_width_max, cfg.max_depth);
    ROS_INFO("  max_extend=%.1fm, min_extend=%.1fm",
             cfg.max_extend_length, cfg.min_extend_length);
    ROS_INFO("  goal_tolerance: xy=%.1fm, z=%.1fm",
             cfg.goal_tolerance_xy, cfg.goal_tolerance_z);
    ROS_INFO("  R_min=%.1fm, gamma_max=%.1f°, r_body=%.1fm",
             cfg.R_min, cfg.gamma_max_rad * 180.0 / M_PI, cfg.r_body);
    ROS_INFO("  scorer: w_h=%.2f, w_c=%.2f, w_p=%.2f",
             cfg.w_h, cfg.w_c, cfg.w_p);

    // ── 创建 ServiceServer ────────────────────────────────────
    // 使用 lambda 捕获 cfg（只读配置，线程安全）
    ros::ServiceServer service = nh.advertiseService<beam_dubins::PlanPath::Request,
                                                      beam_dubins::PlanPath::Response>(
        "/beam_dubins/plan_path",
        [&cfg](beam_dubins::PlanPath::Request&  req,
               beam_dubins::PlanPath::Response& res) -> bool {
            return handle_plan_path(req, res, cfg);
        });

    ROS_INFO("[planner_server] 服务就绪: /beam_dubins/plan_path");
    ROS_INFO("[planner_server] 等待规划请求...");

    // ── 事件循环 ──────────────────────────────────────────────
    ros::spin();

    return 0;
}
