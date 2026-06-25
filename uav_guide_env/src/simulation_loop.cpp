/**
 * @file simulation_loop.cpp
 * @brief 主仿真循环节点（Phase 3 v0.1：目标节点解耦 + 单模式 + Tracking）
 *
 * 架构变更：
 *  - 目标轨迹由独立 target_publisher 节点发布，本节点通过订阅获取
 *  - 删除双规划竞标，改为 YAML 配置的单模式（尾追 / 迎头）
 *  - 保留 Tracking 模式状态机
 *
 * 回调/业务函数全部外提到 main() 之前，main() 中仅保留顶层调度。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "beam_dubins/PlanPath.h"
#include "uav_guide_env/uav_model.h"

// ═══════════════════════════════════════════════════════════════
// 仿真状态结构体（Phase 3 v0.1 精简版）
// ═══════════════════════════════════════════════════════════════

struct SimulationState {
    // ── 时间与步数 ──
    double sim_time = 0.0;
    int    sim_step = 0;
    int    plan_interval  = 2;
    int    reset_interval = 30;
    int    max_steps      = 8000;
    bool   reached        = false;

    // ── UAV ──
    std::array<double, 5> uav_pose;
    std::vector<double> trail_x, trail_y, trail_z;
    int    path_ptr = 0;
    size_t path_generation = 0;

    // ── 目标（从 /target/* 话题订阅，每周期从缓存同步）──
    std::array<double, 5> goal_state;
    std::array<double, 5> virtual_goal;
    std::string           intercept_mode = "tail";

    // ── 规划结果 ──
    std::vector<std::array<double, 4>> best_path;
    double best_cost = INFINITY;
    int    nodes_explored = 0;
    int    depth_reached  = 0;
    double planning_time_ms = 0.0;
    std::string plan_status;

    // ── 空间边界 ──
    double lx = 10000.0, ly = 5000.0, lz = 1000.0;
    double boundary_margin = 50.0;

    // ── Tracking 模式 ─────────────────────────────────────
    bool   tracking_mode           = false;
    double tracking_entry_dist     = 2000.0;
    double tracking_window         = 100.0;
    double tracking_distance_accum = 0.0;
    double last_tracking_dist      = INFINITY;
    int    consecutive_diverging   = 0;
    int    divergence_threshold    = 3;
    double virtual_goal_tolerance  = 200.0;

    // ── 拦截偏移距离（从 YAML 加载）─────────────────────
    double approach_L_tail   = 1000.0;
    double approach_L_headon = 1800.0;

    // ── 扇形捕获区（§3.8，解决近场重规划振荡）──────────
    double sector_half_angle_rad   = 20.0 * M_PI / 180.0;
};

// ═══════════════════════════════════════════════════════════════
// 订阅回调缓存（文件级静态变量，主循环同步读取）
// ═══════════════════════════════════════════════════════════════

static std::array<double, 5> g_target_state = {4000.0, 2500.0, 600.0, 0.0, 0.0};
static std::array<double, 5> g_virtual_goal = {4000.0, 2500.0, 600.0, 0.0, 0.0};
static std::string           g_intercept_mode = "tail";
static bool g_has_target = false;
static bool g_has_vgoal  = false;

static void targetStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_target_state[0] = msg->pose.pose.position.x;
    g_target_state[1] = msg->pose.pose.position.y;
    g_target_state[2] = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    g_target_state[3] = 2.0 * std::atan2(qz, qw);
    g_target_state[4] = 0.0;
    g_has_target = true;
}

static void virtualGoalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_virtual_goal[0] = msg->pose.pose.position.x;
    g_virtual_goal[1] = msg->pose.pose.position.y;
    g_virtual_goal[2] = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    g_virtual_goal[3] = 2.0 * std::atan2(qz, qw);
    g_virtual_goal[4] = 0.0;
    g_has_vgoal = true;
}

static void interceptModeCallback(const std_msgs::String::ConstPtr& msg)
{
    g_intercept_mode = msg->data;
}

// ═══════════════════════════════════════════════════════════════
// 辅助函数
// ═══════════════════════════════════════════════════════════════

static double dist2d(const std::array<double, 5>& a, const std::array<double, 5>& b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

static nav_msgs::Odometry makeOdometry(const std::array<double, 5>& pose,
                                       double speed, const std::string& frame_id,
                                       const std::string& child_frame,
                                       const ros::Time& stamp)
{
    nav_msgs::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id  = child_frame;
    odom.pose.pose.position.x = pose[0];
    odom.pose.pose.position.y = pose[1];
    odom.pose.pose.position.z = pose[2];
    double cy = std::cos(pose[3] * 0.5);
    double sy = std::sin(pose[3] * 0.5);
    odom.pose.pose.orientation.w = cy;
    odom.pose.pose.orientation.z = sy;
    odom.twist.twist.linear.x = speed * std::cos(pose[3]);
    odom.twist.twist.linear.y = speed * std::sin(pose[3]);
    return odom;
}

static nav_msgs::Path makePath(const std::vector<std::array<double, 4>>& waypoints,
                               const std::string& frame_id, const ros::Time& stamp)
{
    nav_msgs::Path msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = wp[0];
        ps.pose.position.y = wp[1];
        ps.pose.position.z = (wp.size() > 2) ? wp[2] : 0.0;
        double yaw = (wp.size() > 3) ? wp[3] : 0.0;
        ps.pose.orientation.w = std::cos(yaw * 0.5);
        ps.pose.orientation.z = std::sin(yaw * 0.5);
        msg.poses.push_back(ps);
    }
    return msg;
}

static void broadcastTF(tf::TransformBroadcaster& br,
                        const std::string& parent, const std::string& child,
                        const std::array<double, 5>& pose, const ros::Time& stamp)
{
    geometry_msgs::TransformStamped ts;
    ts.header.stamp    = stamp;
    ts.header.frame_id = parent;
    ts.child_frame_id  = child;
    ts.transform.translation.x = pose[0];
    ts.transform.translation.y = pose[1];
    ts.transform.translation.z = pose[2];
    double cy = std::cos(pose[3] * 0.5);
    double sy = std::sin(pose[3] * 0.5);
    ts.transform.rotation.w = cy;
    ts.transform.rotation.z = sy;
    br.sendTransform(ts);
}

// ═══════════════════════════════════════════════════════════════
// 业务函数（规划 / 状态机）
// ═══════════════════════════════════════════════════════════════

/// 检查 UAV 是否在目标扇形捕获区内
/// @return true=UAV 在扇形内，应直飞实际目标
static bool isUavInSector(const std::array<double,5>& uav,
                          const std::array<double,5>& target,
                          const std::string& mode,
                          double L,
                          double half_angle_rad)
{
    double dx = uav[0] - target[0];
    double dy = uav[1] - target[1];
    double d = std::sqrt(dx*dx + dy*dy);
    if (d > L) return false;

    // 扇形朝向：统一使用目标航向（tail=后方扇形，head-on=前方扇形，均沿目标航向）
    double psi_sector = target[3];
    double angle_to_uav = std::atan2(dy, dx);
    double dangle = angle_to_uav - psi_sector;
    while (dangle >  M_PI) dangle -= 2.0*M_PI;
    while (dangle < -M_PI) dangle += 2.0*M_PI;
    return std::abs(dangle) <= half_angle_rad;
}

/// 发起规划服务调用（滑动虚拟目标；tracking 模式下直接规划到实际目标）
/// @param force_goal 若非空则使用此目标替代滑动虚拟目标（tracking 模式用）
static bool requestPlan(SimulationState& sim,
                        ros::ServiceClient& client,
                        const ros::Time& now,
                        const std::array<double,5>* force_goal = nullptr)
{
    std::array<double,5> plan_goal;

    if (force_goal) {
        // tracking 模式：直接使用传入的实际目标
        plan_goal = *force_goal;
    } else {
        // ── 滑动虚拟目标（扇形内外平滑过渡）───────────────
        double approach_L = (sim.intercept_mode == "head-on") ? sim.approach_L_headon : sim.approach_L_tail;
        double d_to_target = dist2d(sim.uav_pose, sim.goal_state);
        double ratio = std::min(1.0, d_to_target / approach_L);
        double effective_L = approach_L * ratio;

        double psi = sim.goal_state[3];
        double sign = (sim.intercept_mode == "head-on") ? 1.0 : -1.0;
        double vx = sim.goal_state[0] + sign * effective_L * std::cos(psi);
        double vy = sim.goal_state[1] + sign * effective_L * std::sin(psi);
        double vz = sim.goal_state[2];

        double vyaw = psi;
        if (sim.intercept_mode == "head-on") {
            vyaw += M_PI;
            while (vyaw >  M_PI) vyaw -= 2.0 * M_PI;
            while (vyaw < -M_PI) vyaw += 2.0 * M_PI;
        }
        plan_goal = {vx, vy, vz, vyaw, sim.goal_state[4]};
    }

    beam_dubins::PlanPath srv;
    srv.request.header.stamp    = now;
    srv.request.header.frame_id = "map";
    for (int i = 0; i < 5; ++i) {
        srv.request.start[i] = sim.uav_pose[i];
        srv.request.goal[i]  = plan_goal[i];
    }
    srv.request.space_lx        = sim.lx;
    srv.request.space_ly        = sim.ly;
    srv.request.space_lz        = sim.lz;
    srv.request.boundary_margin = sim.boundary_margin;
    srv.request.obstacles.clear();
    srv.request.time_limit_ms = 0.0;
    srv.request.use_3d        = false;

    if (!client.call(srv)) {
        ROS_ERROR("[simulation_loop] step=%d: Service 调用失败", sim.sim_step);
        return false;
    }

    sim.best_path.clear();
    sim.nodes_explored   = srv.response.nodes_explored;
    sim.depth_reached    = srv.response.depth_reached;
    sim.planning_time_ms = srv.response.planning_time_ms;
    sim.plan_status      = srv.response.status_message;

    if (srv.response.success) {
        sim.best_cost = srv.response.cost;
        for (const auto& pp : srv.response.path) {
            sim.best_path.push_back({pp.x, pp.y, pp.z, pp.yaw});
        }
        sim.path_generation++;
        sim.path_ptr = 0;
        ROS_INFO("[simulation_loop] step=%d: 规划成功 cost=%.1f path=%zu nodes=%d time=%.1fms",
                 sim.sim_step, sim.best_cost, sim.best_path.size(),
                 sim.nodes_explored, sim.planning_time_ms);
        return true;
    } else {
        ROS_WARN("[simulation_loop] step=%d: 规划失败 %s", sim.sim_step, sim.plan_status.c_str());
        return false;
    }
}

/// 检查是否满足 Tracking 进入条件（§3.8：扇形内近距离直接进入）
static void evaluateTrackingEntry(SimulationState& sim)
{
    if (sim.tracking_mode) return;

    double d_to_target = dist2d(sim.uav_pose, sim.goal_state);
    if (d_to_target >= sim.tracking_entry_dist) return;

    // 扇形半径 = 拦截偏移距离（tail/head-on 配置值）
    double approach_L = (sim.intercept_mode == "head-on") ? sim.approach_L_headon : sim.approach_L_tail;

    // 扇形内 + 足够近 → 直接进入 tracking，无需等待路径耗尽
    if (d_to_target < approach_L) {
        sim.tracking_mode = true;
        sim.tracking_distance_accum = 0.0;
        sim.consecutive_diverging   = 0;
        sim.last_tracking_dist      = d_to_target;
        ROS_INFO("[simulation_loop] step=%d: >>> 进入 TRACKING 模式 (扇形内 dist=%.0fm mode=%s)",
                 sim.sim_step, d_to_target, sim.intercept_mode.c_str());
        return;
    }

    // 扇形外 → 需等待路径耗尽后才进入 tracking
    if (sim.best_path.empty()) return;
    if (sim.path_ptr < static_cast<int>(sim.best_path.size()) - 1) return;

    sim.tracking_mode = true;
    sim.tracking_distance_accum = 0.0;
    sim.consecutive_diverging   = 0;
    sim.last_tracking_dist      = d_to_target;
    ROS_INFO("[simulation_loop] step=%d: >>> 进入 TRACKING 模式 (dist=%.0fm mode=%s)",
             sim.sim_step, d_to_target, sim.intercept_mode.c_str());
}

/// Tracking 模式下评估距离趋势，判断是否退出
static void evaluateTrackingTrend(SimulationState& sim, double advance_dist)
{
    if (!sim.tracking_mode) return;

    sim.tracking_distance_accum += advance_dist;
    if (sim.tracking_distance_accum < sim.tracking_window) return;

    double current_dist = dist2d(sim.uav_pose, sim.goal_state);
    double delta = current_dist - sim.last_tracking_dist;
    if (delta > 0) {
        sim.consecutive_diverging++;
        ROS_WARN("[simulation_loop] step=%d: tracking 距离增大 +%.0fm (%d/%d)",
                 sim.sim_step, delta, sim.consecutive_diverging, sim.divergence_threshold);
    } else {
        sim.consecutive_diverging = 0;
    }
    sim.last_tracking_dist = current_dist;
    sim.tracking_distance_accum = 0.0;

    if (sim.consecutive_diverging >= sim.divergence_threshold) {
        sim.tracking_mode = false;
        sim.best_path.clear();
        sim.path_ptr = 0;
        sim.path_generation++;
        sim.consecutive_diverging = 0;
        ROS_WARN("[simulation_loop] step=%d: <<< 退出 TRACKING 模式，恢复规划", sim.sim_step);
    }
}

// ═══════════════════════════════════════════════════════════════
// 主函数（精简调度）
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "simulation_loop");
    ros::NodeHandle nh;

    // ── 加载参数 ──────────────────────────────────────────
    double cruise_speed = 50.0, turn_radius = 500.0, pitch_deg = 5.0, collision_r = 5.0;
    ros::param::param<double>("/uav/cruise_speed_mps",    cruise_speed,  cruise_speed);
    ros::param::param<double>("/uav/min_turn_radius_m",   turn_radius,   turn_radius);
    ros::param::param<double>("/uav/max_pitch_angle_deg", pitch_deg,     pitch_deg);
    ros::param::param<double>("/uav/collision_radius_m",  collision_r,   collision_r);

    double sim_dt = 0.1;
    int plan_int = 2, reset_int = 30, max_steps = 8000;
    ros::param::param<double>("/simulation/dt",                  sim_dt,    0.1);
    ros::param::param<int>   ("/simulation/plan_interval",       plan_int,  2);
    ros::param::param<int>   ("/simulation/beam_reset_interval", reset_int, 30);
    ros::param::param<int>   ("/simulation/max_sim_steps",       max_steps, 8000);

    std::vector<double> start_vec;
    ros::param::get("/scenario/start", start_vec);
    std::array<double, 5> uav_start = {1000.0, 1000.0, 500.0, 0.0, 0.0};
    if (start_vec.size() >= 5)
        uav_start = {start_vec[0], start_vec[1], start_vec[2], start_vec[3], start_vec[4]};

    double lx=10000, ly=5000, lz=1000, bm=50;
    ros::param::param<double>("/space/lx", lx, 10000.0);
    ros::param::param<double>("/space/ly", ly, 5000.0);
    ros::param::param<double>("/space/lz", lz, 1000.0);
    ros::param::param<double>("/space/boundary_margin", bm, 50.0);

    bool heading_continuity_enabled = true;
    int  path_lookahead_n = 5;
    ros::param::param<bool>("/uav/heading_continuity_enabled", heading_continuity_enabled, true);
    ros::param::param<int> ("/uav/path_lookahead_n",          path_lookahead_n,          5);

    // ── 初始化组件 ────────────────────────────────────────
    uav_guide_env::FixedWingUAV uav(cruise_speed, turn_radius, pitch_deg, collision_r, sim_dt);
    SimulationState sim;
    sim.plan_interval  = plan_int;
    sim.reset_interval = reset_int;
    sim.max_steps      = max_steps;
    sim.uav_pose       = uav_start;
    sim.lx = lx; sim.ly = ly; sim.lz = lz;
    sim.boundary_margin = bm;
    sim.trail_x.push_back(uav_start[0]);
    sim.trail_y.push_back(uav_start[1]);
    sim.trail_z.push_back(uav_start[2]);

    // Tracking 参数
    ros::param::param<double>("/target/tracking_entry_dist",    sim.tracking_entry_dist,    2000.0);
    ros::param::param<double>("/target/tracking_window",        sim.tracking_window,        100.0);
    ros::param::param<int>   ("/target/divergence_threshold",   sim.divergence_threshold,   3);
    ros::param::param<double>("/target/virtual_goal_tolerance", sim.virtual_goal_tolerance, 200.0);

    // 拦截偏移距离
    ros::param::param<double>("/target/approach_distance_tail",   sim.approach_L_tail,   1000.0);
    ros::param::param<double>("/target/approach_distance_headon", sim.approach_L_headon, 1800.0);

    // 扇形捕获区参数（§3.8）
    double sector_half_deg = 20.0;
    ros::param::param<double>("/target/sector_half_angle_deg", sector_half_deg, 20.0);
    sim.sector_half_angle_rad = sector_half_deg * M_PI / 180.0;
    ROS_INFO("[simulation_loop] 扇形半角=%.1f° L_tail=%.0fm L_headon=%.0fm tracking_entry=%.0fm",
             sector_half_deg, sim.approach_L_tail, sim.approach_L_headon, sim.tracking_entry_dist);

    // ── ServiceClient ──────────────────────────────────────
    ros::ServiceClient plan_client = nh.serviceClient<beam_dubins::PlanPath>("/beam_dubins/plan_path");
    ROS_INFO("[simulation_loop] 等待规划服务...");
    if (!plan_client.waitForExistence(ros::Duration(10.0))) {
        ROS_ERROR("[simulation_loop] 规划服务未就绪");
        return 1;
    }

    // ── Subscribers（绑定回调，缓存最新目标数据）───────────
    ros::Subscriber sub_target = nh.subscribe<nav_msgs::Odometry>("/target/state", 10, targetStateCallback);
    ros::Subscriber sub_vgoal  = nh.subscribe<nav_msgs::Odometry>("/target/virtual_goal", 10, virtualGoalCallback);
    ros::Subscriber sub_mode   = nh.subscribe<std_msgs::String>("/target/intercept_mode", 10, interceptModeCallback);

    // ── Publishers ─────────────────────────────────────────
    ros::Publisher pub_uav_state  = nh.advertise<nav_msgs::Odometry>("/uav/state", 10);
    ros::Publisher pub_uav_trail  = nh.advertise<nav_msgs::Path>("/uav/trail", 10);
    ros::Publisher pub_bd_path    = nh.advertise<nav_msgs::Path>("/beam_dubins/path", 10);
    ros::Publisher pub_bd_cost    = nh.advertise<std_msgs::Float64>("/beam_dubins/path_cost", 10);
    ros::Publisher pub_sim_status = nh.advertise<std_msgs::String>("/sim/status", 10);

    tf::TransformBroadcaster tf_br;

    // ── 等待首条目标消息 ──────────────────────────────────
    ROS_INFO("[simulation_loop] 等待 /target/state ...");
    ros::topic::waitForMessage<nav_msgs::Odometry>("/target/state", nh, ros::Duration(10.0));
    ROS_INFO("[simulation_loop] 仿真循环启动");

    // ═══════════════════════════════════════════════════════════
    // 主循环（精简调度，每步骤调用一个函数）
    // ═══════════════════════════════════════════════════════════
    ros::Rate rate(1.0 / sim_dt);
    static size_t s_last_gen = 0;

    while (ros::ok() && sim.sim_step < max_steps) {
        ros::Time now = ros::Time::now();

        // 0. map→odom TF
        {
            geometry_msgs::TransformStamped map_ts;
            map_ts.header.stamp    = now;
            map_ts.header.frame_id = "map";
            map_ts.child_frame_id  = "odom";
            map_ts.transform.rotation.w = 1.0;
            tf_br.sendTransform(map_ts);
        }

        // 1. 从回调缓存同步目标数据
        sim.goal_state     = g_target_state;
        sim.virtual_goal   = g_virtual_goal;
        sim.intercept_mode = g_intercept_mode;

        // 2. 发布目标 TF
        broadcastTF(tf_br, "map", "target_link", sim.goal_state, now);

        // 3. 定期重置 beam
        if (sim.sim_step > 0 && sim.sim_step % reset_int == 0) {
            ROS_INFO("[simulation_loop] step=%d: 重置 beam", sim.sim_step);
            sim.path_ptr = 0;
            sim.best_path.clear();
        }

        // 4. 规划周期（tracking 模式也规划，但目标为实际目标）
        if (sim.sim_step % plan_int == 0) {
            if (sim.tracking_mode) {
                // tracking 模式：直接以实际目标为规划终点（迎头=反向朝向，尾追=同向）
                std::array<double,5> actual_goal = sim.goal_state;
                if (sim.intercept_mode == "head-on") {
                    actual_goal[3] += M_PI;
                    while (actual_goal[3] >  M_PI) actual_goal[3] -= 2.0 * M_PI;
                    while (actual_goal[3] < -M_PI) actual_goal[3] += 2.0 * M_PI;
                }
                requestPlan(sim, plan_client, now, &actual_goal);
            } else {
                requestPlan(sim, plan_client, now);
            }
        }

        // 5. 发布规划路径
        if (!sim.best_path.empty()) {
            pub_bd_path.publish(makePath(sim.best_path, "map", now));
            std_msgs::Float64 cost_msg;
            cost_msg.data = sim.best_cost;
            pub_bd_cost.publish(cost_msg);
        }

        // 6. UAV 推进（tracking 模式也沿规划路径飞行）
        double advance_dist = cruise_speed * sim_dt;
        if (sim.path_generation != s_last_gen) {
            s_last_gen = sim.path_generation;
            double best_d = INFINITY;
            int best_i = 0;
            for (int i = 0; i < static_cast<int>(sim.best_path.size()); ++i) {
                double dx = sim.best_path[i][0] - sim.uav_pose[0];
                double dy = sim.best_path[i][1] - sim.uav_pose[1];
                double d = std::sqrt(dx*dx + dy*dy);
                if (d < best_d) { best_d = d; best_i = i; }
            }
            sim.path_ptr = best_i;
        }
        uav.interpolateAlongPathWithYawLimit(sim.uav_pose, sim.best_path,
                                              sim.path_ptr, sim.path_generation,
                                              sim.goal_state, advance_dist,
                                              path_lookahead_n);

        // 7. 发布 UAV 状态 + 航向连续性保护 + 航迹
        pub_uav_state.publish(makeOdometry(sim.uav_pose, cruise_speed, "map", "uav_base_link", now));
        broadcastTF(tf_br, "map", "uav_base_link", sim.uav_pose, now);

        if (heading_continuity_enabled) {
            static double s_prev_yaw = sim.uav_pose[3];
            static bool   s_yaw_init = false;
            if (!s_yaw_init) { s_prev_yaw = sim.uav_pose[3]; s_yaw_init = true; }
            double max_dpsi = (cruise_speed / turn_radius) * sim_dt;
            double dpsi = sim.uav_pose[3] - s_prev_yaw;
            while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
            while (dpsi < -M_PI) dpsi += 2.0 * M_PI;
            if (std::abs(dpsi) > max_dpsi * 1.5) {
                sim.uav_pose[3] = s_prev_yaw + std::copysign(max_dpsi, dpsi);
            }
            s_prev_yaw = sim.uav_pose[3];
        }

        sim.trail_x.push_back(sim.uav_pose[0]);
        sim.trail_y.push_back(sim.uav_pose[1]);
        sim.trail_z.push_back(sim.uav_pose[2]);
        {
            nav_msgs::Path trail_msg;
            trail_msg.header.stamp = now;
            trail_msg.header.frame_id = "map";
            for (size_t i = 0; i < sim.trail_x.size(); ++i) {
                geometry_msgs::PoseStamped ps;
                ps.header = trail_msg.header;
                ps.pose.position.x = sim.trail_x[i];
                ps.pose.position.y = sim.trail_y[i];
                ps.pose.position.z = sim.trail_z[i];
                ps.pose.orientation.w = 1.0;
                trail_msg.poses.push_back(ps);
            }
            pub_uav_trail.publish(trail_msg);
        }

        // 8. Tracking 进入/退出 + 到达检测
        evaluateTrackingEntry(sim);
        evaluateTrackingTrend(sim, advance_dist);

        double tol_xy = 50.0;
        if (!sim.reached && dist2d(sim.uav_pose, sim.goal_state) < tol_xy) {
            sim.reached = true;
            ROS_INFO("[simulation_loop] ★★★ 到达目标邻域 step=%d ★★★", sim.sim_step);
        }
        if (sim.reached && dist2d(sim.uav_pose, sim.goal_state) > tol_xy * 3.0)
            sim.reached = false;

        // 9. 发布仿真状态
        {
            std_msgs::String status_msg;
            std::ostringstream oss;
            oss << "Step " << sim.sim_step << "/" << max_steps
                << " | t=" << std::fixed << std::setprecision(1) << sim.sim_time << "s"
                << " | cost=" << (sim.best_cost < INFINITY ? std::to_string(static_cast<int>(sim.best_cost)) + "m" : "N/A")
                << " | " << (sim.tracking_mode ? "TRACKING" : (sim.reached ? "REACHED" : "planning"))
                << " | mode=" << sim.intercept_mode;
            status_msg.data = oss.str();
            pub_sim_status.publish(status_msg);
        }

        sim.sim_step++;
        sim.sim_time += sim_dt;
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[simulation_loop] 仿真完成: %s", sim.reached ? "抵近目标" : "达到最大步数");
    return 0;
}
