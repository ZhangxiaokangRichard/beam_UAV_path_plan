/**
 * @file simulation_loop.cpp
 * @brief 主仿真循环节点（Phase 2c：环境更新 → 规划请求 → UAV 推进 → Topic/TF 发布）
 *
 * 从 Python beam_dubins_2d_dynamic.py → BeamDubinsDynamicSim 迁移。
 *
 * 时序（10Hz Timer，dt=0.1s）：
 *   奇数周期（0.0s, 0.2s, 0.4s, ...）5Hz 规划：
 *     1. 更新目标位置（TargetTrajectory::positionAt）
 *     2. 更新动态障碍物（跟随目标 Y）
 *     3. 构建 PlanPath 请求 → 调用 /beam_dubins/plan_path 服务
 *     4. 存储/发布最优路径
 *     5. UAV 沿路径推进 V×dt
 *   偶数周期（0.1s, 0.3s, 0.5s, ...）仅推进：
 *     1. 更新目标位置
 *     2. UAV 沿路径推进 V×dt
 *
 * 设计要点：
 *  - 同步 Service 调用：规划阻塞 0.1~0.3s，在偶数周期补偿
 *  - 定期重置 beam（BEAM_RESET_INTERVAL）
 *  - 路径耗尽时死推算朝目标方向前进
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
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "beam_dubins/PlanPath.h"
#include "beam_dubins/Obstacle.h"
#include "uav_guide_env/uav_model.h"
#include "uav_guide_env/target_trajectory.h"
#include "uav_guide_env/obstacle_manager.h"

// ═══════════════════════════════════════════════════════════════
// 全局仿真状态（在 main 中初始化，timer 回调中访问）
// ═══════════════════════════════════════════════════════════════

struct SimulationState {
    // ── 时间与步数 ──
    double sim_time = 0.0;
    int    sim_step = 0;
    int    plan_interval   = 2;       // 每 N 个 tick 规划一次
    int    reset_interval  = 30;      // 每 N 步重置 beam
    int    max_steps       = 5000;
    bool   reached         = false;

    // ── UAV ──
    std::array<double, 5> uav_pose;   // [x, y, z, yaw, pitch]
    std::vector<double>    trail_x;    // 航迹 X
    std::vector<double>    trail_y;    // 航迹 Y
    std::vector<double>    trail_z;    // 航迹 Z
    int    path_ptr     = 0;          // 路径推进指针（只进不退）
    size_t path_generation = 0;       // 路径代际计数器（递增 = 新路径）

    // ── 目标 ──
    std::array<double, 5> goal_state; // [x, y, z, yaw, pitch]

    // ── 规划结果 ──
    std::vector<std::array<double, 4>> best_path;  // [[x,y,z,yaw], ...]
    double   best_cost     = INFINITY;
    int      nodes_explored = 0;
    int      depth_reached  = 0;
    double   planning_time_ms = 0.0;
    std::string plan_status;

    // ── 空间边界 ──
    double lx = 10000.0, ly = 5000.0, lz = 1000.0;
    double boundary_margin = 50.0;

    // ── 终端逼近 ──
    double approach_distance = 1000.0;  // 规划到目标后方 L 米（alg_replan.md 方案 A）
};

// ═══════════════════════════════════════════════════════════════
// 辅助函数
// ═══════════════════════════════════════════════════════════════

/// 2D 距离
static double dist2d(const std::array<double, 5>& a, const std::array<double, 5>& b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

/// 发布 Odometry 消息
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

    // 航向 → 四元数（仅 yaw 旋转）
    double cy = std::cos(pose[3] * 0.5);
    double sy = std::sin(pose[3] * 0.5);
    odom.pose.pose.orientation.w = cy;
    odom.pose.pose.orientation.z = sy;

    // 速度（沿航向方向）
    odom.twist.twist.linear.x = speed * std::cos(pose[3]);
    odom.twist.twist.linear.y = speed * std::sin(pose[3]);
    odom.twist.twist.linear.z = 0.0;

    return odom;
}

/// 发布 Path 消息（从 std::array 列表构建）
static nav_msgs::Path makePath(const std::vector<std::array<double, 4>>& waypoints,
                               const std::string& frame_id, const ros::Time& stamp)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp    = stamp;
    path_msg.header.frame_id = frame_id;

    for (const auto& wp : waypoints) {
        geometry_msgs::PoseStamped pose_st;
        pose_st.header.stamp    = stamp;
        pose_st.header.frame_id = frame_id;
        pose_st.pose.position.x = wp[0];
        pose_st.pose.position.y = wp[1];
        pose_st.pose.position.z = (wp.size() > 2) ? wp[2] : 0.0;
        // 航向 → 四元数
        double yaw = (wp.size() > 3) ? wp[3] : 0.0;
        pose_st.pose.orientation.w = std::cos(yaw * 0.5);
        pose_st.pose.orientation.z = std::sin(yaw * 0.5);
        path_msg.poses.push_back(pose_st);
    }
    return path_msg;
}

/// 构建搜索树 Marker（LineList，青色半透明）
static visualization_msgs::Marker makeSearchTreeMarker(
    const std::vector<std::array<double, 4>>& path,
    const std::string& frame_id, const ros::Time& stamp)
{
    visualization_msgs::Marker marker;
    marker.header.stamp    = ros::Time(0);  // 使用最新 TF
    marker.header.frame_id = frame_id;
    marker.ns      = "search_tree";
    marker.id      = 0;
    marker.type    = visualization_msgs::Marker::LINE_LIST;
    marker.action  = visualization_msgs::Marker::ADD;
    marker.scale.x = 2.0;  // 线宽 (m)
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.55;
    marker.color.b = 0.55;
    marker.color.a = 0.25;

    // 每对连续路径点生成一段线段
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        geometry_msgs::Point p;
        p.x = path[i][0];     p.y = path[i][1];     p.z = (path[i].size() > 2) ? path[i][2] : 0.0;
        marker.points.push_back(p);
        p.x = path[i+1][0];   p.y = path[i+1][1];   p.z = (path[i+1].size() > 2) ? path[i+1][2] : 0.0;
        marker.points.push_back(p);
    }
    return marker;
}

/// 广播 TF（map→odom 固定，odom→child 动态）
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
// 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{   
    // 中文支持
    setlocale(LC_ALL, "");  // 设置系统区域为默认环境
    // ── ROS 初始化 ────────────────────────────────────────────
    ros::init(argc, argv, "simulation_loop");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ── 加载参数 ──────────────────────────────────────────────
    // UAV 动力学
    double cruise_speed    = 50.0;
    double turn_radius     = 250.0;
    double pitch_deg       = 5.0;
    double collision_r     = 5.0;
    ros::param::param<double>("/uav/cruise_speed_mps",    cruise_speed,  cruise_speed);
    ros::param::param<double>("/uav/min_turn_radius_m",   turn_radius,   turn_radius);
    ros::param::param<double>("/uav/max_pitch_angle_deg", pitch_deg,     pitch_deg);
    ros::param::param<double>("/uav/collision_radius_m",  collision_r,   collision_r);

    // 目标轨迹
    std::string traj_type = "line";
    std::vector<double> init_pos_vec;
    ros::param::param<std::string>("/target/trajectory_type", traj_type, traj_type);
    ros::param::get("/target/initial_position", init_pos_vec);
    std::array<double, 3> init_pos = {4000.0, 2500.0, 600.0};
    if (init_pos_vec.size() >= 3) {
        init_pos = {init_pos_vec[0], init_pos_vec[1], init_pos_vec[2]};
    }
    // 轨迹参数
    std::array<double, 5> traj_params = {90.0, 20.0, 4000.0, 0.0, 0.0};  // 默认 line
    if (traj_type == "circle") {
        double cx=8000,cy=2500,cz=600,r=500,omega=5;
        ros::param::param<double>("/target/trajectories/circle/center_x", cx, cx);  // 这些路径名需要与 YAML 对齐
        // 简化：直接用默认值，实际由 YAML 结构决定
        traj_params = {2000.0, 2500.0, 600.0, 500.0, 3.0};
    }
    if (traj_type == "line") {
        double dir=90, spd=20, len=4000;
        ros::param::param<double>("/target/trajectories/line/direction_deg", dir, 90.0);
        ros::param::param<double>("/target/trajectories/line/speed_mps",     spd, 20.0);
        ros::param::param<double>("/target/trajectories/line/length_m",      len, 4000.0);
        traj_params = {dir, spd, len, 0.0, 0.0};
    }

    // 仿真参数
    double sim_dt    = 0.1;
    int plan_int     = 2;
    int reset_int    = 30;
    int max_steps    = 8000;
    ros::param::param<double>("/simulation/dt",                  sim_dt,    0.1);
    ros::param::param<int>   ("/simulation/plan_interval",       plan_int,  2);
    ros::param::param<int>   ("/simulation/beam_reset_interval", reset_int, 30);
    ros::param::param<int>   ("/simulation/max_sim_steps",       max_steps, 8000);

    // 场景起点
    std::vector<double> start_vec;
    ros::param::get("/scenario/start", start_vec);
    std::array<double, 5> uav_start = {2000.0, 1000.0, 500.0, M_PI/2.0, 0.0};
    if (start_vec.size() >= 5) {
        uav_start = {start_vec[0], start_vec[1], start_vec[2], start_vec[3], start_vec[4]};
    }

    // 空间边界
    double lx=10000,ly=5000,lz=1000,bm=50;
    ros::param::param<double>("/space/lx", lx, 10000.0);
    ros::param::param<double>("/space/ly", ly, 5000.0);
    ros::param::param<double>("/space/lz", lz, 1000.0);
    ros::param::param<double>("/space/boundary_margin", bm, 50.0);

    // ── 初始化组件 ────────────────────────────────────────────
    uav_guide_env::FixedWingUAV uav(cruise_speed, turn_radius, pitch_deg, collision_r, sim_dt);
    uav_guide_env::TargetTrajectory target(traj_type, init_pos, traj_params);
    uav_guide_env::ObstacleManager obs_mgr(lx, ly, lz, bm);

    // [已废弃] 通道墙壁障碍物 — 由 approach_distance + 虚拟目标替代
    // uav_guide_env::ChannelParams ch_params;
    // ros::param::param<double>("/target/channel/inner_width",    ch_params.inner_width,    100.0);
    // ...
    // { auto state0 = target.positionAt(0.0);
    //   auto walls = target.computeChannelWalls(state0);
    //   auto left_wall = std::make_unique<uav_guide_env::DynamicObstacle>(...);
    //   obs_mgr.addObstacle(std::move(left_wall));
    //   auto right_wall = std::make_unique<uav_guide_env::DynamicObstacle>(...);
    //   obs_mgr.addObstacle(std::move(right_wall));
    // }

    // ── 仿真状态 ──────────────────────────────────────────────
    SimulationState sim;

    // 加载终端逼近距离 L（alg_replan.md 方案 A）
    ros::param::param<double>("/target/approach_distance", sim.approach_distance, 1000.0);
    ROS_INFO("[simulation_loop] 终端逼近距离: L=%.0fm（通道墙壁已禁用）", sim.approach_distance);

    sim.plan_interval  = plan_int;
    sim.reset_interval = reset_int;
    sim.max_steps      = max_steps;
    sim.uav_pose       = uav_start;
    sim.lx = lx; sim.ly = ly; sim.lz = lz;
    sim.boundary_margin = bm;
    sim.trail_x.push_back(uav_start[0]);
    sim.trail_y.push_back(uav_start[1]);
    sim.trail_z.push_back(uav_start[2]);
    sim.goal_state = target.positionAt(0.0);

    // ── ServiceClient ─────────────────────────────────────────
    ros::ServiceClient plan_client = nh.serviceClient<beam_dubins::PlanPath>(
        "/beam_dubins/plan_path");

    ROS_INFO("[simulation_loop] 等待规划服务 /beam_dubins/plan_path ...");
    if (!plan_client.waitForExistence(ros::Duration(10.0))) {
        ROS_ERROR("[simulation_loop] 规划服务未就绪，退出");
        return 1;
    }
    ROS_INFO("[simulation_loop] 规划服务已就绪");

    // ── Publishers ────────────────────────────────────────────
    ros::Publisher pub_uav_state   = nh.advertise<nav_msgs::Odometry>("/uav/state", 10);
    ros::Publisher pub_uav_trail   = nh.advertise<nav_msgs::Path>("/uav/trail", 10);
    ros::Publisher pub_target_state = nh.advertise<nav_msgs::Odometry>("/target/state", 10);
    ros::Publisher pub_bd_path     = nh.advertise<nav_msgs::Path>("/beam_dubins/path", 10);
    ros::Publisher pub_bd_cost     = nh.advertise<std_msgs::Float64>("/beam_dubins/path_cost", 10);
    ros::Publisher pub_bd_tree     = nh.advertise<visualization_msgs::Marker>("/beam_dubins/search_tree", 10);
    ros::Publisher pub_sim_status  = nh.advertise<std_msgs::String>("/sim/status", 10);

    // ── TF 广播器 ─────────────────────────────────────────────
    tf::TransformBroadcaster tf_br;

    // ── 主循环 ────────────────────────────────────────────────
    ROS_INFO("[simulation_loop] 启动仿真循环: dt=%.2fs, plan_interval=%d, max_steps=%d",
             sim_dt, plan_int, max_steps);

    ros::Rate rate(1.0 / sim_dt);  // 10Hz
    bool first_plan_done = false;

    while (ros::ok() && sim.sim_step < max_steps) {
        ros::Time now = ros::Time::now();

        // ═══════════════════════════════════════════════════════
        // 0. 持续发布 map→odom 变换（确保 Rviz TF 树不断裂）
        // ═══════════════════════════════════════════════════════
        {
            geometry_msgs::TransformStamped map_ts;
            map_ts.header.stamp    = now;
            map_ts.header.frame_id = "map";
            map_ts.child_frame_id  = "odom";
            map_ts.transform.rotation.w = 1.0;
            tf_br.sendTransform(map_ts);
        }

        // ═══════════════════════════════════════════════════════
        // 1. 更新目标位置
        // ═══════════════════════════════════════════════════════
        sim.goal_state = target.positionAt(sim.sim_time);
        pub_target_state.publish(
            makeOdometry(sim.goal_state, 0.0, "map", "target_link", now));
        broadcastTF(tf_br, "map", "target_link", sim.goal_state, now);

        // [已废弃] 通道墙壁更新 — 由 approach_distance + 虚拟目标替代
        // {
        //     auto walls = target.computeChannelWalls(sim.goal_state);
        //     ...
        // }

        // ═══════════════════════════════════════════════════════
        // 3. 定期重置 beam（从 UAV 当前位置）
        // ═══════════════════════════════════════════════════════
        if (sim.sim_step > 0 && sim.sim_step % reset_int == 0) {
            ROS_INFO("[simulation_loop] step=%d: 重置 beam 从 UAV=(%.0f,%.0f)",
                     sim.sim_step, sim.uav_pose[0], sim.uav_pose[1]);
            // 清空路径，让下次规划从 UAV 当前位置开始
            sim.path_ptr = 0;
            sim.best_path.clear();
        }

        // ═══════════════════════════════════════════════════════
        // 4. 奇数周期：调用规划服务（5Hz）
        // ═══════════════════════════════════════════════════════
        bool is_plan_tick = (sim.sim_step % plan_int == 0);
        if (is_plan_tick) {
            beam_dubins::PlanPath srv;
            srv.request.header.stamp    = now;
            srv.request.header.frame_id = "map";

            // 起点 = UAV 当前位置
            srv.request.start[0] = sim.uav_pose[0];
            srv.request.start[1] = sim.uav_pose[1];
            srv.request.start[2] = sim.uav_pose[2];
            srv.request.start[3] = sim.uav_pose[3];
            srv.request.start[4] = sim.uav_pose[4];

            // ── 虚拟目标（alg_replan.md 方案 A）────────────────
            // virtual_goal = 目标 - L * (cos(yaw), sin(yaw))
            // 即目标后方 L 米处，保证规划不会越过目标
            double L = sim.approach_distance;
            double gx = sim.goal_state[0];
            double gy = sim.goal_state[1];
            double gz = sim.goal_state[2];
            double gyaw = sim.goal_state[3];
            double virtual_x = gx - L * std::cos(gyaw);
            double virtual_y = gy - L * std::sin(gyaw);
            double virtual_z = gz;

            // 终点 = 虚拟目标（而非实际目标），引导 UAV 从后方逼近
            srv.request.goal[0] = virtual_x;
            srv.request.goal[1] = virtual_y;
            srv.request.goal[2] = virtual_z;
            srv.request.goal[3] = gyaw;
            srv.request.goal[4] = sim.goal_state[4];

            // 空间边界
            srv.request.space_lx        = sim.lx;
            srv.request.space_ly        = sim.ly;
            srv.request.space_lz        = sim.lz;
            srv.request.boundary_margin = sim.boundary_margin;

            // [已废弃] 障碍物为空（通道墙壁已禁用）
            srv.request.obstacles.clear();

            srv.request.time_limit_ms = 0.0;  // 不限制
            srv.request.use_3d        = false;

            // 同步调用
            if (plan_client.call(srv)) {
                sim.best_path.clear();
                sim.nodes_explored = srv.response.nodes_explored;
                sim.depth_reached  = srv.response.depth_reached;
                sim.planning_time_ms = srv.response.planning_time_ms;
                sim.plan_status    = srv.response.status_message;

                if (srv.response.success) {
                    sim.best_cost = srv.response.cost;
                    // 将 ROS PathPoint → std::array<double,4>
                    for (const auto& pp : srv.response.path) {
                        sim.best_path.push_back({pp.x, pp.y, pp.z, pp.yaw});
                    }
                    sim.path_generation++;  // 路径代际递增，触发 UAV 重新定位路径指针
                    // 发布搜索树 Marker（基于路径近似）
                    auto tree_marker = makeSearchTreeMarker(sim.best_path, "map", now);
                    pub_bd_tree.publish(tree_marker);

                    ROS_INFO("[simulation_loop] step=%d: 规划成功 cost=%.1f path=%zu nodes=%d depth=%d time=%.1fms %s",
                             sim.sim_step, sim.best_cost, sim.best_path.size(),
                             sim.nodes_explored, sim.depth_reached, sim.planning_time_ms,
                             sim.plan_status.c_str());
                } else {
                    ROS_WARN("[simulation_loop] step=%d: 规划失败 %s (nodes=%d depth=%d time=%.1fms)",
                             sim.sim_step, sim.plan_status.c_str(),
                             sim.nodes_explored, sim.depth_reached, sim.planning_time_ms);
                }
            } else {
                ROS_ERROR("[simulation_loop] step=%d: Service 调用失败", sim.sim_step);
            }
            first_plan_done = true;
        }

        // ═══════════════════════════════════════════════════════
        // 5. 发布规划路径和代价（如果有）
        // ═══════════════════════════════════════════════════════
        if (!sim.best_path.empty()) {
            auto path_msg = makePath(sim.best_path, "map", now);
            pub_bd_path.publish(path_msg);

            std_msgs::Float64 cost_msg;
            cost_msg.data = sim.best_cost;
            pub_bd_cost.publish(cost_msg);
        }

        // ═══════════════════════════════════════════════════════
        // 6. UAV 沿路径推进 V×dt（每周期都执行）
        // ═══════════════════════════════════════════════════════
        double advance_dist = cruise_speed * sim_dt;

        // 路径代际变化 → 从 UAV 最近点重新定位指针
        static size_t s_last_gen = 0;
        if (sim.path_generation != s_last_gen) {
            s_last_gen = sim.path_generation;
            // 搜索最近路径点作为新起点
            double best_d = INFINITY;
            int best_i = sim.path_ptr;
            for (int i = sim.path_ptr; i < static_cast<int>(sim.best_path.size()); ++i) {
                double dx = sim.best_path[i][0] - sim.uav_pose[0];
                double dy = sim.best_path[i][1] - sim.uav_pose[1];
                double d = std::sqrt(dx*dx + dy*dy);
                if (d < best_d) { best_d = d; best_i = i; }
            }
            sim.path_ptr = std::max(0, best_i);
        }

        uav.advanceAlongPath(sim.uav_pose, sim.best_path,
                             sim.path_ptr, sim.path_generation,
                             sim.goal_state, advance_dist);

        // ═══════════════════════════════════════════════════════
        // 7. 发布 UAV 状态和航迹
        // ═══════════════════════════════════════════════════════
        pub_uav_state.publish(
            makeOdometry(sim.uav_pose, cruise_speed, "map", "uav_base_link", now));
        broadcastTF(tf_br, "map", "uav_base_link", sim.uav_pose, now);

        // 追加航迹
        sim.trail_x.push_back(sim.uav_pose[0]);
        sim.trail_y.push_back(sim.uav_pose[1]);
        sim.trail_z.push_back(sim.uav_pose[2]);

        // 构建 Path 消息（航迹）
        {
            nav_msgs::Path trail_msg;
            trail_msg.header.stamp    = now;
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

        // ═══════════════════════════════════════════════════════
        // 8. 到达检测（不终止仿真，持续跟踪目标）
        // ═══════════════════════════════════════════════════════
        double tol_xy = 50.0;
        ros::param::param<double>("/beam_dubins/goal_tolerance_xy", tol_xy, 50.0);
        if (!sim.reached && dist2d(sim.uav_pose, sim.goal_state) < tol_xy) {
            sim.reached = true;  // 仅标记"曾经到达"，不影响循环
            ROS_INFO("[simulation_loop] ★★★ 到达目标邻域！ step=%d time=%.1fs（继续跟踪）★★★",
                     sim.sim_step, sim.sim_time);
        }
        // 若已到达过但再次远离，重置标记以便再次记录到达
        if (sim.reached && dist2d(sim.uav_pose, sim.goal_state) > tol_xy * 3.0) {
            sim.reached = false;
            ROS_INFO("[simulation_loop] step=%d: UAV 已远离目标，重新跟踪", sim.sim_step);
        }

        // ═══════════════════════════════════════════════════════
        // 9. 发布仿真状态
        // ═══════════════════════════════════════════════════════
        {
            std_msgs::String status_msg;
            std::ostringstream oss;
            oss << "Step " << sim.sim_step << "/" << max_steps
                << " | t=" << std::fixed << std::setprecision(1) << sim.sim_time << "s"
                << " | cost=" << (sim.best_cost < INFINITY ? std::to_string(static_cast<int>(sim.best_cost)) + "m" : "N/A")
                << " | " << (sim.reached ? "REACHED (tracking)" : "tracking");
            status_msg.data = oss.str();
            pub_sim_status.publish(status_msg);
        }

        // ── 步进 ─────────────────────────────────────────────
        sim.sim_step++;
        sim.sim_time += sim_dt;

        ros::spinOnce();
        rate.sleep();
    }

    if (sim.reached) {
        ROS_INFO("[simulation_loop] 仿真完成：已达最大步数 %d（期间成功抵近目标）", max_steps);
    } else {
        ROS_WARN("[simulation_loop] 仿真终止：达到最大步数 %d（未抵近目标）", max_steps);
    }

    return 0;
}
