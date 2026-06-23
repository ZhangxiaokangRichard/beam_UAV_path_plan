/**
 * @file simulation_loop.cpp
 * @brief 主仿真循环节点（Phase 3 Full：Tracking 模式 + 双虚拟目标竞标）
 *
 * 从 Python beam_dubins_2d_dynamic.py → BeamDubinsDynamicSim 迁移，
 * 依据 alg_phase3_full.md 扩展。
 *
 * 状态机：NORMAL ⇄ TRACKING
 *   NORMAL：  5Hz 双规划竞标（尾追 rear / 迎头 front），选代价较低路径
 *   TRACKING：路径耗尽后直线追踪实际目标，监视 d(dist)/dt 趋势
 *
 * 时序（10Hz Timer，dt=0.1s）：
 *   每周期：
 *     1. 更新目标位置（TargetTrajectory::positionAt）
 *     2. 发布 /target/state, TF
 *   奇数周期（5Hz）：
 *     3. NORMAL 模式：交替规划 rear/front → 竞标 → best_path
 *        TRACKING 模式：跳过规划
 *   每周期：
 *     4. NORMAL：UAV 沿 best_path 推进 V×dt
 *        TRACKING：直线死推算朝实际目标
 *     5. 检查 Tracking 进入/退出条件
 *     6. 发布状态 /sim/status
 *
 * 设计要点：
 *  - 交替竞标：奇数周期=尾追，偶数周期=迎头，每两个周期完成一次完整竞标
 *  - 追踪监视：每 tracking_window 距离评估距离趋势，连续远离 N 次退出
 *  - 向后兼容：dual_plan_enabled=false → 纯尾追；tracking_entry_dist=0 → 禁用 Tracking
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

    // ── Tracking 模式（alg_phase3_full.md 问题一）────────
    bool   tracking_mode           = false;   // 是否处于直线追踪模式
    double tracking_entry_dist     = 2000.0;   // 进入 tracking 的最大 UAV-目标距离 (m)
    double tracking_window         = 100.0;    // 趋势评估窗口距离 (m)
    double tracking_distance_accum = 0.0;      // 当前窗口累积距离 (m)
    double last_tracking_dist      = INFINITY; // 上一次评估时的 UAV-目标距离
    int    consecutive_diverging   = 0;        // 连续远离计数器
    int    divergence_threshold    = 3;        // 连续远离多少次后退出 tracking
    double virtual_goal_tolerance  = 200.0;    // 到达 virtual_goal 的容忍距离 (m)

    // ── 双模式竞标（alg_phase3_full.md 问题二）────────
    bool   dual_plan_enabled  = true;     // 是否启用尾追+迎头双规划竞标
    bool   plan_phase         = false;    // false=尾追周期, true=迎头周期（交替）
    bool   first_plan_pending = true;     // 首次规划需跑满两相才能竞标
    double cached_cost_rear   = INFINITY; // 缓存的尾追代价
    double cached_cost_front  = INFINITY; // 缓存的迎头代价
    std::vector<std::array<double, 4>> cached_path_rear;   // 缓存的尾追路径
    std::vector<std::array<double, 4>> cached_path_front;  // 缓存的迎头路径
    std::string active_mode   = "tail";   // 当前活跃模式："tail" / "head-on"
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

/// 2D 距离（对 array<double,5> 和 array<double,5> 的简化版，用于虚拟目标距离计算）
static double dist2d_vg(const std::array<double, 5>& a, const std::array<double, 5>& b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

// ── 双虚拟目标结构体 ──────────────────────────────────────
struct VirtualGoals {
    std::array<double, 5> rear;   // 尾追虚拟目标 [x,y,z,yaw,pitch]
    std::array<double, 5> front;  // 迎头虚拟目标 [x,y,z,yaw,pitch]
};

/// 计算双虚拟目标（尾追 + 迎头截击）
static VirtualGoals computeVirtualGoals(
    const std::array<double, 5>& target,
    double approach_distance)
{
    VirtualGoals vg;
    double tx = target[0], ty = target[1], tz = target[2];
    double psi = target[3], theta = target[4];
    double L = approach_distance;
    double c = std::cos(psi), s = std::sin(psi);

    // 尾追（后方）：目标后方 L 米，同向
    vg.rear = {tx - L * c, ty - L * s, tz, psi, theta};

    // 迎头（前方）：目标前方 L 米，反向
    double front_yaw = psi + M_PI;
    while (front_yaw >  M_PI) front_yaw -= 2.0 * M_PI;
    while (front_yaw < -M_PI) front_yaw += 2.0 * M_PI;
    vg.front = {tx + L * c, ty + L * s, tz, front_yaw, theta};

    return vg;
}

/// 调用规划服务（提取公共逻辑）
static bool callPlanner(
    ros::ServiceClient& client,
    const std::array<double, 5>& start,
    const std::array<double, 5>& goal,
    const SimulationState& sim,
    const ros::Time& now,
    double& out_cost,
    std::vector<std::array<double, 4>>& out_path,
    int& out_nodes, int& out_depth, double& out_time, std::string& out_status)
{
    beam_dubins::PlanPath srv;
    srv.request.header.stamp    = now;
    srv.request.header.frame_id = "map";
    srv.request.start[0] = start[0];
    srv.request.start[1] = start[1];
    srv.request.start[2] = start[2];
    srv.request.start[3] = start[3];
    srv.request.start[4] = start[4];
    srv.request.goal[0] = goal[0];
    srv.request.goal[1] = goal[1];
    srv.request.goal[2] = goal[2];
    srv.request.goal[3] = goal[3];
    srv.request.goal[4] = goal[4];
    srv.request.space_lx        = sim.lx;
    srv.request.space_ly        = sim.ly;
    srv.request.space_lz        = sim.lz;
    srv.request.boundary_margin = sim.boundary_margin;
    srv.request.obstacles.clear();
    srv.request.time_limit_ms = 0.0;
    srv.request.use_3d        = false;

    if (!client.call(srv)) return false;

    out_cost   = srv.response.success ? srv.response.cost : INFINITY;
    out_nodes  = srv.response.nodes_explored;
    out_depth  = srv.response.depth_reached;
    out_time   = srv.response.planning_time_ms;
    out_status = srv.response.status_message;

    out_path.clear();
    if (srv.response.success) {
        for (const auto& pp : srv.response.path) {
            out_path.push_back({pp.x, pp.y, pp.z, pp.yaw});
        }
    }
    return true;
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
        double cx=8000,cy=2500,cz=600,r=200,omega=0.5;
        ros::param::param<double>("/target/trajectories/circle/center_x", cx, cx);  // 这些路径名需要与 YAML 对齐
        // 简化：直接用默认值，实际由 YAML 结构决定
        traj_params = {2000.0, 2500.0, 600.0, 200.0, 1.5};
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

    // 航向连续性保护参数
    bool heading_continuity_enabled = true;
    ros::param::param<bool>("/uav/heading_continuity_enabled", heading_continuity_enabled, true);
    ROS_INFO("[simulation_loop] 航向连续性保护: %s", heading_continuity_enabled ? "启用" : "禁用");

    // 路径跟踪 lookahead 参数（uav_dynamicR2.md）
    int path_lookahead_n = 5;
    ros::param::param<int>("/uav/path_lookahead_n", path_lookahead_n, 5);
    ROS_INFO("[simulation_loop] 路径跟踪 lookahead_n=%d", path_lookahead_n);

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

    // 加载 Tracking 模式参数（alg_phase3_full.md 问题一）
    ros::param::param<double>("/target/tracking_entry_dist",    sim.tracking_entry_dist,    2000.0);
    ros::param::param<double>("/target/tracking_window",        sim.tracking_window,        100.0);
    ros::param::param<int>   ("/target/divergence_threshold",   sim.divergence_threshold,   3);
    ros::param::param<double>("/target/virtual_goal_tolerance", sim.virtual_goal_tolerance, 200.0);
    ROS_INFO("[simulation_loop] Tracking: entry=%.0fm window=%.0fm div_thresh=%d vg_tol=%.0fm",
             sim.tracking_entry_dist, sim.tracking_window,
             sim.divergence_threshold, sim.virtual_goal_tolerance);

    // 加载双模式竞标参数（alg_phase3_full.md 问题二）
    ros::param::param<bool>("/target/dual_plan_enabled", sim.dual_plan_enabled, true);
    ROS_INFO("[simulation_loop] 双模式竞标: %s", sim.dual_plan_enabled ? "启用" : "禁用（仅尾追）");

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
        // 4. 规划周期（5Hz）— Tracking 模式下跳过
        // ═══════════════════════════════════════════════════════
        bool is_plan_tick = (sim.sim_step % plan_int == 0);
        if (is_plan_tick && !sim.tracking_mode) {
            // ── 计算双虚拟目标 ────────────────────────────
            auto vg = computeVirtualGoals(sim.goal_state, sim.approach_distance);

            if (sim.dual_plan_enabled) {
                // ── 交替竞标：尾追 / 迎头 ──────────────────
                if (!sim.plan_phase) {
                    // 尾追相位：规划 uav → rear
                    ROS_DEBUG("[simulation_loop] step=%d: 尾追规划 phase", sim.sim_step);
                    bool ok = callPlanner(plan_client, sim.uav_pose, vg.rear, sim, now,
                                          sim.cached_cost_rear, sim.cached_path_rear,
                                          sim.nodes_explored, sim.depth_reached,
                                          sim.planning_time_ms, sim.plan_status);
                    if (ok) {
                        ROS_INFO("[simulation_loop] step=%d: 尾追 cost=%.1f path=%zu %s",
                                 sim.sim_step, sim.cached_cost_rear,
                                 sim.cached_path_rear.size(), sim.plan_status.c_str());
                    } else {
                        ROS_ERROR("[simulation_loop] step=%d: 尾追规划 Service 调用失败", sim.sim_step);
                        sim.cached_cost_rear = INFINITY;
                        sim.cached_path_rear.clear();
                    }
                    sim.plan_phase = true;  // 下周期切换迎头
                    sim.first_plan_pending = false;
                } else {
                    // 迎头相位：规划 uav → front
                    ROS_DEBUG("[simulation_loop] step=%d: 迎头规划 phase", sim.sim_step);
                    bool ok = callPlanner(plan_client, sim.uav_pose, vg.front, sim, now,
                                          sim.cached_cost_front, sim.cached_path_front,
                                          sim.nodes_explored, sim.depth_reached,
                                          sim.planning_time_ms, sim.plan_status);
                    if (ok) {
                        ROS_INFO("[simulation_loop] step=%d: 迎头 cost=%.1f path=%zu %s",
                                 sim.sim_step, sim.cached_cost_front,
                                 sim.cached_path_front.size(), sim.plan_status.c_str());
                    } else {
                        ROS_ERROR("[simulation_loop] step=%d: 迎头规划 Service 调用失败", sim.sim_step);
                        sim.cached_cost_front = INFINITY;
                        sim.cached_path_front.clear();
                    }
                    sim.plan_phase = false; // 下周期切换尾追

                    // ── 竞标：选代价较低者 ──────────────────
                    // 快速启发式：若 UAV 到两个虚拟目标的直线距离差 > 2x，跳过劣势方向
                    double d_rear  = dist2d(sim.uav_pose, vg.rear);
                    double d_front = dist2d(sim.uav_pose, vg.front);
                    bool use_rear  = (sim.cached_cost_rear <= sim.cached_cost_front);
                    bool rear_ok   = !sim.cached_path_rear.empty();
                    bool front_ok  = !sim.cached_path_front.empty();

                    if (rear_ok && !front_ok) {
                        use_rear = true;
                    } else if (!rear_ok && front_ok) {
                        use_rear = false;
                    } else if (rear_ok && front_ok) {
                        use_rear = (sim.cached_cost_rear <= sim.cached_cost_front);
                    } else {
                        // 两者都失败，保持旧路径
                        ROS_WARN("[simulation_loop] step=%d: 双规划均失败，保持旧路径", sim.sim_step);
                        use_rear = true;  // 不影响下面逻辑，只是不更新
                    }

                    if (use_rear && rear_ok) {
                        sim.best_path = sim.cached_path_rear;
                        sim.best_cost = sim.cached_cost_rear;
                        sim.active_mode = "tail";
                    } else if (!use_rear && front_ok) {
                        sim.best_path = sim.cached_path_front;
                        sim.best_cost = sim.cached_cost_front;
                        sim.active_mode = "head-on";
                    }

                    if (rear_ok || front_ok) {
                        sim.path_generation++;
                        sim.path_ptr = 0;
                        auto tree_marker = makeSearchTreeMarker(sim.best_path, "map", now);
                        pub_bd_tree.publish(tree_marker);
                        ROS_INFO("[simulation_loop] step=%d: 竞标结果 %s cost=%.1f path=%zu (rear=%.1f front=%.1f)",
                                 sim.sim_step, sim.active_mode.c_str(), sim.best_cost,
                                 sim.best_path.size(),
                                 rear_ok ? sim.cached_cost_rear : -1.0,
                                 front_ok ? sim.cached_cost_front : -1.0);
                    }
                }
            } else {
                // ── 单尾追模式（向后兼容）──────────────────
                bool ok = callPlanner(plan_client, sim.uav_pose, vg.rear, sim, now,
                                      sim.best_cost, sim.best_path,
                                      sim.nodes_explored, sim.depth_reached,
                                      sim.planning_time_ms, sim.plan_status);
                if (ok) {
                    sim.path_generation++;
                    sim.path_ptr = 0;
                    sim.active_mode = "tail";
                    auto tree_marker = makeSearchTreeMarker(sim.best_path, "map", now);
                    pub_bd_tree.publish(tree_marker);
                    ROS_INFO("[simulation_loop] step=%d: 规划成功 cost=%.1f path=%zu nodes=%d depth=%d time=%.1fms %s",
                             sim.sim_step, sim.best_cost, sim.best_path.size(),
                             sim.nodes_explored, sim.depth_reached, sim.planning_time_ms,
                             sim.plan_status.c_str());
                } else {
                    ROS_ERROR("[simulation_loop] step=%d: Service 调用失败", sim.sim_step);
                }
            }
            first_plan_done = true;
        } else if (is_plan_tick && sim.tracking_mode) {
            ROS_DEBUG("[simulation_loop] step=%d: TRACKING 模式 — 跳过规划", sim.sim_step);
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
        // 6. UAV 沿路径推进 V×dt（Tracking 模式下用直线追踪）
        // ═══════════════════════════════════════════════════════
        double advance_dist = cruise_speed * sim_dt;

        if (sim.tracking_mode) {
            // Tracking 模式：直线朝实际目标飞行
            double y_before = sim.uav_pose[1];
            double yaw_before = sim.uav_pose[3];
            double goal_y = sim.goal_state[1];
            uav.deadReckonToward(sim.uav_pose, sim.goal_state, advance_dist);
            sim.tracking_distance_accum += advance_dist;
            ROS_INFO_THROTTLE(2.0,
                "[simulation_loop] TRACKING: y %.0f→%.0f ψ %.1f° goal_y=%.0f dy=%.0f",
                y_before, sim.uav_pose[1],
                yaw_before * 180.0 / M_PI,
                goal_y,
                goal_y - y_before);
        } else {
            // NORMAL 模式：带偏航角约束的连续插值路径跟踪（uav_dynamicR2.md）
            // 路径代际变化 → 从 UAV 最近点重定位指针（跳过已过时前段）
            static size_t s_last_gen = 0;
            if (sim.path_generation != s_last_gen) {
                s_last_gen = sim.path_generation;
                // 找到距 UAV 当前位置最近的路径点，跳过缓存路径的前段
                double best_d = INFINITY;
                int best_i = 0;
                for (int i = 0; i < static_cast<int>(sim.best_path.size()); ++i) {
                    double dx = sim.best_path[i][0] - sim.uav_pose[0];
                    double dy = sim.best_path[i][1] - sim.uav_pose[1];
                    double d = std::sqrt(dx*dx + dy*dy);
                    if (d < best_d) { best_d = d; best_i = i; }
                }
                sim.path_ptr = best_i;
                ROS_DEBUG("[simulation_loop] step=%d: 路径代际变更 gen=%zu mode=%s ptr=%d",
                          sim.sim_step, sim.path_generation, sim.active_mode.c_str(), best_i);
            }

            uav.interpolateAlongPathWithYawLimit(sim.uav_pose, sim.best_path,
                                                  sim.path_ptr, sim.path_generation,
                                                  sim.goal_state, advance_dist,
                                                  path_lookahead_n);
        }

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

        // ── 航向连续性保护（防御性安全网）────────────────────
        if (heading_continuity_enabled) {
            static double s_prev_yaw = sim.uav_pose[3];
            static bool   s_yaw_init = false;
            if (!s_yaw_init) { s_prev_yaw = sim.uav_pose[3]; s_yaw_init = true; }

            double max_dpsi = (cruise_speed / turn_radius) * sim_dt;
            double dpsi = sim.uav_pose[3] - s_prev_yaw;
            while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
            while (dpsi < -M_PI) dpsi += 2.0 * M_PI;

            if (std::abs(dpsi) > max_dpsi * 1.5) {
                double clamped = s_prev_yaw + std::copysign(max_dpsi, dpsi);
                while (clamped >  M_PI) clamped -= 2.0 * M_PI;
                while (clamped < -M_PI) clamped += 2.0 * M_PI;
                ROS_WARN_THROTTLE(1.0,
                    "[simulation_loop] step=%d: 航向跳变拦截 Δψ=%.1f°→限制为%.1f° (max=%.1f°/tick)",
                    sim.sim_step,
                    dpsi * 180.0 / M_PI,
                    (clamped - s_prev_yaw) * 180.0 / M_PI,
                    max_dpsi * 180.0 / M_PI);
                sim.uav_pose[3] = clamped;
            }
            s_prev_yaw = sim.uav_pose[3];
        }

        // ═══════════════════════════════════════════════════════
        // 8. 到达检测 (不终止仿真) + Tracking 进入/退出检查
        // ═══════════════════════════════════════════════════════
        double tol_xy = 50.0;
        ros::param::param<double>("/beam_dubins/goal_tolerance_xy", tol_xy, 50.0);
        double d_to_target = dist2d(sim.uav_pose, sim.goal_state);

        if (!sim.reached && d_to_target < tol_xy) {
            sim.reached = true;
            ROS_INFO("[simulation_loop] ★★★ 到达目标邻域！ step=%d time=%.1fs（继续跟踪）★★★",
                     sim.sim_step, sim.sim_time);
        }
        if (sim.reached && d_to_target > tol_xy * 3.0) {
            sim.reached = false;
            ROS_INFO("[simulation_loop] step=%d: UAV 已远离目标，重新跟踪", sim.sim_step);
        }

        // ── Tracking 进入条件检查 ─────────────────────────
        if (!sim.tracking_mode && sim.path_ptr >= static_cast<int>(sim.best_path.size()) - 1
            && !sim.best_path.empty()) {
            if (d_to_target < sim.tracking_entry_dist) {
                // 计算当前活跃模式的 virtual_goal 距离
                auto vg = computeVirtualGoals(sim.goal_state, sim.approach_distance);
                const auto& vgoal = (sim.active_mode == "head-on") ? vg.front : vg.rear;
                double d_to_virtual = dist2d(sim.uav_pose, vgoal);
                if (d_to_virtual < sim.virtual_goal_tolerance) {
                    sim.tracking_mode = true;
                    sim.tracking_distance_accum = 0.0;
                    sim.consecutive_diverging = 0;
                    sim.last_tracking_dist = d_to_target;
                    ROS_INFO("[simulation_loop] step=%d: >>> 进入 TRACKING 模式 (dist=%.0fm mode=%s)",
                             sim.sim_step, d_to_target, sim.active_mode.c_str());
                }
            }
        }

        // ── Tracking 趋势评估（每 tracking_window 评估一次）──
        if (sim.tracking_mode && sim.tracking_distance_accum >= sim.tracking_window) {
            double delta = d_to_target - sim.last_tracking_dist;
            if (delta > 0) {
                sim.consecutive_diverging++;
                ROS_WARN("[simulation_loop] step=%d: tracking 距离增大 +%.1fm (%d/%d)",
                         sim.sim_step, delta, sim.consecutive_diverging, sim.divergence_threshold);
            } else {
                sim.consecutive_diverging = 0;
            }
            sim.last_tracking_dist = d_to_target;
            sim.tracking_distance_accum = 0.0;

            if (sim.consecutive_diverging >= sim.divergence_threshold) {
                sim.tracking_mode = false;
                sim.best_path.clear();
                sim.path_ptr = 0;
                sim.path_generation++;
                sim.consecutive_diverging = 0;
                sim.first_plan_pending = true;  // 重新竞标
                ROS_WARN("[simulation_loop] step=%d: <<< 退出 TRACKING 模式，恢复规划", sim.sim_step);
            }
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
                << " | " << (sim.tracking_mode ? "TRACKING" : (sim.reached ? "REACHED" : "planning"))
                << " | mode=" << sim.active_mode;
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
