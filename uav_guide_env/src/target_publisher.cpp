/**
 * @file target_publisher.cpp
 * @brief 目标轨迹独立发布节点（Phase 3 v0.1, alg_phase3_v01.md §2）
 *
 * 将目标轨迹逻辑从 simulation_loop 解耦为独立 ROS 节点。
 * 发布实际目标位姿、虚拟目标（尾追/迎头偏移点）、拦截模式。
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <string>
#include <vector>

#include "uav_guide_env/target_trajectory.h"

// ═══════════════════════════════════════════════════════════════
// 辅助：构建 Odometry 消息
// ═══════════════════════════════════════════════════════════════

static nav_msgs::Odometry makeOdometry(const std::array<double, 5>& state,
                                       const std::string& frame_id,
                                       const std::string& child_frame,
                                       const ros::Time& stamp)
{
    nav_msgs::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id  = child_frame;
    odom.pose.pose.position.x = state[0];
    odom.pose.pose.position.y = state[1];
    odom.pose.pose.position.z = state[2];
    double cy = std::cos(state[3] * 0.5);
    double sy = std::sin(state[3] * 0.5);
    odom.pose.pose.orientation.w = cy;
    odom.pose.pose.orientation.z = sy;
    return odom;
}

// ═══════════════════════════════════════════════════════════════
// external 模式：订阅真实 /target/state（来自 ros_udp_bridge）
// ═══════════════════════════════════════════════════════════════

static std::array<double, 5> g_real_target = {0.0, 0.0, 0.0, 0.0, 0.0};
static bool g_has_real = false;

static void realTargetCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_real_target[0] = msg->pose.pose.position.x;
    g_real_target[1] = msg->pose.pose.position.y;
    g_real_target[2] = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    g_real_target[3] = 2.0 * std::atan2(qz, qw);
    g_real_target[4] = 0.0;
    g_has_real = true;
}

// ═══════════════════════════════════════════════════════════════
// 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{   
    setlocale(LC_ALL, "");  // 支持中文输出
    ros::init(argc, argv, "target_publisher");
    ros::NodeHandle nh;

    // ── 加载 YAML 参数 ──────────────────────────────────────
    std::string traj_type = "line";
    std::string intercept_mode = "tail";
    double L_tail   = 1000.0;
    double L_headon = 1500.0;
    std::vector<double> init_pos_vec;
    ros::param::param<std::string>("/target/trajectory_type",     traj_type,      traj_type);
    ros::param::param<std::string>("/target/intercept_mode",      intercept_mode, intercept_mode);
    ros::param::param<double>("/target/approach_distance_tail",   L_tail,   1000.0);
    ros::param::param<double>("/target/approach_distance_headon", L_headon, 1500.0);
    ros::param::get("/target/initial_position", init_pos_vec);
    std::array<double, 3> init_pos = {4000.0, 2500.0, 600.0};
    if (init_pos_vec.size() >= 3) {
        init_pos = {init_pos_vec[0], init_pos_vec[1], init_pos_vec[2]};
    }

    // 轨迹参数
    std::array<double, 5> traj_params = {90.0, 20.0, 4000.0, 0.0, 0.0};
    if (traj_type == "line") {
        double dir=90, spd=20, len=4000;
        ros::param::param<double>("/target/trajectories/line/direction_deg", dir, 90.0);
        ros::param::param<double>("/target/trajectories/line/speed_mps",     spd, 20.0);
        ros::param::param<double>("/target/trajectories/line/length_m",      len, 4000.0);
        traj_params = {dir, spd, len, 0.0, 0.0};
    } else if (traj_type == "circle") {
        traj_params = {2000.0, 2500.0, 600.0, 500.0, 3.0};
    }

    // ── 拦截模式（latched 发布）────────────────────────────
    ros::Publisher pub_vgoal = nh.advertise<nav_msgs::Odometry>("/target/virtual_goal", 10);
    ros::Publisher pub_mode  = nh.advertise<std_msgs::String>("/target/intercept_mode", 10, true);
    std_msgs::String mode_msg;
    mode_msg.data = intercept_mode;
    pub_mode.publish(mode_msg);

    double L = (intercept_mode == "head-on") ? L_headon : L_tail;

    // ── 真实 target 外部模式（阶段 7b）─────────────────────
    // external=true：不生成合成轨迹，订阅真实 /target/state（ros_udp_bridge
    // udp_receiver 发布），仅转发 /target/virtual_goal + /target/intercept_mode，
    // 供 simulation_loop 在获得真实 target 数据的前提下做内部 rviz 仿真。
    bool external_target = false;
    ros::param::param<bool>("/target/external_target", external_target, false);
    if (external_target) {
        std::string ext_topic = "/target/state";
        ros::param::param<std::string>("/target/external_topic", ext_topic, ext_topic);
        ROS_INFO("[target_publisher] EXTERNAL 模式：订阅 %s → 转发 virtual_goal/intercept_mode",
                 ext_topic.c_str());
        std::array<double, 5> ext_state = {init_pos[0], init_pos[1], init_pos[2], 0.0, 0.0};
        bool ext_has = false;
        auto ext_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
            ext_state[0] = msg->pose.pose.position.x;
            ext_state[1] = msg->pose.pose.position.y;
            ext_state[2] = msg->pose.pose.position.z;
            ext_state[3] = 2.0 * std::atan2(msg->pose.pose.orientation.z,
                                            msg->pose.pose.orientation.w);
            ext_state[4] = 0.0;
            ext_has = true;
        };
        ros::Subscriber sub_ext = nh.subscribe<nav_msgs::Odometry>(ext_topic, 10, ext_cb);
        ros::Rate ext_rate(10.0);
        while (ros::ok()) {
            if (ext_has) {
                auto vg = uav_guide_env::TargetTrajectory::computeVirtualGoal(
                    ext_state, intercept_mode, L);
                pub_vgoal.publish(makeOdometry(vg, "map", "virtual_goal_link", ros::Time::now()));
            }
            ros::spinOnce();
            ext_rate.sleep();
        }
        return 0;
    }

    // ── 合成轨迹模式（原逻辑）──────────────────────────────
    uav_guide_env::TargetTrajectory target(traj_type, init_pos, traj_params);
    ROS_INFO("[target_publisher] 轨迹类型=%s 拦截模式=%s L=%.0fm",
             traj_type.c_str(), intercept_mode.c_str(), L);

    ros::Publisher pub_state = nh.advertise<nav_msgs::Odometry>("/target/state", 10);
    ros::Publisher pub_traj  = nh.advertise<nav_msgs::Path>("/target/trajectory_pred", 10, true);

    // ── 10Hz 主循环 ────────────────────────────────────────
    ros::Rate rate(10.0);
    double t = 0.0;
    int tick = 0;

    // 预生成目标预测轨迹（1Hz 更新）
    nav_msgs::Path pred_path;
    pred_path.header.frame_id = "map";

    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        // 计算实际目标位姿
        auto target_state = target.positionAt(t);

        // 计算虚拟目标（尾追/迎头偏移点）
        auto virtual_goal = uav_guide_env::TargetTrajectory::computeVirtualGoal(
            target_state, intercept_mode, L);

        // 发布
        pub_state.publish(makeOdometry(target_state, "map", "target_link", now));
        pub_vgoal.publish(makeOdometry(virtual_goal, "map", "virtual_goal_link", now));

        // 1Hz 更新目标预测轨迹
        if (tick % 10 == 0) {
            pred_path.header.stamp = now;
            pred_path.poses.clear();
            // 预测未来 10 秒，每秒 1 个点
            for (int i = 0; i <= 10; ++i) {
                auto fut = target.positionAt(t + i * 1.0);
                geometry_msgs::PoseStamped ps;
                ps.header = pred_path.header;
                ps.pose.position.x = fut[0];
                ps.pose.position.y = fut[1];
                ps.pose.position.z = fut[2];
                double cy = std::cos(fut[3] * 0.5);
                double sy = std::sin(fut[3] * 0.5);
                ps.pose.orientation.w = cy;
                ps.pose.orientation.z = sy;
                pred_path.poses.push_back(ps);
            }
            pub_traj.publish(pred_path);
        }

        t += 0.1;
        tick++;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
