/**
 * @file uav_logger.cpp
 * @brief 仿真数据记录节点 — 将 UAV 和目标轨迹保存为 JSON
 *
 * 订阅 /uav/state 和 /target/state，记录时间戳 + 5D 位姿，
 * 仿真结束时写入 ${beam_dubins_ws}/log/sim_log_YYYYMMDD_HHMMSS.json
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <vector>
#include <cmath>

// ═══════════════════════════════════════════════════════════════
// 数据点结构
// ═══════════════════════════════════════════════════════════════

struct TrajectoryPoint {
    double t;       // 仿真时间 (s)
    double x, y, z;
    double yaw;
};

static std::vector<TrajectoryPoint> g_uav_log;
static std::vector<TrajectoryPoint> g_target_log;
static double g_sim_start = 0.0;  // 首个数据点的墙上时间

// ═══════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════

static void uavCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (g_sim_start == 0.0) g_sim_start = msg->header.stamp.toSec();

    TrajectoryPoint pt;
    pt.t   = msg->header.stamp.toSec() - g_sim_start;
    pt.x   = msg->pose.pose.position.x;
    pt.y   = msg->pose.pose.position.y;
    pt.z   = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    pt.yaw = 2.0 * std::atan2(qz, qw);
    g_uav_log.push_back(pt);
}

static void targetCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    TrajectoryPoint pt;
    pt.t   = msg->header.stamp.toSec() - g_sim_start;
    pt.x   = msg->pose.pose.position.x;
    pt.y   = msg->pose.pose.position.y;
    pt.z   = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    pt.yaw = 2.0 * std::atan2(qz, qw);
    g_target_log.push_back(pt);
}

// ═══════════════════════════════════════════════════════════════
// JSON 写入
// ═══════════════════════════════════════════════════════════════

static void writeJson(const std::string& filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        ROS_ERROR("[uav_logger] 无法写入文件: %s", filepath.c_str());
        return;
    }

    ofs << std::fixed << std::setprecision(3);
    ofs << "{\n";
    ofs << "  \"description\": \"Beam Dubins UAV simulation log\",\n";
    ofs << "  \"uav_points\": " << g_uav_log.size() << ",\n";
    ofs << "  \"target_points\": " << g_target_log.size() << ",\n";

    // ── UAV 轨迹 ──
    ofs << "  \"uav_trajectory\": [\n";
    for (size_t i = 0; i < g_uav_log.size(); ++i) {
        const auto& p = g_uav_log[i];
        ofs << "    {\"t\":" << p.t << ",\"x\":" << p.x
            << ",\"y\":" << p.y << ",\"z\":" << p.z
            << ",\"yaw\":" << p.yaw << "}";
        if (i + 1 < g_uav_log.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ],\n";

    // ── 目标轨迹 ──
    ofs << "  \"target_trajectory\": [\n";
    for (size_t i = 0; i < g_target_log.size(); ++i) {
        const auto& p = g_target_log[i];
        ofs << "    {\"t\":" << p.t << ",\"x\":" << p.x
            << ",\"y\":" << p.y << ",\"z\":" << p.z
            << ",\"yaw\":" << p.yaw << "}";
        if (i + 1 < g_target_log.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";
    ofs.close();

    ROS_INFO("[uav_logger] 日志已保存: %s (UAV=%zu points, Target=%zu points)",
             filepath.c_str(), g_uav_log.size(), g_target_log.size());
}

// ═══════════════════════════════════════════════════════════════
// 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_logger");
    ros::NodeHandle nh;

    // ── 订阅 ──────────────────────────────────────────────
    ros::Subscriber sub_uav    = nh.subscribe("/uav/state",    100, uavCallback);
    ros::Subscriber sub_target = nh.subscribe("/target/state", 100, targetCallback);

    // ── 生成输出路径 ──────────────────────────────────────
    std::string log_dir;
    ros::param::param<std::string>("/uav_logger/log_dir", log_dir,
                                    std::string(getenv("HOME")) + "/beam_dubins_ws/log");
    // 创建目录
    std::string mkdir_cmd = "mkdir -p " + log_dir;
    system(mkdir_cmd.c_str());

    // 时间戳文件名
    time_t now = time(nullptr);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", localtime(&now));
    std::string filepath = log_dir + "/sim_log_" + ts + ".json";

    ROS_INFO("[uav_logger] 开始记录，输出: %s", filepath.c_str());

    // ── 事件循环 ──────────────────────────────────────────
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // ── 退出时写入 JSON ──────────────────────────────────
    writeJson(filepath);
    return 0;
}
