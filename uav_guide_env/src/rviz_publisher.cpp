/**
 * @file rviz_publisher.cpp
 * @brief Rviz 3D 可视化发布节点（Phase 3）
 *
 * 从设计文档 ros_design.md §5.3 迁移。
 *
 * 功能：
 *  1. 发布障碍物 MarkerArray（半透明 AABB 方块，1Hz 更新）
 *  2. 发布空间边界 Marker（线框，latched）
 *  3. 订阅并增强 UAV/目标/路径等可视化主题
 *
 * 设计要点：
 *  - 障碍物 Marker 从 /scenario/obstacles 参数初始化
 *  - 动态障碍物根据当前目标 Y 坐标更新（订阅 /target/state）
 *  - 空间边界从 /space/* 参数构建
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <sstream>
#include <vector>

// ═══════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════

struct VizState {
    // 空间边界
    double lx = 10000.0, ly = 5000.0, lz = 1000.0;
    double margin = 50.0;

    // 障碍物
    struct ObstacleInfo {
        int id;
        double ox, oy, oz;    // 初始角点
        double dx, dy, dz;    // 尺寸
        double margin;        // 安全边距
        bool is_dynamic;
        double offset_y;      // 动态障碍物的 Y 偏移
        std::string follow;   // "goal_y_neg" / "goal_y_pos"
    };
    std::vector<ObstacleInfo> obs_info;

    // 当前目标 Y（用于动态障碍物更新）
    double current_goal_y = 2500.0;
};

// ═══════════════════════════════════════════════════════════════
// Marker 构建函数
// ═══════════════════════════════════════════════════════════════

/// 构建单个 AABB 障碍物的半透明 Cube Marker
static visualization_msgs::Marker makeObstacleMarker(
    int id, double cx, double cy, double cz,
    double dx, double dy, double dz,
    const ros::Time& stamp)
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = "odom";
    m.ns      = "obstacles";
    m.id      = id;
    m.type    = visualization_msgs::Marker::CUBE;
    m.action  = visualization_msgs::Marker::ADD;
    m.pose.position.x = cx;
    m.pose.position.y = cy;
    m.pose.position.z = cz;
    m.pose.orientation.w = 1.0;
    m.scale.x = dx;
    m.scale.y = dy;
    m.scale.z = dz;
    // 灰色半透明
    m.color.r = 0.4f;
    m.color.g = 0.4f;
    m.color.b = 0.4f;
    m.color.a = 0.5f;
    m.lifetime = ros::Duration(2.0);  // 2 秒存活（持续刷新）
    return m;
}

/// 构建空间边界线框 Marker
static visualization_msgs::Marker makeBoundsMarker(
    double lx, double ly, double lz,
    double margin, const ros::Time& stamp)
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = "odom";
    m.ns      = "env_bounds";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::LINE_LIST;
    m.action  = visualization_msgs::Marker::ADD;
    m.scale.x = 3.0;  // 线宽 (m)
    // 浅灰色半透明
    m.color.r = 0.6f;
    m.color.g = 0.6f;
    m.color.b = 0.6f;
    m.color.a = 0.3f;
    m.lifetime = ros::Duration(0);  // 持久

    // 8 个角点
    double x0 = margin, x1 = lx - margin;
    double y0 = margin, y1 = ly - margin;
    double z0 = margin, z1 = lz - margin;

    // 辅助 lambda：添加线段
    auto add_line = [&m](double xa, double ya, double za,
                         double xb, double yb, double zb) {
        geometry_msgs::Point p;
        p.x = xa; p.y = ya; p.z = za; m.points.push_back(p);
        p.x = xb; p.y = yb; p.z = zb; m.points.push_back(p);
    };

    // 底面 4 条边
    add_line(x0, y0, z0, x1, y0, z0);
    add_line(x1, y0, z0, x1, y1, z0);
    add_line(x1, y1, z0, x0, y1, z0);
    add_line(x0, y1, z0, x0, y0, z0);
    // 顶面 4 条边
    add_line(x0, y0, z1, x1, y0, z1);
    add_line(x1, y0, z1, x1, y1, z1);
    add_line(x1, y1, z1, x0, y1, z1);
    add_line(x0, y1, z1, x0, y0, z1);
    // 4 条竖棱
    add_line(x0, y0, z0, x0, y0, z1);
    add_line(x1, y0, z0, x1, y0, z1);
    add_line(x1, y1, z0, x1, y1, z1);
    add_line(x0, y1, z0, x0, y1, z1);

    return m;
}

/// 构建状态文本 Marker（Text View Facing）
static visualization_msgs::Marker makeStatusTextMarker(
    const std::string& text, const ros::Time& stamp)
{
    visualization_msgs::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = "odom";
    m.ns      = "sim_status";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action  = visualization_msgs::Marker::ADD;
    // 文本位置：空间左上角上方
    m.pose.position.x = 500.0;
    m.pose.position.y = 4500.0;
    m.pose.position.z = 950.0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 80.0;  // 文字高度 (m)
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 0.9f;
    m.text    = text;
    m.lifetime = ros::Duration(2.0);
    return m;
}

// ═══════════════════════════════════════════════════════════════
// 目标状态回调（获取当前 goal_y 用于动态障碍物更新）
// ═══════════════════════════════════════════════════════════════

static void targetStateCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                VizState* state)
{
    state->current_goal_y = msg->pose.pose.position.y;
}

// ═══════════════════════════════════════════════════════════════
// 仿真状态回调（更新文本 Marker）
// ═══════════════════════════════════════════════════════════════

static std::string g_status_text = "Beam Dubins Simulation";

static void simStatusCallback(const std_msgs::String::ConstPtr& msg)
{
    g_status_text = msg->data;
}

// ═══════════════════════════════════════════════════════════════
// 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_publisher");
    ros::NodeHandle nh;

    VizState state;

    // ── 加载参数 ──────────────────────────────────────────────
    ros::param::param<double>("/space/lx", state.lx, 10000.0);
    ros::param::param<double>("/space/ly", state.ly, 5000.0);
    ros::param::param<double>("/space/lz", state.lz, 1000.0);
    ros::param::param<double>("/space/boundary_margin", state.margin, 50.0);

    // 从 scenario.yaml 读取障碍物配置
    XmlRpc::XmlRpcValue obs_list;
    if (ros::param::get("/scenario/obstacles", obs_list)) {
        for (int i = 0; i < obs_list.size(); ++i) {
            auto& o = obs_list[i];
            VizState::ObstacleInfo info;
            info.id = o.hasMember("id") ? static_cast<int>(o["id"]) : i;
            info.ox = 0; info.oy = 0; info.oz = 0;
            info.dx = 100; info.dy = 100; info.dz = 100;
            info.margin = 5.0;
            if (o.hasMember("position") && o["position"].size() >= 3) {
                info.ox = static_cast<double>(o["position"][0]);
                info.oy = static_cast<double>(o["position"][1]);
                info.oz = static_cast<double>(o["position"][2]);
            }
            if (o.hasMember("size") && o["size"].size() >= 3) {
                info.dx = static_cast<double>(o["size"][0]);
                info.dy = static_cast<double>(o["size"][1]);
                info.dz = static_cast<double>(o["size"][2]);
            }
            if (o.hasMember("safe_margin"))
                info.margin = static_cast<double>(o["safe_margin"]);
            info.is_dynamic = false;
            info.offset_y = 0.0;
            if (o.hasMember("type")) {
                std::string t = o["type"];
                info.is_dynamic = (t == "dynamic");
            }
            if (o.hasMember("offset_y"))
                info.offset_y = static_cast<double>(o["offset_y"]);
            if (o.hasMember("follow"))
                info.follow = std::string(o["follow"]);

            state.obs_info.push_back(info);
        }
    }

    ROS_INFO("[rviz_publisher] 加载 %zu 个障碍物, 空间=%.0f×%.0f×%.0fm",
             state.obs_info.size(), state.lx, state.ly, state.lz);

    // ── 订阅 ──────────────────────────────────────────────────
    ros::Subscriber sub_target = nh.subscribe<nav_msgs::Odometry>(
        "/target/state", 10,
        boost::bind(targetStateCallback, _1, &state));
    ros::Subscriber sub_status = nh.subscribe<std_msgs::String>(
        "/sim/status", 10, simStatusCallback);

    // ── 发布 ──────────────────────────────────────────────────
    ros::Publisher pub_obstacles   = nh.advertise<visualization_msgs::MarkerArray>(
        "/obstacles", 10, true);  // latched
    ros::Publisher pub_bounds      = nh.advertise<visualization_msgs::Marker>(
        "/env/bounds", 10, true); // latched
    ros::Publisher pub_status_text = nh.advertise<visualization_msgs::Marker>(
        "/sim/status_text", 10);

    // ── 发布一次性（latched）Markers ─────────────────────────
    // 空间边界（不变，发布一次即可）
    {
        auto bounds = makeBoundsMarker(state.lx, state.ly, state.lz,
                                       state.margin, ros::Time::now());
        pub_bounds.publish(bounds);
        ROS_INFO("[rviz_publisher] 空间边界已发布 (latched)");
    }

    // ── 定时器：1Hz 更新障碍物 Marker 和状态文本 ────────────
    ros::Timer timer = nh.createTimer(ros::Duration(1.0),
        [&](const ros::TimerEvent&) {
            ros::Time now = ros::Time::now();

            // ── 障碍物 MarkerArray ────────────────────────────
            visualization_msgs::MarkerArray obs_array;
            for (size_t i = 0; i < state.obs_info.size(); ++i) {
                const auto& info = state.obs_info[i];

                double cx, cy, cz;
                double dx = info.dx + 2.0 * info.margin;  // AABB 尺寸（含边距）
                double dy = info.dy + 2.0 * info.margin;
                double dz = info.dz + 2.0 * info.margin;

                if (info.is_dynamic) {
                    // 动态障碍物：根据当前 goal_y 计算中心位置
                    // 左墙 (follow=goal_y_neg): center_y = goal_y + offset_y + dy/2
                    // 右墙 (follow=goal_y_pos): center_y = goal_y + offset_y + dy/2
                    cy = state.current_goal_y + info.offset_y + dy / 2.0;
                    cx = info.ox + dx / 2.0;
                    cz = info.oz + dz / 2.0;
                } else {
                    // 静态障碍物
                    cx = info.ox + dx / 2.0;
                    cy = info.oy + dy / 2.0;
                    cz = info.oz + dz / 2.0;
                }

                obs_array.markers.push_back(
                    makeObstacleMarker(info.id, cx, cy, cz, dx, dy, dz, now));
            }
            pub_obstacles.publish(obs_array);

            // ── 状态文本 Marker ───────────────────────────────
            auto text_marker = makeStatusTextMarker(g_status_text, now);
            pub_status_text.publish(text_marker);
        });

    ROS_INFO("[rviz_publisher] 可视化节点就绪，发布 /obstacles /env/bounds /sim/status_text");
    ROS_INFO("[rviz_publisher] 在 Rviz 中可通过以下 Display 订阅:");
    ROS_INFO("  - MarkerArray: /obstacles (障碍物)");
    ROS_INFO("  - Marker: /env/bounds (边界线框)");
    ROS_INFO("  - Marker: /sim/status_text (状态文本)");
    ROS_INFO("  - Odometry: /uav/state (UAV 位姿)");
    ROS_INFO("  - Path: /uav/trail (航迹)");
    ROS_INFO("  - Path: /beam_dubins/path (规划路径)");
    ROS_INFO("  - Odometry: /target/state (目标)");
    ROS_INFO("  - TF: uav_base_link, target_link");

    ros::spin();
    return 0;
}
