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

    // 通道参数（从 /target/channel/* 加载）
    double channel_inner_width    = 100.0;
    double channel_wall_length    = 2000.0;
    double channel_wall_thickness = 2000.0;
    double channel_wall_height    = 1000.0;
    double channel_wall_z_base    = 0.0;
    double channel_safe_margin    = 5.0;

    // 当前目标状态（用于计算墙壁位置）
    double target_x   = 4000.0;
    double target_y   = 2500.0;
    double target_z   = 600.0;
    double target_yaw = M_PI / 2.0;  // 默认朝北
};

// ═══════════════════════════════════════════════════════════════
// Marker 构建函数
// ═══════════════════════════════════════════════════════════════

/// 构建单个 AABB 障碍物的半透明 Cube Marker
/// frame_id 使用 "map"（map→odom 恒等变换，坐标一致）
static visualization_msgs::Marker makeObstacleMarker(
    int id, double cx, double cy, double cz,
    double dx, double dy, double dz)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);  // 使用最新 TF，标准可视化惯例
    m.header.frame_id = "map";           // 直接使用 map 帧，避免 TF 查询失败
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
    // 深灰底色 + 白色线框边框 → 在白背景上清晰可见
    m.color.r = 0.25f;
    m.color.g = 0.25f;
    m.color.b = 0.25f;
    m.color.a = 0.7f;
    m.lifetime = ros::Duration(2.0);  // 2 秒存活（持续刷新）
    return m;
}

/// 构建空间边界线框 Marker
static visualization_msgs::Marker makeBoundsMarker(
    double lx, double ly, double lz,
    double margin)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns      = "env_bounds";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::LINE_LIST;
    m.action  = visualization_msgs::Marker::ADD;
    m.scale.x = 3.0;  // 线宽 (m)
    m.pose.orientation.w = 1.0;
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
    const std::string& text)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
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
// UAV / 目标 位姿缓存（用于 3D 模型 Marker 发布）
// ═══════════════════════════════════════════════════════════════

static nav_msgs::Odometry g_last_uav_odom;
static nav_msgs::Odometry g_last_target_odom;
static bool g_has_uav    = false;
static bool g_has_target = false;

// ═══════════════════════════════════════════════════════════════
// 目标状态回调（获取当前 goal_y 用于动态障碍物更新）
// ═══════════════════════════════════════════════════════════════

static void targetStateCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                VizState* state)
{
    state->target_x   = msg->pose.pose.position.x;
    state->target_y   = msg->pose.pose.position.y;
    state->target_z   = msg->pose.pose.position.z;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    state->target_yaw = 2.0 * std::atan2(qz, qw);
    // 同时缓存用于 3D 模型 Marker
    g_last_target_odom = *msg;
    g_has_target = true;
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
// UAV 状态回调
// ═══════════════════════════════════════════════════════════════

static void uavStateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_last_uav_odom = *msg;
    g_has_uav = true;
}

/// 构建 UAV 3D 模型 Marker（大号橙色箭头 + 发光球体，确保 10km 尺度可见）
static visualization_msgs::Marker makeUavModelMarker(
    const nav_msgs::Odometry& odom)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);  // 使用最新 TF
    m.header.frame_id = "map";          // 直接使用 map 帧
    m.ns      = "uav_model";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::ARROW;
    m.action  = visualization_msgs::Marker::ADD;

    // 箭头从 UAV 位置沿航向延伸
    double yaw = 2.0 * std::atan2(odom.pose.pose.orientation.z,
                                   odom.pose.pose.orientation.w);
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;
    double shaft_len = 600.0;   // 箭头总长 600m（在 10km 地图上清晰可见）

    m.points.resize(2);
    m.points[0].x = x;
    m.points[0].y = y;
    m.points[0].z = z;
    m.points[1].x = x + shaft_len * std::cos(yaw);
    m.points[1].y = y + shaft_len * std::sin(yaw);
    m.points[1].z = z;

    m.pose.orientation.w = 1.0;
    m.scale.x = 30.0;   // 箭杆直径 (m)
    m.scale.y = 80.0;   // 箭头直径 (m)

    m.color.r = 1.0f;
    m.color.g = 0.45f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m.lifetime = ros::Duration(0.5);  // 短存活期，持续刷新
    return m;
}

/// 构建 UAV 位置球体 Marker（确保即使箭头不渲染也有可见标记）
static visualization_msgs::Marker makeUavSphereMarker(
    const nav_msgs::Odometry& odom)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns      = "uav_sphere";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::SPHERE;
    m.action  = visualization_msgs::Marker::ADD;
    m.pose.position.x = odom.pose.pose.position.x;
    m.pose.position.y = odom.pose.pose.position.y;
    m.pose.position.z = odom.pose.pose.position.z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 100.0;   // 直径 100m 的球
    m.scale.y = 100.0;
    m.scale.z = 100.0;
    m.color.r = 1.0f;
    m.color.g = 0.45f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m.lifetime = ros::Duration(0.5);
    return m;
}

/// 构建目标 3D 模型 Marker（大号红色球体，10km 尺度可见）
static visualization_msgs::Marker makeTargetModelMarker(
    const nav_msgs::Odometry& odom)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns      = "target_model";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::SPHERE;
    m.action  = visualization_msgs::Marker::ADD;
    m.pose.position.x = odom.pose.pose.position.x;
    m.pose.position.y = odom.pose.pose.position.y;
    m.pose.position.z = odom.pose.pose.position.z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 120.0;   // 直径 120m
    m.scale.y = 120.0;
    m.scale.z = 120.0;
    m.color.r = 1.0f;
    m.color.g = 0.15f;
    m.color.b = 0.15f;
    m.color.a = 0.9f;
    m.lifetime = ros::Duration(0.5);
    return m;
}

/// 构建目标朝向箭头 Marker
static visualization_msgs::Marker makeTargetArrowMarker(
    const nav_msgs::Odometry& odom)
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns      = "target_arrow";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::ARROW;
    m.action  = visualization_msgs::Marker::ADD;

    double yaw = 2.0 * std::atan2(odom.pose.pose.orientation.z,
                                   odom.pose.pose.orientation.w);
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;
    double shaft_len = 400.0;

    m.points.resize(2);
    m.points[0].x = x;
    m.points[0].y = y;
    m.points[0].z = z;
    m.points[1].x = x + shaft_len * std::cos(yaw);
    m.points[1].y = y + shaft_len * std::sin(yaw);
    m.points[1].z = z;

    m.pose.orientation.w = 1.0;
    m.scale.x = 20.0;
    m.scale.y = 60.0;

    m.color.r = 1.0f;
    m.color.g = 0.15f;
    m.color.b = 0.15f;
    m.color.a = 1.0f;
    m.lifetime = ros::Duration(0.5);
    return m;
}

// ═══════════════════════════════════════════════════════════════
// 默认 Marker（无仿真数据时的备用显示）
// ═══════════════════════════════════════════════════════════════

/// 默认 UAV 箭头 — 起始位置 (2000, 1000, 500)，朝北 (yaw=π/2)
static visualization_msgs::Marker makeDefaultUavMarker()
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns      = "uav_model";
    m.id      = 0;
    m.type    = visualization_msgs::Marker::ARROW;
    m.action  = visualization_msgs::Marker::ADD;
    double x = 2000.0, y = 1000.0, z = 500.0, yaw = M_PI / 2.0, len = 600.0;
    m.points.resize(2);
    m.points[0].x = x;          m.points[0].y = y;          m.points[0].z = z;
    m.points[1].x = x + len * std::cos(yaw);
    m.points[1].y = y + len * std::sin(yaw);
    m.points[1].z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 30.0;  m.scale.y = 80.0;
    m.color.r = 1.0f;  m.color.g = 0.45f;  m.color.b = 0.0f;  m.color.a = 1.0f;
    m.lifetime = ros::Duration(2.0);
    return m;
}

static visualization_msgs::Marker makeDefaultUavSphere()
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns = "uav_sphere";  m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 2000.0;  m.pose.position.y = 1000.0;  m.pose.position.z = 500.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 100.0;  m.scale.y = 100.0;  m.scale.z = 100.0;
    m.color.r = 1.0f;  m.color.g = 0.45f;  m.color.b = 0.0f;  m.color.a = 1.0f;
    m.lifetime = ros::Duration(2.0);
    return m;
}

/// 默认目标球体 — 起始位置 (4000, 2500, 600)
static visualization_msgs::Marker makeDefaultTargetMarker()
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns = "target_model";  m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = 4000.0;  m.pose.position.y = 2500.0;  m.pose.position.z = 600.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = 120.0;  m.scale.y = 120.0;  m.scale.z = 120.0;
    m.color.r = 1.0f;  m.color.g = 0.15f;  m.color.b = 0.15f;  m.color.a = 0.9f;
    m.lifetime = ros::Duration(2.0);
    return m;
}

/// 默认目标箭头 — 朝北
static visualization_msgs::Marker makeDefaultTargetArrow()
{
    visualization_msgs::Marker m;
    m.header.stamp    = ros::Time(0);
    m.header.frame_id = "map";
    m.ns = "target_arrow";  m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    double x = 4000.0, y = 2500.0, z = 600.0, yaw = M_PI / 2.0, len = 400.0;
    m.points.resize(2);
    m.points[0].x = x;          m.points[0].y = y;          m.points[0].z = z;
    m.points[1].x = x + len * std::cos(yaw);
    m.points[1].y = y + len * std::sin(yaw);
    m.points[1].z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = 20.0;  m.scale.y = 60.0;
    m.color.r = 1.0f;  m.color.g = 0.15f;  m.color.b = 0.15f;  m.color.a = 1.0f;
    m.lifetime = ros::Duration(2.0);
    return m;
}

// ═══════════════════════════════════════════════════════════════
// 主函数
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{   
    // 中文支持
    setlocale(LC_ALL, "");  // 设置系统区域为默认环境

    ros::init(argc, argv, "rviz_publisher");
    ros::NodeHandle nh;
    ROS_INFO("[rviz_publisher] === 节点启动 ===");

    VizState state;

    // ── 加载空间参数 ──────────────────────────────────────────
    ros::param::param<double>("/space/lx", state.lx, 10000.0);
    ros::param::param<double>("/space/ly", state.ly, 5000.0);
    ros::param::param<double>("/space/lz", state.lz, 1000.0);
    ros::param::param<double>("/space/boundary_margin", state.margin, 50.0);
    ROS_INFO("[rviz_publisher] 空间参数: %.0f×%.0f×%.0f margin=%.0f",
             state.lx, state.ly, state.lz, state.margin);

    // ── 加载通道障碍物参数（从 /target/channel/*）─────────
    ros::param::param<double>("/target/channel/inner_width",    state.channel_inner_width,    100.0);
    ros::param::param<double>("/target/channel/wall_length",    state.channel_wall_length,    2000.0);
    ros::param::param<double>("/target/channel/wall_thickness", state.channel_wall_thickness, 2000.0);
    ros::param::param<double>("/target/channel/wall_height",    state.channel_wall_height,    1000.0);
    ros::param::param<double>("/target/channel/wall_z_base",    state.channel_wall_z_base,    0.0);
    ros::param::param<double>("/target/channel/safe_margin",    state.channel_safe_margin,    5.0);
    ROS_INFO("[rviz_publisher] 通道参数: inner_width=%.0f wall=%.0f×%.0f×%.0f",
             state.channel_inner_width, state.channel_wall_thickness,
             state.channel_wall_length, state.channel_wall_height);

    // ── 订阅 ──────────────────────────────────────────────────
    ros::Subscriber sub_target = nh.subscribe<nav_msgs::Odometry>(
        "/target/state", 10,
        boost::bind(targetStateCallback, _1, &state));
    ros::Subscriber sub_status = nh.subscribe<std_msgs::String>(
        "/sim/status", 10, simStatusCallback);
    ros::Subscriber sub_uav_state = nh.subscribe<nav_msgs::Odometry>(
        "/uav/state", 10, uavStateCallback);

    // ── 发布 ──────────────────────────────────────────────────
    ros::Publisher pub_obstacles   = nh.advertise<visualization_msgs::MarkerArray>(
        "/obstacles", 10, true);  // latched
    ros::Publisher pub_bounds      = nh.advertise<visualization_msgs::Marker>(
        "/env/bounds", 10, true); // latched
    ros::Publisher pub_status_text = nh.advertise<visualization_msgs::Marker>(
        "/sim/status_text", 10);
    ros::Publisher pub_uav_model   = nh.advertise<visualization_msgs::Marker>(
        "/uav/model", 10);
    ros::Publisher pub_uav_sphere  = nh.advertise<visualization_msgs::Marker>(
        "/uav/sphere", 10);
    ros::Publisher pub_target_model = nh.advertise<visualization_msgs::Marker>(
        "/target/model", 10);
    ros::Publisher pub_target_arrow = nh.advertise<visualization_msgs::Marker>(
        "/target/arrow", 10);

    // ── 发布一次性（latched）Markers ─────────────────────────
    // 空间边界（不变，发布一次即可）
    {
        auto bounds = makeBoundsMarker(state.lx, state.ly, state.lz,
                                       state.margin);
        pub_bounds.publish(bounds);
        ROS_INFO("[rviz_publisher] 空间边界已发布 (latched)");
    }

    // ── 定时器：1Hz 更新障碍物 Marker 和状态文本 ────────────
    ros::Timer timer = nh.createTimer(ros::Duration(1.0),
        [&](const ros::TimerEvent&) {
            ros::Time now = ros::Time::now();
            static int tick_count = 0;
            tick_count++;

            // ── 障碍物 MarkerArray（通道墙壁，跟随目标）────────
            visualization_msgs::MarkerArray obs_array;
            {
                double hw = state.channel_wall_thickness / 2.0;
                double hl = state.channel_wall_length    / 2.0;
                double hh = state.channel_wall_height    / 2.0;
                double gap2 = state.channel_inner_width  / 2.0;
                double sm  = state.channel_safe_margin;
                double tx = state.target_x, ty = state.target_y;
                double tz = state.channel_wall_z_base + hh;
                double yaw = state.target_yaw;

                // 左方向 = yaw+π/2 (北偏西)，右方向 = yaw-π/2 (北偏东)
                double lx = std::cos(yaw + M_PI / 2.0);
                double ly = std::sin(yaw + M_PI / 2.0);
                double rx = std::cos(yaw - M_PI / 2.0);
                double ry = std::sin(yaw - M_PI / 2.0);

                double lcx = tx + (gap2 + hw + sm) * lx;
                double lcy = ty + (gap2 + hw + sm) * ly;
                double rcx = tx + (gap2 + hw + sm) * rx;
                double rcy = ty + (gap2 + hw + sm) * ry;

                obs_array.markers.push_back(
                    makeObstacleMarker(0, lcx, lcy, tz,
                        state.channel_wall_thickness + 2.0*sm,
                        state.channel_wall_length    + 2.0*sm,
                        state.channel_wall_height    + 2.0*sm));
                obs_array.markers.push_back(
                    makeObstacleMarker(1, rcx, rcy, tz,
                        state.channel_wall_thickness + 2.0*sm,
                        state.channel_wall_length    + 2.0*sm,
                        state.channel_wall_height    + 2.0*sm));
            }
            pub_obstacles.publish(obs_array);

            // ── 状态文本 Marker ───────────────────────────────
            auto text_marker = makeStatusTextMarker(g_status_text);
            pub_status_text.publish(text_marker);

            // ── UAV 3D 模型 Marker（橙色箭头）───────────────
            if (g_has_uav) {
                pub_uav_model.publish(makeUavModelMarker(g_last_uav_odom));
                pub_uav_sphere.publish(makeUavSphereMarker(g_last_uav_odom));
            } else {
                pub_uav_model.publish(makeDefaultUavMarker());
                pub_uav_sphere.publish(makeDefaultUavSphere());
            }

            // ── 目标 3D 模型 Marker（红色球体 + 箭头）─────
            if (g_has_target) {
                pub_target_model.publish(makeTargetModelMarker(g_last_target_odom));
                pub_target_arrow.publish(makeTargetArrowMarker(g_last_target_odom));
            } else {
                pub_target_model.publish(makeDefaultTargetMarker());
                pub_target_arrow.publish(makeDefaultTargetArrow());
            }

            ROS_INFO_THROTTLE(5.0, "[rviz_publisher] 1Hz tick#%d: has_uav=%d has_target=%d tgt=(%.0f,%.0f) yaw=%.1f°",
                              tick_count, g_has_uav, g_has_target,
                              state.target_x, state.target_y,
                              state.target_yaw * 180.0 / M_PI);
        });

    // ── 定时器2：5Hz 高频刷新 UAV/目标 3D 模型 Marker ────────
    ros::Timer model_timer = nh.createTimer(ros::Duration(0.2),
        [&](const ros::TimerEvent&) {
            if (g_has_uav) {
                pub_uav_model.publish(makeUavModelMarker(g_last_uav_odom));
                pub_uav_sphere.publish(makeUavSphereMarker(g_last_uav_odom));
            } else {
                pub_uav_model.publish(makeDefaultUavMarker());
                pub_uav_sphere.publish(makeDefaultUavSphere());
            }
            if (g_has_target) {
                pub_target_model.publish(makeTargetModelMarker(g_last_target_odom));
                pub_target_arrow.publish(makeTargetArrowMarker(g_last_target_odom));
            } else {
                pub_target_model.publish(makeDefaultTargetMarker());
                pub_target_arrow.publish(makeDefaultTargetArrow());
            }
            ROS_INFO_THROTTLE(5.0, "[rviz_publisher] 5Hz tick: has_uav=%d uav_pos=(%.0f,%.0f) has_target=%d tgt_pos=(%.0f,%.0f)",
                              g_has_uav, g_last_uav_odom.pose.pose.position.x, g_last_uav_odom.pose.pose.position.y,
                              g_has_target, g_last_target_odom.pose.pose.position.x, g_last_target_odom.pose.pose.position.y);
        });

    ROS_INFO("[rviz_publisher] 可视化节点就绪");
    ROS_INFO("[rviz_publisher] 在 Rviz 中可通过以下 Display 订阅:");
    ROS_INFO("  - Grid: 白色网格背景（200m 单元格）");
    ROS_INFO("  - MarkerArray: /obstacles (障碍物)");
    ROS_INFO("  - Marker: /env/bounds (边界线框)");
    ROS_INFO("  - Marker: /sim/status_text (状态文本)");
    ROS_INFO("  - Marker: /uav/model (UAV 3D 箭头)");
    ROS_INFO("  - Marker: /target/model (目标 3D 球体)");
    ROS_INFO("  - Odometry: /uav/state (UAV 位姿箭头)");
    ROS_INFO("  - Path: /uav/trail (航迹)");
    ROS_INFO("  - Path: /beam_dubins/path (规划路径)");
    ROS_INFO("  - Odometry: /target/state (目标位姿箭头)");
    ROS_INFO("  - TF: uav_base_link, target_link");

    ROS_INFO("[rviz_publisher] 进入 spin 循环...");
    ros::spin();
    ROS_INFO("[rviz_publisher] spin 退出");
    return 0;
}
