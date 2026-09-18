/**
 * @file uav_viz_node.cpp
 * @brief 3D 可视化节点（P7）——1.0.1 `rviz_publisher` 的 ROS2 精简版
 *
 * 保留 1.0.1 的可视化设计（命名空间/颜色/尺寸/存活期完全一致）：
 *   UAV    : ARROW  ns=uav_model   scale 30/80   橙色 (1.0, 0.45, 0.0, 1.0)，杆长 600 m
 *            SPHERE ns=uav_sphere  直径 100 m
 *   目标   : SPHERE ns=target_model 直径 120 m  红色 (1.0, 0.15, 0.15, 0.9)
 *            ARROW  ns=target_arrow scale 20/60  杆长 400 m
 *   边界   : LINE_LIST ns=env_bounds 线宽 3.0  浅灰 (0.6,0.6,0.6,0.3)，latched
 *   状态   : TEXT_VIEW_FACING scale.z 80 m，位置 (500, 4500, 950)
 *
 * 订阅：aoa/uav/state、aoa/target/state、aoa/uav/planed_path、aoa/viz/status_text
 * 发布：aoa/viz/{env_bounds,uav_model,uav_sphere,target_model,target_arrow,status,cost_text}
 *       + aoa/viz/uav_trail(nav_msgs/Path) + aoa/viz/planed_path(nav_msgs/Path)
 * 移除：障碍物 MarkerArray、/sim/status 仿真状态（1.0.1 中已废弃 / 由真实链路替代）
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "uav_guide/msg/uav_planned_path.hpp"

namespace {

// ── 参数助手 ──
double p_double(const rclcpp::Node& n, const std::string& k, double d)
{
    if (!n.has_parameter(k)) return d;
    const auto v = n.get_parameter(k);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.as_double();
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(v.as_int());
    return d;
}

std::string p_string(const rclcpp::Node& n, const std::string& k, const std::string& d)
{
    if (!n.has_parameter(k)) return d;
    const auto v = n.get_parameter(k);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return v.as_string();
    return d;
}

double yaw_from_quat(double z, double w) { return 2.0 * std::atan2(z, w); }

/// yaw → 四元数（仅偏航）
void set_yaw(geometry_msgs::msg::Quaternion& q, double yaw)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
}

// ═══════════════════════════════════════════════════════════════
// Marker 构建（尺寸/颜色与 1.0.1 完全一致）
// ═══════════════════════════════════════════════════════════════

visualization_msgs::msg::Marker make_bounds_marker(double lx, double ly, double lz,
                                                   double margin, const std::string& frame)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = "env_bounds";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 3.0;
    m.pose.orientation.w = 1.0;
    m.color.r = 0.6f;
    m.color.g = 0.6f;
    m.color.b = 0.6f;
    m.color.a = 0.3f;
    m.lifetime = rclcpp::Duration(0, 0);   // 持久（配合 latched QoS）

    const double x0 = margin, x1 = lx - margin;
    const double y0 = margin, y1 = ly - margin;
    const double z0 = margin, z1 = lz - margin;

    auto add_line = [&m](double xa, double ya, double za, double xb, double yb, double zb) {
        geometry_msgs::msg::Point p;
        p.x = xa; p.y = ya; p.z = za; m.points.push_back(p);
        p.x = xb; p.y = yb; p.z = zb; m.points.push_back(p);
    };

    add_line(x0, y0, z0, x1, y0, z0);
    add_line(x1, y0, z0, x1, y1, z0);
    add_line(x1, y1, z0, x0, y1, z0);
    add_line(x0, y1, z0, x0, y0, z0);
    add_line(x0, y0, z1, x1, y0, z1);
    add_line(x1, y0, z1, x1, y1, z1);
    add_line(x1, y1, z1, x0, y1, z1);
    add_line(x0, y1, z1, x0, y0, z1);
    add_line(x0, y0, z0, x0, y0, z1);
    add_line(x1, y0, z0, x1, y0, z1);
    add_line(x1, y1, z0, x1, y1, z1);
    add_line(x0, y1, z0, x0, y1, z1);
    return m;
}

/// 箭头：从 (x,y,z) 沿 yaw 延伸 len
visualization_msgs::msg::Marker make_arrow_marker(const std::string& frame, const std::string& ns,
                                                  double x, double y, double z, double yaw,
                                                  double len, double shaft_dia, double head_dia,
                                                  float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.points.resize(2);
    m.points[0].x = x;
    m.points[0].y = y;
    m.points[0].z = z;
    m.points[1].x = x + len * std::cos(yaw);
    m.points[1].y = y + len * std::sin(yaw);
    m.points[1].z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = shaft_dia;
    m.scale.y = head_dia;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    return m;
}

visualization_msgs::msg::Marker make_sphere_marker(const std::string& frame, const std::string& ns,
                                                   double x, double y, double z, double dia,
                                                   float r, float g, float b, float a)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = dia; m.scale.y = dia; m.scale.z = dia;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    return m;
}

visualization_msgs::msg::Marker make_text_marker(const std::string& frame, const std::string& ns,
                                                 const std::string& text, double x, double y,
                                                 double z, double height, float r, float g, float b,
                                                 float a, double lifetime_s)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.z = height;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.text = text;
    m.lifetime = rclcpp::Duration::from_seconds(lifetime_s);
    return m;
}

}  // namespace

namespace {

class UavVizNode : public rclcpp::Node {
public:
    UavVizNode()
        : rclcpp::Node("uav_viz_node",
                       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        frame_ = p_string(*this, "frame_id", "map");

        // 空间边界（与 scenario.yaml 同源）
        lx_ = p_double(*this, "space.lx", 10000.0);
        ly_ = p_double(*this, "space.ly", 5000.0);
        lz_ = p_double(*this, "space.lz", 1000.0);
        margin_ = p_double(*this, "space.boundary_margin", 50.0);

        // 无数据时的默认位置（与 1.0.1 参数同源）
        default_uav_ = {p_double(*this, "viz.default_uav_x", 2000.0),
                        p_double(*this, "viz.default_uav_y", 1000.0),
                        p_double(*this, "viz.default_uav_z", 500.0),
                        p_double(*this, "viz.default_uav_yaw", 1.5707963267948966)};
        default_target_ = {p_double(*this, "viz.default_target_x", 4000.0),
                           p_double(*this, "viz.default_target_y", 2500.0),
                           p_double(*this, "viz.default_target_z", 600.0),
                           p_double(*this, "viz.default_target_yaw", 1.5707963267948966)};

        model_hz_ = p_double(*this, "viz.model_rate_hz", 5.0);
        status_hz_ = p_double(*this, "viz.status_rate_hz", 1.0);
        trail_max_ = static_cast<std::size_t>(p_double(*this, "viz.trail_max_points", 2000.0));
        trail_step_m_ = p_double(*this, "viz.trail_min_step_m", 20.0);
        status_text_ = p_string(*this, "viz.status_text", "Beam Dubins 2.0");

        const std::string uav_state_topic = p_string(*this, "topics.uav_state", "aoa/uav/state");
        const std::string target_state_topic =
            p_string(*this, "topics.target_state", "aoa/target/state");
        const std::string planed_path_topic =
            p_string(*this, "topics.planed_path", "aoa/uav/planed_path");
        const std::string status_in_topic =
            p_string(*this, "topics.status_text", "aoa/viz/status_text");

        // ── 订阅 ──
        rclcpp::QoS latched(rclcpp::KeepLast(1));
        latched.transient_local().reliable();

        sub_uav_ = create_subscription<nav_msgs::msg::Odometry>(
            uav_state_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                uav_ = *msg;
                has_uav_ = true;
                appendTrail(msg->pose.pose.position.x, msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
            });
        sub_target_ = create_subscription<nav_msgs::msg::Odometry>(
            target_state_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                target_ = *msg;
                has_target_ = true;
            });
        // 规划路径：订阅侧用 volatile（兼容 transient_local 与 volatile 发布者，
        // 例如 loop 节点的 latching 发布与 `ros2 topic pub` 手注）
        sub_plan_ = create_subscription<uav_guide::msg::UavPlannedPath>(
            planed_path_topic, rclcpp::QoS(10),
            [this](uav_guide::msg::UavPlannedPath::SharedPtr msg) { onPlannedPath(*msg); });
        sub_status_ = create_subscription<std_msgs::msg::String>(
            status_in_topic, rclcpp::QoS(10),
            [this](std_msgs::msg::String::SharedPtr msg) { status_text_ = msg->data; });

        // ── 发布 ──
        pub_bounds_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.env_bounds", "aoa/viz/env_bounds"), latched);
        pub_uav_model_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.uav_model", "aoa/viz/uav_model"), 10);
        pub_uav_sphere_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.uav_sphere", "aoa/viz/uav_sphere"), 10);
        pub_target_model_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.target_model", "aoa/viz/target_model"), 10);
        pub_target_arrow_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.target_arrow", "aoa/viz/target_arrow"), 10);
        pub_status_text_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.status", "aoa/viz/status"), 10);
        pub_plan_text_ = create_publisher<visualization_msgs::msg::Marker>(
            p_string(*this, "topics.plan_text", "aoa/viz/plan_text"), 10);
        pub_trail_ = create_publisher<nav_msgs::msg::Path>(
            p_string(*this, "topics.uav_trail", "aoa/viz/uav_trail"), latched);
        pub_plan_path_ = create_publisher<nav_msgs::msg::Path>(
            p_string(*this, "topics.planed_path_vis", "aoa/viz/planed_path"), latched);

        // 边界只发一次（latched）
        auto bounds = make_bounds_marker(lx_, ly_, lz_, margin_, frame_);
        bounds.header.stamp = now();
        pub_bounds_->publish(bounds);

        // ── 定时器 ──
        const auto period = [](double hz, std::int64_t floor_ms) {
            return std::chrono::milliseconds(
                std::max<std::int64_t>(floor_ms, static_cast<std::int64_t>(1000.0 / std::max(0.1, hz))));
        };
        model_timer_ = create_wall_timer(period(model_hz_, 50), [this]() { publishModels(); });
        status_timer_ = create_wall_timer(period(status_hz_, 200), [this]() { publishTexts(); });

        RCLCPP_INFO(get_logger(),
                    "[uav_viz_node] ready: frame=%s space=%.0f×%.0f×%.0f margin=%.0f "
                    "model=%.1fHz status=%.1fHz (1.0.1 marker 设计已保留)",
                    frame_.c_str(), lx_, ly_, lz_, margin_, model_hz_, status_hz_);
    }

private:
    // ── 航迹 ──
    void appendTrail(double x, double y, double z)
    {
        if (!trail_.empty()) {
            const auto& last = trail_.back().pose.position;
            const double d = std::hypot(x - last.x, y - last.y);
            if (d < trail_step_m_) return;
        }
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_;
        ps.header.stamp = now();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        set_yaw(ps.pose.orientation, yaw_from_quat(uav_.pose.pose.orientation.z,
                                                  uav_.pose.pose.orientation.w));
        trail_.push_back(ps);
        if (trail_.size() > trail_max_) trail_.erase(trail_.begin(), trail_.begin() + trail_max_ / 10);
        trail_dirty_ = true;
    }

    void publishTrail()
    {
        if (!trail_dirty_) return;
        nav_msgs::msg::Path path;
        path.header.frame_id = frame_;
        path.header.stamp = now();
        path.poses = trail_;
        pub_trail_->publish(path);
        trail_dirty_ = false;
    }

    // ── 规划路径（uav_guide/UavPlannedPath → nav_msgs/Path）──
    void onPlannedPath(const uav_guide::msg::UavPlannedPath& msg)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = msg.header.stamp;
        path.header.frame_id = msg.header.frame_id.empty() ? frame_ : msg.header.frame_id;
        path.poses.reserve(msg.path.size());
        for (const auto& p : msg.path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            ps.pose.position.z = p.z;
            set_yaw(ps.pose.orientation, p.yaw);
            path.poses.push_back(ps);
        }
        pub_plan_path_->publish(path);

        std::ostringstream oss;
        oss << "plan " << (msg.success ? "OK" : "FAIL")
            << "  cost=" << static_cast<long>(msg.cost) << " m"
            << "  pts=" << msg.path.size();
        if (!msg.success && !msg.status_message.empty()) oss << "  (" << msg.status_message << ")";
        plan_text_ = oss.str();
        RCLCPP_INFO(get_logger(), "[uav_viz_node] %s → %s (poses=%zu)", oss.str().c_str(),
                    pub_plan_path_->get_topic_name(), path.poses.size());
    }

    // ── 5 Hz：UAV / 目标 3D 模型 ──
    void publishModels()
    {
        const double ux = has_uav_ ? uav_.pose.pose.position.x : default_uav_[0];
        const double uy = has_uav_ ? uav_.pose.pose.position.y : default_uav_[1];
        const double uz = has_uav_ ? uav_.pose.pose.position.z : default_uav_[2];
        const double uyaw = has_uav_ ? yaw_from_quat(uav_.pose.pose.orientation.z,
                                                     uav_.pose.pose.orientation.w)
                                     : default_uav_[3];
        const double tx = has_target_ ? target_.pose.pose.position.x : default_target_[0];
        const double ty = has_target_ ? target_.pose.pose.position.y : default_target_[1];
        const double tz = has_target_ ? target_.pose.pose.position.z : default_target_[2];
        const double tyaw = has_target_ ? yaw_from_quat(target_.pose.pose.orientation.z,
                                                        target_.pose.pose.orientation.w)
                                        : default_target_[3];

        // UAV：橙色箭头（600 m，scale 30/80）+ 100 m 球
        auto uav_arrow = make_arrow_marker(frame_, "uav_model", ux, uy, uz, uyaw, 600.0, 30.0, 80.0,
                                           1.0f, 0.45f, 0.0f, 1.0f);
        uav_arrow.header.stamp = now();
        auto uav_sphere = make_sphere_marker(frame_, "uav_sphere", ux, uy, uz, 100.0, 1.0f, 0.45f,
                                             0.0f, 1.0f);
        uav_sphere.header.stamp = now();
        pub_uav_model_->publish(uav_arrow);
        pub_uav_sphere_->publish(uav_sphere);

        // 目标：红色球（120 m）+ 400 m 箭头（scale 20/60）
        auto tgt_sphere = make_sphere_marker(frame_, "target_model", tx, ty, tz, 120.0, 1.0f, 0.15f,
                                             0.15f, 0.9f);
        tgt_sphere.header.stamp = now();
        auto tgt_arrow = make_arrow_marker(frame_, "target_arrow", tx, ty, tz, tyaw, 400.0, 20.0,
                                           60.0, 1.0f, 0.15f, 0.15f, 1.0f);
        tgt_arrow.header.stamp = now();
        pub_target_model_->publish(tgt_sphere);
        pub_target_arrow_->publish(tgt_arrow);

        publishTrail();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "[uav_viz_node] has_uav=%d uav=(%.0f,%.0f,%.0f) has_target=%d "
                             "target=(%.0f,%.0f,%.0f) trail=%zu",
                             static_cast<int>(has_uav_), ux, uy, uz, static_cast<int>(has_target_),
                             tx, ty, tz, trail_.size());
    }

    // ── 1 Hz：状态文本 / 规划摘要文本 ──
    void publishTexts()
    {
        std::ostringstream oss;
        oss << status_text_;
        if (has_uav_) oss << "   UAV (" << static_cast<long>(uav_.pose.pose.position.x) << ","
                          << static_cast<long>(uav_.pose.pose.position.y) << ","
                          << static_cast<long>(uav_.pose.pose.position.z) << ")";
        if (has_target_) oss << "   目标 (" << static_cast<long>(target_.pose.pose.position.x) << ","
                             << static_cast<long>(target_.pose.pose.position.y) << ","
                             << static_cast<long>(target_.pose.pose.position.z) << ")";

        auto text = make_text_marker(frame_, "sim_status", oss.str(), 500.0, 4500.0, 950.0, 80.0,
                                     1.0f, 1.0f, 1.0f, 0.9f, 2.0);
        text.header.stamp = now();
        pub_status_text_->publish(text);

        if (!plan_text_.empty()) {
            auto plan_text = make_text_marker(frame_, "plan_text", plan_text_, 500.0, 4500.0, 800.0,
                                              60.0, 1.0f, 0.9f, 0.2f, 0.9f, 3.0);
            plan_text.header.stamp = now();
            pub_plan_text_->publish(plan_text);
        }
    }

    // ── 成员 ──
    std::string frame_ = "map";
    double lx_ = 10000.0, ly_ = 5000.0, lz_ = 1000.0, margin_ = 50.0;
    std::vector<double> default_uav_{2000.0, 1000.0, 500.0, 1.5707963267948966};
    std::vector<double> default_target_{4000.0, 2500.0, 600.0, 1.5707963267948966};
    double model_hz_ = 5.0, status_hz_ = 1.0, trail_step_m_ = 20.0;
    std::size_t trail_max_ = 2000;
    std::string status_text_, plan_text_;

    nav_msgs::msg::Odometry uav_, target_;
    bool has_uav_ = false, has_target_ = false;
    std::vector<geometry_msgs::msg::PoseStamped> trail_;
    bool trail_dirty_ = false;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_bounds_, pub_uav_model_,
        pub_uav_sphere_, pub_target_model_, pub_target_arrow_, pub_status_text_, pub_plan_text_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_trail_, pub_plan_path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_uav_, sub_target_;
    rclcpp::Subscription<uav_guide::msg::UavPlannedPath>::SharedPtr sub_plan_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
    rclcpp::TimerBase::SharedPtr model_timer_, status_timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<UavVizNode>());
    } catch (const std::exception& ex) {
        RCLCPP_FATAL(rclcpp::get_logger("uav_viz_node"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
