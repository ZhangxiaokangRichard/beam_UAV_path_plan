/**
 * @file ros_utils.cpp
 * @brief 节点侧复用层实现
 */

#include "uav_guide/ros_utils.hpp"

#include "uav_guide/mode.hpp"
#include "uav_guide/state.hpp"

namespace uav_guide::ros_utils {

// ── 参数读取 ─────────────────────────────────────────────────

double get_double(const rclcpp::Node& node, const std::string& name, double fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return value.as_double();
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return static_cast<double>(value.as_int());
    return fallback;
}

std::int64_t get_int(const rclcpp::Node& node, const std::string& name, std::int64_t fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return value.as_int();
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        return static_cast<std::int64_t>(value.as_double());
    return fallback;
}

std::string get_string(const rclcpp::Node& node, const std::string& name,
                       const std::string& fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return value.as_string();
    return fallback;
}

bool get_bool(const rclcpp::Node& node, const std::string& name, bool fallback)
{
    if (!node.has_parameter(name)) return fallback;
    const auto value = node.get_parameter(name);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) return value.as_bool();
    return fallback;
}

// ── 配置装配 ─────────────────────────────────────────────────

GeoOrigin read_origin(const rclcpp::Node& node)
{
    GeoOrigin origin;
    origin.lat_deg = get_double(node, "geodesy.origin_latitude_deg", 0.0);
    origin.lon_deg = get_double(node, "geodesy.origin_longitude_deg", 0.0);
    origin.alt_m = get_double(node, "geodesy.origin_altitude_m", 0.0);
    return origin;
}

void read_setpoint_config(const rclcpp::Node& node, SetpointConfig& cfg)
{
    cfg.curvature_eps = get_double(node, "segmentation.curvature_eps", cfg.curvature_eps);
    cfg.yaw_segment_eps = get_double(node, "segmentation.yaw_segment_eps", cfg.yaw_segment_eps);
    cfg.default_turn_radius_m =
        get_double(node, "segmentation.default_turn_radius_m", cfg.default_turn_radius_m);
    cfg.offset_m = get_double(node, "finalshot.offset_m", cfg.offset_m);
    cfg.offset_m_t = get_double(node, "finalshot.offset_m_t", cfg.offset_m_t);
    cfg.offset_radius_m = get_double(node, "finalshot.radius_m", cfg.offset_radius_m);
}

void read_guidance_config(const rclcpp::Node& node, GuidanceConfig& cfg)
{
    cfg.curvature_eps = get_double(node, "guidance.curvature_eps", cfg.curvature_eps);
    cfg.fly_speed_mps = get_double(node, "guidance.fly_speed_mps", cfg.fly_speed_mps);

    const std::string sample_rule =
        get_string(node, "guidance.sample_rule", "first_zero_curvature");
    cfg.sample_rule = (sample_rule == "first_nonzero_curvature")
                          ? SampleRule::FirstNonzeroCurvature
                          : SampleRule::FirstZeroCurvature;

    const std::string heads_source = get_string(node, "guidance.heads_source", "sample_yaw");
    cfg.heads_source = (heads_source == "bearing_to_sample") ? HeadsSource::BearingToSample
                                                             : HeadsSource::SampleYaw;

    const std::string height_source = get_string(node, "guidance.height_source", "sample_point_z");
    cfg.height_source = (height_source == "target_z") ? HeightSource::TargetZ
                                                      : HeightSource::SamplePointZ;
}

void read_orchestrator_config(const rclcpp::Node& node, OrchestratorConfig& cfg)
{
    cfg.approach_distance_tail =
        get_double(node, "target.approach_distance_tail", cfg.approach_distance_tail);
    cfg.approach_distance_headon =
        get_double(node, "target.approach_distance_headon", cfg.approach_distance_headon);
    cfg.tracking_entry_dist = get_double(node, "target.tracking_entry_dist", cfg.tracking_entry_dist);
    cfg.tracking_window = get_double(node, "target.tracking_window", cfg.tracking_window);
    cfg.velocity_align_deg = get_double(node, "target.velocity_align_deg", cfg.velocity_align_deg);
    cfg.min_closure_m = get_double(node, "target.min_closure_m", cfg.min_closure_m);
    cfg.min_closure_m_tail = get_double(node, "target.min_closure_m_tail", cfg.min_closure_m_tail);
    cfg.divergence_threshold = static_cast<int>(
        get_int(node, "target.divergence_threshold", cfg.divergence_threshold));
    cfg.entry_hold_count =
        static_cast<int>(get_int(node, "target.entry_hold_count", cfg.entry_hold_count));
}

void read_space_config(const rclcpp::Node& node, SpaceConfig& cfg)
{
    cfg.lx = get_double(node, "space.lx", cfg.lx);
    cfg.ly = get_double(node, "space.ly", cfg.ly);
    cfg.lz = get_double(node, "space.lz", cfg.lz);
    cfg.boundary_margin = get_double(node, "space.boundary_margin", cfg.boundary_margin);
}

void read_plan_params(const rclcpp::Node& node, PlanParams& params)
{
    read_space_config(node, params.space);
    params.time_limit_ms = get_double(node, "planning.time_limit_ms", params.time_limit_ms);
    params.use_3d = get_bool(node, "planning.use_3d", params.use_3d);
    params.frame_id = get_string(node, "planning.frame_id", params.frame_id);
}

// ── 状态/路径转换 ────────────────────────────────────────────

State5 to_state5(const nav_msgs::msg::Odometry& odom)
{
    return state::from_odometry(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                odom.pose.pose.position.z,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z,
                                odom.pose.pose.orientation.w);
}

std::vector<PathPoint> to_path_points(const std::vector<beam_dubins::msg::PathPoint>& msg_path)
{
    std::vector<PathPoint> out;
    out.reserve(msg_path.size());
    for (const auto& p : msg_path) {
        PathPoint pp;
        pp.x = p.x;
        pp.y = p.y;
        pp.z = p.z;
        pp.yaw = p.yaw;
        pp.pitch = p.pitch;
        pp.curvature = p.curvature;
        out.push_back(pp);
    }
    return out;
}

// ── 输出→消息 ────────────────────────────────────────────────

msg::Setpoint make_setpoint_msg(const SetpointOutput& out, const std_msgs::msg::Header& header)
{
    msg::Setpoint msg;
    msg.header = header;
    msg.target_longitude = out.longitude_deg;
    msg.target_latitude = out.latitude_deg;
    msg.target_altitude = out.altitude_m;
    msg.target_radius = out.radius_m;
    return msg;
}

msg::Guidance make_guidance_msg(const GuidanceOutput& out, const std_msgs::msg::Header& header)
{
    msg::Guidance msg;
    msg.header = header;
    msg.fly_height = out.fly_height;
    msg.heads = out.heads_deg;
    msg.fly_speed = out.fly_speed_mps;
    return msg;
}

}  // namespace uav_guide::ros_utils
