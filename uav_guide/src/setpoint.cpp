/**
 * @file setpoint.cpp
 * @brief 指点制导解算（移植 1.0.1 的 guide_point_builder.py）
 */

#include "uav_guide/setpoint.hpp"

#include <cmath>

#include "uav_guide/geo.hpp"
#include "uav_guide/path.hpp"

namespace uav_guide::setpoint {
namespace {

/// 目标实时高度（WGS84 椭球高）
double target_altitude_wgs84(const GeoOrigin& origin, const State5& target)
{
    const geo::EnuPoint enu{target.x, target.y, target.z};
    return geo::to_geodetic(origin, enu).alt_m;
}

SetpointOutput offset_guide(const State5& target, InterceptMode mode,
                            const GeoOrigin& origin, const SetpointConfig& cfg)
{
    double x = 0.0;
    double y = 0.0;
    offset_goal_xy(target, mode, cfg.offset_m, cfg.offset_m_t, x, y);

    const auto geodetic = geo::to_geodetic(origin, geo::EnuPoint{x, y, target.z});
    SetpointOutput out;
    out.longitude_deg = geodetic.lon_deg;
    out.latitude_deg = geodetic.lat_deg;
    out.altitude_m = target_altitude_wgs84(origin, target);
    out.radius_m = cfg.offset_radius_m;
    return out;
}

}  // namespace

void offset_goal_xy(const State5& target, InterceptMode mode,
                    double offset_m, double offset_m_t,
                    double& x, double& y)
{
    const double psi = target.yaw;
    if (mode == InterceptMode::HeadOn) {
        // 目标**身后**（与 1.0.1 一致：0.4·offset_m_t 的加权）
        const double d = offset_m + 0.4 * offset_m_t;
        x = target.x - d * std::cos(psi);
        y = target.y - d * std::sin(psi);
    } else {
        // 目标**前方**
        const double d = offset_m + offset_m_t;
        x = target.x + d * std::cos(psi);
        y = target.y + d * std::sin(psi);
    }
}

SetpointOutput build(const std::vector<PathPoint>& path,
                     const State5& uav,
                     const State5& target,
                     InterceptMode mode,
                     const GeoOrigin& origin,
                     const SetpointConfig& cfg)
{
    // 无本机状态 / 无路径 → 偏移引导点
    if (!uav.valid || path.empty()) {
        return offset_guide(target, mode, origin, cfg);
    }

    const auto segments = path::segment(path, cfg.curvature_eps,
                                        cfg.default_turn_radius_m, cfg.yaw_segment_eps);
    const std::size_t ptr = path::closest_index(path, uav.x, uav.y);
    const auto arc = path::farthest_remaining_arc(segments, ptr);

    // 无剩余弧段（末段直线 / 全直线）→ 偏移引导点
    if (!arc.has_value()) {
        return offset_guide(target, mode, origin, cfg);
    }

    // 最远剩余弧段的圆心
    const auto& p0 = path[arc->start_index];
    const auto center = path::arc_center_xy(p0.x, p0.y, p0.yaw, arc->radius_m, arc->is_left);
    const auto geodetic = geo::to_geodetic(origin, geo::EnuPoint{center.first, center.second, p0.z});

    SetpointOutput out;
    out.longitude_deg = geodetic.lon_deg;
    out.latitude_deg = geodetic.lat_deg;
    out.altitude_m = target_altitude_wgs84(origin, target);
    out.radius_m = arc->is_left ? -arc->radius_m : arc->radius_m;  // 左转负 / 右转正
    return out;
}

}  // namespace uav_guide::setpoint
