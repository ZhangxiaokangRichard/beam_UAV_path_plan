/**
 * @file geo.hpp
 * @brief 几何/大地坐标工具（零 ROS；**两制导节点共享**）
 *
 * - 角度规范化与"ENU 航向 ↔ 航空航向"换算
 * - 距离/方位角
 * - WGS84 ↔ ECEF ↔ ENU（算法与 1.0.1 的 ros_mqtt_bridge/geodesy、uav_guide/geodesy.py 一致）
 */

#pragma once

#include <cmath>

#include "uav_guide/types.hpp"

namespace uav_guide::geo {

constexpr double kPi = 3.14159265358979323846;

// ── 角度 ─────────────────────────────────────────────────────

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

/// 规范化到 (-π, π]
double wrap_angle(double a);

/// ENU 航向 (rad, 0=东、逆时针) → 航空航向 (deg, 0=北、顺时针, [0,360))
inline double enu_yaw_to_heading_deg(double yaw_enu_rad)
{
    double heading = 90.0 - rad2deg(yaw_enu_rad);
    heading = std::fmod(heading, 360.0);
    if (heading < 0.0) heading += 360.0;
    return heading;
}

/// 航空航向 (deg, 0=北、顺时针) → ENU 航向 (rad)
inline double heading_deg_to_enu_yaw(double heading_deg)
{
    return deg2rad(90.0 - heading_deg);
}

// ── 距离/方位 ────────────────────────────────────────────────

inline double distance_2d(double x1, double y1, double x2, double y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}

/// ENU 平面方位角 = atan2(dNorth, dEast)
inline double bearing_enu(double dx_east, double dy_north)
{
    return std::atan2(dy_north, dx_east);
}

// ── WGS84 ↔ ENU ─────────────────────────────────────────────

struct Geodetic {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;
};

struct EnuPoint {
    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
};

/// WGS84 → 以 origin 为原点的 ENU（x=East, y=North, z=Up）
EnuPoint to_enu(const GeoOrigin& origin, const Geodetic& p);

/// ENU → WGS84
Geodetic to_geodetic(const GeoOrigin& origin, const EnuPoint& p);

}  // namespace uav_guide::geo
