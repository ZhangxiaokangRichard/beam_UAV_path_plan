/**
 * @file geo.cpp
 * @brief 几何/大地坐标实现（算法与 1.0.1 的 geodesy 一致）
 */

#include "uav_guide/geo.hpp"

namespace uav_guide::geo {
namespace {

// WGS84 椭球参数
constexpr double kA = 6378137.0;
constexpr double kE2 = 6.6943799901413165e-3;

void geodetic_to_ecef(double lat_deg, double lon_deg, double alt_m,
                      double& x, double& y, double& z)
{
    const double lat = deg2rad(lat_deg);
    const double lon = deg2rad(lon_deg);
    const double sin_lat = std::sin(lat);
    const double radius = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
    x = (radius + alt_m) * std::cos(lat) * std::cos(lon);
    y = (radius + alt_m) * std::cos(lat) * std::sin(lon);
    z = (radius * (1.0 - kE2) + alt_m) * sin_lat;
}

Geodetic ecef_to_geodetic(double x, double y, double z)
{
    const double longitude = std::atan2(y, x);
    const double p = std::hypot(x, y);
    double latitude = std::atan2(z, p * (1.0 - kE2));
    double height = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double sin_lat = std::sin(latitude);
        const double radius = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
        height = p / std::cos(latitude) - radius;
        latitude = std::atan2(z, p * (1.0 - kE2 * radius / (radius + height)));
    }
    Geodetic out;
    out.lat_deg = rad2deg(latitude);
    out.lon_deg = rad2deg(longitude);
    out.alt_m = height;
    return out;
}

void origin_ecef(const GeoOrigin& origin, double& ox, double& oy, double& oz)
{
    geodetic_to_ecef(origin.lat_deg, origin.lon_deg, origin.alt_m, ox, oy, oz);
}

}  // namespace

double wrap_angle(double a)
{
    const double two_pi = 2.0 * kPi;
    double r = std::fmod(a + kPi, two_pi);
    if (r < 0.0) r += two_pi;
    return r - kPi;
}

EnuPoint to_enu(const GeoOrigin& origin, const Geodetic& p)
{
    double ox = 0.0, oy = 0.0, oz = 0.0;
    origin_ecef(origin, ox, oy, oz);

    double x = 0.0, y = 0.0, z = 0.0;
    geodetic_to_ecef(p.lat_deg, p.lon_deg, p.alt_m, x, y, z);

    const double dx = x - ox;
    const double dy = y - oy;
    const double dz = z - oz;

    const double lat = deg2rad(origin.lat_deg);
    const double lon = deg2rad(origin.lon_deg);
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    EnuPoint out;
    out.east = -sin_lon * dx + cos_lon * dy;
    out.north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    out.up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
    return out;
}

Geodetic to_geodetic(const GeoOrigin& origin, const EnuPoint& p)
{
    double ox = 0.0, oy = 0.0, oz = 0.0;
    origin_ecef(origin, ox, oy, oz);

    const double lat = deg2rad(origin.lat_deg);
    const double lon = deg2rad(origin.lon_deg);
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    // ENU → ECEF 增量（旋转矩阵转置）
    const double dx = -sin_lon * p.east - sin_lat * cos_lon * p.north + cos_lat * cos_lon * p.up;
    const double dy = cos_lon * p.east - sin_lat * sin_lon * p.north + cos_lat * sin_lon * p.up;
    const double dz = cos_lat * p.north + sin_lat * p.up;

    return ecef_to_geodetic(ox + dx, oy + dy, oz + dz);
}

}  // namespace uav_guide::geo
