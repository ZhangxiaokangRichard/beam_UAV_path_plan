/**
 * @file geodesy.cpp
 * @brief WGS84 经纬高与局部 ENU 米制坐标之间的转换。
 *
 * 规划器和 TF 只使用 map 局部坐标：x=East、y=North、z=Up。
 * 经纬度不能直接作为规划坐标，因此这里统一经过 ECEF 中间坐标转换。
 */

#include "ros_mqtt_bridge/geodesy.h"

#include <cmath>

namespace ros_mqtt_bridge {
namespace {
// WGS84 椭球参数：长半轴和第一偏心率平方。
constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0;
constexpr double kE2 = 6.6943799901413165e-3;
double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double value) { return value * 180.0 / kPi; }
}

Geodesy::Geodesy(double latitude_deg, double longitude_deg, double altitude_m)
    : origin_latitude_rad_(radians(latitude_deg))
    , origin_longitude_rad_(radians(longitude_deg))
    , origin_altitude_m_(altitude_m)
    , origin_x_(0.0)
    , origin_y_(0.0)
    , origin_z_(0.0)
{
    // 原点的 ECEF 坐标只计算一次，后续正逆变换都围绕该原点进行。
    geodeticToEcef({latitude_deg, longitude_deg, altitude_m}, origin_x_, origin_y_, origin_z_);
}

void Geodesy::geodeticToEcef(const GeodeticPosition& position,
                             double& x, double& y, double& z) const
{
    // WGS84 经纬高 -> 地心地固坐标 ECEF。
    const double lat = radians(position.latitude_deg);
    const double lon = radians(position.longitude_deg);
    const double sin_lat = std::sin(lat);
    const double radius = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
    x = (radius + position.altitude_m) * std::cos(lat) * std::cos(lon);
    y = (radius + position.altitude_m) * std::cos(lat) * std::sin(lon);
    z = (radius * (1.0 - kE2) + position.altitude_m) * sin_lat;
}

LocalPose Geodesy::toLocal(const GeodeticPosition& position) const
{
    // 先计算目标与原点的 ECEF 差值，再旋转到原点切平面 ENU。
    double x, y, z;
    geodeticToEcef(position, x, y, z);
    const double dx = x - origin_x_;
    const double dy = y - origin_y_;
    const double dz = z - origin_z_;
    const double sin_lat = std::sin(origin_latitude_rad_);
    const double cos_lat = std::cos(origin_latitude_rad_);
    const double sin_lon = std::sin(origin_longitude_rad_);
    const double cos_lon = std::cos(origin_longitude_rad_);
    // ROS map 坐标约定：x=东、y=北、z=上；姿态字段由调用方补入。
    return {
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
        0.0, 0.0};
}

GeodeticPosition Geodesy::ecefToGeodetic(double x, double y, double z) const
{
    // ECEF -> WGS84。纬度和高度使用迭代法收敛，固定次数足够满足局部航点输出精度。
    const double longitude = std::atan2(y, x);
    const double p = std::sqrt(x * x + y * y);
    double latitude = std::atan2(z, p * (1.0 - kE2));
    double height = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double sin_lat = std::sin(latitude);
        const double radius = kA / std::sqrt(1.0 - kE2 * sin_lat * sin_lat);
        height = p / std::cos(latitude) - radius;
        latitude = std::atan2(z, p * (1.0 - kE2 * radius / (radius + height)));
    }
    return {degrees(latitude), degrees(longitude), height};
}

GeodeticPosition Geodesy::toGeodetic(double x, double y, double z) const
{
    // map ENU -> ECEF，再转换为地面站需要的经纬高。
    const double sin_lat = std::sin(origin_latitude_rad_);
    const double cos_lat = std::cos(origin_latitude_rad_);
    const double sin_lon = std::sin(origin_longitude_rad_);
    const double cos_lon = std::cos(origin_longitude_rad_);
    const double dx = -sin_lon * x - sin_lat * cos_lon * y + cos_lat * cos_lon * z;
    const double dy = cos_lon * x - sin_lat * sin_lon * y + cos_lat * sin_lon * z;
    const double dz = cos_lat * y + sin_lat * z;
    return ecefToGeodetic(origin_x_ + dx, origin_y_ + dy, origin_z_ + dz);
}

}  // namespace ros_mqtt_bridge