#pragma once

#include "ros_udp_bridge/types.h"

namespace ros_udp_bridge {

/// WGS84 经纬高 <-> 局部 ENU 米制坐标转换（自包含，不依赖外部包）。
class Geodesy {
public:
    Geodesy(double origin_latitude_deg, double origin_longitude_deg,
            double origin_altitude_m);

    LocalPose toLocal(const GeodeticPosition& position) const;
    GeodeticPosition toGeodetic(double x, double y, double z) const;

private:
    double origin_latitude_rad_;
    double origin_longitude_rad_;
    double origin_altitude_m_;
    double origin_x_;
    double origin_y_;
    double origin_z_;

    void geodeticToEcef(const GeodeticPosition& position,
                        double& x, double& y, double& z) const;
    GeodeticPosition ecefToGeodetic(double x, double y, double z) const;
};

}  // namespace ros_udp_bridge
