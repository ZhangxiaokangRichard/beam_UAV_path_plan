#pragma once

namespace ros_udp_bridge {

/// WGS84 经纬高。
struct GeodeticPosition {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

/// 局部 ENU 坐标（map：x=东、y=北、z=上）与姿态（弧度，ENU 约定）。
struct LocalPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};

}  // namespace ros_udp_bridge
