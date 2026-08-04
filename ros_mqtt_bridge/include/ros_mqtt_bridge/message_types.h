#pragma once

#include <cstdint>
#include <string>

namespace ros_mqtt_bridge {

struct GeodeticPosition {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

struct LocalPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw   = 0.0;   // ENU 约定 (0=东, π/2=北), rad
    double pitch = 0.0;
    double roll  = 0.0;
};

struct TargetState {
    std::uint32_t target_id = 0;     // 由 uav_sn 哈希生成
    std::string   uav_sn;            // UAV 序列号，如 "V2.4AOACRAFT-SIL3NF0"
    std::string   mode;              // 飞行模式，如 "Auto"
    std::uint32_t sequence = 0;
    std::string   mid;               // 地面站消息唯一标识
    std::string   timestamp;
    GeodeticPosition geodetic;
    LocalPose local;
    double east_mps = 0.0;
    double north_mps = 0.0;
    double up_mps = 0.0;
    double ground_speed_mps = 0.0;   // vvg：地速
    double valid_for_ms = 500.0;
};

struct Waypoint {
    std::uint32_t route_id = 0;
    std::uint32_t sequence = 0;
    std::uint32_t wp_id = 0;
    std::uint32_t total = 0;
    std::string timestamp;
    LocalPose local;
    GeodeticPosition geodetic;
    double curvature_1pm = 0.0;
    double speed_mps = 0.0;
};

/// 用于 MQTT 状态上报的 UAV 位姿快照（取自 nav_msgs/Odometry）。
struct UavStateJson {
    double px = 0.0, py = 0.0, pz = 0.0;            // pose.pose.position
    double ox = 0.0, oy = 0.0, oz = 0.0, ow = 1.0;  // pose.pose.orientation (四元数)
    double vx = 0.0, vy = 0.0, vz = 0.0;            // twist.twist.linear
};

}  // namespace ros_mqtt_bridge