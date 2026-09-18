/**
 * @file state.hpp
 * @brief 5D 状态提取（零 ROS；两制导节点共享）
 *
 * 输入为原始标量（由节点从 nav_msgs/Odometry 取值后传入），保持算法层零 ROS 依赖。
 */

#pragma once

#include <cmath>

#include "uav_guide/types.hpp"

namespace uav_guide::state {

/// 四元数 → ENU 偏航角（rad，0=东、逆时针）
inline double quaternion_to_yaw_enu(double qx, double qy, double qz, double qw)
{
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

/// 由位姿/姿态构造 State5（yaw 可为四元数或直接给定）
inline State5 make(double x, double y, double z, double yaw_enu_rad, double pitch_rad = 0.0)
{
    State5 s;
    s.x = x;
    s.y = y;
    s.z = z;
    s.yaw = yaw_enu_rad;
    s.pitch = pitch_rad;
    s.valid = true;
    return s;
}

/// 由 Odometry 原始字段构造（四元数取 yaw）
inline State5 from_odometry(double px, double py, double pz,
                            double qx, double qy, double qz, double qw,
                            double pitch_rad = 0.0)
{
    return make(px, py, pz, quaternion_to_yaw_enu(qx, qy, qz, qw), pitch_rad);
}

}  // namespace uav_guide::state
