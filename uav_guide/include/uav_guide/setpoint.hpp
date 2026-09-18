/**
 * @file setpoint.hpp
 * @brief 指点（setpoint）制导：路径几何 → WGS84 引导点（零 ROS；**uav_guide_point_node 独有**）
 *
 * 语义与 1.0.1 的 /uav/guide_point 完全一致：
 *  - 存在未飞过弧段 → 沿航路**最远**剩余弧段的**圆心**（radius = ±R）
 *  - 无剩余弧段 / 全直线 / 无路径 → 目标**偏移引导点**
 *      head-on：目标身后；tail：目标前方
 */

#pragma once

#include <vector>

#include "uav_guide/types.hpp"

namespace uav_guide::setpoint {

/// 偏移引导点的 map 平面坐标（head-on 身后 / tail 前方）
void offset_goal_xy(const State5& target, InterceptMode mode,
                    double offset_m, double offset_m_t,
                    double& x, double& y);

/// 引导点解算（→ Setpoint.msg）
SetpointOutput build(const std::vector<PathPoint>& path,
                     const State5& uav,
                     const State5& target,
                     InterceptMode mode,
                     const GeoOrigin& origin,
                     const SetpointConfig& cfg);

}  // namespace uav_guide::setpoint
