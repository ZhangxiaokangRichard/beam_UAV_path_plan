/**
 * @file guidance.hpp
 * @brief 巡航（cruise）制导：路径几何 → 航向/高度/速度（零 ROS；**uav_guidance_node 独有**）
 *
 * 采样点规则（Q12/Q16 裁决，默认）：自本机沿 planed_path 起，
 * 取**首个曲率为 0 的点**（即首个直线段起点），其 ENU 航向即巡航目标航向。
 */

#pragma once

#include <vector>

#include "uav_guide/types.hpp"

namespace uav_guide::guidance {

/// 制导解算（→ Guidance.msg）
GuidanceOutput build(const std::vector<PathPoint>& path,
                     const State5& uav,
                     const State5& target,
                     const GuidanceConfig& cfg);

}  // namespace uav_guide::guidance
