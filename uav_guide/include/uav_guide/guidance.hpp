/**
 * @file guidance.hpp
 * @brief 巡航（cruise）制导：路径几何 → 航向/高度/速度（零 ROS；**uav_guidance_node 独有**）
 *
 * heads 取法（`guidance.heads_source`）：
 *   - `sample_yaw` / `bearing_to_sample`：采样点规则（Q12/Q16 裁决）——自本机沿路径扫
 *     **首个曲率为 0 的点**（首个直线段起点），取其航向；
 *   - `l1`：**L1 路径跟踪**——沿路径在前视距离 L1 处取参考点，偏差角 β 决定期望曲率
 *     `κ = 2·sinβ/L1`（按盘旋能力 `1/R_c` 饱和），输出 `ψ_cmd = ψ + κ·V·T_lead`。
 *     参考点距离恒定、指令连续、前方 L1 内的转弯可见。设计见 `doc/L1_tracking_plan.md`。
 */

#pragma once

#include <cstddef>
#include <vector>

#include "uav_guide/path.hpp"
#include "uav_guide/types.hpp"

namespace uav_guide::guidance {

/// L1 解算结果（供单测、诊断与节点降级策略使用）
struct L1Solution {
    bool ok = false;           // 解算成功（位姿与路径均可用）
    bool guarded = false;      // |β| > beta_guard_deg → 本周期应保持上一指令
    bool saturated = false;    // κ 被盘旋能力上限饱和
    bool near_end = false;     // 参考点收在路径末端（前视收缩）
    double beta_rad = 0.0;     // 偏差角 β（正 = 参考点在本机航向左侧）
    double beta_deg = 0.0;
    double kappa_cmd = 0.0;    // 期望曲率 (1/m)，已饱和
    double l1_eff_m = 0.0;     // 本周期实际前视距离（m）
    double psi_cmd_enu = 0.0;  // 期望航向（ENU）
    double los_yaw_enu = 0.0;  // 本机 → 参考点方位（ENU）
    double speed_mps = 0.0;    // 本次解算采用的地速
    std::size_t ref_index = 0; // 参考点所在段起点索引
    PathPoint ref;             // 参考点（插值后）
    path::Projection proj;     // 本机在路径上的投影
};

/// L1 解算（纯几何 + 控制律，不依赖 target）
L1Solution solve_l1(const std::vector<PathPoint>& path,
                    const State5& uav,
                    const GuidanceConfig& cfg);

/// 制导解算（→ Guidance.msg）
GuidanceOutput build(const std::vector<PathPoint>& path,
                     const State5& uav,
                     const State5& target,
                     const GuidanceConfig& cfg);

}  // namespace uav_guide::guidance
