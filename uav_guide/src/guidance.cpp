/**
 * @file guidance.cpp
 * @brief 巡航制导解算（新增；Q12/Q16 裁决的采样点规则）
 *
 * 采样点：自本机沿路径向后扫描，取**首个曲率为 0 的点**（首个直线段起点）。
 * 若扫到末尾仍无（全弧段）→ 回退路径末端点，并置 sample_fallback=true 供节点限流告警。
 */

#include "uav_guide/guidance.hpp"

#include <cmath>

#include "uav_guide/geo.hpp"
#include "uav_guide/path.hpp"

namespace uav_guide::guidance {

GuidanceOutput build(const std::vector<PathPoint>& path,
                     const State5& uav,
                     const State5& target,
                     const GuidanceConfig& cfg)
{
    GuidanceOutput out;
    out.fly_speed_mps = cfg.fly_speed_mps;

    // 无路径 → 以目标高度与目标航向兜底（不发布陈旧值由节点负责）
    if (path.empty()) {
        out.fly_height = target.z;
        out.heads_deg = geo::enu_yaw_to_heading_deg(target.yaw);
        out.sample_fallback = true;
        return out;
    }

    const std::size_t from = uav.valid ? path::closest_index(path, uav.x, uav.y) : 0;
    std::size_t idx = path::find_sample_index(path, from, cfg.sample_rule, cfg.curvature_eps);
    if (idx >= path.size()) {
        idx = path.size() - 1;  // 全弧段：回退末端点
        out.sample_fallback = true;
    }
    out.sample_index = idx;

    const PathPoint& sample = path[idx];

    // ── heads（局部 ENU → 航空航向）──
    double psi_enu = sample.yaw;
    if (cfg.heads_source == HeadsSource::BearingToSample && uav.valid) {
        psi_enu = geo::bearing_enu(sample.x - uav.x, sample.y - uav.y);
    }
    out.heads_deg = geo::enu_yaw_to_heading_deg(psi_enu);

    // ── 期望高度 ──
    out.fly_height = (cfg.height_source == HeightSource::TargetZ) ? target.z : sample.z;
    return out;
}

}  // namespace uav_guide::guidance
