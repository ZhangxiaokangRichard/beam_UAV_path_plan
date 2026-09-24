/**
 * @file guidance.cpp
 * @brief 巡航制导解算（采样点规则 Q12/Q16 + L1 路径跟踪）
 *
 * `sample_yaw` / `bearing_to_sample`：自本机沿路径扫描，取**首个曲率为 0 的点**
 *（首个直线段起点）。若扫到末尾仍无（全弧段）→ 回退路径末端点，
 *并置 sample_fallback=true 供节点限流告警。
 *
 * `l1`：沿路径在前视距离 L1 处取参考点，偏差角 β 决定期望曲率
 * `κ = 2·sinβ/L1`（按盘旋能力 `1/R_c` 饱和），输出 `ψ_cmd = ψ + κ·V·T_lead`。
 * 参考点距离恒定、指令连续、前方 L1 内的转弯可见（设计见 doc/L1_tracking_plan.md）。
 */

#include "uav_guide/guidance.hpp"

#include <algorithm>
#include <cmath>

#include "uav_guide/geo.hpp"
#include "uav_guide/path.hpp"

namespace uav_guide::guidance {
namespace {

/// 末端前视最小距离（m）：end_shrink_ratio 收缩后的下限，避免增益发散
constexpr double kMinLookaheadM = 50.0;
/// 地速下限（m/s）：低于此值视为无效（未起飞/悬停），回退配置常量
constexpr double kMinSpeedMps = 1.0;

}  // namespace

L1Solution solve_l1(const std::vector<PathPoint>& path,
                    const State5& uav,
                    const GuidanceConfig& cfg)
{
    L1Solution s;
    if (path.empty() || !uav.valid) return s;

    // ── ① 投影到路径折线（段内插值，消除大点距造成的指令跳变）──
    s.proj = path::project(path, uav.x, uav.y);
    if (!s.proj.valid) return s;

    s.speed_mps = (uav.speed_mps > kMinSpeedMps) ? uav.speed_mps : cfg.speed_mps;

    // ── ② 前视距离：剩余不足 L1 时按 end_shrink_ratio 收缩（默认收到末点）──
    const double total = path::length(path);
    const double remaining = std::max(0.0, total - s.proj.s_m);
    double l1 = cfg.l1_distance_m;
    if (remaining < l1) {
        s.near_end = true;
        l1 = std::min(l1, cfg.end_shrink_ratio * remaining);
    }
    l1 = std::max(l1, std::min(remaining, kMinLookaheadM));
    if (l1 < 1e-6) l1 = std::max(cfg.l1_distance_m, 1.0);   // 退化保护（零长路径）
    s.l1_eff_m = l1;

    // ── ③ 参考点（沿弧长前进 L1，插值）与偏差角 ──
    bool clamped = false;
    s.ref = path::point_at_arclength(path, s.proj.s_m + l1, &s.ref_index, &clamped);
    const double dx = s.ref.x - uav.x;
    const double dy = s.ref.y - uav.y;
    if (std::hypot(dx, dy) < 1e-6) {
        // 参考点与本机重合（已在路径末点）：保持当前航向
        s.ok = true;
        s.los_yaw_enu = uav.yaw;
        s.psi_cmd_enu = uav.yaw;
        return s;
    }
    const double los = std::atan2(dy, dx);
    s.los_yaw_enu = los;
    s.beta_rad = geo::wrap_angle(los - uav.yaw);
    s.beta_deg = s.beta_rad * 180.0 / geo::kPi;

    // ── ④ 保护：参考点跑到本机侧后方（投影跳变/路径倒退）→ 本周期保持上一指令 ──
    if (std::abs(s.beta_deg) > cfg.beta_guard_deg) {
        s.ok = true;
        s.guarded = true;
        return s;
    }

    // ── ⑤ L1 律 → 期望曲率（死区 + 盘旋能力饱和）──
    double kappa = 0.0;
    if (std::abs(s.beta_deg) >= cfg.beta_deadband_deg) {
        kappa = 2.0 * std::sin(s.beta_rad) / l1;
    }
    const double kappa_max = 1.0 / std::max(cfg.turn_radius_m, 1e-3);
    if (kappa > kappa_max) {
        kappa = kappa_max;
        s.saturated = true;
    }
    if (kappa < -kappa_max) {
        kappa = -kappa_max;
        s.saturated = true;
    }
    s.kappa_cmd = kappa;

    // ── ⑤ 期望航向：在本机当前航向基础上给一个前馈角（补偿飞控航向环滞后）──
    s.psi_cmd_enu = geo::wrap_angle(uav.yaw + kappa * s.speed_mps * cfg.lead_time_s);
    s.ok = true;
    return s;
}

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

    // ── L1 路径跟踪（heads_source = l1）──
    if (cfg.heads_source == HeadsSource::L1) {
        const L1Solution sol = solve_l1(path, uav, cfg);
        const bool to_target_z = (cfg.height_source == HeightSource::TargetZ);

        if (!sol.ok) {
            // 位姿或路径不可用 → 不给指令（节点保持上一值，无上一值时跳过发布）
            out.hold_previous = true;
            out.sample_fallback = true;
            out.fly_height = to_target_z ? target.z : path.front().z;
            return out;
        }

        out.beta_deg = sol.beta_deg;
        out.kappa_cmd = sol.kappa_cmd;
        out.l1_eff_m = sol.l1_eff_m;
        out.l1_point_x = sol.ref.x;
        out.l1_point_y = sol.ref.y;
        out.l1_index = sol.ref_index;
        out.l1_saturated = sol.saturated;
        out.near_end = sol.near_end;
        out.los_heading_deg = geo::enu_yaw_to_heading_deg(sol.los_yaw_enu);
        out.sample_index = sol.proj.seg_index;   // L1 模式下为投影片段起点索引（诊断）
        out.fly_height = to_target_z ? target.z : sol.ref.z;

        if (sol.guarded) {
            // β 越界：节点保持上一指令；连续保持 N 周期后降级为“朝参考点直飞”
            out.hold_previous = true;
            out.heads_deg = out.los_heading_deg;
            return out;
        }

        out.heads_deg = geo::enu_yaw_to_heading_deg(sol.psi_cmd_enu);
        return out;
    }

    // ── 采样点规则（sample_yaw / bearing_to_sample）──
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
