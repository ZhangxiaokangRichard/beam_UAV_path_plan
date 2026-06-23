/**
 * @file uav_model.cpp
 * @brief 固定翼 UAV 运动学模型实现
 *
 * 从 Python env/uav.py → FixedWingUAV + beam_dubins_2d_dynamic.py → _advance_uav() 迁移。
 */

#include "uav_guide_env/uav_model.h"

#include <algorithm>
#include <cmath>

namespace uav_guide_env {

// ═══════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════

FixedWingUAV::FixedWingUAV(double cruise_speed_mps, double min_turn_radius_m,
                           double max_pitch_angle_deg, double collision_radius_m,
                           double sim_dt)
    : V_(cruise_speed_mps)
    , R_min_(min_turn_radius_m)
    , gamma_max_(max_pitch_angle_deg * M_PI / 180.0)
    , r_body_(collision_radius_m)
    , dt_(sim_dt)
    , g_(9.81)
{}

// ═══════════════════════════════════════════════════════════════
// 单步运动学积分
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> FixedWingUAV::step(
    const std::array<double, 5>& pose,
    double bank, double gamma, double dt) const
{
    double x   = pose[0];
    double y   = pose[1];
    double z   = pose[2];
    double psi = pose[3];
    // pitch = pose[4]

    // 俯仰角约束在 [-gamma_max, +gamma_max]
    gamma = std::clamp(gamma, -gamma_max_, gamma_max_);

    // 水平位移
    double x_new = x + V_ * std::cos(gamma) * std::cos(psi) * dt;
    double y_new = y + V_ * std::cos(gamma) * std::sin(psi) * dt;
    double z_new = z + V_ * std::sin(gamma) * dt;

    // 航向变化率：协调转弯 ω = g·tan(bank)/V
    double psi_dot = 0.0;
    if (std::abs(bank) > 1e-9) {
        psi_dot = g_ * std::tan(bank) / V_;
    }
    double psi_new = psi + psi_dot * dt;

    return {x_new, y_new, z_new, psi_new, gamma};
}

// ═══════════════════════════════════════════════════════════════
// 死推算（路径耗尽时 / Tracking 模式的兜底策略，Dubins 约束）
//
// 设计：位置朝目标直线插值推进（保跟踪），航向 Dubins 约束平滑过渡
// 与 interpolateAlongPathWithYawLimit 设计一致
// ═══════════════════════════════════════════════════════════════

void FixedWingUAV::deadReckonToward(
    std::array<double, 5>& pose,
    const std::array<double, 5>& goal,
    double distance) const
{
    double dx = goal[0] - pose[0];
    double dy = goal[1] - pose[1];
    double dist_to_goal = std::sqrt(dx * dx + dy * dy);
    if (dist_to_goal < 1e-6) return;

    double fly_dist = std::min(distance, dist_to_goal);

    // ── 位置更新：朝目标直线插值推进（保跟踪精度）──────
    double ratio = fly_dist / dist_to_goal;
    pose[0] += ratio * dx;
    pose[1] += ratio * dy;

    // ── 航向更新：Dubins 约束平滑转向 ──────────────────
    double psi_desired = std::atan2(dy, dx);
    double dpsi = psi_desired - pose[3];
    while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
    while (dpsi < -M_PI) dpsi += 2.0 * M_PI;

    // Δψ_max = fly_dist / R_min（弧长 = R × Δψ）
    double dpsi_max = fly_dist / R_min_;
    double dpsi_actual = std::clamp(dpsi, -dpsi_max, dpsi_max);
    pose[3] += dpsi_actual;

    // 规范化航向到 (-π, π]
    while (pose[3] >  M_PI) pose[3] -= 2.0 * M_PI;
    while (pose[3] < -M_PI) pose[3] += 2.0 * M_PI;
}

// ═══════════════════════════════════════════════════════════════
// 带偏航角约束的连续位置插值路径跟踪（uav_dynamicR2.md）
//
// 核心思路：解耦位置与航向
//   · 位置沿路径点几何插值推进 — 保证路径跟踪精度
//   · 航向以 lookahead 纯追踪方式平滑转向 — 提前预判前方直段，避免"转向死循环"
//   · 位置-航向短期解耦在仿真中是可接受的近似
// ═══════════════════════════════════════════════════════════════

void FixedWingUAV::interpolateAlongPathWithYawLimit(
    std::array<double, 5>& pose,
    const std::vector<std::array<double, 4>>& path,
    int& path_ptr, size_t& last_path_gen,
    const std::array<double, 5>& goal,
    double distance,
    int lookahead_n) const
{
    // 无路径 → 朝目标 Dubins 约束死推算
    if (path.empty()) {
        deadReckonToward(pose, goal, distance);
        return;
    }

    double dist_left = distance;
    // 每周期最大航向变化（Dubins 约束：Δψ_max = V/R_min × dt）
    const double max_dpsi = (V_ / R_min_) * dt_;

    while (dist_left > 1e-3 && path_ptr < static_cast<int>(path.size()) - 1) {
        const auto& nxt = path[path_ptr + 1];
        double dx = nxt[0] - pose[0];
        double dy = nxt[1] - pose[1];
        double d_to_nxt = std::sqrt(dx * dx + dy * dy);

        // ── 位置更新：沿路径几何连续插值推进 ──────────────
        if (d_to_nxt < 1e-3) {
            // 已到达该路径点 → 跳到下一个
            path_ptr++;
            continue;
        }

        if (d_to_nxt <= dist_left) {
            // 完整跨越该路径点
            pose[0] = nxt[0];
            pose[1] = nxt[1];
            if (nxt.size() > 2) pose[2] = nxt[2];  // z 跟随
            path_ptr++;
            dist_left -= d_to_nxt;
        } else {
            // 部分插值推进（沿路径点连线方向）
            double ratio = dist_left / d_to_nxt;
            pose[0] += ratio * dx;
            pose[1] += ratio * dy;
            dist_left = 0.0;
        }

        // ── 航向更新：lookahead 纯追踪 + Dubins 速率限制 ──
        // 前视 N 个路径点（或路径终点），提前预判前方直段方向
        int la_idx = std::min(path_ptr + lookahead_n,
                              static_cast<int>(path.size()) - 1);
        const auto& la = path[la_idx];

        // 期望航向：指向 lookahead 点的几何方向
        double psi_desired = std::atan2(la[1] - pose[1], la[0] - pose[0]);

        // 航向混合：若 lookahead 点有规划航向且与几何航向差距 < 45°，
        // 则信任规划航向（final_shot 段规划航向 = 目标航向）
        if (la.size() > 3) {
            double psi_path = la[3];
            double dpsi_mix = psi_path - psi_desired;
            while (dpsi_mix >  M_PI) dpsi_mix -= 2.0 * M_PI;
            while (dpsi_mix < -M_PI) dpsi_mix += 2.0 * M_PI;
            if (std::abs(dpsi_mix) < M_PI / 4.0) {  // 45° 阈值
                psi_desired = psi_path;  // 规划航向可信，直接采用
            }
        }

        // 航向误差
        double dpsi = psi_desired - pose[3];
        while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
        while (dpsi < -M_PI) dpsi += 2.0 * M_PI;

        // Dubins 约束：单周期航向变化不能超过 ω_max × dt
        double dpsi_actual = std::clamp(dpsi, -max_dpsi, max_dpsi);
        pose[3] += dpsi_actual;
    }

    // 路径耗尽 → Dubins 约束朝目标死推算
    if (dist_left > 1e-3) {
        deadReckonToward(pose, goal, dist_left);
    }

    // 规范化航向到 (-π, π]
    while (pose[3] >  M_PI) pose[3] -= 2.0 * M_PI;
    while (pose[3] < -M_PI) pose[3] += 2.0 * M_PI;
}

// ═══════════════════════════════════════════════════════════════
// 辅助方法
// ═══════════════════════════════════════════════════════════════

std::array<double, 3> FixedWingUAV::headingVector(double yaw, double pitch) const
{
    double cp = std::cos(pitch);
    return {cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch)};
}

UAVState FixedWingUAV::getState(const std::array<double, 4>& wp) const
{
    UAVState state;
    state.pose = {wp[0], wp[1], wp[2], wp[3], 0.0};  // pitch 恒 0 (Phase 2b)
    state.sphere = {wp[0], wp[1], wp[2], r_body_};
    state.heading = headingVector(wp[3], 0.0);
    return state;
}

}  // namespace uav_guide_env
