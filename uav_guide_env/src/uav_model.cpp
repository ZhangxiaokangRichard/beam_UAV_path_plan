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
// 沿路径推进（"只进不退指针"策略）
// ═══════════════════════════════════════════════════════════════

void FixedWingUAV::advanceAlongPath(
    std::array<double, 5>& pose,
    const std::vector<std::array<double, 4>>& path,
    int& path_ptr, size_t& last_path_gen,
    const std::array<double, 5>& goal,
    double distance) const
{
    if (path.empty()) {
        // 无路径 → 朝目标方向死推算
        deadReckonToward(pose, goal, distance);
        return;
    }

    // ── 路径更新检测由调用方处理（simulation_loop 通过 path_generation 判断）──
    double dist_left = distance;

    while (dist_left > 1e-3 && path_ptr < static_cast<int>(path.size()) - 1) {
        const auto& nxt = path[path_ptr + 1];
        double dx = nxt[0] - pose[0];
        double dy = nxt[1] - pose[1];
        double d = std::sqrt(dx * dx + dy * dy);

        if (d < 1e-3) {
            // 已到达该路径点 → 跳到下一个
            path_ptr++;
            continue;
        }

        if (d <= dist_left) {
            // 完整跨越该路径点
            pose[0] = nxt[0];
            pose[1] = nxt[1];
            // z 如有变化也可跟随
            if (nxt.size() > 2) pose[2] = nxt[2];
            // 航向从路径点获取
            if (path[path_ptr + 1].size() > 3) {
                pose[3] = path[path_ptr + 1][3];
            }
            path_ptr++;
            dist_left -= d;
        } else {
            // 部分插值推进
            double ratio = dist_left / d;
            pose[0] += ratio * dx;
            pose[1] += ratio * dy;
            // 航向平滑插值
            if (path[path_ptr + 1].size() > 3) {
                double psi_target = path[path_ptr + 1][3];
                double dpsi = psi_target - pose[3];
                // 规范角度差到 (-π, π]
                while (dpsi >  M_PI) dpsi -= 2.0 * M_PI;
                while (dpsi < -M_PI) dpsi += 2.0 * M_PI;
                pose[3] += ratio * dpsi;
                // 规范到 (-π, π]
                while (pose[3] >  M_PI) pose[3] -= 2.0 * M_PI;
                while (pose[3] < -M_PI) pose[3] += 2.0 * M_PI;
            }
            dist_left = 0.0;
        }
    }

    // 路径耗尽 → 朝目标方向死推算
    if (dist_left > 1e-3) {
        deadReckonToward(pose, goal, dist_left);
    }
}

// ═══════════════════════════════════════════════════════════════
// 死推算（路径耗尽时的兜底策略）
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

    double step = std::min(distance, dist_to_goal);
    double psi  = std::atan2(dy, dx);
    pose[0] += step * std::cos(psi);
    pose[1] += step * std::sin(psi);
    pose[3] = psi;
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
