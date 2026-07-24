/**
 * @file collision.cpp
 * @brief 碰撞检测实现
 *
 * 从 Python utils/collision.py 逐函数迁移。
 * 核心算法：
 *  - sphere_aabb_intersect 使用 Arvo 算法（最近点 + 距离平方比较）
 *  - capsule_aabb_intersect 使用自适应步长沿线采样
 */

#include "beam_dubins/collision.h"
#include "beam_dubins/types.h"    // AABB 定义

#include <algorithm>
#include <cmath>

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 球体 vs AABB（Arvo 算法）
// ═══════════════════════════════════════════════════════════════

bool sphere_aabb_intersect(const double* center, double radius,
                           const double* aabb_min, const double* aabb_max)
{
    // 计算球心到 AABB 最近点的距离平方
    double d2 = 0.0;
    for (int i = 0; i < 3; ++i) {
        if (center[i] < aabb_min[i]) {
            double diff = aabb_min[i] - center[i];
            d2 += diff * diff;
        } else if (center[i] > aabb_max[i]) {
            double diff = center[i] - aabb_max[i];
            d2 += diff * diff;
        }
        // 在 AABB 内部的分量贡献 0
    }
    return d2 <= radius * radius;
}

// ═══════════════════════════════════════════════════════════════
// 线段最近点
// ═══════════════════════════════════════════════════════════════

void closest_point_on_segment(const double* p1, const double* p2,
                              const double* q, double* out)
{
    // seg = p2 - p1
    double seg[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    double seg_len2 = seg[0] * seg[0] + seg[1] * seg[1] + seg[2] * seg[2];

    if (seg_len2 < 1e-12) {
        // 退化为点
        out[0] = p1[0]; out[1] = p1[1]; out[2] = p1[2];
        return;
    }

    // t = clamp( dot(q-p1, seg) / seg_len2, 0, 1 )
    double dq[3] = {q[0] - p1[0], q[1] - p1[1], q[2] - p1[2]};
    double dot = dq[0] * seg[0] + dq[1] * seg[1] + dq[2] * seg[2];
    double t = std::clamp(dot / seg_len2, 0.0, 1.0);

    out[0] = p1[0] + t * seg[0];
    out[1] = p1[1] + t * seg[1];
    out[2] = p1[2] + t * seg[2];
}

// ═══════════════════════════════════════════════════════════════
// 胶囊体 vs AABB（扫掠球）
// ═══════════════════════════════════════════════════════════════

bool capsule_aabb_intersect(const double* p1, const double* p2, double radius,
                            const double* aabb_min, const double* aabb_max)
{
    // 计算线段长度
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double dz = p2[2] - p1[2];
    double seg_len = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (seg_len < 1e-9) {
        // 线段退化为点 → 直接用球体检测
        return sphere_aabb_intersect(p1, radius, aabb_min, aabb_max);
    }

    // 自适应步长：采样间距 ≈ radius/2，至少 2 步
    int n_steps = std::max(2, static_cast<int>(seg_len / std::max(radius * 0.5, 1.0)) + 1);

    for (int i = 0; i <= n_steps; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(n_steps);
        double pt[3] = {
            p1[0] + t * dx,
            p1[1] + t * dy,
            p1[2] + t * dz
        };
        if (sphere_aabb_intersect(pt, radius, aabb_min, aabb_max)) {
            return true;  // 任意采样点碰撞即返回
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════
// 扫掠球路径碰撞
// ═══════════════════════════════════════════════════════════════

bool swept_sphere_path(const std::vector<const double*>& waypoints,
                       double radius, const std::vector<AABB>& obstacles)
{
    if (obstacles.empty() || waypoints.size() < 2) {
        return false;  // 无障碍物或路径太短 → 安全
    }

    // 逐段检查
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const double* p1 = waypoints[i];
        const double* p2 = waypoints[i + 1];
        for (const auto& obs : obstacles) {
            // 使用 AABB 的 min/max 分量
            double aabb_min[3] = {obs.min.x, obs.min.y, obs.min.z};
            double aabb_max[3] = {obs.max.x, obs.max.y, obs.max.z};
            if (capsule_aabb_intersect(p1, p2, radius, aabb_min, aabb_max)) {
                return true;  // 任一段碰撞即返回
            }
        }
    }
    return false;
}

}  // namespace beam_dubins
