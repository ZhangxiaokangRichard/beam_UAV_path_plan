/**
 * @file beam_scorer.cpp
 * @brief Beam 评分器实现
 *
 * 从 Python methods/beam_dubins.py → BeamScorer 迁移。
 *
 * 评分公式：f = g + w_h*h - w_c*c - w_p*p
 *  - g：节点累计路径长度（越小越好）
 *  - h：到目标的启发式距离估计（可纳启发式）
 *  - c：节点局部 clearance（越大越好，因此用 - 号）
 *  - p：全局进度 (d_start_goal - d_current_goal)（越大越好，因此用 - 号）
 */

#include "beam_dubins/beam_scorer.h"

#include <algorithm>
#include <cmath>

#include "beam_dubins/dubins_path.h"
#include "beam_dubins/geometry.h"

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════

BeamScorer::BeamScorer(const DubinsPath3D& dubins,
                       const double* start, const double* goal,
                       double w_h, double w_c, double w_p)
    : dubins_(dubins)
    , w_h_(w_h), w_c_(w_c), w_p_(w_p)
{
    // 保存起点和目标的副本
    std::copy(start, start + 5, start_.begin());
    std::copy(goal,  goal  + 5, goal_.begin());

    // 计算起点到目标的 3D 距离（用于进度 p 的计算基准）
    d_start_goal_ = distance_3d(start, goal);
}

// ═══════════════════════════════════════════════════════════════
// 两阶段评分
// ═══════════════════════════════════════════════════════════════

double BeamScorer::evaluate_fast(const BeamNode& node) const
{
    double g = node.g;
    double h = fast_heuristic(&node.pos.x);

    // clearance：无穷大视为很大值（无障碍区）
    double c = (node.clearance < INFINITY) ? node.clearance : 1000.0;

    // 进度：起点→目标的全局进度 = 已缩短的距离
    double d_cur = distance_3d(&node.pos.x, goal_.data());
    double p = std::max(0.0, d_start_goal_ - d_cur);

    return g + w_h_ * h - w_c_ * c - w_p_ * p;
}

double BeamScorer::evaluate_exact(const BeamNode& node)
{
    double g = node.g;
    double h = dubins_heuristic(&node.pos.x, goal_.data());

    double c = (node.clearance < INFINITY) ? node.clearance : 1000.0;

    double d_cur = distance_3d(&node.pos.x, goal_.data());
    double p = std::max(0.0, d_start_goal_ - d_cur);

    return g + w_h_ * h - w_c_ * c - w_p_ * p;
}

// ═══════════════════════════════════════════════════════════════
// 快速启发式（欧氏距离 × 放大系数）
// ═══════════════════════════════════════════════════════════════

double BeamScorer::fast_heuristic(const double* pos) const
{
    // 欧氏距离 × DUBINS_FACTOR ≥ 真实 Dubins 距离（可纳性保证）
    return distance_3d(pos, goal_.data()) * DUBINS_FACTOR;
}

// ═══════════════════════════════════════════════════════════════
// Dubins 精确启发式（带缓存）
// ═══════════════════════════════════════════════════════════════

double BeamScorer::dubins_heuristic(const double* pos, const double* goal)
{
    // ── 缓存键：50m 网格 ──
    int rx = static_cast<int>(std::round(pos[0] / 50.0)) * 50;
    int ry = static_cast<int>(std::round(pos[1] / 50.0)) * 50;
    CacheKey key = {rx, ry};

    auto it = h_cache_.find(key);
    if (it != h_cache_.end()) {
        return it->second;  // 命中缓存
    }

    // ── 计算最短 Dubins 距离 ──
    auto candidates = dubins_.find_horizontal_tangents(pos, goal);
    double h;
    if (!candidates.empty()) {
        h = candidates[0].total_len;  // 最短的
    } else {
        // 不可行 → 退化为欧氏距离估计
        h = distance_3d(pos, goal) * DUBINS_FACTOR;
    }

    h_cache_[key] = h;
    return h;
}

// ═══════════════════════════════════════════════════════════════
// 目标更新
// ═══════════════════════════════════════════════════════════════

void BeamScorer::set_goal(const double* new_goal)
{
    std::copy(new_goal, new_goal + 5, goal_.begin());
    // 清空缓存（目标变化后，缓存的启发式值失效）
    h_cache_.clear();
}

}  // namespace beam_dubins
