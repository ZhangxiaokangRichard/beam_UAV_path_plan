/**
 * @file beam_scorer.h
 * @brief Beam 搜索启发式评分器（纯 C++，无 ROS 依赖）
 *
 * 从 Python methods/beam_dubins.py → BeamScorer 迁移。
 *
 * 评分公式：f = g + w_h*h - w_c*c - w_p*p
 *  - g：累计路径长度
 *  - h：到目标的启发式距离（Dubins 下界估计）
 *  - c：局部 clearance（障碍物距离奖励）
 *  - p：进度奖励（起点到终点的全局进度）
 *
 * 两阶段评分：
 *  阶段1（快速）：欧氏距离 × DUBINS_FACTOR 作为启发式下界
 *  阶段2（精确）：对 top N 候选调用 Dubins 精确距离
 */

#pragma once

#include <array>
#include <unordered_map>
#include <vector>

#include "beam_dubins/types.h"

namespace beam_dubins {

// 前向声明
class DubinsPath3D;

// ── std::pair<int,int> 哈希（用于启发式缓存）─────────────────

struct PairHash {
    size_t operator()(const std::pair<int, int>& p) const {
        return static_cast<size_t>(p.first) * 31 + static_cast<size_t>(p.second);
    }
};

/**
 * @class BeamScorer
 * @brief 启发式评分器
 *
 * 为 BeamNode 计算评价分数 f，用于 Beam 搜索的节点排序和剪枝。
 * 内置缓存机制，避免重复计算相同网格位置的 Dubins 启发式距离。
 */
class BeamScorer {
public:
    /// Dubins→欧氏 放大系数下界：Dubins 路径 ∈ [欧氏, 欧氏×1.5]
    static constexpr double DUBINS_FACTOR = 1.08;

    /**
     * @brief 构造函数
     * @param dubins  Dubins 路径计算器引用（用于精确启发式）
     * @param start  搜索起点 [x, y, z, yaw, pitch]
     * @param goal   搜索目标 [x, y, z, yaw, pitch]
     * @param w_h    启发式权重（默认 1.2）
     * @param w_c    clearance 权重（默认 0.05）
     * @param w_p    进度权重（默认 0.3）
     */
    BeamScorer(const DubinsPath3D& dubins,
               const double* start, const double* goal,
               double w_h = 1.2, double w_c = 0.05, double w_p = 0.3);

    /**
     * @brief 快速评分（欧氏距离启发式），用于候选初筛
     * @param node  待评分节点
     * @return f = g + w_h*欧氏*dubins_factor - w_c*clearance - w_p*progress
     */
    double evaluate_fast(const BeamNode& node) const;

    /**
     * @brief 精确评分（Dubins 距离启发式），用于最终排名
     * @param node  待评分节点
     * @return f = g + w_h*Dubins_heuristic - w_c*clearance - w_p*progress
     */
    double evaluate_exact(const BeamNode& node);

    /**
     * @brief 获取 Dubins 精确启发式距离（带缓存）
     * @param pos  当前位置 [x, y, z, yaw, pitch]
     * @param goal  目标位置
     * @return 忽略障碍物的最短 Dubins 距离
     */
    double dubins_heuristic(const double* pos, const double* goal);

    // ── 目标更新（用于 IncrementalBeamDubins）────────────────

    /// 更新目标位置（用于动态目标场景）
    void set_goal(const double* new_goal);

    /// 获取当前目标指针（只读，用于 Dubins 启发式计算）
    const double* get_goal_ptr() const { return goal_.data(); }

    // ── 访问器 ───────────────────────────────────────────────
    double get_w_h() const { return w_h_; }
    double get_w_c() const { return w_c_; }
    double get_w_p() const { return w_p_; }

private:
    /**
     * @brief 快速启发式：欧氏距离 × DUBINS_FACTOR
     * @param pos  当前位置 [x, y, z, ...]
     * @return ≥ 真实 Dubins 路径长度的下界估计
     */
    double fast_heuristic(const double* pos) const;

    // ── 成员 ──
    const DubinsPath3D& dubins_;
    std::array<double, 5> start_;   // 搜索起点
    std::array<double, 5> goal_;    // 搜索目标
    double w_h_, w_c_, w_p_;        // 评分权重
    double d_start_goal_;           // 起点→目标 距离（用于进度计算）

    /// 缓存：网格键 (rx, ry) → Dubins 启发式值
    /// 网格分辨率 50m
    using CacheKey = std::pair<int, int>;
    std::unordered_map<CacheKey, double, PairHash> h_cache_;
};

}  // namespace beam_dubins
