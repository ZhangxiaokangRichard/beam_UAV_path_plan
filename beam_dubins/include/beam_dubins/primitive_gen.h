/**
 * @file primitive_gen.h
 * @brief 自适应 Dubins 运动基元生成器（纯 C++，无 ROS 依赖）
 *
 * 从 Python methods/beam_dubins.py → PrimitiveGenerator 迁移。
 *
 * 根据局部 clearance 自动选择角度粒度和步长：
 *  - 开阔区：大步长、粗粒度（0°, ±30°, ±60°）
 *  - 密集区：小步长、细粒度（0°, ±15°, ±30°, ±45°, ±60°, ±90°）
 */

#pragma once

#include <vector>

#include "beam_dubins/types.h"

namespace beam_dubins {

// 前向声明
struct AABB;

/**
 * @class PrimitiveGenerator
 * @brief 自适应 Dubins 基元生成器
 *
 * 根据当前节点的局部 clearance 自适应选择：
 *  1. 转弯角度集合（粗/中/细三种粒度）
 *  2. 扩展步长
 *
 * 生成基元包括直行（S）和左右转弯（L/R）。
 */
class PrimitiveGenerator {
public:
    /**
     * @brief 构造函数
     * @param R_min  最小转弯半径 (m)
     * @param gamma_max  最大俯仰角 (rad)，Phase 1 中使用 0
     * @param r_body  碰撞球半径 (m)
     * @param obstacles  障碍物列表指针
     */
    PrimitiveGenerator(double R_min, double gamma_max, double r_body,
                       const std::vector<AABB>* obstacles = nullptr);

    /**
     * @brief 根据当前节点和局部 clearance 生成候选基元集
     * @param node  当前 BeamNode（提供位置和航向）
     * @param step_size  本步扩展步长 (m)
     * @param clearance  局部清除距离 (m)，用于自适应角度选择
     * @return 候选基元列表
     */
    std::vector<DubinsPrimitive> generate(const BeamNode& node,
                                          double step_size,
                                          double clearance) const;

    // ── 访问器 ───────────────────────────────────────────────
    double get_R_min() const { return R_min_; }

private:
    // ── 自适应角度选择 ───────────────────────────────────────

    /// 根据 clearance 选择转弯角度集合（度）
    static std::vector<double> select_angles(double clearance);

    // ── 基元构造 ─────────────────────────────────────────────

    /// 生成直行基元
    DubinsPrimitive make_straight(const BeamNode& node, double step_size) const;

    /// 生成转弯基元（L 或 R）
    DubinsPrimitive make_turn(const BeamNode& node, double angle_deg,
                              char direction, double step_size) const;

    // ── 碰撞检测辅助 ─────────────────────────────────────────

    /// 检查基元路径段是否无碰撞
    bool is_primitive_safe(const DubinsPrimitive& prim) const;

    // ── 成员 ──
    double R_min_;
    double gamma_max_;
    double r_body_;
    const std::vector<AABB>* obstacles_;
};

}  // namespace beam_dubins
