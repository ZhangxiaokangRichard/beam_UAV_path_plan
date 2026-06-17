/**
 * @file dubins_path.h
 * @brief 3D Dubins 路径规划（横纵解耦，纯 C++，无 ROS 依赖）
 *
 * 从 Python methods/dubins_path.py 迁移。
 *
 * 规划层职责：纯几何计算，只接收约束参数 R_min/gamma_max/r_body，
 * 不持有 UAV 运动学对象，不做运动学积分。
 *
 * 输出路径点格式：[[x, y, z, ψ], ...]
 */

#pragma once

#include <functional>
#include <vector>

#include "beam_dubins/types.h"

namespace beam_dubins {

// 前向声明（避免循环依赖）
struct AABB;

/**
 * @class DubinsPath3D
 * @brief 3D Dubins 路径计算器（横纵解耦）
 *
 * 水平面：计算 LSL/LSR/RSL/RSR 四种标准 Dubins 路径
 * 垂直面：求解满足 γ_max 约束的高度剖面
 *
 * 该类的所有方法均为无状态计算，仅使用构造时传入的约束参数。
 */
class DubinsPath3D {
public:
    /**
     * @brief 构造函数
     * @param R_min  水平最小转弯半径 (m)
     * @param gamma_max  最大俯仰角 (rad)
     * @param r_body  UAV 碰撞球半径 (m)
     * @param obstacles  障碍物列表（只读指针，碰撞检测用）
     */
    DubinsPath3D(double R_min, double gamma_max, double r_body,
                 const std::vector<AABB>* obstacles = nullptr);

    // ── 水平 Dubins 路径 ─────────────────────────────────────

    /**
     * @brief 计算起点到终点的 4 种标准 Dubins 路径参数
     * @param start  起点状态 [x, y, z, ψ, γ]（至少包含前 4 分量）
     * @param goal   终点状态 [x, y, z, ψ, γ]
     * @return 4 种路径参数（LSL/LSR/RSL/RSR），按总长升序排列
     *         某些类型可能因几何不可行而不在结果中
     */
    std::vector<HorizontalDubinsParams> find_horizontal_tangents(
        const double* start, const double* goal) const;

    // ── 纵向高度剖面 ─────────────────────────────────────────

    /**
     * @brief 求解满足 gamma_max 约束的纵向爬升剖面
     * @param s_total  水平路径总弧长 (m)
     * @param z_start  起点高度 (m)
     * @param z_end    终点高度 (m)
     * @return z(s) 函数：给定弧长 s ∈ [0, s_total]，返回高度 z
     */
    std::function<double(double)> solve_altitude_profile(
        double s_total, double z_start, double z_end) const;

    // ── 路径采样与输出 ───────────────────────────────────────

    /**
     * @brief 对 HorizontalDubinsParams 采样生成离散路径点
     * @param params  Dubins 路径参数
     * @param n_arc  每段弧的采样点数
     * @param n_straight  直线段采样点数
     * @return 离散路径点列 [[x, y, z, ψ], ...]
     */
    std::vector<Waypoint> sample_path(
        const HorizontalDubinsParams& params,
        int n_arc = 20, int n_straight = 10) const;

    /**
     * @brief 计算从起点到终点的最优 Dubins 路径
     * @param start  起点 [x, y, z, ψ, γ]
     * @param goal   终点 [x, y, z, ψ, γ]
     * @param out_path  输出路径点列
     * @return 最佳路径的总长度，若无可行为 -1
     */
    double best_path(const double* start, const double* goal,
                     std::vector<Waypoint>& out_path) const;

    /**
     * @brief 检查路径是否与障碍物碰撞
     * @param path  路径点列
     * @return true 表示无碰撞（安全）
     */
    bool is_path_safe(const std::vector<Waypoint>& path) const;

    // ── 访问器 ───────────────────────────────────────────────
    double get_R_h()      const { return R_h_; }
    double get_gamma_max() const { return gamma_max_; }
    double get_r_body()   const { return r_body_; }

private:
    /**
     * @brief 计算单种 Dubins 类型的切线参数
     * @param sp  起点状态（至少 4 分量：x, y, z, ψ）
     * @param ep  终点状态
     * @param type  类型字符串："LSL"/"RSR"/"LSR"/"RSL"
     * @param R  转弯半径
     * @return 路径参数；几何不可行返回 std::nullopt
     */
    HorizontalDubinsParams compute_horizontal(
        const double* sp, const double* ep,
        const std::string& type, double R) const;

    // ── 约束参数 ──
    double R_h_;           // 水平最小转弯半径 (m)
    double gamma_max_;     // 最大俯仰角 (rad)
    double r_body_;        // 碰撞球半径 (m)
    const std::vector<AABB>* obstacles_;  // 障碍物引用（可能为 nullptr）
};

}  // namespace beam_dubins
