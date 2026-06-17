/**
 * @file collision.h
 * @brief 碰撞检测函数（纯 C++，无 ROS 依赖）
 *
 * 从 Python utils/collision.py 迁移。
 * 提供球体-AABB、胶囊体-AABB、扫掠球路径碰撞检测。
 */

#pragma once

#include <vector>

namespace beam_dubins {

struct AABB;  // 前向声明，定义在 types.h

// ── 单点检测 ──────────────────────────────────────────────────

/// Arvo 算法：球体与 AABB 相交检测
/// @param center  球心 [x, y, z]
/// @param radius  球半径 (m)
/// @param aabb_min  AABB 最小角 [x_min, y_min, z_min]
/// @param aabb_max  AABB 最大角 [x_max, y_max, z_max]
/// @return true 表示相交（有碰撞）
bool sphere_aabb_intersect(const double* center, double radius,
                           const double* aabb_min, const double* aabb_max);

// ── 线段检测 ──────────────────────────────────────────────────

/// 线段 p1→p2 上距离点 q 最近的点
/// @param p1  线段起点 [x, y, z]
/// @param p2  线段终点 [x, y, z]
/// @param q   查询点 [x, y, z]
/// @param out  输出最近点 [x, y, z]
void closest_point_on_segment(const double* p1, const double* p2,
                              const double* q, double* out);

/// 胶囊体（扫掠球）与 AABB 碰撞检测
/// 沿线段 p1→p2 以自适应步长采样多个球体进行检测
/// @param p1  起点 [x, y, z]
/// @param p2  终点 [x, y, z]
/// @param radius  球半径
/// @param aabb_min, aabb_max  AABB 范围
/// @return true 表示有碰撞
bool capsule_aabb_intersect(const double* p1, const double* p2, double radius,
                            const double* aabb_min, const double* aabb_max);

// ── 路径级检测 ────────────────────────────────────────────────

/// 逐段检查离散路径的扫掠球碰撞
/// @param waypoints  路径点列，每点 [x, y, z]（至少3分量）
/// @param radius  碰撞球半径
/// @param obstacles  AABB 障碍物列表
/// @return true 表示路径有碰撞
bool swept_sphere_path(const std::vector<const double*>& waypoints,
                       double radius, const std::vector<AABB>& obstacles);

}  // namespace beam_dubins
