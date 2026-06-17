/**
 * @file obstacle_manager.cpp
 * @brief AABB 障碍物管理实现
 *
 * 从 Python env/obstacle.py + env/environment.py 合并迁移。
 * 碰撞检测复用 beam_dubins 包的 sphere_aabb_intersect / capsule_aabb_intersect。
 */

#include "uav_guide_env/obstacle_manager.h"

#include <algorithm>
#include <cmath>
#include <memory>

// 复用 Phase 1 的碰撞检测函数
#include "beam_dubins/collision.h"

namespace uav_guide_env {

// ═══════════════════════════════════════════════════════════════
// BoxObstacle
// ═══════════════════════════════════════════════════════════════

BoxObstacle::BoxObstacle(double x, double y, double z,
                         double dx, double dy, double dz,
                         double safe_margin)
    : origin_({x, y, z})
    , size_({dx, dy, dz})
    , safe_margin_(safe_margin)
{
    updateAABB();
}

void BoxObstacle::updateAABB()
{
    // AABB = origin ± margin, max = origin + size + margin
    for (int i = 0; i < 3; ++i) {
        aabb_min_[i] = origin_[i] - safe_margin_;
        aabb_max_[i] = origin_[i] + size_[i] + safe_margin_;
    }
}

bool BoxObstacle::sphereIntersects(const double* center, double radius) const
{
    return beam_dubins::sphere_aabb_intersect(
        center, radius, aabb_min_.data(), aabb_max_.data());
}

bool BoxObstacle::sweptSphereIntersects(const double* p1, const double* p2,
                                        double radius) const
{
    return beam_dubins::capsule_aabb_intersect(
        p1, p2, radius, aabb_min_.data(), aabb_max_.data());
}

void BoxObstacle::update(double /*t*/)
{
    // 静态障碍物默认不更新
}

// ═══════════════════════════════════════════════════════════════
// DynamicObstacle
// ═══════════════════════════════════════════════════════════════

DynamicObstacle::DynamicObstacle(double x, double y, double z,
                                 double dx, double dy, double dz,
                                 double vx, double vy, double vz,
                                 double safe_margin)
    : BoxObstacle(x, y, z, dx, dy, dz, safe_margin)
    , velocity_({vx, vy, vz})
    , t0_origin_({x, y, z})
{}

void DynamicObstacle::update(double t)
{
    // 根据时间 t 更新位置：origin = t0_origin + velocity × t
    for (int i = 0; i < 3; ++i) {
        origin_[i] = t0_origin_[i] + velocity_[i] * t;
    }
    updateAABB();
}

void DynamicObstacle::followGoalY(double goal_y, double offset_y)
{
    // 直接设置 Y 坐标（通道墙壁跟随场景）
    origin_[1] = goal_y + offset_y;
    updateAABB();
}

// ═══════════════════════════════════════════════════════════════
// ObstacleManager
// ═══════════════════════════════════════════════════════════════

ObstacleManager::ObstacleManager(double lx, double ly, double lz,
                                 double boundary_margin)
    : lx_(lx), ly_(ly), lz_(lz), margin_(boundary_margin)
{}

void ObstacleManager::addObstacle(std::unique_ptr<BoxObstacle> obs)
{
    obs->setId(static_cast<int>(obstacles_.size()));
    obstacles_.push_back(std::move(obs));
}

bool ObstacleManager::isInBounds(const double* pos) const
{
    double x = pos[0];
    double y = pos[1];
    double z = (pos[2] != pos[2]) ? 0.0 : pos[2];  // NaN 保护（不应出现）

    return (x >= margin_ && x <= lx_ - margin_ &&
            y >= margin_ && y <= ly_ - margin_ &&
            z >= margin_ && z <= lz_ - margin_);
}

bool ObstacleManager::isObstacleFree(const double* pos, double radius) const
{
    for (const auto& obs : obstacles_) {
        if (obs->sphereIntersects(pos, radius)) {
            return false;  // 碰撞
        }
    }
    return true;  // 安全
}

bool ObstacleManager::isPathSafe(
    const std::vector<std::array<double, 3>>& path_points,
    double radius) const
{
    if (path_points.size() < 2 || obstacles_.empty()) {
        return true;
    }

    // 逐段检查：对每一对连续路径点做扫掠球碰撞检测
    for (size_t i = 0; i < path_points.size() - 1; ++i) {
        const double* p1 = path_points[i].data();
        const double* p2 = path_points[i + 1].data();

        // 先检查端点是否超出边界
        if (!isInBounds(p1) || !isInBounds(p2)) {
            return false;
        }

        // 对每个障碍物做胶囊体检测
        for (const auto& obs : obstacles_) {
            if (obs->sweptSphereIntersects(p1, p2, radius)) {
                return false;  // 碰撞
            }
        }
    }

    return true;  // 整条路径安全
}

void ObstacleManager::update(double t, double goal_y)
{
    for (auto& obs : obstacles_) {
        // 尝试转型为 DynamicObstacle
        auto* dyn = dynamic_cast<DynamicObstacle*>(obs.get());
        if (dyn) {
            // 动态障碍物：检查是否为通道墙壁跟随模式
            // （通过速度是否为0和是否有 follow 标记区分——此处简化处理）
            dyn->update(t);
        }
    }
}

std::vector<std::array<double, 6>> ObstacleManager::snapshotAABBs() const
{
    std::vector<std::array<double, 6>> result;
    result.reserve(obstacles_.size());

    for (const auto& obs : obstacles_) {
        std::array<double, 6> aabb;
        const auto& lo = obs->getAABBMin();
        const auto& hi = obs->getAABBMax();
        aabb[0] = lo[0]; aabb[1] = lo[1]; aabb[2] = lo[2];
        aabb[3] = hi[0]; aabb[4] = hi[1]; aabb[5] = hi[2];
        result.push_back(aabb);
    }

    return result;
}

}  // namespace uav_guide_env
