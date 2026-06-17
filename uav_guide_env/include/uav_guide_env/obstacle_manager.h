/**
 * @file obstacle_manager.h
 * @brief AABB 障碍物管理与空间边界检查（从 Python env/obstacle.py + env/environment.py 合并迁移）
 *
 * 提供：
 *  - BoxObstacle：静态轴对齐长方体障碍物
 *  - DynamicObstacle：可沿直线运动的动态障碍物
 *  - ObstacleManager：障碍物集合管理 + 空间边界 + 碰撞/安全查询
 */

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace uav_guide_env {

// ═══════════════════════════════════════════════════════════════
// BoxObstacle — 静态 AABB 障碍物
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 轴对齐长方体障碍物
 *
 * 位置 (origin) + 尺寸 (size) + 安全边距 (safe_margin) → 计算 AABB。
 * 碰撞检测使用球-AABB (Arvo 算法) 和 胶囊-AABB (扫掠球)。
 */
class BoxObstacle {
public:
    /**
     * @brief 构造函数
     * @param x, y, z   底面角点坐标 (m)
     * @param dx, dy, dz  长方体尺寸 (m)
     * @param safe_margin 安全膨胀边距 (m)，默认 5.0
     */
    BoxObstacle(double x, double y, double z,
                double dx, double dy, double dz,
                double safe_margin = 5.0);

    virtual ~BoxObstacle() = default;

    // ── 碰撞检测 ───────────────────────────────────────────

    /// 球体与 AABB 相交检测
    bool sphereIntersects(const double* center, double radius) const;

    /// 扫掠球（胶囊体）与 AABB 相交检测
    bool sweptSphereIntersects(const double* p1, const double* p2,
                               double radius) const;

    // ── AABB 更新（动态障碍物子类使用）────────────────────

    /// 根据时间更新障碍物位置（默认实现：无操作）
    virtual void update(double t);

    // ── 访问器 ─────────────────────────────────────────────

    const std::array<double, 3>& getOrigin()    const { return origin_; }
    const std::array<double, 3>& getSize()      const { return size_; }
    const std::array<double, 3>& getAABBMin()   const { return aabb_min_; }
    const std::array<double, 3>& getAABBMax()   const { return aabb_max_; }
    double getSafeMargin() const { return safe_margin_; }
    int getId()           const { return id_; }
    void setId(int id) { id_ = id; }

protected:
    /// 重新计算 AABB（子类修改 origin_ 后调用）
    void updateAABB();

    std::array<double, 3> origin_;     // 障碍物底面角点 [x, y, z]
    std::array<double, 3> size_;       // 尺寸 [dx, dy, dz]
    std::array<double, 3> aabb_min_;   // AABB 最小角
    std::array<double, 3> aabb_max_;   // AABB 最大角
    double safe_margin_;
    int id_ = -1;
};

// ═══════════════════════════════════════════════════════════════
// DynamicObstacle — 动态障碍物
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 可沿直线运动的动态障碍物
 *
 * 障碍物以恒定速度沿直线移动。
 * 可通过 follow 模式跟随目标 Y 坐标（通道墙壁场景）。
 */
class DynamicObstacle : public BoxObstacle {
public:
    DynamicObstacle(double x, double y, double z,
                    double dx, double dy, double dz,
                    double vx = 0.0, double vy = 0.0, double vz = 0.0,
                    double safe_margin = 5.0);

    void update(double t) override;

    /// 跟随目标 Y 坐标偏移（通道墙壁跟随场景）
    /// offset_y = 墙壁内边距离目标中心 Y 的偏移
    void followGoalY(double goal_y, double offset_y);

private:
    std::array<double, 3> velocity_;   // [vx, vy, vz]
    std::array<double, 3> t0_origin_;  // t=0 时的原点
};

// ═══════════════════════════════════════════════════════════════
// ObstacleManager — 障碍物集合管理 + 空间边界
// ═══════════════════════════════════════════════════════════════

/**
 * @brief 障碍物管理器
 *
 * 合并 Python env/environment.py → Environment3D 的功能：
 *  - 空间边界检查 (isInBounds)
 *  - 障碍物列表管理
 *  - 全局碰撞查询 (isObstacleFree, isPathSafe)
 *  - 动态障碍物时间更新
 */
class ObstacleManager {
public:
    /**
     * @brief 构造函数
     * @param lx, ly, lz  空间最大范围 (m)
     * @param boundary_margin  边界安全距离 (m)
     */
    ObstacleManager(double lx, double ly, double lz, double boundary_margin = 50.0);

    // ── 障碍物管理 ─────────────────────────────────────────

    /// 添加障碍物（转移所有权）
    void addObstacle(std::unique_ptr<BoxObstacle> obs);

    /// 获取所有障碍物（只读）
    const std::vector<std::unique_ptr<BoxObstacle>>& getObstacles() const {
        return obstacles_;
    }

    // ── 空间边界 ───────────────────────────────────────────

    /// 检查点是否在空间边界内（允许区间：[margin, L-margin]）
    bool isInBounds(const double* pos) const;

    /// 获取空间范围
    double getLx() const { return lx_; }
    double getLy() const { return ly_; }
    double getLz() const { return lz_; }
    double getBoundaryMargin() const { return margin_; }

    // ── 碰撞查询 ───────────────────────────────────────────

    /// 单点球体碰撞检测（true = 无碰撞）
    bool isObstacleFree(const double* pos, double radius) const;

    /// 离散路径扫掠球碰撞检测（true = 安全无碰撞）
    bool isPathSafe(const std::vector<std::array<double, 3>>& path_points,
                    double radius) const;

    // ── 动态障碍物更新 ─────────────────────────────────────

    /// 根据时间和目标位置更新动态障碍物
    void update(double t, double goal_y);

    /// 快照：获取当前所有障碍物的 AABB 列表（用于规划请求）
    std::vector<std::array<double, 6>> snapshotAABBs() const;

private:
    double lx_, ly_, lz_;     // 空间最大范围 (m)
    double margin_;           // 边界安全距离 (m)
    std::vector<std::unique_ptr<BoxObstacle>> obstacles_;
};

}  // namespace uav_guide_env
