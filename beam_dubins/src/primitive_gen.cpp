/**
 * @file primitive_gen.cpp
 * @brief 自适应基元生成器实现
 *
 * 从 Python methods/beam_dubins.py → PrimitiveGenerator 迁移。
 *
 * 基元类型：
 *  - S（直行）：沿当前航向直线延伸 step_size 米
 *  - L_θ（左转）：以 R_min 为半径左转 θ 度
 *  - R_θ（右转）：以 R_min 为半径右转 θ 度
 *
 * 碰撞检测：每个基元生成后即进行 swept-sphere 检测，
 * 有碰撞的基元直接丢弃（不在 generate() 中返回）。
 */

#include "beam_dubins/primitive_gen.h"

#include <cmath>
#include <string>

#include "beam_dubins/collision.h"
#include "beam_dubins/geometry.h"

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════

PrimitiveGenerator::PrimitiveGenerator(double R_min, double gamma_max,
                                       double r_body,
                                       const std::vector<AABB>* obstacles)
    : R_min_(R_min)
    , gamma_max_(gamma_max)
    , r_body_(r_body)
    , obstacles_(obstacles)
{}

// ═══════════════════════════════════════════════════════════════
// 公开接口：生成候选基元集
// ═══════════════════════════════════════════════════════════════

std::vector<DubinsPrimitive> PrimitiveGenerator::generate(
    const BeamNode& node, double step_size, double clearance) const
{
    // 根据 clearance 自适应选择角度集合
    auto angles = select_angles(clearance);

    std::vector<DubinsPrimitive> primitives;
    primitives.reserve(angles.size() * 2);  // 每种角度有 L、R 两个方向

    for (double angle_deg : angles) {
        if (std::abs(angle_deg) < 1e-6) {
            // ── 直行基元 ──
            auto prim = make_straight(node, step_size);
            if (!prim.waypoints.empty() && is_primitive_safe(prim)) {
                primitives.push_back(std::move(prim));
            }
        } else {
            // ── 左转基元 ──
            auto prim_l = make_turn(node, angle_deg, 'L', step_size);
            if (!prim_l.waypoints.empty() && is_primitive_safe(prim_l)) {
                primitives.push_back(std::move(prim_l));
            }
            // ── 右转基元 ──
            auto prim_r = make_turn(node, angle_deg, 'R', step_size);
            if (!prim_r.waypoints.empty() && is_primitive_safe(prim_r)) {
                primitives.push_back(std::move(prim_r));
            }
        }
    }

    return primitives;
}

// ═══════════════════════════════════════════════════════════════
// 自适应角度选择
// ═══════════════════════════════════════════════════════════════

std::vector<double> PrimitiveGenerator::select_angles(double clearance)
{
    // 根据 clearance 返回不同粒度的转弯角度集合
    // 角度以度为单位，0° 表示直行
    if (clearance > 500.0) {
        // 开阔区：粗粒度
        return {0.0, 30.0, 60.0};
    } else if (clearance > 100.0) {
        // 中密度区
        return {0.0, 15.0, 30.0, 45.0};
    } else {
        // 密集区：细粒度
        return {0.0, 15.0, 30.0, 45.0, 60.0, 90.0};
    }
}

// ═══════════════════════════════════════════════════════════════
// 直行基元构造
// ═══════════════════════════════════════════════════════════════

DubinsPrimitive PrimitiveGenerator::make_straight(
    const BeamNode& node, double step_size) const
{
    DubinsPrimitive prim;
    prim.type           = "S";
    prim.turn_angle_deg = 0.0;
    prim.turn_direction = 'S';
    prim.arc_length     = step_size;

    // 获取当前节点状态
    double psi = node.pos.yaw;
    double z   = node.pos.z;
    double x0  = node.pos.x;
    double y0  = node.pos.y;

    // 离散化直行段：按 r_body 间距采样
    int n_steps = std::max(2, static_cast<int>(step_size / std::max(r_body_, 1.0)));
    prim.waypoints.reserve(n_steps + 1);

    for (int k = 0; k <= n_steps; ++k) {
        double frac = static_cast<double>(k) / static_cast<double>(n_steps);
        Waypoint pp;
        pp.x     = x0 + frac * step_size * std::cos(psi);
        pp.y     = y0 + frac * step_size * std::sin(psi);
        pp.z     = z;
        pp.yaw   = psi;
        pp.pitch = 0.0;      // Phase 1: 2D 模式，俯仰角恒为 0
        pp.curvature = 0.0;  // 直行曲率为 0
        prim.waypoints.push_back(pp);
    }

    return prim;
}

// ═══════════════════════════════════════════════════════════════
// 转弯基元构造
// ═══════════════════════════════════════════════════════════════

DubinsPrimitive PrimitiveGenerator::make_turn(
    const BeamNode& node, double angle_deg,
    char direction, double step_size) const
{
    DubinsPrimitive prim;
    prim.turn_angle_deg = angle_deg;
    prim.turn_direction = direction;

    double R = R_min_;
    double angle_rad = angle_deg * M_PI / 180.0;
    double arc_length = R * angle_rad;

    // 按步长截断：如果弧长超过 step_size，截断到 step_size
    if (arc_length > step_size) {
        angle_rad  = step_size / R;
        arc_length = step_size;
        prim.turn_angle_deg = angle_rad * 180.0 / M_PI;
    }

    if (arc_length < 1e-6) {
        prim.arc_length = 0.0;
        prim.type = std::string(1, direction) + "_0";
        return prim;  // 过短的弧 → 空基元
    }

    prim.arc_length = arc_length;
    prim.type = std::string(1, direction) + "_"
              + std::to_string(static_cast<int>(std::round(angle_deg)));

    // ── 计算圆心和弧的采样 ──
    double z       = node.pos.z;
    double psi_cur = node.pos.yaw;
    double pos_xypsi[3] = {node.pos.x, node.pos.y, psi_cur};

    // 圆心
    double center[2];
    if (direction == 'L') {
        left_center(pos_xypsi, R, center);
    } else {
        right_center(pos_xypsi, R, center);
    }

    // 从圆心指向当前节点的起始角
    double a0 = std::atan2(node.pos.y - center[1], node.pos.x - center[0]);

    // 扫掠角：左转为正（逆时针），右转为负（顺时针）
    double sweep = (direction == 'L') ? angle_rad : -angle_rad;

    // 采样点数
    int n_steps = std::max(2, static_cast<int>(arc_length / std::max(r_body_, 1.0)));

    // 使用 sample_arc_2d 生成圆弧点
    auto arc_pts = sample_arc_2d(center, R, a0, sweep, n_steps + 1);

    prim.waypoints.reserve(arc_pts.size());
    for (size_t k = 0; k < arc_pts.size(); ++k) {
        double frac = static_cast<double>(k) / static_cast<double>(arc_pts.size() - 1);
        // 弧上航向 = 圆心角 + π/2(左) 或 -π/2(右)
        double yaw_on_arc = wrap_angle(
            a0 + frac * sweep + ((direction == 'L') ? M_PI / 2.0 : -M_PI / 2.0));

        Waypoint pp;
        pp.x     = arc_pts[k][0];
        pp.y     = arc_pts[k][1];
        pp.z     = z;
        pp.yaw   = yaw_on_arc;
        pp.pitch = 0.0;
        pp.curvature = 1.0 / R;  // 转弯曲率
        prim.waypoints.push_back(pp);
    }

    return prim;
}

// ═══════════════════════════════════════════════════════════════
// 基元碰撞检测
// ═══════════════════════════════════════════════════════════════

bool PrimitiveGenerator::is_primitive_safe(const DubinsPrimitive& prim) const
{
    if (!obstacles_ || obstacles_->empty()) {
        return true;  // 无障碍物 → 安全
    }

    // 构建 double* 数组
    std::vector<std::array<double, 3>> storage;
    storage.reserve(prim.waypoints.size());
    for (const auto& pp : prim.waypoints) {
        storage.push_back({pp.x, pp.y, pp.z});
    }
    std::vector<const double*> wps;
    wps.reserve(storage.size());
    for (const auto& s : storage) {
        wps.push_back(s.data());
    }

    // swept_sphere_path 返回 true=有碰撞
    return !swept_sphere_path(wps, r_body_, *obstacles_);
}

}  // namespace beam_dubins
