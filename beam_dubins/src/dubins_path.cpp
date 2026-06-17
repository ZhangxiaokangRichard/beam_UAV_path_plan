/**
 * @file dubins_path.cpp
 * @brief 3D Dubins 路径计算实现
 *
 * 从 Python methods/dubins_path.py 迁移。
 * 核心算法：
 *  - 4 种标准 Dubins 类型（LSL/LSR/RSL/RSR）的圆心-切点几何计算
 *  - 外公切线（LSL/RSR）与内公切线（LSR/RSL）的区分
 *  - 纵向高度剖面的三段式求解
 */

#include "beam_dubins/dubins_path.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "beam_dubins/collision.h"
#include "beam_dubins/geometry.h"

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════

DubinsPath3D::DubinsPath3D(double R_min, double gamma_max, double r_body,
                           const std::vector<AABB>* obstacles)
    : R_h_(R_min)
    , gamma_max_(gamma_max)
    , r_body_(r_body)
    , obstacles_(obstacles)
{}

// ═══════════════════════════════════════════════════════════════
// 水平 Dubins 4 型枚举
// ═══════════════════════════════════════════════════════════════

std::vector<HorizontalDubinsParams> DubinsPath3D::find_horizontal_tangents(
    const double* start, const double* goal) const
{
    // 对 4 种 Dubins 类型逐一计算，按总长升序排列
    static const char* types[4] = {"LSL", "RSR", "LSR", "RSL"};
    std::vector<HorizontalDubinsParams> results;

    for (const auto& t : types) {
        auto p = compute_horizontal(start, goal, t, R_h_);
        // compute_horizontal 返回即使 "不可行" 也会构造对象（total_len=0 表示失败）
        // 我们使用 total_len > 0 来判断有效性
        if (p.total_len > 1e-6) {
            results.push_back(std::move(p));
        }
    }

    // 按总长升序排列
    std::sort(results.begin(), results.end(),
              [](const HorizontalDubinsParams& a, const HorizontalDubinsParams& b) {
                  return a.total_len < b.total_len;
              });
    return results;
}

// ═══════════════════════════════════════════════════════════════
// 单类型 Dubins 计算
// ═══════════════════════════════════════════════════════════════

HorizontalDubinsParams DubinsPath3D::compute_horizontal(
    const double* sp, const double* ep,
    const std::string& path_type, double R) const
{
    HorizontalDubinsParams result;
    result.type = path_type;

    // ── 提取起点/终点朝向角 ──
    // sp 格式: [x, y, z, ψ, γ] 或 [x, y, ψ]
    double psi_s = sp[3];  // 偏航角 ψ
    double psi_e = ep[3];

    // ── 计算两圆心 ──
    double c1[2], c2[2];
    double sp_xypsi[3] = {sp[0], sp[1], psi_s};
    double ep_xypsi[3] = {ep[0], ep[1], psi_e};

    if (path_type[0] == 'L') {
        left_center(sp_xypsi, R, c1);
    } else {
        right_center(sp_xypsi, R, c1);
    }
    if (path_type[2] == 'L') {
        left_center(ep_xypsi, R, c2);
    } else {
        right_center(ep_xypsi, R, c2);
    }

    // ── 圆心距 ──
    double d = distance_2d(c1, c2);

    // 切点坐标 和 切线角
    double t1_xy[2], t2_xy[2];
    double psi_t = 0.0;       // 切线方向角
    double straight_len = 0.0;

    if (path_type == "LSL" || path_type == "RSR") {
        // ═══ 外公切线 ═══
        // 两圆心连线方向角
        double alpha = std::atan2(c2[1] - c1[1], c2[0] - c1[0]);

        // 垂直方向（从 c1 指向切线点）
        double perp_x, perp_y;
        if (path_type == "LSL") {
            // L 圆切线点 = c1 + R * (旋转 alpha 顺时针90°)
            perp_x =  std::sin(alpha);
            perp_y = -std::cos(alpha);
        } else {  // RSR
            // R 圆切线点 = c1 + R * (旋转 alpha 逆时针90°)
            perp_x = -std::sin(alpha);
            perp_y =  std::cos(alpha);
        }

        t1_xy[0] = c1[0] + R * perp_x;
        t1_xy[1] = c1[1] + R * perp_y;
        t2_xy[0] = c2[0] + R * perp_x;
        t2_xy[1] = c2[1] + R * perp_y;

        psi_t = std::atan2(t2_xy[1] - t1_xy[1], t2_xy[0] - t1_xy[0]);
        straight_len = distance_2d(t1_xy, t2_xy);

    } else {
        // ═══ 内公切线（LSR / RSL） ═══
        if (d < 2.0 * R - 1e-9) {
            // 两圆相交/重叠，无内公切线
            return result;
        }

        double ratio = 2.0 * R / d;
        if (std::abs(ratio) > 1.0) {
            return result;
        }
        double beta  = std::acos(ratio);
        double alpha = std::atan2(c2[1] - c1[1], c2[0] - c1[0]);

        double angle1, angle2;
        if (path_type == "LSR") {
            // c1 左圆顺时针切出，c2 右圆逆时针切入
            angle1 = alpha - beta;
            angle2 = alpha - beta + M_PI;
        } else {  // RSL
            angle1 = alpha + beta;
            angle2 = alpha + beta - M_PI;
        }

        t1_xy[0] = c1[0] + R * std::cos(angle1);
        t1_xy[1] = c1[1] + R * std::sin(angle1);
        t2_xy[0] = c2[0] + R * std::cos(angle2);
        t2_xy[1] = c2[1] + R * std::sin(angle2);

        psi_t = std::atan2(t2_xy[1] - t1_xy[1], t2_xy[0] - t1_xy[0]);
        straight_len = distance_2d(t1_xy, t2_xy);
    }

    // ── 各段弧长计算 ──
    // 第一段弧角（从起点航向到切线方向）
    double ang1;
    if (path_type[0] == 'L') {
        ang1 = ccw_arc_angle(psi_s, psi_t);
    } else {
        ang1 = -cw_arc_angle(psi_s, psi_t);
    }

    // 第三段弧角（从切线方向到终点航向）
    double ang2;
    if (path_type[2] == 'L') {
        ang2 = ccw_arc_angle(psi_t, psi_e);
    } else {
        ang2 = -cw_arc_angle(psi_t, psi_e);
    }

    double arc1_len = arc_length_2d(R, ang1);
    double arc2_len = arc_length_2d(R, ang2);
    double total_len = arc1_len + straight_len + arc2_len;

    if (total_len < 1e-6) {
        return result;  // 退化（起点=终点）
    }

    // ── 填充结果 ──
    result.c1.x = c1[0]; result.c1.y = c1[1]; result.c1.z = 0.0;
    result.c2.x = c2[0]; result.c2.y = c2[1]; result.c2.z = 0.0;
    result.t1.x = t1_xy[0]; result.t1.y = t1_xy[1];
    result.t1.z = sp[2];     // 起点高度
    result.t1.yaw = psi_t; result.t1.pitch = 0.0;
    result.t2.x = t2_xy[0]; result.t2.y = t2_xy[1];
    result.t2.z = ep[2];     // 终点高度
    result.t2.yaw = psi_t; result.t2.pitch = 0.0;
    result.arc1_angle   = ang1;
    result.arc2_angle   = ang2;
    result.arc1_len     = arc1_len;
    result.straight_len = straight_len;
    result.arc2_len     = arc2_len;
    result.total_len    = total_len;

    return result;
}

// ═══════════════════════════════════════════════════════════════
// 高度剖面求解
// ═══════════════════════════════════════════════════════════════

std::function<double(double)> DubinsPath3D::solve_altitude_profile(
    double s_total, double z_start, double z_end) const
{
    double dz = z_end - z_start;
    double tan_gmax = std::tan(gamma_max_);

    if (s_total < 1e-6) {
        // 路径长度为零 → 返回恒定高度
        return [z_end](double /*s*/) { return z_end; };
    }

    // ── 判断是否可在 gamma_max 约束内完成高度变化 ──
    if (std::abs(dz) <= s_total * tan_gmax + 1e-6) {
        // 在约束内 → 简单线性剖面
        double slope = dz / s_total;
        return [z_start, slope](double s) {
            return z_start + slope * s;
        };
    }

    // ── 超出约束 → 以最大坡度尽力接近目标高度 ──
    double sign = (dz > 0.0) ? 1.0 : -1.0;
    double s_climb = std::abs(dz) / tan_gmax;

    if (s_climb > s_total) {
        // 全程爬升/俯冲也无法到达 → 以最大坡度
        double actual_dz = sign * s_total * tan_gmax;
        double slope = actual_dz / s_total;
        return [z_start, slope](double s) {
            return z_start + slope * s;
        };
    }

    // ── 三段式剖面：爬升 + 平飞 + 下降 ──
    double s1 = s_climb;
    return [z_start, sign, tan_gmax, s1](double s) -> double {
        if (s <= s1) {
            return z_start + sign * s * tan_gmax;
        }
        // s1 以后保持目标高度（已到达）
        return z_start + sign * s1 * tan_gmax;
    };
}

// ═══════════════════════════════════════════════════════════════
// 路径采样
// ═══════════════════════════════════════════════════════════════

std::vector<Waypoint> DubinsPath3D::sample_path(
    const HorizontalDubinsParams& params,
    int n_arc, int n_straight) const
{
    std::vector<Waypoint> path;

    // ── 辅助 lambda：添加点 ──
    auto add_point = [&path](double x, double y, double z, double yaw) {
        path.push_back({x, y, z, yaw, 0.0, 0.0});
    };

    double z_start = params.t1.z;
    double z_end   = params.t2.z;

    // ── 第一段弧 ──
    double ang1 = params.arc1_angle;
    double sweep1 = ang1;  // 已带符号
    // 起始角：从 c1 指向起点
    double a0_1 = std::atan2(params.t1.y - params.c1.y - M_PI/2 * 0,  // 跳过
                             0);  // 此处需要计算正确的起始角
    // 重新计算：起点 psi_s → 圆心角
    // 从 c1 到起点的方向 = psi_s ± π/2
    // 更简单：直接用 sample_arc_2d 从切点开始

    // 实际上，我们采样时用简化方式：
    //  第一段弧：从起点位置 (通过 params.t1 的位置) 沿圆弧到 t1
    //  我们用到 sample_arc_2d 来采样
    // 
    // 为了简化，这里用起点->切点 直接采弧
    // 
    // 起点位置 = [sp[0], sp[1]]，c1 已知，切点 t1 已知
    // 从 c1 指向起点位置的角 start_angle_1

    // 第一个弧的起始点即起点位置（不是 t1）
    // 我们需要计算 start → t1 的弧
    // start 的圆心角: atan2(sp[1] - c1[1], sp[0] - c1[0])
    // t1 的圆心角:   atan2(t1[1] - c1[1], t1[0] - c1[0])

    // 由于我们没有保存起点坐标，这里使用 t1 近似
    // Phase 1 简化: 仅返回 3 个关键点（起点 / 切点1 / 切点2 / 终点）
    // 完整采样在 Phase 3 补充

    // ── 计算起点/终点位置（从 params 推导）──
    // 注意：params 中未存储原始起点/终点，这里需要外部传入
    // Phase 1 简化方案：返回 3 段关键点
    //  起点→t1（弧1）→直线→t2→终点（弧2）
    // 每段采 n 个点

    double c1_arr[2] = {params.c1.x, params.c1.y};
    double c2_arr[2] = {params.c2.x, params.c2.y};

    // 弧1：从 t1 的"前一位置"开始——实际是 start 点
    // 我们使用圆弧的角范围
    double sweep_arc1 = params.arc1_angle;  // 已带符号（CCW>0, CW<0）

    // 计算 c1→t1 的方向角
    double ang_t1 = std::atan2(params.t1.y - c1_arr[1],
                               params.t1.x - c1_arr[0]);
    // 圆弧起始角（从 c1 看 t1 的反方向 sweep）
    double start_ang1 = ang_t1 - sweep_arc1;  // t1 在弧末端

    // 采样弧1
    int n1 = std::max(2, n_arc);
    for (int i = 0; i < n1; ++i) {
        double frac = static_cast<double>(i) / (n1 - 1);
        double a = start_ang1 + frac * sweep_arc1;
        double x = c1_arr[0] + R_h_ * std::cos(a);
        double y = c1_arr[1] + R_h_ * std::sin(a);
        // 弧上的航向：半径方向旋转 π/2
        double yaw_on_arc = a + ((sweep_arc1 > 0) ? M_PI / 2.0 : -M_PI / 2.0);
        double z = z_start + frac * (z_end - z_start) * (params.arc1_len / params.total_len);
        add_point(x, y, z, yaw_on_arc);
    }

    // 直线段
    int n_s = std::max(2, n_straight);
    for (int i = 1; i < n_s; ++i) {  // 跳过 i=0（避免与弧1末端重复）
        double frac = static_cast<double>(i) / (n_s - 1);
        double x = params.t1.x + frac * (params.t2.x - params.t1.x);
        double y = params.t1.y + frac * (params.t2.y - params.t1.y);
        double z = z_start + (params.arc1_len + frac * params.straight_len) / params.total_len * (z_end - z_start);
        add_point(x, y, z, params.t1.yaw);  // 直行航向恒定
    }

    // 弧2：从 t2 到终点
    double ang_t2 = std::atan2(params.t2.y - c2_arr[1],
                               params.t2.x - c2_arr[0]);
    double sweep_arc2 = params.arc2_angle;
    double start_ang2 = ang_t2;  // t2 在弧起始

    int n2 = std::max(2, n_arc);
    for (int i = 1; i < n2; ++i) {  // 跳过 i=0
        double frac = static_cast<double>(i) / (n2 - 1);
        double a = start_ang2 + frac * sweep_arc2;
        double x = c2_arr[0] + R_h_ * std::cos(a);
        double y = c2_arr[1] + R_h_ * std::sin(a);
        double yaw_on_arc = a + ((sweep_arc2 > 0) ? M_PI / 2.0 : -M_PI / 2.0);
        double z = z_start + (params.arc1_len + params.straight_len + frac * params.arc2_len) / params.total_len * (z_end - z_start);
        add_point(x, y, z, yaw_on_arc);
    }

    return path;
}

// ═══════════════════════════════════════════════════════════════
// 最优路径查找
// ═══════════════════════════════════════════════════════════════

double DubinsPath3D::best_path(const double* start, const double* goal,
                               std::vector<Waypoint>& out_path) const
{
    auto candidates = find_horizontal_tangents(start, goal);
    if (candidates.empty()) {
        return -1.0;  // 不可行
    }

    // 取最短的（已按 total_len 升序排列）
    const auto& best = candidates[0];

    // 采样路径
    out_path = sample_path(best, 20, 10);

    return best.total_len;
}

// ═══════════════════════════════════════════════════════════════
// 路径安全检查
// ═══════════════════════════════════════════════════════════════

bool DubinsPath3D::is_path_safe(const std::vector<Waypoint>& path) const
{
    if (!obstacles_ || obstacles_->empty() || path.size() < 2) {
        return true;  // 无障碍物或路径太短 → 安全
    }

    // 构建连续存储的坐标数组用于 swept_sphere_path
    std::vector<std::array<double, 3>> storage;
    storage.reserve(path.size());
    for (const auto& pp : path) {
        storage.push_back({pp.x, pp.y, pp.z});
    }

    std::vector<const double*> wps;
    wps.reserve(storage.size());
    for (const auto& s : storage) {
        wps.push_back(s.data());
    }

    // swept_sphere_path 返回 true=有碰撞，取反表示安全
    return !swept_sphere_path(wps, r_body_, *obstacles_);
}

}  // namespace beam_dubins
