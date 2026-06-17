/**
 * @file geometry.cpp
 * @brief 几何工具函数实现
 *
 * 从 Python utils/geometry.py 逐函数迁移，保持数学逻辑完全一致。
 */

#include "beam_dubins/geometry.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 距离计算
// ═══════════════════════════════════════════════════════════════

double distance_2d(const double* p1, const double* p2) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    return std::sqrt(dx * dx + dy * dy);
}

double distance_3d(const double* p1, const double* p2, double z_weight) {
    double dx = p2[0] - p1[0];
    double dy = p2[1] - p1[1];
    double dz = (p2[2] - p1[2]) * z_weight;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// ═══════════════════════════════════════════════════════════════
// 角度工具
// ═══════════════════════════════════════════════════════════════

double wrap_angle(double a) {
    // Python: (a + π) % (2π) - π，输出范围 (-π, π]
    double two_pi = 2.0 * M_PI;
    a = std::fmod(a + M_PI, two_pi);
    if (a < 0.0) {
        a += two_pi;    // C++ fmod 对负数行为与 Python % 不同，手动修正
    }
    // 处理浮点精度导致的边界情况：如果结果 ≈ -π，转为 +π
    double result = a - M_PI;
    if (std::abs(result + M_PI) < 1e-12) {
        result = M_PI;
    }
    return result;
}

double angle_diff(double a, double b) {
    return wrap_angle(b - a);
}

double ccw_arc_angle(double start_ang, double end_ang) {
    // 从 start 逆时针到 end 的角度 ∈ [0, 2π)
    double d = wrap_angle(end_ang - start_ang);
    return (d >= 0.0) ? d : d + 2.0 * M_PI;
}

double cw_arc_angle(double start_ang, double end_ang) {
    // 从 start 顺时针到 end 的角度 ∈ [0, 2π)
    double d = wrap_angle(start_ang - end_ang);
    return (d >= 0.0) ? d : d + 2.0 * M_PI;
}

double arc_length_2d(double r, double dtheta) {
    return r * std::abs(dtheta);
}

// ═══════════════════════════════════════════════════════════════
// 方向角计算
// ═══════════════════════════════════════════════════════════════

double directional_yaw(const double* p1, const double* p2) {
    return std::atan2(p2[1] - p1[1], p2[0] - p1[0]);
}

double directional_pitch(const double* p1, const double* p2) {
    double dist_xy = distance_2d(p1, p2);
    double dz = p2[2] - p1[2];
    if (dist_xy < 1e-9) {
        // 纯垂直方向：正负 90°
        return std::copysign(M_PI / 2.0, dz);
    }
    return std::atan2(dz, dist_xy);
}

// ═══════════════════════════════════════════════════════════════
// 向量运算
// ═══════════════════════════════════════════════════════════════

void rotate_2d(const double* v, double theta, double* out) {
    double c = std::cos(theta);
    double s = std::sin(theta);
    out[0] = c * v[0] - s * v[1];
    out[1] = s * v[0] + c * v[1];
}

void normalize_2d(const double* v, double* out) {
    double n = std::sqrt(v[0] * v[0] + v[1] * v[1]);
    if (n < 1e-12) {
        out[0] = 0.0;
        out[1] = 0.0;
    } else {
        out[0] = v[0] / n;
        out[1] = v[1] / n;
    }
}

void left_normal_2d(const double* v, double* out) {
    // 逆时针旋转 90°：[-y, x]
    out[0] = -v[1];
    out[1] =  v[0];
}

// ═══════════════════════════════════════════════════════════════
// 圆弧与圆心
// ═══════════════════════════════════════════════════════════════

std::vector<std::vector<double>> sample_arc_2d(
    const double* center, double r, double start_ang, double sweep, int n)
{
    // 在 [start_ang, start_ang + sweep] 上均匀采样 n 个点
    std::vector<std::vector<double>> points;
    points.reserve(n);

    double cx = center[0];
    double cy = center[1];

    for (int i = 0; i < n; ++i) {
        double frac = static_cast<double>(i) / static_cast<double>(n - 1);
        double ang  = start_ang + frac * sweep;
        points.push_back({cx + r * std::cos(ang), cy + r * std::sin(ang)});
    }
    return points;
}

void left_center(const double* pos, double R, double* out) {
    // pos = [x, y, ψ]
    // 左圆心 = (x - R*sin(ψ), y + R*cos(ψ))
    double psi = pos[2];
    out[0] = pos[0] - R * std::sin(psi);
    out[1] = pos[1] + R * std::cos(psi);
}

void right_center(const double* pos, double R, double* out) {
    // pos = [x, y, ψ]
    // 右圆心 = (x + R*sin(ψ), y - R*cos(ψ))
    double psi = pos[2];
    out[0] = pos[0] + R * std::sin(psi);
    out[1] = pos[1] - R * std::cos(psi);
}

}  // namespace beam_dubins
