#include "uav_dynamic/dwa_tracker.h"

#include <algorithm>
#include <cmath>

namespace uav_dynamic {
namespace {
constexpr double kPi = 3.14159265358979323846;

inline double wrap_angle(double a)
{
    while (a >  kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}

double horizontalSolve(const std::vector<beam_dubins::PathPoint>& path,
                       const MotionParams& p,
                       double cur_yaw,
                       double& roll_out)
{
    roll_out = 0.0;
    if (path.size() < 2) return cur_yaw;

    const double v = p.speed_mps;
    const double roll_max = p.bank_angle_max_deg * kPi / 180.0;
    const double yaw_tol = 1e-3;   // 相邻点 yaw 差阈值（rad），> 此值视为转弯段

    // 1) 找 path 中最后一个转弯段（相邻点 yaw 差 ≠ 0，Dubins 末尾圆弧段）。
    //    注意：beam_dubins 输出的 path 中 curvature 字段可能恒为 0（未填充），
    //    因此改用相邻点 yaw 差检测转弯，而非依赖 curvature。
    int last_turn = -1;
    for (int i = static_cast<int>(path.size()) - 1; i > 0; --i) {
        if (std::abs(wrap_angle(path[i].yaw - path[i - 1].yaw)) > yaw_tol) {
            last_turn = i;
            break;
        }
    }
    if (last_turn < 0) {
        // 无转弯（纯直线 Dubins）→ 沿当前 yaw 巡航
        roll_out = 0.0;
        return cur_yaw;
    }
    // 2) 入口切点：从最后转弯点向前跳过整个转弯段，
    //    停在进入圆弧前的切点（直线段末点，其 yaw = 进入圆前的切线航向）
    int entry = last_turn;
    while (entry > 0 &&
           std::abs(wrap_angle(path[entry].yaw - path[entry - 1].yaw)) > yaw_tol) {
        entry--;
    }

    // 3) 转弯方向 sign（左转+ / 右转-）：切点之后第一个转弯点的 yaw 变化
    const int next = std::min(entry + 1, static_cast<int>(path.size()) - 1);
    const double d = wrap_angle(path[next].yaw - path[entry].yaw);
    const int sign = (d > 0.0) ? +1 : -1;

    // 4) 转弯半径 R：由转弯段弧长 / 转角估计（ds/dψ），兜底 min_turn_radius_m
    double sum_ds = 0.0, sum_dpsi = 0.0;
    for (int i = entry + 1; i <= last_turn; ++i) {
        const double dd = wrap_angle(path[i].yaw - path[i - 1].yaw);
        sum_dpsi += std::abs(dd);
        sum_ds += std::hypot(path[i].x - path[i - 1].x,
                             path[i].y - path[i - 1].y);
    }
    double R = (sum_dpsi > 1e-6) ? (sum_ds / sum_dpsi) : p.min_turn_radius_m;
    R = std::max(R, p.min_turn_radius_m);

    // 5) 目标航向 = 进入该圆前的切线航向指向（入口切点 yaw），
    //    更新到当前 uav 航向（不对 yaw 限幅，仅归一化）→ 航向快速调整跟上 target
    double yaw = path[entry].yaw;
    yaw = wrap_angle(yaw);

    // 6) 协调转弯律滚转：roll = sign·atan2(v², g·R)（左转正/右转负），限幅 ±25°
    double roll = static_cast<double>(sign) * std::atan2(v * v, p.g * R);
    roll = std::clamp(roll, -roll_max, roll_max);
    roll_out = roll;
    return yaw;
}

double verticalSolve(double height_err,
                     const VerticalParams& vp,
                     double dt,
                     double& alpha_out,
                     double& integral)
{
    const double climb_raw = vp.kp * height_err + vp.ki * integral;
    double climb = std::clamp(climb_raw, -vp.climb_vel_limit_mps, vp.climb_vel_limit_mps);

    // anti-windup：输出饱和且误差驱动方向与饱和方向一致时冻结积分，
    // 避免退出饱和后积分滞后造成的超调/振荡，保证纵向渐近稳定。
    const bool saturated = (climb_raw <= -vp.climb_vel_limit_mps) ||
                           (climb_raw >= vp.climb_vel_limit_mps);
    const bool same_dir = (climb_raw >= 0.0) == (height_err >= 0.0);
    if (!(saturated && same_dir)) {
        integral += height_err * dt;
        integral = std::clamp(integral, -vp.integral_limit, vp.integral_limit);
    }

    const double alpha_max = vp.alpha_limit_deg * kPi / 180.0;
    const double alpha = vp.alpha_gain * climb;   // 垂直速度 → 迎角（rad）
    alpha_out = std::clamp(alpha, -alpha_max, alpha_max);
    return climb;
}

}  // namespace uav_dynamic
