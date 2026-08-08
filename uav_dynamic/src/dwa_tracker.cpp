#include "uav_dynamic/dwa_tracker.h"

#include <algorithm>
#include <cmath>

namespace uav_dynamic {
namespace {
constexpr double kPi = 3.14159265358979323846;
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

    // 1) 前视点：沿路径弧长累计到 lookahead
    std::size_t idx = path.size() - 1;
    double acc = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        acc += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        if (acc >= p.lookahead_m) { idx = i; break; }
    }
    const double dx = path[idx].x - path[0].x;
    const double dy = path[idx].y - path[0].y;
    const double desired_yaw = std::atan2(dy, dx);

    // 2) DWA：在期望航向 ± window 采样候选航向
    //    协调转弯：R = lookahead/|Δψ|，roll = atan2(v², g·R)，限幅 bank_angle_max
    //    代价 = |候选−期望航向|（主项，保证与期望轨迹误差最小）
    //          + roll_weight·|roll|（轻微平滑，抑制过度滚转）
    double best_yaw = desired_yaw;
    double best_roll = 0.0;
    double best_cost = 1e18;
    const int n = p.sample_num;
    for (int i = -(n / 2); i <= n / 2; ++i) {
        const double cand = desired_yaw + static_cast<double>(i) *
                              (2.0 * p.sample_window_rad / static_cast<double>(n - 1));
        double dyaw = cand - cur_yaw;   // 转向量（决定协调转弯 roll）
        while (dyaw >  kPi) dyaw -= 2.0 * kPi;
        while (dyaw < -kPi) dyaw += 2.0 * kPi;

        // 协调转弯律：转弯半径 R = lookahead / |Δψ|（小角度近似，最小转弯半径兜底）
        double R = (std::abs(dyaw) < 1e-6)
                       ? 1e6
                       : p.lookahead_m / std::abs(dyaw);
        R = std::max(R, p.min_turn_radius_m);
        double roll = std::atan2(v * v, p.g * R);
        roll = std::clamp(roll, -roll_max, roll_max);

        // 对期望航向的偏差（wrap）
        double d_des = cand - desired_yaw;
        while (d_des >  kPi) d_des -= 2.0 * kPi;
        while (d_des < -kPi) d_des += 2.0 * kPi;

        const double cost = std::abs(d_des) + p.roll_weight * std::abs(roll);
        if (cost < best_cost) {
            best_cost = cost;
            best_yaw = cand;
            best_roll = roll;
        }
    }

    roll_out = best_roll;
    return best_yaw;
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
