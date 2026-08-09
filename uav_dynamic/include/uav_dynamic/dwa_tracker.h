#pragma once

#include <vector>

#include "beam_dubins/PathPoint.h"

namespace uav_dynamic {

/// 水平运动参数（前向步插值跟踪 + 协调转弯律）。
struct MotionParams {
    double speed_mps = 50.0;
    double min_turn_radius_m = 500.0;
    double bank_angle_max_deg = 25.0;
    double g = 9.81;
    double solve_hz = 20.0;    // 主循环更新频率（前向步 = speed_mps / solve_hz，如 50/20=2.5m）
    double lookahead_m = 100.0;
    int sample_num = 21;
    double sample_window_rad = 0.2;
    double roll_weight = 0.2;   // 滚转惩罚（仅轻微平滑，保证航向收敛到期望，误差最小）
};

/// 纵向运动参数（高度误差 PI）。
struct VerticalParams {
    double kp = 0.6;
    double ki = 0.02;
    double climb_vel_limit_mps = 3.0;
    double alpha_limit_deg = 30.0;
    double alpha_gain = 0.175;  // =30°/3m/s：climb 限幅内迎角线性对应，不提前饱和
    double integral_limit = 8.0;
};

/// 解算输出。
struct SolveCmd {
    bool success = false;
    double heading_rad = 0.0;   // 期望航向（ENU 弧度）
    double roll_rad = 0.0;      // 期望滚转角（协调转弯律）
    double pitch_rad = 0.0;     // 期望迎角（纵向 PI，限幅）
    double climb_mps = 0.0;     // 期望垂直速度（限幅）
};

/**
 * @brief 水平跟踪：取 path 最后一个圆弧的入口切线航向（快速调整航向跟上 target）。
 *   - 找 path 最后一个圆弧段（curvature≠0）的入口切点，提取圆心 C 与半径 R
 *   - 目标航向 = 进入该圆前的切线航向指向（入口切点相对圆心角 ± 90°，不对 yaw 限幅）
 *   - roll = 协调转弯律 sign·atan2(v², g·R)（左转正/右转负），限幅 ±bank_angle_max_deg
 *   - 无圆弧（纯直线 Dubins）→ 沿当前 yaw 以巡航速度向前推算（roll=0）
 * @param path  全局离散路径（起点≈uav 当前位置，PathPoint 含 curvature）
 * @param p     运动参数
 * @param cur_yaw  uav 当前航向（ENU 弧度）
 * @param roll_out  输出协调转弯滚转角（限幅）
 * @return 期望航向（ENU 弧度，归一化，不对 yaw 限幅）
 */
double horizontalSolve(const std::vector<beam_dubins::PathPoint>& path,
                       const MotionParams& p,
                       double cur_yaw,
                       double& roll_out);

/**
 * @brief 纵向跟踪：高度误差 PI。
 * @param height_err  target_height - uav_height
 * @param vp          纵向参数
 * @param dt          积分步长（秒）
 * @param alpha_out   输出迎角（限幅，弧度）
 * @param integral    积分状态（调用方维护）
 * @return 期望垂直速度（限幅）
 */
double verticalSolve(double height_err,
                     const VerticalParams& vp,
                     double dt,
                     double& alpha_out,
                     double& integral);

}  // namespace uav_dynamic
