/**
 * @file target_trajectory.cpp
 * @brief 目标轨迹生成器实现
 *
 * 从 Python test_cases/scenario_3d.py → TargetTrajectory 迁移。
 * 四种轨迹类型的运动学公式与原 Python 代码保持一致。
 */

#include "uav_guide_env/target_trajectory.h"

#include <cmath>
#include <stdexcept>

namespace uav_guide_env {

// ═══════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════

TargetTrajectory::TargetTrajectory(const std::string& traj_type,
                                   const std::array<double, 3>& init_pos,
                                   const std::array<double, 5>& params)
    : traj_type_(traj_type)
    , init_pos_(init_pos)
{
    // 根据轨迹类型解析参数（YAML 角度：0°=北+Y，90°=东+X，180°=南-Y）
    if (traj_type == "line") {
        double nav_deg = params[0];
        line_direction_ = (90.0 - nav_deg) * M_PI / 180.0;  // 导航→数学坐标
        line_speed_     = params[1];
        line_length_    = params[2];
    } else if (traj_type == "circle") {
        circle_cx_    = params[0];
        circle_cy_    = params[1];
        circle_cz_    = params[2];
        circle_radius_ = params[3];
        circle_omega_  = params[4] * M_PI / 180.0;    // °/s→rad/s
    } else if (traj_type == "racetrack") {
        rt_straight_len_ = params[0];
        rt_turn_radius_  = params[1];
        double nav_deg = params[2];
        rt_direction_    = (90.0 - nav_deg) * M_PI / 180.0;
        rt_altitude_     = params[3];
        rt_lap_speed_    = params[4];
    }
    // "static" 不需要额外参数
}

// ═══════════════════════════════════════════════════════════════
// 公开接口
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::positionAt(double t) const
{
    if (traj_type_ == "static")    return staticPos(t);
    if (traj_type_ == "line")      return linePos(t);
    if (traj_type_ == "circle")    return circlePos(t);
    if (traj_type_ == "racetrack") return racetrackPos(t);

    // 未知类型 → 回退为静止
    return staticPos(t);
}

ChannelWalls TargetTrajectory::computeChannelWalls(
    const std::array<double, 5>& target_state) const
{
    const auto& ch = channel_;
    double tx = target_state[0];
    double ty = target_state[1];
    double tz = target_state[2];
    double yaw = target_state[3];

    // 通道方向 = 目标航向；缺口垂直航向
    // 墙壁设计（以目标朝北 yaw=π/2 为例）：
    //   - 通道沿 Y 轴（航向方向），墙壁长 wall_length
    //   - 缺口沿 X 轴（垂直航向），墙壁厚 wall_thickness
    //   - 左墙在西 (-X)，右墙在东 (+X)，目标居中
    // 航向左侧方向 = yaw + π/2（北偏西），右侧 = yaw - π/2（北偏东）
    double left_x  = std::cos(yaw + M_PI / 2.0);  // 左方向 X
    double left_y  = std::sin(yaw + M_PI / 2.0);  // 左方向 Y
    double right_x = std::cos(yaw - M_PI / 2.0);  // 右方向 X
    double right_y = std::sin(yaw - M_PI / 2.0);  // 右方向 Y

    // 墙中心在缺口方向的偏移（含安全边距）
    double half_gap = ch.inner_width / 2.0 + ch.safe_margin;
    double wall_half_thick = ch.wall_thickness / 2.0;
    double wall_half_len   = ch.wall_length / 2.0;
    double wall_half_h     = ch.wall_height / 2.0;

    // 左墙中心 = 目标 + (半通道宽 + 半墙厚) 向左
    double lcx = tx + (half_gap + wall_half_thick) * left_x;
    double lcy = ty + (half_gap + wall_half_thick) * left_y;

    // 右墙中心 = 目标 + (半通道宽 + 半墙厚) 向右
    double rcx = tx + (half_gap + wall_half_thick) * right_x;
    double rcy = ty + (half_gap + wall_half_thick) * right_y;

    double wall_cz = ch.wall_z_base + wall_half_h;

    ChannelWalls walls;
    double sm = ch.safe_margin;

    // ── 计算 AABB 半尺寸（投影到世界 XY 轴）─────────────────
    // wall_thickness 沿缺口方向（垂直航向），wall_length 沿通道方向（航向）
    // 将正交方向的半尺寸投影到世界 XY 坐标，支持任意航向
    double hw = wall_half_thick + sm;   // 缺口方向半尺寸（含安全边距）
    double hl = wall_half_len   + sm;   // 通道方向半尺寸（含安全边距）
    double abs_sin = std::abs(std::sin(yaw));
    double abs_cos = std::abs(std::cos(yaw));
    double hx = hw * abs_sin + hl * abs_cos;  // AABB X 半尺寸
    double hy = hw * abs_cos + hl * abs_sin;  // AABB Y 半尺寸

    // 左墙 AABB（墙中心 ± 半尺寸）
    walls.left_min  = {lcx - hx, lcy - hy, ch.wall_z_base - sm};
    walls.left_max  = {lcx + hx, lcy + hy, ch.wall_z_base + ch.wall_height + sm};

    // 右墙 AABB
    walls.right_min = {rcx - hx, rcy - hy, ch.wall_z_base - sm};
    walls.right_max = {rcx + hx, rcy + hy, ch.wall_z_base + ch.wall_height + sm};

    return walls;
}

// ═══════════════════════════════════════════════════════════════
// Phase 3 v0.1：虚拟目标计算（alg_phase3_v01.md §2.4）
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::computeVirtualGoal(
    const std::array<double, 5>& target_state,
    const std::string& mode,
    double L)
{
    double tx = target_state[0], ty = target_state[1], tz = target_state[2];
    double psi = target_state[3], theta = target_state[4];
    double c = std::cos(psi), s = std::sin(psi);

    if (mode == "head-on") {
        // 迎头截击：目标前方 L 米，航向反向（面对面）
        double front_yaw = psi + M_PI;
        while (front_yaw >  M_PI) front_yaw -= 2.0 * M_PI;
        while (front_yaw < -M_PI) front_yaw += 2.0 * M_PI;
        return {tx + L * c, ty + L * s, tz, front_yaw, theta};
    } else {
        // 尾追（默认）：目标后方 L 米，航向同向
        return {tx - L * c, ty - L * s, tz, psi, theta};
    }
}

// ═══════════════════════════════════════════════════════════════
// static：静止目标
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::staticPos(double /*t*/) const
{
    return {init_pos_[0], init_pos_[1], init_pos_[2], 0.0, 0.0};
}

// ═══════════════════════════════════════════════════════════════
// line：匀速直线运动
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::linePos(double t) const
{
    // line_direction_ 已是数学坐标弧度（0=东+X, π/2=北+Y）
    double dist = std::min(line_speed_ * t, line_length_);
    double x = init_pos_[0] + dist * std::cos(line_direction_);
    double y = init_pos_[1] + dist * std::sin(line_direction_);
    double z = init_pos_[2];
    double yaw = line_direction_;  // 输出数学坐标 yaw，与系统其他部分一致
    return {x, y, z, yaw, 0.0};
}

// ═══════════════════════════════════════════════════════════════
// circle：匀速圆周运动
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::circlePos(double t) const
{
    double ang = circle_omega_ * t;
    double x = circle_cx_ + circle_radius_ * std::cos(ang);
    double y = circle_cy_ + circle_radius_ * std::sin(ang);
    double z = circle_cz_;
    // 航向为切线方向（垂直于半径）
    double yaw = ang + M_PI / 2.0;
    return {x, y, z, yaw, 0.0};
}

// ═══════════════════════════════════════════════════════════════
// racetrack：田径赛道（直线 + 180° 半圆弯道）
// ═══════════════════════════════════════════════════════════════

std::array<double, 5> TargetTrajectory::racetrackPos(double t) const
{
    double lap_dist = rt_lap_speed_ * t;                     // 累计路程
    double half_circle_len = M_PI * rt_turn_radius_;          // 半圆周长
    double lap_len = 2.0 * (rt_straight_len_ + half_circle_len); // 一圈总长
    double remainder = std::fmod(lap_dist, lap_len);
    if (remainder < 0.0) remainder += lap_len;

    double x, y, yaw;

    if (remainder < rt_straight_len_) {
        // ── 第一段直线 ──
        double frac = remainder / rt_straight_len_;
        x = init_pos_[0] + frac * rt_straight_len_ * std::cos(rt_direction_);
        y = init_pos_[1] + frac * rt_straight_len_ * std::sin(rt_direction_);
        yaw = rt_direction_;
    } else if (remainder < rt_straight_len_ + half_circle_len) {
        // ── 第一弯道（180° 右转或左转）────────────────────
        double frac = (remainder - rt_straight_len_) / half_circle_len;
        // 弯曲中心：直线终点 + 右转90°方向 × R
        double cx = init_pos_[0] + rt_straight_len_ * std::cos(rt_direction_)
                    + rt_turn_radius_ * std::cos(rt_direction_ - M_PI / 2.0);
        double cy = init_pos_[1] + rt_straight_len_ * std::sin(rt_direction_)
                    + rt_turn_radius_ * std::sin(rt_direction_ - M_PI / 2.0);
        double ang_start = rt_direction_ + M_PI / 2.0;
        double ang = ang_start + frac * M_PI;  // 180° 半圆
        x = cx + rt_turn_radius_ * std::cos(ang);
        y = cy + rt_turn_radius_ * std::sin(ang);
        yaw = ang + M_PI / 2.0;  // 切线方向
    } else if (remainder < 2.0 * rt_straight_len_ + half_circle_len) {
        // ── 第二段直线（反向）────────────────────────────
        double frac = (remainder - rt_straight_len_ - half_circle_len) / rt_straight_len_;
        // 第二直线起点 = 第一弯道终点（即第一直线终点反向平移 2R）
        double x0 = init_pos_[0] + rt_straight_len_ * std::cos(rt_direction_)
                    + 2.0 * rt_turn_radius_ * std::cos(rt_direction_ - M_PI / 2.0);
        double y0 = init_pos_[1] + rt_straight_len_ * std::sin(rt_direction_)
                    + 2.0 * rt_turn_radius_ * std::sin(rt_direction_ - M_PI / 2.0);
        double rev_dir = rt_direction_ + M_PI;  // 反方向
        x = x0 + frac * rt_straight_len_ * std::cos(rev_dir);
        y = y0 + frac * rt_straight_len_ * std::sin(rev_dir);
        yaw = rev_dir;
    } else {
        // ── 第二弯道（回到起点）──────────────────────────
        double frac = (remainder - 2.0 * rt_straight_len_ - half_circle_len) / half_circle_len;
        // 弯曲中心：第二直线终点 + 右转90°方向 × R
        double x0 = init_pos_[0] + rt_straight_len_ * std::cos(rt_direction_)
                    + 2.0 * rt_turn_radius_ * std::cos(rt_direction_ - M_PI / 2.0);
        double y0 = init_pos_[1] + rt_straight_len_ * std::sin(rt_direction_)
                    + 2.0 * rt_turn_radius_ * std::sin(rt_direction_ - M_PI / 2.0);
        double rev_dir = rt_direction_ + M_PI;
        double cx = x0 + rt_straight_len_ * std::cos(rev_dir)
                    + rt_turn_radius_ * std::cos(rt_direction_ + M_PI / 2.0);
        double cy = y0 + rt_straight_len_ * std::sin(rev_dir)
                    + rt_turn_radius_ * std::sin(rt_direction_ + M_PI / 2.0);
        double ang_start = rt_direction_ - M_PI / 2.0;
        double ang = ang_start + frac * M_PI;
        x = cx + rt_turn_radius_ * std::cos(ang);
        y = cy + rt_turn_radius_ * std::sin(ang);
        yaw = ang + M_PI / 2.0;
    }

    return {x, y, rt_altitude_, yaw, 0.0};
}

}  // namespace uav_guide_env
