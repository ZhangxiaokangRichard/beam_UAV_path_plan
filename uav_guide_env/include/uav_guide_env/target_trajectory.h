/**
 * @file target_trajectory.h
 * @brief 动态目标轨迹生成器（从 Python test_cases/scenario_3d.py → TargetTrajectory 迁移）
 *
 * 支持四种轨迹类型：
 *  - static：静止目标
 *  - line：匀速直线运动
 *  - racetrack：田径赛道（直线 + 半圆弯道）
 *  - circle：匀速圆周运动
 *
 * 接口对齐 Python 原型：position_at(t) → [x, y, z, yaw, pitch]
 */

#pragma once

#include <array>
#include <string>

namespace uav_guide_env {

/**
 * @struct ChannelParams
 * @brief 通道障碍物配置（AABB 墙壁，形成引导通道）
 */
struct ChannelParams {
    double inner_width    = 100.0;   // 通道内部宽度（两墙内边间距，m）
    double wall_length    = 2000.0;  // 墙壁沿通道方向长度（m）
    double wall_thickness = 2000.0;  // 墙壁垂直通道方向厚度（m）
    double wall_height    = 1000.0;  // 墙壁高度（m）
    double safe_margin    = 5.0;     // 安全边距（m）
    double wall_z_base    = 0.0;     // 墙壁底部 Z（m）
};

/**
 * @struct ChannelWalls
 * @brief 通道两侧墙壁的 AABB（min/max 角点）
 */
struct ChannelWalls {
    std::array<double, 3> left_min;   // 左墙 aabb_min
    std::array<double, 3> left_max;   // 左墙 aabb_max
    std::array<double, 3> right_min;  // 右墙 aabb_min
    std::array<double, 3> right_max;  // 右墙 aabb_max
};

/**
 * @class TargetTrajectory
 * @brief 动态目标轨迹生成器
 *
 * 无状态：给定时间 t，返回目标在 t 时刻的 5D 位姿。
 * 配置参数（初始位置、速度、方向等）在构造时传入。
 */
class TargetTrajectory {
public:
    /**
     * @brief 构造函数
     * @param traj_type  轨迹类型："static" / "line" / "racetrack" / "circle"
     * @param init_pos   初始位置 [x, y, z, yaw_deg=0]
     * @param params     轨迹参数（按类型不同）：
     *   line:    {direction_deg, speed_mps, length_m}
     *   circle:  {center_x, center_y, center_z, radius_m, angular_speed_dps}
     *   racetrack: {straight_len_m, turn_radius_m, direction_deg, altitude_m, lap_speed_mps}
     */
    TargetTrajectory(const std::string& traj_type,
                     const std::array<double, 3>& init_pos,
                     const std::array<double, 5>& params = {});

    /**
     * @brief 获取目标在时刻 t 的位姿
     * @param t  仿真时间 (s)
     * @return [x, y, z, yaw, pitch]
     */
    std::array<double, 5> positionAt(double t) const;

    /**
     * @brief 设置通道障碍物参数
     */
    void setChannelParams(const ChannelParams& params) { channel_ = params; }

    /**
     * @brief 获取通道障碍物参数
     */
    const ChannelParams& getChannelParams() const { return channel_; }

    /**
     * @brief 计算通道两侧墙壁的 AABB
     * @param target_state  目标当前 5D 位姿 [x, y, z, yaw, pitch]
     * @return 左右墙的 min/max 角点
     *
     * 墙壁沿目标航向延伸（dy=wall_length），
     * 通道缺口垂直航向（dx=wall_thickness），
     * 左墙在航向左侧（yaw-π/2 方向），右墙在航向右侧（yaw+π/2 方向）。
     */
    ChannelWalls computeChannelWalls(const std::array<double, 5>& target_state) const;

    // ── 访问器 ───────────────────────────────────────────────
    const std::string& getType()          const { return traj_type_; }
    const std::array<double, 3>& getInitPos() const { return init_pos_; }

private:
    // ── 各轨迹类型的内部计算 ───────────────────────────────

    std::array<double, 5> staticPos(double t) const;
    std::array<double, 5> linePos(double t) const;
    std::array<double, 5> circlePos(double t) const;
    std::array<double, 5> racetrackPos(double t) const;

    // ── 配置 ──
    std::string traj_type_;
    std::array<double, 3> init_pos_;   // [x0, y0, z0]

    // ── line 参数 ──
    double line_direction_ = 0.0;      // rad
    double line_speed_     = 20.0;     // m/s
    double line_length_    = 4000.0;   // m

    // ── circle 参数 ──
    double circle_cx_ = 8000.0, circle_cy_ = 2500.0, circle_cz_ = 600.0;
    double circle_radius_ = 100.0;     // m
    double circle_omega_  = 0.0087266;  // rad/s (=0.5°/s)

    // ── racetrack 参数 ──
    double rt_straight_len_ = 1000.0;  // m
    double rt_turn_radius_  = 300.0;   // m
    double rt_direction_    = 0.0;     // rad
    double rt_altitude_     = 600.0;   // m
    double rt_lap_speed_    = 30.0;    // m/s

    // ── 通道障碍物 ──
    ChannelParams channel_;
};

}  // namespace uav_guide_env
