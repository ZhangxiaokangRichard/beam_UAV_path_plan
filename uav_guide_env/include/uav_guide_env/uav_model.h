/**
 * @file uav_model.h
 * @brief 固定翼 UAV 运动学模型（从 Python env/uav.py → FixedWingUAV 迁移）
 *
 * 提供 UAV 运动学约束参数（供规划层只读）和运动学推进方法（供仿真层调用）。
 * 模型假设：
 *  - 定速巡航 V（无加减速）
 *  - 协调转弯（bank-to-turn），有最小转弯半径 R_min
 *  - 最大俯仰角 γ_max 约束垂直爬升/俯冲速率
 *  - 球形碰撞体 radius=r_body
 */

#pragma once

#include <array>
#include <vector>

namespace uav_guide_env {

/**
 * @brief UAV 完整状态（仿真可视化使用）
 */
struct UAVState {
    std::array<double, 5> pose;     // [x, y, z, yaw, pitch]
    std::array<double, 4> sphere;   // [x, y, z, r] 碰撞球
    std::array<double, 3> heading;  // [dx, dy, dz] 归一化方向向量
};

/**
 * @class FixedWingUAV
 * @brief 固定翼无人机运动学模型
 *
 * 规划约束参数 (V, R_min, gamma_max, r_body) 供规划层只读访问。
 * 运动学方法 (step, advanceAlongPath) 供仿真循环调用。
 */
class FixedWingUAV {
public:
    /**
     * @brief 从参数构造 UAV 模型
     * @param cruise_speed_mps   巡航速度 (m/s)
     * @param min_turn_radius_m  最小水平转弯半径 (m)
     * @param max_pitch_angle_deg 最大俯仰角 (°)
     * @param collision_radius_m 碰撞球半径 (m)
     * @param sim_dt             仿真步长 (s)，默认 0.1
     */
    FixedWingUAV(double cruise_speed_mps, double min_turn_radius_m,
                 double max_pitch_angle_deg, double collision_radius_m,
                 double sim_dt = 0.1);

    // ── 运动学积分（低层级，一步推进）──────────────────────

    /**
     * @brief 运动学积分一步
     * @param pose  当前状态 [x, y, z, yaw, pitch]
     * @param bank  当前坡度角 (rad)，用于转弯
     * @param gamma 目标俯仰角 (rad)
     * @param dt    积分时长 (s)
     * @return 新状态 [x, y, z, yaw, pitch]
     */
    std::array<double, 5> step(const std::array<double, 5>& pose,
                               double bank, double gamma, double dt) const;

    // ── 路径跟随（高层级，沿离散路径推进）──────────────────

    /**
     * @brief 沿路径推进一段距离（"只进不退指针"策略）
     *
     * 从 Python beam_dubins_2d_dynamic.py → _advance_uav() 迁移。
     * 路径耗尽时朝目标方向死推算。
     *
     * @param pose       UAV 当前状态 [x, y, z, yaw, pitch]（输入输出）
     * @param path       待跟随的路径点列 [[x,y,z,yaw], ...]
     * @param path_ptr   路径推进指针（输入输出，只增不减）
     * @param last_path_id 上次路径的标识（输入输出，用于检测路径更新）
     * @param goal       目标位置 [x, y, z, yaw, pitch]（死推算用）
     * @param distance   本次推进距离 (m) = V × dt
     */
    void advanceAlongPath(std::array<double, 5>& pose,
                          const std::vector<std::array<double, 4>>& path,
                          int& path_ptr, size_t& last_path_gen,
                          const std::array<double, 5>& goal,
                          double distance) const;

    // ── 辅助方法 ───────────────────────────────────────────

    /// 获取归一化方向向量 [dx, dy, dz]
    std::array<double, 3> headingVector(double yaw, double pitch) const;

    /// 从路径点构建完整 UAVState（用于可视化）
    UAVState getState(const std::array<double, 4>& wp) const;

    // ── 访问器（规划约束参数，只读）────────────────────────
    double getCruiseSpeed()     const { return V_; }
    double getMinTurnRadius()   const { return R_min_; }
    double getMaxPitchAngle()   const { return gamma_max_; }
    double getCollisionRadius() const { return r_body_; }
    double getSimDt()           const { return dt_; }

private:
    /// 朝目标方向直线死推算（路径耗尽时的兜底策略）
    void deadReckonToward(std::array<double, 5>& pose,
                          const std::array<double, 5>& goal,
                          double distance) const;

    // ── 运动学参数 ──
    double V_;           // 巡航速度 (m/s)
    double R_min_;       // 最小水平转弯半径 (m)
    double gamma_max_;   // 最大俯仰角 (rad)
    double r_body_;      // 碰撞球半径 (m)
    double dt_;          // 默认仿真步长 (s)
    double g_;           // 重力加速度 (m/s²)
};

}  // namespace uav_guide_env
