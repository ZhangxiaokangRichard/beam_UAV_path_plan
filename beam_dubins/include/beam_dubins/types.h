/**
 * @file types.h
 * @brief Beam Dubins 算法核心数据结构定义（纯 C++，无 ROS 依赖）
 *
 * 本文件定义规划过程中使用的所有结构体：
 *  - Vec3 / Vec5：三维/五维向量
 *  - AABB：轴对齐包围盒（障碍物）
 *  - BeamNode：束搜索树节点
 *  - DubinsPrimitive：运动基元（转弯/直行段）
 *  - HorizontalDubinsParams：水平 Dubins 路径参数
 *  - Waypoint：输出路径点
 *  - BeamConfig：算法参数配置
 *  - PlanRequest / PlanResponse：规划输入/输出
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace beam_dubins {

// ── 基本向量类型 ────────────────────────────────────────────────

/// 三维向量（位置）
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/// 五维向量（完整状态：位置 + 姿态）
struct Vec5 {
    double x     = 0.0;  // X 坐标 (m)
    double y     = 0.0;  // Y 坐标 (m)
    double z     = 0.0;  // Z 坐标 (m)
    double yaw   = 0.0;  // 偏航角 ψ (rad)
    double pitch = 0.0;  // 俯仰角 γ (rad)
};

/// 路径点（核心库使用，注意：不与 ROS msg beam_dubins::Waypoint 冲突）
struct Waypoint {
    double x         = 0.0;  // X (m)
    double y         = 0.0;  // Y (m)
    double z         = 0.0;  // Z (m)
    double yaw       = 0.0;  // ψ (rad)
    double pitch     = 0.0;  // γ (rad)
    double curvature = 0.0;  // 曲率 κ = 1/R (1/m)
};

/// 轴对齐包围盒（障碍物碰撞体）
struct AABB {
    Vec3 min;  // 最小角点 [x_min, y_min, z_min]
    Vec3 max;  // 最大角点 [x_max, y_max, z_max]
};

// ── 搜索核心数据结构 ──────────────────────────────────────────

/// 束搜索树节点
struct BeamNode {
    Vec5 pos;                           // 节点状态 [x, y, z, yaw, pitch]
    int parent_idx = -1;                // 父节点在 all_nodes 中的索引（根节点=-1）
    std::vector<Waypoint> branch;      // 从父节点到当前节点的路径段点列
    double g          = 0.0;            // 累计路径长度 (m)
    double t          = 0.0;            // 累计飞行时间 (s)
    int depth         = 0;              // 搜索深度（根=0）
    double clearance  = INFINITY;       // 局部障碍物距离探测值 (m)
};

/// Dubins 运动基元：一段满足最小转弯半径约束的路径段
struct DubinsPrimitive {
    std::string type;                   // 类型标识：如 "L_30", "S", "R_45"
    double turn_angle_deg = 0.0;        // 转弯角度（°），直行 S=0
    char turn_direction   = 'S';        // 转弯方向：'L' 左转, 'S' 直行, 'R' 右转
    double arc_length     = 0.0;        // 弧长 (m)
    std::vector<Waypoint> waypoints;   // 基元离散路径点列
};

/// 水平 Dubins 路径参数（LSL / LSR / RSL / RSR 四种类型）
struct HorizontalDubinsParams {
    std::string type;                   // "LSL", "LSR", "RSL", "RSR"
    Vec3 c1, c2;                        // 两段圆弧的圆心坐标 [x, y, z=0]
    Vec5 t1, t2;                        // 两切点状态 [x, y, z, yaw, pitch]
    double arc1_angle   = 0.0;          // 第一段转角（有符号，CCW>0）
    double arc2_angle   = 0.0;          // 第三段转角（有符号，CCW>0）
    double arc1_len     = 0.0;          // 第一段弧长
    double straight_len = 0.0;          // 中间直线段长度
    double arc2_len     = 0.0;          // 第三段弧长
    double total_len    = 0.0;          // 总路径长度
};

// ── 配置与接口结构体 ──────────────────────────────────────────

/// Beam Dubins 算法参数配置
struct BeamConfig {
    // ── 搜索参数 ──
    int beam_width               = 4;     // 束宽度（每层保留节点数）
    int beam_width_max           = 8;     // 最大束宽度（早期自适应使用）
    int max_depth                = 200;   // 最大搜索深度
    double max_extend_length     = 500.0; // 最大步长 (m)
    double min_extend_length     = 100.0; // 最小步长 (m)
    double goal_tolerance_xy     = 50.0;  // 目标命中容差 XY (m)
    double goal_tolerance_z      = 30.0;  // 目标命中容差 Z (m)
    int dubins_shot_interval     = 1;     // 每 N 层尝试一次 Dubins final-shot
    double min_diversity_separation = 100.0; // 多样性剪枝最小节点间距 (m)

    // ── 评分器权重 ──
    double w_h = 1.2;                      // 启发式权重（目标距离）
    double w_c = 0.05;                     // clearance 奖励权重
    double w_p = 0.3;                      // 进度奖励权重

    // ── UAV 运动学约束 ──
    double R_min       = 250.0;            // 最小水平转弯半径 (m)
    double gamma_max_rad = 0.087266;       // 最大俯仰角 (rad) = 5°
    double r_body      = 5.0;              // 碰撞球半径 (m)
};

/// 规划请求（纯 C++ 结构体，不依赖 ROS）
struct PlanRequest {
    std::array<double, 5> start = {0, 0, 0, 0, 0};  // [x, y, z, yaw, pitch]
    std::array<double, 5> goal  = {0, 0, 0, 0, 0};  // [x, y, z, yaw, pitch]
    double space_lx         = 10000.0;  // X 上限 (m)
    double space_ly         = 5000.0;   // Y 上限 (m)
    double space_lz         = 1000.0;   // Z 上限 (m)
    double boundary_margin  = 50.0;     // 边界安全距离 (m)
    std::vector<AABB> obstacles;        // 障碍物列表
    double time_limit_ms    = 0.0;      // 规划超时 (ms)，0=不限
    bool use_3d             = false;    // 是否启用 3D 模式
};

/// 规划响应（纯 C++ 结构体，不依赖 ROS）
struct PlanResponse {
    bool success              = false;  // 是否成功
    double cost               = 0.0;    // 路径总长度 (m)
    std::vector<Waypoint> path;        // 路径点序列
    int nodes_explored        = 0;      // 探索节点数
    int depth_reached         = 0;      // 到达深度
    double planning_time_ms   = 0.0;    // 规划耗时 (ms)
    std::string status_message;         // 状态消息
};

}  // namespace beam_dubins
