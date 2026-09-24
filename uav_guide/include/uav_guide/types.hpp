/**
 * @file types.hpp
 * @brief uav_guide 算法层公共数据类型（**零 ROS 依赖**）
 *
 * 设计原则（2026-09-18 裁决）：不再 1:1 复刻 1.0.1 的 7 个 Python 模块，
 * 而是按"两个制导节点是否共享"切分为命名空间 + 自由函数。
 * 本文件只放 POD / 枚举，供 geo / path / setpoint / guidance / state / mode / orchestrator 共享。
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace uav_guide {

// ── 几何/状态 ─────────────────────────────────────────────────

/// 路径点（字段与 beam_dubins/PathPoint 一一对应；算法层不依赖 rosidl）
struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;        // ENU：0=东、逆时针为正
    double pitch = 0.0;
    double curvature = 0.0;  // 1/R（直行 = 0）
};

/// 5D 状态 [x, y, z, yaw, pitch]（map/ENU：x=East, y=North, z=Up）
struct State5 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double speed_mps = 0.0;  // 地速（m/s；0 = 未知，L1 解算回退到 cfg.speed_mps）
    bool valid = false;      // 是否已收到有效状态
};

/// 拦截模式（与 1.0.1 一致：head-on / tail）
enum class InterceptMode { HeadOn, Tail };

/// 制导阶段
enum class Phase { Approach, Tracking };

// ── 制导输出 ─────────────────────────────────────────────────

/// 指点制导输出（→ uav_guide/msg/Setpoint）
struct SetpointOutput {
    double longitude_deg = 0.0;
    double latitude_deg = 0.0;
    double altitude_m = 0.0;
    double radius_m = 0.0;   // >0 顺时针 / <0 逆时针
};

/// 巡航制导输出（→ uav_guide/msg/Guidance）
struct GuidanceOutput {
    double fly_height = 0.0;   // 期望高度（类型由 bridge 参数决定）
    double heads_deg = 0.0;    // 航向，0=正北、顺时针，[0,360)
    double fly_speed_mps = 0.0;
    bool sample_fallback = false;   // 采样点回退（全弧段/空路径）→ 节点限流告警
    std::size_t sample_index = 0;   // 采样点索引（日志/调参用）

    // ── L1 跟踪（heads_source = L1）控制量与诊断 ──
    double beta_deg = 0.0;         // 偏差角 β（正 = 参考点在本机航向左侧）
    double kappa_cmd = 0.0;        // 期望曲率 (1/m)，已按 turn_radius_m 饱和
    double l1_eff_m = 0.0;         // 本周期实际前视距离（末端会收缩）
    double l1_point_x = 0.0;       // 参考点位置（ENU）
    double l1_point_y = 0.0;
    double los_heading_deg = 0.0;  // "朝参考点直飞"的航向（保护降级用）
    std::size_t l1_index = 0;      // 参考点所在段起点索引（诊断）
    bool l1_saturated = false;     // κ 是否被盘旋能力上限饱和
    bool near_end = false;         // 参考点收在路径末端
    bool hold_previous = false;    // |β| 越界 → 本周期保持上一指令（节点负责）
};

// ── 配置 ─────────────────────────────────────────────────────

/// 地理原点（WGS84 ↔ ENU 的基准点）
struct GeoOrigin {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_m = 0.0;

    bool valid() const { return lat_deg != 0.0 || lon_deg != 0.0; }
};

/// 规划空间（随 PlanPath 请求下发）
struct SpaceConfig {
    double lx = 10000.0;
    double ly = 5000.0;
    double lz = 1000.0;
    double boundary_margin = 50.0;
};

/// 规划调用参数
struct PlanParams {
    SpaceConfig space;
    double time_limit_ms = 0.0;
    bool use_3d = false;
    std::string frame_id = "map";
};

/// 指点制导配置（对应 1.0.1: segmentation.* / finalshot.*）
struct SetpointConfig {
    double curvature_eps = 1e-4;
    double yaw_segment_eps = 0.03;
    double default_turn_radius_m = 500.0;
    double offset_m = 50.0;         // finalshot.offset_m
    double offset_m_t = 750.0;      // finalshot.offset_m_t
    double offset_radius_m = 10.0;  // finalshot.radius_m
};

/// 采样点选取规则（Q12/Q16 裁决：默认"首个曲率为 0 的点"）
enum class SampleRule {
    FirstZeroCurvature,     // 自 uav 起，首个 |curvature| <= eps 的点（默认）
    FirstNonzeroCurvature,  // 旧规则：首个 |curvature| > eps 的点
};

/// heads 取法
enum class HeadsSource {
    SampleYaw,        // 采样点自带 ENU 航向（默认）
    BearingToSample,  // 本机 → 采样点的 ENU 方位角
    L1,               // L1 路径跟踪（前视 L1 点 + 曲率饱和 + 前馈补偿，见 doc/L1_tracking_plan.md）
};

/// 期望高度取法
enum class HeightSource {
    SamplePointZ,  // 采样点 z（默认）
    TargetZ,       // 目标 z
};

/// 巡航制导配置（对应新增 guidance: 段）
struct GuidanceConfig {
    double curvature_eps = 1e-4;
    double fly_speed_mps = 50.0;   // 本轮为常量占位（yaml 巡航速度）
    SampleRule sample_rule = SampleRule::FirstZeroCurvature;
    HeadsSource heads_source = HeadsSource::SampleYaw;
    HeightSource height_source = HeightSource::SamplePointZ;

    // ── L1 路径跟踪（heads_source = L1 时生效；设计见 doc/L1_tracking_plan.md）──
    double l1_distance_m = 1000.0;    // 前视弧长（参考点沿路径里程）
    double turn_radius_m = 500.0;     // 盘旋能力上限（κ 饱和用）
    double lead_time_s = 1.0;         // 飞控航向环时间补偿
    double speed_mps = 50.0;          // 地速回退值（无实测地速时使用）
    double beta_deadband_deg = 0.5;   // 偏差角死区
    double beta_guard_deg = 90.0;     // 偏差角保护阈值（越界则保持上一指令）
    double end_shrink_ratio = 1.0;    // 末端前视收缩比例（1 = 收到路径末点）
};

/// APPROACH / TRACKING 状态机配置（取值与 1.0.1 的 uav_guide.yaml 一致）
struct OrchestratorConfig {
    double approach_distance_tail = 1500.0;
    double approach_distance_headon = 2300.0;
    double tracking_entry_dist = 2400.0;
    double tracking_window = 100.0;        // 每累计飞行 100 m 评估一次（m）
    double velocity_align_deg = 70.0;      // 共线容差（°）
    double min_closure_m = 3.0;            // head-on 每帧最小接近量（m）
    double min_closure_m_tail = 1.5;       // tail 每帧最小接近量（m）
    int divergence_threshold = 3;          // 连续发散次数阈值
    int entry_hold_count = 1;              // 连续满足帧数（防抖）
};

/// 规划结果（服务响应的零 ROS 视图）
struct PlanResult {
    bool success = false;
    double cost = 0.0;
    std::string status_message;
    std::vector<PathPoint> path;
};

}  // namespace uav_guide
