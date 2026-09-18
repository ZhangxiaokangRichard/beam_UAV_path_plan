/**
 * @file orchestrator.hpp
 * @brief APPROACH / TRACKING 制导阶段状态机（零 ROS；**uav_guide_loop_node 独有**）
 *
 * 判据与 1.0.1 的 plan_orchestrator.py 逐条对齐：
 *  进入：d ≤ tracking_entry_dist 且 |wrap(ψ_uav − ψ_ref)| ≤ velocity_align_deg
 *        且 closure ≥ min_closure（head-on 3 m / tail 1.5 m），连续 entry_hold_count 帧
 *  退出：每累计飞行 tracking_window 米评估一次，连续 divergence_threshold 次距离增大
 *  规划终点：APPROACH = 目标前/后 L 的虚拟点；TRACKING = 目标本体（yaw = ψ_ref）
 *
 * 本类不调用服务、不发布话题：由节点把 Decision 转成 PlanPath 请求。
 */

#pragma once

#include "uav_guide/types.hpp"

namespace uav_guide::orchestrator {

/// 期望拦截航向 ψ_ref：head-on = ψ_tgt + π；tail = ψ_tgt
double expected_intercept_yaw(const State5& target, InterceptMode mode);

/// APPROACH 虚拟目标：head-on 目标前方 L；tail 目标后方 L
State5 virtual_goal(const State5& target, InterceptMode mode, double approach_L);

/// 该模式下的 approach 距离
double approach_distance(const OrchestratorConfig& cfg, InterceptMode mode);

/// 该模式下的最小接近量
double min_closure(const OrchestratorConfig& cfg, InterceptMode mode);

class StateMachine {
public:
    explicit StateMachine(const OrchestratorConfig& cfg) : cfg_(cfg) {}

    /// 拦截模式热切换：清空缓存并退出 TRACKING
    void reset(InterceptMode mode);

    struct Decision {
        State5 goal;          // 本轮规划终点（start 由调用方取 uav）
        Phase phase = Phase::Approach;
        bool tracking_changed = false;   // 本帧发生进入/退出（用于日志与话题）
        double distance = 0.0;           // 本机↔目标距离
        double closure = 0.0;            // 上一帧距离 − 本帧距离
        double align_err_deg = 0.0;      // |ψ_uav − ψ_ref|（°）
    };

    /// 每次 state 更新调用一次
    Decision update(const State5& uav, const State5& target, InterceptMode mode);

    bool tracking() const { return tracking_; }
    double last_distance() const { return last_distance_; }

private:
    /// 进入判据评估；返回 true 表示本帧进入 TRACKING
    bool evaluate_entry(InterceptMode mode, double distance,
                        double align_err_deg, double closure);
    /// 退出判据评估；返回 true 表示本帧退出 TRACKING
    bool evaluate_trend(const State5& uav, const State5& target, double advance);

    OrchestratorConfig cfg_;
    bool tracking_ = false;
    double last_distance_ = 0.0;      // 上一帧距离（用于 closure）
    double last_uav_x_ = 0.0;
    double last_uav_y_ = 0.0;
    bool has_last_uav_ = false;
    double tracking_accum_ = 0.0;     // 累计航程（退出评估窗口）
    double tracking_ref_dist_ = 0.0;  // 窗口起点的距离
    int consecutive_diverging_ = 0;
    int entry_hold_ = 0;
};

}  // namespace uav_guide::orchestrator
