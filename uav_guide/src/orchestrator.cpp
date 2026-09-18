/**
 * @file orchestrator.cpp
 * @brief APPROACH / TRACKING 状态机实现（判据与 1.0.1 plan_orchestrator.py 逐条对齐）
 */

#include "uav_guide/orchestrator.hpp"

#include <cmath>

#include "uav_guide/geo.hpp"

namespace uav_guide::orchestrator {

double expected_intercept_yaw(const State5& target, InterceptMode mode)
{
    if (mode == InterceptMode::HeadOn) return geo::wrap_angle(target.yaw + geo::kPi);
    return target.yaw;
}

double approach_distance(const OrchestratorConfig& cfg, InterceptMode mode)
{
    return (mode == InterceptMode::HeadOn) ? cfg.approach_distance_headon
                                           : cfg.approach_distance_tail;
}

double min_closure(const OrchestratorConfig& cfg, InterceptMode mode)
{
    return (mode == InterceptMode::Tail) ? cfg.min_closure_m_tail : cfg.min_closure_m;
}

State5 virtual_goal(const State5& target, InterceptMode mode, double approach_L)
{
    const double sign = (mode == InterceptMode::HeadOn) ? 1.0 : -1.0;
    State5 goal;
    goal.x = target.x + sign * approach_L * std::cos(target.yaw);
    goal.y = target.y + sign * approach_L * std::sin(target.yaw);
    goal.z = target.z;
    goal.yaw = expected_intercept_yaw(target, mode);
    goal.pitch = target.pitch;
    goal.valid = true;
    return goal;
}

// ═══════════════════════════════════════════════════════════════
// StateMachine
// ═══════════════════════════════════════════════════════════════

void StateMachine::reset(InterceptMode /*mode*/)
{
    tracking_ = false;
    last_distance_ = 0.0;
    has_last_uav_ = false;
    tracking_accum_ = 0.0;
    tracking_ref_dist_ = 0.0;
    consecutive_diverging_ = 0;
    entry_hold_ = 0;
}

StateMachine::Decision StateMachine::update(const State5& uav, const State5& target,
                                            InterceptMode mode)
{
    Decision d;
    if (!uav.valid || !target.valid) {
        d.goal = target;
        d.phase = tracking_ ? Phase::Tracking : Phase::Approach;
        return d;
    }

    d.distance = geo::distance_2d(uav.x, uav.y, target.x, target.y);

    // ── 1) TRACKING 趋势评估（可能退出）──
    bool changed = false;
    if (tracking_ && has_last_uav_) {
        const double advance = geo::distance_2d(last_uav_x_, last_uav_y_, uav.x, uav.y);
        changed = evaluate_trend(uav, target, advance);
    }

    // ── 2) 进入评估（仅 APPROACH 阶段）──
    const double psi_ref = expected_intercept_yaw(target, mode);
    d.align_err_deg = geo::rad2deg(std::abs(geo::wrap_angle(uav.yaw - psi_ref)));
    d.closure = has_last_uav_ ? (last_distance_ - d.distance) : 0.0;
    if (!tracking_ && evaluate_entry(mode, d.distance, d.align_err_deg, d.closure)) {
        changed = true;
    }

    // ── 3) 规划终点 ──
    if (tracking_) {
        d.goal = target;                                   // TRACKING：目标本体
        d.goal.yaw = psi_ref;
        d.goal.valid = true;
        d.phase = Phase::Tracking;
    } else {
        d.goal = virtual_goal(target, mode, approach_distance(cfg_, mode));
        d.phase = Phase::Approach;
    }

    // ── 4) 历史更新（供下一帧 closure / advance / 趋势）──
    last_distance_ = d.distance;
    last_uav_x_ = uav.x;
    last_uav_y_ = uav.y;
    has_last_uav_ = true;
    d.tracking_changed = changed;
    return d;
}

bool StateMachine::evaluate_entry(InterceptMode mode, double distance,
                                  double align_err_deg, double closure)
{
    if (distance >= cfg_.tracking_entry_dist) {
        entry_hold_ = 0;
        return false;
    }

    const bool aligned = align_err_deg <= cfg_.velocity_align_deg;
    const bool closing = closure >= min_closure(cfg_, mode);
    if (aligned && closing) {
        ++entry_hold_;
    } else {
        entry_hold_ = 0;
    }
    if (entry_hold_ < cfg_.entry_hold_count) return false;

    tracking_ = true;
    tracking_accum_ = 0.0;
    tracking_ref_dist_ = distance;
    consecutive_diverging_ = 0;
    entry_hold_ = 0;
    return true;
}

bool StateMachine::evaluate_trend(const State5& uav, const State5& target, double advance)
{
    tracking_accum_ += advance;
    if (tracking_accum_ < cfg_.tracking_window) return false;

    const double current = geo::distance_2d(uav.x, uav.y, target.x, target.y);
    const double delta = current - tracking_ref_dist_;
    if (delta > 0.0) {
        ++consecutive_diverging_;
    } else {
        consecutive_diverging_ = 0;
    }
    tracking_ref_dist_ = current;
    tracking_accum_ = 0.0;

    if (consecutive_diverging_ >= cfg_.divergence_threshold) {
        tracking_ = false;
        consecutive_diverging_ = 0;
        tracking_accum_ = 0.0;
        return true;   // 退出 TRACKING（miss）
    }
    return false;
}

}  // namespace uav_guide::orchestrator
