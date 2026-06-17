/**
 * @file beam_search.cpp
 * @brief Beam Dubins 主搜索器实现
 *
 * 从 Python methods/beam_dubins.py + beam_dubins_2d_dynamic.py 迁移合并。
 *
 * 核心循环：
 *   for depth in 1..max_depth:
 *       对 beam 中每个节点：
 *         - Dubins final-shot 尝试直达目标
 *         - 自适应基元扩展 + 碰撞过滤
 *         - 快速评分初筛
 *       对 top N 精确评分 + 多样性剪枝
 *       更新 beam 前沿
 *
 * 关键设计：
 *  - 所有权重和阈值通过 BeamConfig 控制，不硬编码
 *  - plan_step() 和 reset_from() 支持增量模式
 *  - 所有碰撞检测通过 obstacles_ 指针（为空则跳过）
 */

#include "beam_dubins/beam_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "beam_dubins/beam_scorer.h"
#include "beam_dubins/collision.h"
#include "beam_dubins/dubins_path.h"
#include "beam_dubins/geometry.h"
#include "beam_dubins/primitive_gen.h"

namespace beam_dubins {

// ═══════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════

BeamDubins::BeamDubins(const std::vector<AABB>* obstacles,
                       const DubinsPath3D& dubins,
                       const double* start, const double* goal,
                       const BeamConfig& cfg)
    : obstacles_(obstacles)
    , dubins_(dubins)
    , cfg_(cfg)
{
    // 保存起点
    std::copy(start, start + 5, start_state_.begin());

    // 创建组件
    prim_gen_ = new PrimitiveGenerator(
        dubins.get_R_h(), dubins.get_gamma_max(),
        dubins.get_r_body(), obstacles_);

    scorer_ = new BeamScorer(dubins, start, goal,
                             cfg_.w_h, cfg_.w_c, cfg_.w_p);

    // 初始化 beam
    reset_from_impl(start);
}

// 析构（需在 .cpp 中定义以支持前向声明的 unique_ptr）
// 此处使用原始指针管理，在析构时释放
// 注意：若改为 unique_ptr，需在头文件中包含完整类型

// ═══════════════════════════════════════════════════════════════
// 模式1：一次性完整搜索
// ═══════════════════════════════════════════════════════════════

double BeamDubins::search(std::vector<Waypoint>& out_path,
                          std::vector<BeamNode>& out_all_nodes)
{
    // 初始化
    reset_from_impl(start_state_.data());
    out_path.clear();

    // ── 主循环 ──
    for (int d = 1; d <= cfg_.max_depth; ++d) {
        depth_ = d;

        int cur_beam_width = adaptive_beam_width(d);
        std::vector<std::pair<BeamNode, double>> candidates;

        for (const auto& node : beam_) {
            // ── Dubins final-shot：每 N 层尝试一次直达 ──
            if (d % cfg_.dubins_shot_interval == 0) {
                std::vector<Waypoint> shot_path;
                double cost = dubins_.best_path(&node.pos.x, scorer_->get_goal_ptr(), shot_path);
                if (cost > 0) {
                    double total_cost = node.g + cost;
                    if (total_cost < best_cost_) {
                        best_cost_ = total_cost;
                        best_path_ = build_path(node);
                        // 追加 final-shot 路径点
                        best_path_.insert(best_path_.end(),
                                          shot_path.begin(), shot_path.end());
                        path_found_depth_ = d;
                    }
                }
            }

            // ── 自适应基元扩展 ──
            double clr  = estimate_clearance(&node.pos.x);
            double step = adapt_step_size(clr, d);
            auto primitives = prim_gen_->generate(node, step, clr);

            for (const auto& prim : primitives) {
                BeamNode child = extend(node, prim, clr, d);
                double score = scorer_->evaluate_fast(child);
                candidates.emplace_back(std::move(child), score);
            }
        }

        // ── 将新节点加入全局节点列表 ──
        for (auto& [nd, _] : candidates) {
            all_nodes_.push_back(nd);
        }

        // ── 无候选 → 死胡同 ──
        if (candidates.empty()) {
            break;
        }

        // ── 阶段1：快速评分排序 ──
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto& a, const auto& b) {
                      return a.second < b.second;
                  });

        // ── 阶段2：对 top 3×beam_width 精确评分 ──
        int top_n = std::min(static_cast<int>(candidates.size()),
                             cur_beam_width * 3);
        for (int i = 0; i < top_n; ++i) {
            auto& [nd, score] = candidates[i];
            score = scorer_->evaluate_exact(nd);
        }
        // 重新排序
        std::sort(candidates.begin(), candidates.begin() + top_n,
                  [](const auto& a, const auto& b) {
                      return a.second < b.second;
                  });

        // ── 多样性剪枝 ──
        beam_ = prune_diverse(candidates, cur_beam_width);

        // ── 更新最优节点 ──
        update_best_node();
    }

    // ── 输出结果 ──
    out_all_nodes = all_nodes_;

    if (path_found_depth_ > 0) {
        out_path = best_path_;
        return best_cost_;
    }

    // 未到达目标：尝试从 best_node 到目标的 Dubins 连接
    if (best_node_ != nullptr) {
        std::vector<Waypoint> partial;
        double cost = dubins_.best_path(&best_node_->pos.x, scorer_->get_goal_ptr(), partial);
        if (cost > 0) {
            out_path = build_path(*best_node_);
            out_path.insert(out_path.end(), partial.begin(), partial.end());
            return best_node_->g + cost;
        }
        // Dubins 不连通 → 仅返回已探索部分
        out_path = build_path(*best_node_);
        return best_node_->g;
    }

    return INFINITY;
}

// ═══════════════════════════════════════════════════════════════
// 模式2：增量式推进
// ═══════════════════════════════════════════════════════════════

bool BeamDubins::plan_step(const double* goal)
{
    if (depth_ >= cfg_.max_depth) {
        return false;
    }

    depth_++;
    // 更新评分器中的目标
    scorer_->set_goal(goal);

    bool improved = false;
    int cur_beam_width = adaptive_beam_width(depth_);
    std::vector<std::pair<BeamNode, double>> candidates;

    for (const auto& node : beam_) {
        // ── Dubins final-shot ──
        if (depth_ % cfg_.dubins_shot_interval == 0) {
            std::vector<Waypoint> shot_path;
            double cost = dubins_.best_path(&node.pos.x, goal, shot_path);
            if (cost > 0) {
                double total_cost = node.g + cost;
                if (total_cost < best_cost_) {
                    best_cost_ = total_cost;
                    best_path_ = build_path(node);
                    best_path_.insert(best_path_.end(),
                                      shot_path.begin(), shot_path.end());
                    path_found_depth_ = depth_;
                    improved = true;
                }
            }
        }

        // ── 基元扩展 ──
        double clr  = estimate_clearance(&node.pos.x);
        double step = adapt_step_size(clr, depth_);
        auto primitives = prim_gen_->generate(node, step, clr);

        for (const auto& prim : primitives) {
            BeamNode child = extend(node, prim, clr, depth_);
            double score = scorer_->evaluate_fast(child);
            candidates.emplace_back(std::move(child), score);
        }
    }

    // 加入全局列表
    for (auto& [nd, _] : candidates) {
        all_nodes_.push_back(nd);
    }

    if (!candidates.empty()) {
        // 快速排序 → 精确重评 top → 剪枝
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });

        int top_n = std::min(static_cast<int>(candidates.size()),
                             cur_beam_width * 3);
        for (int i = 0; i < top_n; ++i) {
            auto& [nd, score] = candidates[i];
            score = scorer_->evaluate_exact(nd);
        }
        std::sort(candidates.begin(), candidates.begin() + top_n,
                  [](const auto& a, const auto& b) { return a.second < b.second; });

        beam_ = prune_diverse(candidates, cur_beam_width);
        update_best_node();
    } else {
        beam_.clear();
    }

    return improved;
}

// ═══════════════════════════════════════════════════════════════
// 重置
// ═══════════════════════════════════════════════════════════════

void BeamDubins::reset_from(const double* pos)
{
    reset_from_impl(pos);
}

void BeamDubins::reset_from_impl(const double* pos)
{
    BeamNode root;
    root.pos.x = pos[0];
    root.pos.y = pos[1];
    root.pos.z = pos[2];
    root.pos.yaw   = pos[3];
    root.pos.pitch = pos[4];
    root.g         = 0.0;
    root.t         = 0.0;
    root.depth     = 0;
    root.parent_idx = -1;
    root.clearance  = estimate_clearance(pos);

    beam_.clear();
    beam_.push_back(root);

    all_nodes_.clear();
    all_nodes_.push_back(root);

    depth_          = 0;
    best_node_      = &all_nodes_.back();  // 指向 all_nodes_ 中的根节点
    best_path_.clear();
    best_cost_      = INFINITY;
    path_found_depth_ = -1;
}

// ═══════════════════════════════════════════════════════════════
// 查询最优路径
// ═══════════════════════════════════════════════════════════════

std::vector<Waypoint> BeamDubins::get_best_path() const
{
    if (!best_path_.empty()) {
        return best_path_;
    }
    if (best_node_ != nullptr && best_node_->g > 0) {
        return build_path(*best_node_);
    }
    return {};
}

double BeamDubins::get_best_cost() const
{
    if (!best_path_.empty()) {
        return best_cost_;
    }
    if (best_node_ != nullptr) {
        return best_node_->g;
    }
    return INFINITY;
}

// ═══════════════════════════════════════════════════════════════
// 节点扩展
// ═══════════════════════════════════════════════════════════════

BeamNode BeamDubins::extend(const BeamNode& parent, const DubinsPrimitive& prim,
                            double clearance, int depth) const
{
    BeamNode child;

    // 基元末端状态
    const auto& last_wp = prim.waypoints.back();
    child.pos.x     = last_wp.x;
    child.pos.y     = last_wp.y;
    child.pos.z     = last_wp.z;
    child.pos.yaw   = last_wp.yaw;
    child.pos.pitch = last_wp.pitch;  // Phase 1: 0.0

    child.parent_idx = static_cast<int>(all_nodes_.size()) - 1;  // 父节点在 all_nodes_ 中的位置
    // 注意：这里假定父节点是 all_nodes_ 中最后添加的
    // 实际使用时由调用方负责维护索引关系
    // Phase 1 简化：使用偏移量

    child.branch    = prim.waypoints;  // 复制基元的路径点
    child.g         = parent.g + prim.arc_length;
    child.t         = parent.t + prim.arc_length / std::max(dubins_.get_R_h(), 1.0);
    child.depth     = depth;
    child.clearance = clearance;

    return child;
}

// ═══════════════════════════════════════════════════════════════
// 碰撞检测
// ═══════════════════════════════════════════════════════════════

bool BeamDubins::is_primitive_safe(const DubinsPrimitive& prim) const
{
    if (!obstacles_ || obstacles_->empty()) {
        return true;
    }

    // 构建 double* 数组用于 swept_sphere_path
    std::vector<std::array<double, 3>> storage;
    storage.reserve(prim.waypoints.size());
    for (const auto& pp : prim.waypoints) {
        storage.push_back({pp.x, pp.y, pp.z});
    }
    std::vector<const double*> wps;
    wps.reserve(storage.size());
    for (const auto& s : storage) {
        wps.push_back(s.data());
    }

    return !swept_sphere_path(wps, dubins_.get_r_body(), *obstacles_);
}

// ═══════════════════════════════════════════════════════════════
// 局部 clearance 估算
// ═══════════════════════════════════════════════════════════════

double BeamDubins::estimate_clearance(const double* pos) const
{
    // 在 8 个方向、4 个距离层级探测障碍物
    // 返回首次碰撞发现的距离；全通则返回一个大值
    static const double probe_dists[] = {100.0, 200.0, 400.0, 600.0};
    double z = pos[2];
    double r = dubins_.get_r_body();

    for (double d : probe_dists) {
        for (int k = 0; k < 8; ++k) {
            double angle = k * M_PI / 4.0;  // 0°, 45°, 90°, ..., 315°
            double px = pos[0] + d * std::cos(angle);
            double py = pos[1] + d * std::sin(angle);

            // 检查该探测点是否与障碍物碰撞
            bool collision = false;
            if (obstacles_) {
                double pt[3] = {px, py, z};
                for (const auto& obs : *obstacles_) {
                    double aabb_min[3] = {obs.min.x, obs.min.y, obs.min.z};
                    double aabb_max[3] = {obs.max.x, obs.max.y, obs.max.z};
                    if (sphere_aabb_intersect(pt, r, aabb_min, aabb_max)) {
                        collision = true;
                        break;
                    }
                }
            }
            if (collision) {
                return d;  // 返回首次碰撞的距离
            }
        }
    }

    // 全通 → 返回比最大探测距离更大的值
    return probe_dists[3] + 200.0;  // 800m
}

// ═══════════════════════════════════════════════════════════════
// 自适应步长
// ═══════════════════════════════════════════════════════════════

double BeamDubins::adapt_step_size(double clearance, int depth) const
{
    double step;
    if (clearance > 500.0) {
        step = cfg_.max_extend_length;      // 500m
    } else if (clearance > 200.0) {
        step = 350.0;
    } else if (clearance > 100.0) {
        step = 200.0;
    } else if (clearance > 50.0) {
        step = 150.0;
    } else {
        step = cfg_.min_extend_length;      // 100m
    }

    // 搜索早期略放大步长（加速探索）
    if (depth < 10) {
        step = std::min(step * 1.5, 600.0);
    }

    return step;
}

// ═══════════════════════════════════════════════════════════════
// 自适应 beam 宽度
// ═══════════════════════════════════════════════════════════════

int BeamDubins::adaptive_beam_width(int depth) const
{
    if (depth < 10) {
        // 早期：宽探索
        return std::min(cfg_.beam_width_max, cfg_.beam_width + 2);
    } else if (depth < 30) {
        // 中期：正常
        return cfg_.beam_width;
    } else {
        // 后期：收窄聚焦
        return std::max(2, cfg_.beam_width - 1);
    }
}

// ═══════════════════════════════════════════════════════════════
// 多样性剪枝
// ═══════════════════════════════════════════════════════════════

std::vector<BeamNode> BeamDubins::prune_diverse(
    std::vector<std::pair<BeamNode, double>>& candidates,
    int beam_width) const
{
    std::vector<BeamNode> selected;
    selected.reserve(beam_width);

    for (auto& [node, score] : candidates) {
        bool too_close = false;
        for (const auto& s : selected) {
            // 检查与已选节点的间距
            double d = distance_3d(&node.pos.x, &s.pos.x);
            if (d < cfg_.min_diversity_separation) {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            selected.push_back(std::move(node));
            if (static_cast<int>(selected.size()) >= beam_width) {
                break;
            }
        }
    }

    // 如果多样性过滤导致不足 beam_width，补充剩余最优
    if (static_cast<int>(selected.size()) < beam_width) {
        for (auto& [node, score] : candidates) {
            // 简单检查是否已被选中（比较地址不可靠，比较 g 值近似判断）
            bool already_selected = false;
            for (const auto& s : selected) {
                if (std::abs(node.g - s.g) < 1e-6 &&
                    distance_3d(&node.pos.x, &s.pos.x) < 1e-3) {
                    already_selected = true;
                    break;
                }
            }
            if (!already_selected) {
                selected.push_back(std::move(node));
                if (static_cast<int>(selected.size()) >= beam_width) {
                    break;
                }
            }
        }
    }

    return selected;
}

// ═══════════════════════════════════════════════════════════════
// 更新最优节点
// ═══════════════════════════════════════════════════════════════

void BeamDubins::update_best_node()
{
    if (beam_.empty()) return;

    // 用 Dubins 启发式评估各节点到目标的距离
    best_node_ = &beam_[0];
    double best_f = best_node_->g
                  + scorer_->dubins_heuristic(&best_node_->pos.x, scorer_->get_goal_ptr());

    for (size_t i = 1; i < beam_.size(); ++i) {
        double h = scorer_->dubins_heuristic(&beam_[i].pos.x, scorer_->get_goal_ptr());
        double f = beam_[i].g + h;
        if (f < best_f) {
            best_f = f;
            best_node_ = &beam_[i];
        }
    }
}

// ═══════════════════════════════════════════════════════════════
// 路径回溯
// ═══════════════════════════════════════════════════════════════

std::vector<Waypoint> BeamDubins::build_path(const BeamNode& node) const
{
    // 从叶节点回溯到根，收集所有路径段
    std::vector<std::vector<Waypoint>> segments;
    const BeamNode* cur = &node;

    while (cur != nullptr) {
        if (!cur->branch.empty()) {
            segments.push_back(cur->branch);
        } else {
            // 叶节点或根节点：用 pos 构造单点
            Waypoint pp;
            pp.x     = cur->pos.x;
            pp.y     = cur->pos.y;
            pp.z     = cur->pos.z;
            pp.yaw   = cur->pos.yaw;
            pp.pitch = cur->pos.pitch;
            pp.curvature = 0.0;
            segments.push_back({pp});
        }

        if (cur->parent_idx < 0) {
            break;  // 到达根节点
        }
        cur = &all_nodes_[cur->parent_idx];
    }

    // 反转（回溯是从叶到根）
    std::reverse(segments.begin(), segments.end());

    // 展平为单一路径
    std::vector<Waypoint> result;
    for (const auto& seg : segments) {
        result.insert(result.end(), seg.begin(), seg.end());
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════
// 死推算（路径耗尽时的兜底策略）
// ═══════════════════════════════════════════════════════════════

std::vector<Waypoint> BeamDubins::dead_reckon_toward(
    const double* goal, double distance) const
{
    // 从当前位置朝目标方向直线延伸
    // 此方法在 Phase 2 的 UAV 推进中使用
    std::vector<Waypoint> path;
    // 使用 best_node 或 start_state 作为当前位置
    const double* cur = (best_node_ != nullptr) ? &best_node_->pos.x : start_state_.data();

    double dx = goal[0] - cur[0];
    double dy = goal[1] - cur[1];
    double dist_to_goal = std::sqrt(dx * dx + dy * dy);
    if (dist_to_goal < 1e-6) return path;

    double step = std::min(distance, dist_to_goal);
    double psi  = std::atan2(dy, dx);

    Waypoint pp;
    pp.x     = cur[0] + step * std::cos(psi);
    pp.y     = cur[1] + step * std::sin(psi);
    pp.z     = cur[2];
    pp.yaw   = psi;
    pp.pitch = 0.0;
    pp.curvature = 0.0;
    path.push_back(pp);
    return path;
}

// ═══════════════════════════════════════════════════════════════
// 独立包装函数：plan_path()
// ═══════════════════════════════════════════════════════════════

PlanResponse plan_path(const PlanRequest& req, const BeamConfig& cfg)
{
    // ── 记录起始时间 ──────────────────────────────────────────
    auto t_start = std::chrono::high_resolution_clock::now();

    // ── 构建障碍物列表 ────────────────────────────────────────
    // 使用传入的 obstacles 向量（不需要拷贝，只读引用）
    const auto& obstacles = req.obstacles;

    // ── 构建 DubinsPath3D ────────────────────────────────────
    DubinsPath3D dubins(cfg.R_min, cfg.gamma_max_rad, cfg.r_body,
                        obstacles.empty() ? nullptr : &obstacles);

    // ── 构建 BeamDubins 搜索器 ───────────────────────────────
    BeamDubins planner(
        obstacles.empty() ? nullptr : &obstacles,
        dubins,
        req.start.data(),
        req.goal.data(),
        cfg);

    // ── 执行搜索 ─────────────────────────────────────────────
    std::vector<Waypoint> path;
    std::vector<BeamNode> all_nodes;
    double cost = planner.search(path, all_nodes);

    // ── 记录结束时间 ──────────────────────────────────────────
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // ── 填充响应 ──────────────────────────────────────────────
    PlanResponse resp;
    resp.nodes_explored  = planner.get_nodes_explored();
    resp.depth_reached   = planner.get_depth();
    resp.planning_time_ms = elapsed_ms;

    if (!path.empty()) {
        resp.success = true;
        resp.cost    = cost;
        // 判断是否真正到达目标（在 move 之前检查）
        double dx = path.back().x - req.goal[0];
        double dy = path.back().y - req.goal[1];
        double dz = path.back().z - req.goal[2];
        double dist_to_goal = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist_to_goal < cfg.goal_tolerance_xy) {
            resp.status_message = "goal reached";
        } else {
            resp.status_message = "partial path";
        }
        resp.path = std::move(path);
    } else {
        resp.success = false;
        resp.cost    = INFINITY;
        resp.status_message = "no solution";
    }

    return resp;
}

}  // namespace beam_dubins
