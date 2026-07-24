/**
 * @file beam_search.h
 * @brief Beam Dubins 全局路径规划器（纯 C++，无 ROS 依赖）
 *
 * 从 Python methods/beam_dubins.py → BeamDubins + IncrementalBeamDubins 合并迁移。
 *
 * 在 Dubins 运动基元空间内进行受限宽度束搜索 (Beam Search)：
 *  - 逐层推进：每层生成候选基元 → 评分 → 排序 → 多样性剪枝
 *  - 自适应步长：根据局部 clearance 调整扩展步长
 *  - 自适应 beam 宽度：早期宽探索，后期收窄聚焦
 *  - Dubins final-shot：定期尝试直接 Dubins 路径连接目标
 *
 * 本文件提供两种模式：
 *  1. search()  — 一次性完整搜索（静态目标场景）
 *  2. plan_step() / reset_from() — 增量式推进（动态目标跟踪场景）
 */

#pragma once

#include <vector>

#include "beam_dubins/types.h"

namespace beam_dubins {

// 前向声明
class DubinsPath3D;
class PrimitiveGenerator;
class BeamScorer;
struct AABB;

/**
 * @class BeamDubins
 * @brief Beam Dubins 主搜索器
 *
 * 提供两种搜索模式：
 *  - search()：一次性运行完整搜索，返回最优路径
 *  - plan_step() / reset_from()：增量式推进，与 Python IncrementalBeamDubins 对齐
 */
class BeamDubins {
public:
    /**
     * @brief 构造函数（搜索模式：一次性完整搜索）
     * @param obstacles  障碍物列表指针
     * @param dubins     Dubins 路径计算器
     * @param start      起点 [x, y, z, yaw, pitch]
     * @param goal       终点 [x, y, z, yaw, pitch]
     * @param cfg        算法参数配置
     */
    BeamDubins(const std::vector<AABB>* obstacles,
               const DubinsPath3D& dubins,
               const double* start, const double* goal,
               const BeamConfig& cfg);

    // ── 模式1：一次性完整搜索 ───────────────────────────────

    /**
     * @brief 运行完整 Beam Dubins 搜索
     * @param out_path  输出最优路径点序列（空=无解）
     * @param out_all_nodes  输出所有探索过的节点（用于可视化）
     * @return 最优路径代价，无解返回 INFINITY
     */
    double search(std::vector<Waypoint>& out_path,
                  std::vector<BeamNode>& out_all_nodes);

    // ── 模式2：增量式推进（动态目标跟踪）────────────────────

    /**
     * @brief 推进一个搜索深度层
     * @param goal  当前目标（5D 状态）
     * @return true 表示本轮找到更优的完整路径
     */
    bool plan_step(const double* goal);

    /**
     * @brief 从新位置重置 beam（动态场景用）
     * @param pos  新起点 [x, y, z, yaw, pitch]
     */
    void reset_from(const double* pos);

    /**
     * @brief 获取当前最优可用路径
     * @return 优先返回完整路径，否则返回到 best_node 的部分路径
     */
    std::vector<Waypoint> get_best_path() const;

    /**
     * @brief 获取当前最优路径代价
     */
    double get_best_cost() const;

    // ── 访问器（用于可视化与调试）────────────────────────────

    const std::vector<BeamNode>& get_all_nodes()  const { return all_nodes_; }
    const std::vector<BeamNode>& get_beam()       const { return beam_; }
    int get_depth() const { return depth_; }
    int get_nodes_explored() const { return static_cast<int>(all_nodes_.size()); }
    const BeamNode* get_best_node() const { return best_node_; }

private:
    // ── 扩展与评分 ───────────────────────────────────────────

    /// 从父节点沿基元扩展生成子节点
    BeamNode extend(const BeamNode& parent, const DubinsPrimitive& prim,
                    double clearance, int depth) const;

    /// 检查基元是否无碰撞
    bool is_primitive_safe(const DubinsPrimitive& prim) const;

    // ── 自适应策略 ───────────────────────────────────────────

    /// 根据 clearance 和深度估算局部障碍物距离
    double estimate_clearance(const double* pos) const;

    /// 根据 clearance 和深度自适应步长
    double adapt_step_size(double clearance, int depth) const;

    /// 根据搜索深度自适应 beam 宽度
    int adaptive_beam_width(int depth) const;

    // ── 剪枝 ─────────────────────────────────────────────────

    /**
     * @brief 多样性剪枝：按评分 + 空间多样性保留 top-K 节点
     * @param candidates  候选节点及其评分
     * @param beam_width  目标保留节点数
     * @return 剪枝后的节点列表
     */
    std::vector<BeamNode> prune_diverse(
        std::vector<std::pair<BeamNode, double>>& candidates,
        int beam_width) const;

    /// 更新全局最优节点
    void update_best_node();

    // ── 路径回溯 ─────────────────────────────────────────────

    /// 从叶节点回溯到根，构建正向路径点序列
    std::vector<Waypoint> build_path(const BeamNode& node) const;

    /// 重置 beam（用于 reset_from 和初始化）
    void reset_from_impl(const double* pos);

    /// 朝目标方向死推算（路径耗尽时的兜底）
    std::vector<Waypoint> dead_reckon_toward(const double* goal,
                                              double distance) const;

    // ── 成员 ──
    // 外部依赖（只读引用/指针）
    const std::vector<AABB>* obstacles_;
    const DubinsPath3D& dubins_;
    BeamConfig cfg_;

    // 组件
    PrimitiveGenerator* prim_gen_;   // 基元生成器（unique_ptr 在 .cpp 中管理）
    BeamScorer* scorer_;             // 评分器

    // 搜索状态
    std::vector<BeamNode> beam_;       // 当前 beam 前沿节点
    std::vector<BeamNode> all_nodes_;  // 所有探索过的节点
    int depth_ = 0;                    // 当前搜索深度

    BeamNode* best_node_ = nullptr;           // 最优部分节点
    std::vector<Waypoint> best_path_;         // 最优完整路径
    double best_cost_ = INFINITY;              // 最优代价
    int path_found_depth_ = -1;                // 找到路径的深度（-1=未找到）

    // 起点（用于 reset）
    std::array<double, 5> start_state_;
};

// ═══════════════════════════════════════════════════════════════
// 独立包装函数（无 ROS 依赖，方便直接调用和单元测试）
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Beam Dubins 路径规划的顶层入口函数
 *
 * 内部创建 BeamDubins 对象并调用 search()。
 * 该函数无任何 ROS 依赖，可被 planner_server 或直接测试代码调用。
 *
 * @param req  规划请求（起点、终点、障碍物、空间边界等）
 * @param cfg  算法参数配置
 * @return 规划响应（路径、代价、状态消息等）
 */
PlanResponse plan_path(const PlanRequest& req, const BeamConfig& cfg);

}  // namespace beam_dubins
