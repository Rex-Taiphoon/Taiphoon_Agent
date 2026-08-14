/**
 * @file OptimizedEgoPlanner.hpp
 * @brief continuous B-spline optimized EGO planner
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include "ego_swarm/ITrajectoryPlanner.hpp"
#include "ego_swarm/ProjectedGradientSolver.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 使用單一直線 seed 與 bounded projected gradient 的 optimized backend。
 *
 * 這個 backend 只產生候選 trajectory；dynamic、peer、obstacle safety
 * 仍由外部 ContinuousSafetyValidator 統一決定。
 */
class OptimizedEgoPlanner final : public ITrajectoryPlanner
{
public:
    /**
     * @brief 使用 planner 設定建立 optimized backend。
     * @param config planner 與 solver 設定
     */
    explicit OptimizedEgoPlanner(
        const PlannerConfig& config = PlannerConfig{});

    /**
     * @brief 更新 backend 設定。
     * @param config 新設定
     * @return true 表示設定有效
     */
    bool set_config(const PlannerConfig& config);

    /**
     * @brief 取得 backend 設定。
     * @return 唯讀設定參考
     */
    const PlannerConfig& config() const;

    /**
     * @brief 對單一 immutable snapshot 進行 continuous optimization。
     * @param snapshot 單一 planner cycle 的唯讀資料
     * @return optimized 結果；非 CONVERGED 不可視為可執行軌跡
     */
    PlannerResult plan(
        const PlanningSnapshot& snapshot) const override;

private:
    static bool is_valid_config(const PlannerConfig& config);

    static double compute_seed_duration_s(
        const PlannerConfig& config,
        const Vector3& current,
        const Vector3& target);

    static bool is_hard_dynamically_feasible(
        const BsplineTrajectory& trajectory,
        const PlannerConfig& config);

    PlannerConfig config_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
