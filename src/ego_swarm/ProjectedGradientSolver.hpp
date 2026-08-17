/**
 * @file ProjectedGradientSolver.hpp
 * @brief 有界 projected gradient solver 的純 C++ 測試 seam
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

#include "ego_swarm/BsplineTrajectory.hpp"
#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// Objective 單次評估結果。
struct OptimizationEvaluation
{
    bool is_valid{false};
    double total_cost{0.0};
    double goal_cost{0.0};
    double smoothness_cost{0.0};
    double dynamic_cost{0.0};
    double obstacle_cost{0.0};
    double peer_cost{0.0};
    double minimum_obstacle_distance_m{0.0};
    double minimum_peer_distance_m{0.0};
};

/**
 * @brief solver 與 trajectory objective 之間的最小 interface。
 */
class IOptimizationObjective
{
public:
    virtual ~IOptimizationObjective() = default;

    /**
     * @brief 評估一組控制點的有限成本。
     * @param control_points 待評估控制點
     * @return objective evaluation；無效輸入必須回傳 is_valid=false
     */
    virtual OptimizationEvaluation evaluate(
        const std::vector<Vector3>& control_points) const = 0;
};

/// projected gradient solver 的結束原因。
enum class SolverTerminationReason
{
    CONVERGED = 0,
    INVALID_INPUT,
    NONFINITE_GRADIENT,
    ITERATION_LIMIT,
    TIMEOUT,
    DIVERGED,
    INFEASIBLE
};

/// solver 設定。
struct ProjectedGradientSolverConfig
{
    int max_iterations{40};
    double time_budget_ms{20.0};
    double gradient_epsilon_m{0.05};
    double initial_step_m{0.20};
    double minimum_step_m{0.01};
    double convergence_tolerance{1.0e-3};
    // Optional domain hard-clearance hints used only to recognize that a
    // bounded side-step already satisfies the final safety geometry. The
    // common safety validator remains the authoritative final check.
    double hard_peer_distance_m{3.0};
    double hard_obstacle_clearance_m{1.0};
};

/// solver 結果；尚未經 safety validator 驗證，不可直接發布。
struct ProjectedGradientSolverResult
{
    bool is_valid{false};
    std::vector<Vector3> control_points{};
    OptimizationEvaluation evaluation{};
    std::size_t iterations{0U};
    SolverTerminationReason termination{
        SolverTerminationReason::INVALID_INPUT};
};

/**
 * @brief 具 projection、finite-difference gradient 與時間上限的 solver。
 *
 * projection callback 是 internal seam：production 使用 altitude／搜尋範圍
 * constraint，unit test 可注入簡單 projection 或故意產生數值失效。
 */
class ProjectedGradientSolver
{
public:
    using Projection = std::function<bool(std::vector<Vector3>&)>;

    /**
     * @brief 執行 bounded projected gradient descent。
     * @param initial_control_points 直線 seed 控制點
     * @param objective objective adapter
     * @param config solver 設定
     * @param projection 每次 candidate 更新後套用的硬約束
     * @return solver 結果；只有 CONVERGED 才可作為 optimizer success
     */
    ProjectedGradientSolverResult solve(
        const std::vector<Vector3>& initial_control_points,
        const IOptimizationObjective& objective,
        const ProjectedGradientSolverConfig& config,
        const Projection& projection) const;

private:
    static bool is_valid_config(
        const ProjectedGradientSolverConfig& config);

    static bool is_finite_control_points(
        const std::vector<Vector3>& control_points);
};

}  // namespace ego_swarm
}  // namespace taiphoon
