/**
 * @file ProjectedGradientSolver.cpp
 * @brief 有界 projected gradient solver 實作
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#include "ego_swarm/ProjectedGradientSolver.hpp"

#include <algorithm>
#include <limits>

namespace taiphoon
{
namespace ego_swarm
{

ProjectedGradientSolverResult ProjectedGradientSolver::solve(
    const std::vector<Vector3>& initial_control_points,
    const IOptimizationObjective& objective,
    const ProjectedGradientSolverConfig& config,
    const Projection& projection) const
{
    ProjectedGradientSolverResult result;
    if (!is_valid_config(config) ||
        !is_finite_control_points(initial_control_points) ||
        initial_control_points.size() <= BSPLINE_DEGREE ||
        !projection)
    {
        result.termination = SolverTerminationReason::INVALID_INPUT;
        return result;
    }

    const auto begin = std::chrono::steady_clock::now();
    const auto elapsed_ms = [&begin]()
    {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();
    };

    if (elapsed_ms() >= config.time_budget_ms)
    {
        result.termination = SolverTerminationReason::TIMEOUT;
        return result;
    }

    std::vector<Vector3> current = initial_control_points;
    if (!projection(current) || !is_finite_control_points(current))
    {
        result.termination = SolverTerminationReason::INVALID_INPUT;
        return result;
    }

    OptimizationEvaluation current_evaluation = objective.evaluate(current);
    if (!current_evaluation.is_valid ||
        !is_finite(current_evaluation.total_cost))
    {
        result.termination = SolverTerminationReason::INFEASIBLE;
        return result;
    }

    double step_m = config.initial_step_m;
    const auto find_coordinate_improvement = [
        &objective, &projection, &config, &elapsed_ms](
        const std::vector<Vector3>& base,
        const OptimizationEvaluation& base_evaluation,
        const double step,
        std::vector<Vector3>& improved_points,
        OptimizationEvaluation& improved_evaluation)
    {
        improved_points = base;
        improved_evaluation = base_evaluation;
        for (std::size_t point_index = 1U;
             point_index + 1U < base.size(); ++point_index)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                for (const double direction : {-1.0, 1.0})
                {
                    if (elapsed_ms() >= config.time_budget_ms)
                    {
                        return false;
                    }
                    std::vector<Vector3> candidate = base;
                    candidate.at(point_index).at(axis) += direction * step;
                    if (!projection(candidate) ||
                        !is_finite_control_points(candidate))
                    {
                        continue;
                    }
                    const OptimizationEvaluation evaluation =
                        objective.evaluate(candidate);
                    if (evaluation.is_valid &&
                        is_finite(evaluation.total_cost) &&
                        base_evaluation.total_cost - evaluation.total_cost >
                            config.convergence_tolerance)
                    {
                        improved_points = candidate;
                        improved_evaluation = evaluation;
                        return true;
                    }
                }
            }
        }
        // 對稱障礙群可能讓單一 control point 的有限差分方向都沒有
        // 改善；此時沿同一水平軸同步推移中間控制點，仍是同一個
        // continuous seed 的 bounded line-search，不是新增 topology seed。
        for (std::size_t axis : {0U, 1U, 2U})
        {
            for (const double direction : {-1.0, 1.0})
            {
                if (elapsed_ms() >= config.time_budget_ms)
                {
                    return false;
                }
                std::vector<Vector3> candidate = base;
                for (std::size_t point_index = 1U;
                     point_index + 1U < candidate.size(); ++point_index)
                {
                    candidate.at(point_index).at(axis) += direction * step;
                }
                if (!projection(candidate) ||
                    !is_finite_control_points(candidate))
                {
                    continue;
                }
                const OptimizationEvaluation evaluation =
                    objective.evaluate(candidate);
                if (evaluation.is_valid &&
                    is_finite(evaluation.total_cost) &&
                    base_evaluation.total_cost - evaluation.total_cost >
                        config.convergence_tolerance)
                {
                    improved_points = candidate;
                    improved_evaluation = evaluation;
                    return true;
                }
            }
        }
        return false;
    };
    for (int iteration = 0; iteration < config.max_iterations; ++iteration)
    {
        result.iterations = static_cast<std::size_t>(iteration + 1);
        if (elapsed_ms() >= config.time_budget_ms)
        {
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::TIMEOUT;
            return result;
        }

        std::vector<Vector3> gradient(
            current.size(), Vector3{0.0, 0.0, 0.0});
        double gradient_squared_norm = 0.0;
        for (std::size_t point_index = 1U;
             point_index + 1U < current.size(); ++point_index)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                std::vector<Vector3> plus = current;
                std::vector<Vector3> minus = current;
                plus.at(point_index).at(axis) += config.gradient_epsilon_m;
                minus.at(point_index).at(axis) -= config.gradient_epsilon_m;
                if (!projection(plus) || !projection(minus) ||
                    !is_finite_control_points(plus) ||
                    !is_finite_control_points(minus))
                {
                    result.control_points = current;
                    result.evaluation = current_evaluation;
                    result.is_valid = true;
                    result.termination =
                        SolverTerminationReason::NONFINITE_GRADIENT;
                    return result;
                }

                const OptimizationEvaluation plus_evaluation =
                    objective.evaluate(plus);
                const OptimizationEvaluation minus_evaluation =
                    objective.evaluate(minus);
                if (!plus_evaluation.is_valid ||
                    !minus_evaluation.is_valid ||
                    !is_finite(plus_evaluation.total_cost) ||
                    !is_finite(minus_evaluation.total_cost))
                {
                    result.control_points = current;
                    result.evaluation = current_evaluation;
                    result.is_valid = true;
                    result.termination =
                        SolverTerminationReason::NONFINITE_GRADIENT;
                    return result;
                }

                const double derivative =
                    (plus_evaluation.total_cost - minus_evaluation.total_cost) /
                    (2.0 * config.gradient_epsilon_m);
                if (!is_finite(derivative))
                {
                    result.control_points = current;
                    result.evaluation = current_evaluation;
                    result.is_valid = true;
                    result.termination =
                        SolverTerminationReason::NONFINITE_GRADIENT;
                    return result;
                }
                gradient.at(point_index).at(axis) = derivative;
                gradient_squared_norm += derivative * derivative;
            }
        }

        if (!is_finite(gradient_squared_norm))
        {
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::NONFINITE_GRADIENT;
            return result;
        }
        const double gradient_norm = std::sqrt(gradient_squared_norm);
        if (!is_finite(gradient_norm))
        {
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::NONFINITE_GRADIENT;
            return result;
        }
        if (gradient_norm <= config.convergence_tolerance)
        {
            std::vector<Vector3> coordinate_candidate;
            OptimizationEvaluation coordinate_evaluation;
            if ((current_evaluation.obstacle_cost >
                    config.convergence_tolerance ||
                current_evaluation.peer_cost > config.convergence_tolerance) &&
                find_coordinate_improvement(
                    current, current_evaluation, step_m,
                    coordinate_candidate, coordinate_evaluation))
            {
                current = coordinate_candidate;
                current_evaluation = coordinate_evaluation;
                step_m = std::min(config.initial_step_m, step_m * 1.1);
                continue;
            }
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::CONVERGED;
            return result;
        }

        // 若梯度下降的最小步幅已不足以造成可辨識的狀態改變，視為
        // 數值收斂；避免在有限差分噪聲下把已達到的局部解誤報為 diverged。
        if (step_m * gradient_norm <= config.convergence_tolerance)
        {
            std::vector<Vector3> coordinate_candidate;
            OptimizationEvaluation coordinate_evaluation;
            if ((current_evaluation.obstacle_cost >
                    config.convergence_tolerance ||
                current_evaluation.peer_cost > config.convergence_tolerance) &&
                find_coordinate_improvement(
                    current, current_evaluation, step_m,
                    coordinate_candidate, coordinate_evaluation))
            {
                current = coordinate_candidate;
                current_evaluation = coordinate_evaluation;
                step_m = std::min(config.initial_step_m, step_m * 1.1);
                continue;
            }
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::CONVERGED;
            return result;
        }

        const double normalization = std::max(1.0, gradient_norm);
        std::vector<Vector3> candidate = current;
        for (std::size_t point_index = 1U;
             point_index + 1U < candidate.size(); ++point_index)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                candidate.at(point_index).at(axis) -=
                    step_m * gradient.at(point_index).at(axis) /
                    normalization;
            }
        }

        if (!projection(candidate) || !is_finite_control_points(candidate))
        {
            step_m *= 0.5;
            if (step_m < config.minimum_step_m)
            {
                result.control_points = current;
                result.evaluation = current_evaluation;
                result.is_valid = true;
                result.termination = SolverTerminationReason::DIVERGED;
                return result;
            }
            continue;
        }

        bool projected_candidate_changed = false;
        for (std::size_t point_index = 1U;
             point_index + 1U < candidate.size() &&
             !projected_candidate_changed; ++point_index)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                if (std::abs(
                        candidate.at(point_index).at(axis) -
                        current.at(point_index).at(axis)) >
                    config.convergence_tolerance)
                {
                    projected_candidate_changed = true;
                    break;
                }
            }
        }
        if (!projected_candidate_changed)
        {
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::CONVERGED;
            return result;
        }

        const OptimizationEvaluation candidate_evaluation =
            objective.evaluate(candidate);
        if (!candidate_evaluation.is_valid ||
            !is_finite(candidate_evaluation.total_cost))
        {
            step_m *= 0.5;
            if (step_m < config.minimum_step_m)
            {
                result.control_points = current;
                result.evaluation = current_evaluation;
                result.is_valid = true;
                result.termination = SolverTerminationReason::DIVERGED;
                return result;
            }
            continue;
        }

        const double improvement = current_evaluation.total_cost -
            candidate_evaluation.total_cost;
        if (improvement > config.convergence_tolerance)
        {
            current = candidate;
            current_evaluation = candidate_evaluation;
            step_m = std::min(config.initial_step_m, step_m * 1.1);
            continue;
        }

        // 障礙物 clearance barrier 在幾何轉折或 nearest-point 切換處可能
        // 讓整體 normalized gradient 的 backtracking 方向暫時不降成本。
        // 以同一個 step 做 deterministic coordinate line-search，仍只在
        // bounded projection 內搜尋，不引入 Phase 7 的多 topology seed。
        std::vector<Vector3> best_coordinate;
        OptimizationEvaluation best_coordinate_evaluation;
        if ((current_evaluation.obstacle_cost >
                config.convergence_tolerance ||
            current_evaluation.peer_cost > config.convergence_tolerance) &&
            find_coordinate_improvement(
                current, current_evaluation, step_m,
                best_coordinate, best_coordinate_evaluation))
        {
            current = best_coordinate;
            current_evaluation = best_coordinate_evaluation;
            step_m = std::min(config.initial_step_m, step_m * 1.1);
            continue;
        }

        step_m *= 0.5;
        if (step_m < config.minimum_step_m)
        {
            result.control_points = current;
            result.evaluation = current_evaluation;
            result.is_valid = true;
            result.termination = SolverTerminationReason::DIVERGED;
            return result;
        }
    }

    result.control_points = current;
    result.evaluation = current_evaluation;
    result.is_valid = true;
    result.termination = SolverTerminationReason::ITERATION_LIMIT;
    return result;
}

bool ProjectedGradientSolver::is_valid_config(
    const ProjectedGradientSolverConfig& config)
{
    return config.max_iterations > 0 &&
           is_finite(config.time_budget_ms) &&
           config.time_budget_ms > EGO_SWARM_EPSILON_M &&
           is_finite(config.gradient_epsilon_m) &&
           config.gradient_epsilon_m > EGO_SWARM_EPSILON_M &&
           is_finite(config.initial_step_m) &&
           config.initial_step_m > EGO_SWARM_EPSILON_M &&
           is_finite(config.minimum_step_m) &&
           config.minimum_step_m > EGO_SWARM_EPSILON_M &&
           config.initial_step_m >= config.minimum_step_m &&
           is_finite(config.convergence_tolerance) &&
           config.convergence_tolerance > EGO_SWARM_EPSILON_M;
}

bool ProjectedGradientSolver::is_finite_control_points(
    const std::vector<Vector3>& control_points)
{
    if (control_points.empty())
    {
        return false;
    }
    for (const Vector3& control_point : control_points)
    {
        if (!is_finite_vector(control_point))
        {
            return false;
        }
    }
    return true;
}

}  // namespace ego_swarm
}  // namespace taiphoon
