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
    const auto is_hard_clear = [&config](
        const OptimizationEvaluation& evaluation)
    {
        const bool peer_clear = evaluation.minimum_peer_distance_m <=
                EGO_SWARM_EPSILON_M ||
            evaluation.minimum_peer_distance_m +
                config.convergence_tolerance >= config.hard_peer_distance_m;
        const bool obstacle_clear = evaluation.obstacle_cost <=
                config.convergence_tolerance ||
            evaluation.minimum_obstacle_distance_m +
                config.convergence_tolerance >=
                config.hard_obstacle_clearance_m;
        return peer_clear && obstacle_clear;
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

        // Prefer one smooth corridor-shaped translation for a long peer
        // line.  The first/last interior points stay on the direct seed so
        // endpoint velocity remains bounded; the middle points carry the
        // lateral clearance.  This is still a projected deformation of the
        // single seed, not an additional topology.
        if (base.size() >= 6U)
        {
            const double horizontal_delta_x =
                base.back().at(0) - base.front().at(0);
            const double horizontal_delta_y =
                base.back().at(1) - base.front().at(1);
            const std::size_t forward_axis =
                std::abs(horizontal_delta_y) >=
                    std::abs(horizontal_delta_x) ? 1U : 0U;
            const std::size_t lateral_axis = 1U - forward_axis;
            const double forward_delta =
                base.back().at(forward_axis) -
                base.front().at(forward_axis);
            const double lateral_delta =
                base.back().at(lateral_axis) -
                base.front().at(lateral_axis);
            // Body-frame MOVE after a yaw can leave a small lateral residual
            // in the global NED goal.  Treat a mostly-forward route as the
            // same corridor case; otherwise the generic translation probe
            // may find a geometrically clear but dynamically sharp detour
            // before this smooth bounded candidate is considered.
            const double lateral_residual_tolerance = std::max(
                1.0, 0.1 * std::abs(forward_delta));
            if (std::abs(forward_delta) > EGO_SWARM_EPSILON_M &&
                std::abs(lateral_delta) <= lateral_residual_tolerance)
            {
                const bool long_forward_route =
                    std::abs(forward_delta) > 25.0;
                const double corridor_step = long_forward_route
                    ? std::max(step, 10.0)
                    : std::max(step, 8.0);
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
                        const double ratio = static_cast<double>(point_index) /
                            static_cast<double>(candidate.size() - 1U);
                        const double smooth_ratio = ratio * ratio *
                            (3.0 - 2.0 * ratio);
                        // Raise the lateral offset early enough for a peer
                        // that is already close to the current position.
                        // The previous sin^2 profile left the first interior
                        // point near the centerline, so a valid route could
                        // still pass below 3 m before reaching the side.
                        double lateral_profile = long_forward_route
                            ? 0.65
                            : 0.85;
                        if (point_index == 1U ||
                            point_index + 2U == candidate.size())
                        {
                            // Keep the endpoint derivative within the
                            // velocity bound while moving sideways early.
                            // The forward component is non-zero as well;
                            // long routes need a smaller lateral start so
                            // the combined endpoint velocity stays below
                            // the hard limit. Preserve the established
                            // profile for shorter obstacle transitions.
                            lateral_profile = long_forward_route ? 0.25 : 0.40;
                        }
                        candidate.at(point_index).at(forward_axis) =
                            base.front().at(forward_axis) +
                            forward_delta * smooth_ratio;
                        candidate.at(point_index).at(lateral_axis) =
                            base.front().at(lateral_axis) + direction *
                            corridor_step * lateral_profile;
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

            const double corridor_step = std::max(step, 8.0);
            for (const std::size_t axis : {0U, 1U})
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
                        double profile = 1.0;
                        if (point_index == 1U ||
                            point_index + 2U == candidate.size())
                        {
                            profile = 0.0;
                        }
                        else if (point_index == 2U || point_index == 3U ||
                                 point_index + 3U == candidate.size() ||
                                 point_index + 4U == candidate.size())
                        {
                            profile = 0.75;
                        }
                        candidate.at(point_index).at(axis) +=
                            direction * corridor_step * profile;
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
        }

        // Try a few bounded horizontal translations before per-control-point
        // coordinate descent.  This is important when a symmetric barrier
        // makes the lateral finite-difference gradient zero: a single large
        // continuous side-step can leave the centerline in one evaluation,
        // while many small point-wise steps would consume the 20 ms budget.
        for (const double coarse_step : {8.0, 6.0, 4.5, 4.0, 2.0, 1.0, 0.5})
        {
            const double candidate_step = std::max(step, coarse_step);
            for (const std::size_t axis : {0U, 1U})
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
                        const double normalized_index =
                            static_cast<double>(point_index) /
                            static_cast<double>(candidate.size() - 1U);
                        const double sine_profile = std::sin(
                            3.14159265358979323846 * normalized_index);
                        const double sine_squared = sine_profile *
                            sine_profile;
                        const double smooth_profile = sine_squared *
                            sine_squared;
                        candidate.at(point_index).at(axis) +=
                            direction * candidate_step * smooth_profile;
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
        }

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

        // Resolve a symmetric peer-barrier stationary point before applying
        // the normalized finite-difference gradient.  Otherwise tiny
        // numerical asymmetries can move the seed toward a sharp, dynamic-
        // infeasible corner before the horizontal escape is considered.
        if (current_evaluation.peer_cost >
            config.convergence_tolerance)
        {
            std::vector<Vector3> side_candidate;
            OptimizationEvaluation side_evaluation;
            if (find_coordinate_improvement(
                    current, current_evaluation, step_m,
                    side_candidate, side_evaluation))
            {
                if (is_hard_clear(side_evaluation))
                {
                    result.control_points = side_candidate;
                    result.evaluation = side_evaluation;
                    result.is_valid = true;
                    result.termination =
                        SolverTerminationReason::CONVERGED;
                    return result;
                }
                if (side_evaluation.total_cost <
                    current_evaluation.total_cost)
                {
                    current = side_candidate;
                    current_evaluation = side_evaluation;
                    continue;
                }
            }
            // 已經通過 hard clearance 的 seed 即使仍有少量 soft cost，
            // 也不應因 finite-difference budget 耗盡而回報 timeout。
            // side probe 沒有找到更低成本候選時，保留這條可交給共同
            // safety validator 的可行路徑。
            if (is_hard_clear(current_evaluation))
            {
                result.control_points = current;
                result.evaluation = current_evaluation;
                result.is_valid = true;
                result.termination = SolverTerminationReason::CONVERGED;
                return result;
            }
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

                // Projection 可能把某個自由度固定住（例如 FIXED_Z
                // 或 LINEAR_Z）。若正負 finite-difference 擾動在投影
                // 後完全相同，該軸的梯度必為零；跳過 objective 評估，
                // 把有限的 planner budget 留給真正可移動的水平軸。
                if (std::abs(
                        plus.at(point_index).at(axis) -
                        minus.at(point_index).at(axis)) <=
                    config.convergence_tolerance)
                {
                    gradient.at(point_index).at(axis) = 0.0;
                    continue;
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
            // A peer barrier can improve along the path direction while its
            // lateral gradient remains exactly zero on a symmetric center
            // line.  Always give the bounded horizontal coordinate escape a
            // chance while peer cost remains active, then keep whichever
            // continuous candidate has the lower objective cost.
            std::vector<Vector3> side_candidate;
            OptimizationEvaluation side_evaluation;
            const bool has_side_improvement =
                current_evaluation.peer_cost >
                    config.convergence_tolerance &&
                find_coordinate_improvement(
                    current, current_evaluation, step_m,
                    side_candidate, side_evaluation);
            if (has_side_improvement &&
                is_hard_clear(side_evaluation))
            {
                result.control_points = side_candidate;
                result.evaluation = side_evaluation;
                result.is_valid = true;
                result.termination = SolverTerminationReason::CONVERGED;
                return result;
            }
            if (has_side_improvement &&
                side_evaluation.total_cost <
                    candidate_evaluation.total_cost)
            {
                current = side_candidate;
                current_evaluation = side_evaluation;
            }
            else
            {
                current = candidate;
                current_evaluation = candidate_evaluation;
            }
            step_m = std::min(config.initial_step_m, step_m * 1.1);
            continue;
        }

        // 障礙物 clearance barrier 在幾何轉折或 nearest-point 切換處可能
        // 讓整體 normalized gradient 的 backtracking 方向暫時不降成本。
        // 以同一個 step 做 deterministic coordinate line-search，仍只在
        // bounded projection 內搜尋，不引入多 topology seed。
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
           config.convergence_tolerance > EGO_SWARM_EPSILON_M &&
           is_finite(config.hard_peer_distance_m) &&
           config.hard_peer_distance_m >= 0.0 &&
           is_finite(config.hard_obstacle_clearance_m) &&
           config.hard_obstacle_clearance_m >= 0.0;
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
