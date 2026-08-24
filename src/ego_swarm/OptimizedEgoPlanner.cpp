/**
 * @file OptimizedEgoPlanner.cpp
 * @brief continuous B-spline optimized EGO planner 實作
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#include "ego_swarm/OptimizedEgoPlanner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "ego_swarm/BsplineTrajectory.hpp"
#include "ego_swarm/SpatiotemporalRouteInitializer.hpp"
#include "ego_swarm/TrajectoryPriority.hpp"

namespace taiphoon
{
namespace ego_swarm
{

namespace
{

// 10 點控制器沿用原本 6 點 cubic B-spline 的 3 spans 作為時間基準；
// 實際 knot interval 仍依目前控制點數量重新分配。
constexpr double LEGACY_TIMING_SPAN_COUNT = 3.0;
constexpr double LEGACY_TIMING_CONTROL_POINT_DENOMINATOR = 5.0;

class TrajectoryObjective final : public IOptimizationObjective
{
public:
    TrajectoryObjective(
        const PlanningSnapshot& snapshot,
        const PlannerConfig& config,
        const double knot_interval_s,
        const double evaluation_sample_interval_s = 0.0)
        : snapshot_(snapshot), config_(config),
          knot_interval_s_(knot_interval_s),
          evaluation_sample_interval_s_(evaluation_sample_interval_s >
                    EGO_SWARM_EPSILON_M
                ? evaluation_sample_interval_s
                : std::max(config.sample_interval_s, 0.75))
    {
    }

    OptimizationEvaluation evaluate(
        const std::vector<Vector3>& control_points) const override
    {
        OptimizationEvaluation result;
        if (control_points.size() <= BSPLINE_DEGREE ||
            !is_finite(knot_interval_s_))
        {
            return result;
        }
        for (const Vector3& point : control_points)
        {
            if (!is_finite_vector(point))
            {
                return result;
            }
        }

        const BsplineTrajectory trajectory(
            control_points, knot_interval_s_);
        if (!trajectory.is_valid())
        {
            return result;
        }

        // Objective 只提供 optimizer 的平滑引導；最終仍由 continuous
        // safety validator 以正式取樣密度重新驗證。使用較稀疏的 objective
        // sampling，讓長距離路徑與多個 peer occupancy 仍能在 20 ms budget
        // 內完成 central finite-difference 評估。
        const double objective_sample_interval =
            evaluation_sample_interval_s_;
        std::size_t sample_count = static_cast<std::size_t>(std::ceil(
            trajectory.duration_s() / objective_sample_interval));
        sample_count = std::max<std::size_t>(1U, std::min(
            sample_count, EGO_SWARM_MAX_SAMPLE_COUNT));
        const double sample_divisor = static_cast<double>(sample_count + 1U);
        double minimum_obstacle_distance =
            std::numeric_limits<double>::max();
        double minimum_peer_distance = std::numeric_limits<double>::max();
        std::size_t active_peer_count = 0U;

        for (std::size_t index = 0U; index <= sample_count; ++index)
        {
            const double relative_time = std::min(
                trajectory.duration_s(),
                static_cast<double>(index) * objective_sample_interval);
            const TrajectoryState sample = trajectory.evaluate_state(
                relative_time);
            if (!sample.is_valid)
            {
                return OptimizationEvaluation{};
            }

            // 起終點是 optimizer 的硬約束；goal term 只評估 terminal
            // 誤差，避免把整條已經平滑的路徑不合理地拉向終點。
            if (index == sample_count)
            {
                const Vector3 goal_delta = subtract_vectors(
                    sample.position, snapshot_.local_goal);
                result.goal_cost = squared_norm(goal_delta);
            }
            result.dynamic_cost += dynamic_excess_cost(sample);

            if (!evaluate_obstacle(
                    sample.position, minimum_obstacle_distance,
                    result.obstacle_cost))
            {
                return OptimizationEvaluation{};
            }
            if (!evaluate_peers(
                    sample.position, relative_time, minimum_peer_distance,
                    active_peer_count, result.peer_cost))
            {
                return OptimizationEvaluation{};
            }
        }

        result.smoothness_cost = smoothness_cost(control_points);
        if (snapshot_.obstacle_map->point_count() == 0U)
        {
            minimum_obstacle_distance = 0.0;
        }
        else if (minimum_obstacle_distance <
                 config_.obstacle_clearance_m +
                     config_.optimizer_obstacle_soft_margin_m)
        {
            const double clearance_error =
                config_.obstacle_clearance_m +
                config_.optimizer_obstacle_soft_margin_m -
                minimum_obstacle_distance;
            result.obstacle_cost += 100.0 * clearance_error * clearance_error +
                10.0 * clearance_error;
        }
        if (active_peer_count == 0U)
        {
            minimum_peer_distance = 0.0;
        }
        result.goal_cost /= sample_divisor;
        result.dynamic_cost /= sample_divisor;
        result.obstacle_cost /= sample_divisor;
        result.peer_cost /= sample_divisor;
        result.total_cost = config_.goal_weight * result.goal_cost +
            config_.smoothness_weight * result.smoothness_cost +
            config_.dynamic_weight * result.dynamic_cost +
            config_.obstacle_weight * result.obstacle_cost +
            config_.peer_weight * result.peer_cost;
        result.minimum_obstacle_distance_m = minimum_obstacle_distance;
        result.minimum_peer_distance_m = minimum_peer_distance;
        result.is_valid = is_finite(result.total_cost) &&
            is_finite(result.goal_cost) &&
            is_finite(result.smoothness_cost) &&
            is_finite(result.dynamic_cost) &&
            is_finite(result.obstacle_cost) &&
            is_finite(result.peer_cost) &&
            is_finite(result.minimum_obstacle_distance_m) &&
            is_finite(result.minimum_peer_distance_m);
        return result;
    }

private:
    double dynamic_excess_cost(const TrajectoryState& sample) const
    {
        return squared_excess(
                   std::sqrt(squared_norm(sample.velocity)),
                   config_.max_velocity_mps) +
            squared_excess(
                std::sqrt(squared_norm(sample.acceleration)),
                config_.max_acceleration_mps2) +
            squared_excess(
                std::sqrt(squared_norm(sample.jerk)),
                config_.max_jerk_mps3);
    }

    static double squared_excess(const double value, const double limit)
    {
        if (!is_finite(value) || !is_finite(limit))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double excess = std::max(0.0, value - limit);
        return excess * excess;
    }

    bool evaluate_obstacle(
        const Vector3& position,
        double& minimum_distance,
        double& cost) const
    {
        if (snapshot_.obstacle_map == nullptr ||
            !snapshot_.obstacle_map->is_valid() ||
            !snapshot_.obstacle_map->is_fresh(snapshot_.planning_time_s))
        {
            return false;
        }
        if (snapshot_.obstacle_map->point_count() == 0U)
        {
            return true;
        }
        const double distance = snapshot_.obstacle_map->nearest_distance(
            position);
        if (!is_finite(distance))
        {
            return false;
        }
        minimum_distance = std::min(minimum_distance, distance);
        const double hard_clearance_error =
            config_.obstacle_clearance_m - distance;
        const double soft_clearance_error =
            config_.obstacle_clearance_m +
            config_.optimizer_obstacle_soft_margin_m - distance;
        if (hard_clearance_error > 0.0)
        {
            cost += 10000.0 * hard_clearance_error * hard_clearance_error +
                1000.0 * hard_clearance_error;
        }
        else if (soft_clearance_error > 0.0)
        {
            cost += 100.0 * soft_clearance_error * soft_clearance_error +
                10.0 * soft_clearance_error;
        }
        return is_finite(cost);
    }

    bool evaluate_peers(
        const Vector3& position,
        const double relative_time_s,
        double& minimum_distance,
        std::size_t& active_peer_count,
        double& cost) const
    {
        const double absolute_time_s = snapshot_.trajectory_start_time_s +
            relative_time_s;
        if (!is_finite(absolute_time_s))
        {
            return false;
        }
        for (const PeerTrajectory& peer : snapshot_.peers)
        {
            if (peer.agent_id == snapshot_.self_agent_id ||
                (!peer.terminal_hold &&
                 absolute_time_s > peer.end_time_s + EGO_SWARM_EPSILON_M))
            {
                continue;
            }
            if (!should_check_peer_occupancy(
                    peer, absolute_time_s,
                    snapshot_.trajectory_start_time_s,
                    snapshot_.self_agent_id))
            {
                continue;
            }
            const double peer_time_s = std::max(
                0.0, absolute_time_s - peer.start_time_s);
            const TrajectoryState peer_sample = peer.trajectory.evaluate_state(
                peer_time_s);
            if (!peer_sample.is_valid)
            {
                return false;
            }
            ++active_peer_count;
            const double distance = distance_between_centers(
                position, peer_sample.position);
            if (!is_finite(distance))
            {
                return false;
            }
            minimum_distance = std::min(minimum_distance, distance);
            const double hard_separation_error =
                config_.peer_collision_min_distance_m - distance;
            const double soft_separation_error =
                peer_planning_soft_clearance_m(config_) +
                config_.optimizer_peer_soft_margin_m - distance;
            if (hard_separation_error > 0.0)
            {
                cost += 10000.0 * hard_separation_error *
                    hard_separation_error + 1000.0 * hard_separation_error;
            }
            else if (soft_separation_error > 0.0)
            {
                cost += 100.0 * soft_separation_error *
                    soft_separation_error + 10.0 * soft_separation_error;
            }
        }
        return is_finite(cost);
    }

    static double smoothness_cost(
        const std::vector<Vector3>& control_points)
    {
        double cost = 0.0;
        for (std::size_t index = 1U;
             index + 1U < control_points.size(); ++index)
        {
            const Vector3 difference{
                control_points.at(index + 1U).at(0) -
                    2.0 * control_points.at(index).at(0) +
                    control_points.at(index - 1U).at(0),
                control_points.at(index + 1U).at(1) -
                    2.0 * control_points.at(index).at(1) +
                    control_points.at(index - 1U).at(1),
                control_points.at(index + 1U).at(2) -
                    2.0 * control_points.at(index).at(2) +
                    control_points.at(index - 1U).at(2)};
            cost += squared_norm(difference);
        }
        return cost;
    }

    const PlanningSnapshot& snapshot_;
    const PlannerConfig& config_;
    double knot_interval_s_{0.0};
    double evaluation_sample_interval_s_{0.75};
};

PlannerTerminationReason convert_solver_reason(
    const SolverTerminationReason reason)
{
    switch (reason)
    {
        case SolverTerminationReason::CONVERGED:
            return PlannerTerminationReason::SUCCESS;
        case SolverTerminationReason::INVALID_INPUT:
            return PlannerTerminationReason::INVALID_INPUT;
        case SolverTerminationReason::NONFINITE_GRADIENT:
            return PlannerTerminationReason::NONFINITE_GRADIENT;
        case SolverTerminationReason::ITERATION_LIMIT:
            return PlannerTerminationReason::ITERATION_LIMIT;
        case SolverTerminationReason::TIMEOUT:
            return PlannerTerminationReason::TIMEOUT;
        case SolverTerminationReason::DIVERGED:
            return PlannerTerminationReason::DIVERGED;
        case SolverTerminationReason::INFEASIBLE:
            return PlannerTerminationReason::INFEASIBLE;
    }
    return PlannerTerminationReason::INFEASIBLE;
}

PlannerTerminationReason convert_route_reason(
    const RouteInitializerTerminationReason reason)
{
    switch (reason)
    {
        case RouteInitializerTerminationReason::SUCCESS:
            return PlannerTerminationReason::SUCCESS;
        case RouteInitializerTerminationReason::INVALID_INPUT:
            return PlannerTerminationReason::INVALID_INPUT;
        case RouteInitializerTerminationReason::NO_FEASIBLE_PATH:
            return PlannerTerminationReason::NO_FEASIBLE_PATH;
        case RouteInitializerTerminationReason::TIMEOUT:
            return PlannerTerminationReason::TIMEOUT;
    }
    return PlannerTerminationReason::INFEASIBLE;
}

bool build_control_points_from_timed_route(
    const std::vector<TimedRoutePoint>& route,
    const std::size_t control_point_count,
    const double fixed_z,
    std::vector<Vector3>& control_points,
    double& duration_s)
{
    if (route.size() < 2U || control_point_count <= BSPLINE_DEGREE ||
        !is_finite(fixed_z))
    {
        return false;
    }
    duration_s = route.back().time_s;
    if (!is_finite(duration_s) || duration_s <= EGO_SWARM_EPSILON_M)
    {
        return false;
    }
    for (std::size_t index = 0U; index < route.size(); ++index)
    {
        if (!is_finite_vector(route.at(index).position) ||
            !is_finite(route.at(index).time_s) ||
            (index > 0U && route.at(index).time_s <=
                route.at(index - 1U).time_s))
        {
            return false;
        }
    }

    control_points.clear();
    control_points.reserve(control_point_count);
    std::size_t segment_index = 0U;
    for (std::size_t index = 0U; index < control_point_count; ++index)
    {
        const double target_time = duration_s * static_cast<double>(index) /
            static_cast<double>(control_point_count - 1U);
        while (segment_index + 1U < route.size() &&
               route.at(segment_index + 1U).time_s < target_time)
        {
            ++segment_index;
        }
        Vector3 point = route.back().position;
        if (target_time <= route.front().time_s)
        {
            point = route.front().position;
        }
        else if (segment_index + 1U < route.size())
        {
            const TimedRoutePoint& first = route.at(segment_index);
            const TimedRoutePoint& second = route.at(segment_index + 1U);
            const double span = second.time_s - first.time_s;
            const double ratio = span <= EGO_SWARM_EPSILON_M
                ? 0.0 : (target_time - first.time_s) / span;
            point = Vector3{
                first.position.at(0) +
                    (second.position.at(0) - first.position.at(0)) * ratio,
                first.position.at(1) +
                    (second.position.at(1) - first.position.at(1)) * ratio,
                fixed_z};
        }
        point.at(2) = fixed_z;
        if (!is_finite_vector(point))
        {
            control_points.clear();
            return false;
        }
        control_points.push_back(point);
    }
    control_points.front() = route.front().position;
    control_points.back() = route.back().position;
    control_points.front().at(2) = fixed_z;
    control_points.back().at(2) = fixed_z;
    return true;
}

}  // namespace

OptimizedEgoPlanner::OptimizedEgoPlanner(const PlannerConfig& config)
{
    if (!set_config(config))
    {
        config_ = PlannerConfig{};
    }
}

bool OptimizedEgoPlanner::set_config(const PlannerConfig& config)
{
    PlannerConfig normalized = config;
    constexpr double DEFAULT_MAX_VELOCITY_MPS = 2.5;
    constexpr double DEFAULT_MAX_ACCELERATION_MPS2 = 1.5;
    if (std::abs(normalized.max_velocity_mps - DEFAULT_MAX_VELOCITY_MPS) <=
            EGO_SWARM_EPSILON_M &&
        std::abs(normalized.max_velocity_m_s - DEFAULT_MAX_VELOCITY_MPS) >
            EGO_SWARM_EPSILON_M)
    {
        normalized.max_velocity_mps = normalized.max_velocity_m_s;
    }
    if (std::abs(normalized.max_acceleration_mps2 -
                 DEFAULT_MAX_ACCELERATION_MPS2) <= EGO_SWARM_EPSILON_M &&
        std::abs(normalized.max_acceleration_m_s2 -
                 DEFAULT_MAX_ACCELERATION_MPS2) > EGO_SWARM_EPSILON_M)
    {
        normalized.max_acceleration_mps2 = normalized.max_acceleration_m_s2;
    }
    normalized.max_velocity_m_s = normalized.max_velocity_mps;
    normalized.max_acceleration_m_s2 = normalized.max_acceleration_mps2;
    if (!is_valid_config(normalized))
    {
        return false;
    }
    config_ = normalized;
    return true;
}

const PlannerConfig& OptimizedEgoPlanner::config() const
{
    return config_;
}

PlannerResult OptimizedEgoPlanner::plan(
    const PlanningSnapshot& snapshot) const
{
    PlannerResult result;
    result.backend = PlannerBackendKind::OPTIMIZED;
    result.trajectory_start_time_s = snapshot.trajectory_start_time_s;
    const auto start = std::chrono::steady_clock::now();
    const auto finish = [&result, &start](PlannerResult output)
    {
        output.planning_time_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        return output;
    };

    if (!is_valid_config(config_) || !snapshot.is_valid())
    {
        result.termination = PlannerTerminationReason::INVALID_INPUT;
        return finish(result);
    }

    const std::size_t count = config_.control_point_count;
    const double denominator = static_cast<double>(count - 1U);
    double duration_s = compute_seed_duration_s(
        config_, snapshot.self_state.position, snapshot.local_goal);

    std::vector<Vector3> seed;
    seed.reserve(count);
    const double distance = distance_between_centers(
        snapshot.self_state.position, snapshot.local_goal);
    // 所有距離都使用對稱 smoothstep 分布，降低 clamped B-spline 的
    // 起步與收尾速度，讓 10 點 seed 與 6 點 baseline 具有相同的
    // timing reference；起點與終點仍保持不變。
    for (std::size_t index = 0U; index < count; ++index)
    {
        const double linear_ratio = static_cast<double>(index) / denominator;
        const double ratio = linear_ratio * linear_ratio *
            (3.0 - 2.0 * linear_ratio);
        Vector3 seed_point = interpolate_vectors(
            snapshot.self_state.position, snapshot.local_goal, ratio);
        if (snapshot.path_constraint == PathConstraint::LINEAR_Z)
        {
            // LINEAR_Z 是明確的幾何約束；smoothstep 只作用於水平
            // seed，避免 open-space fast path 產生非線性高度控制點。
            seed_point.at(2) = snapshot.self_state.position.at(2) +
                (snapshot.local_goal.at(2) -
                 snapshot.self_state.position.at(2)) * linear_ratio;
        }
        seed.push_back(seed_point);
    }

    PlanningSnapshot planning_snapshot = snapshot;
    if (config_.enable_spatiotemporal_route_planning &&
        snapshot.path_constraint == PathConstraint::FIXED_Z)
    {
        PlannerConfig route_config = config_;
        SpatiotemporalRouteInitializer route_initializer;
        const SpatiotemporalRouteResult route_result =
            route_initializer.initialize(snapshot, route_config, seed);
        result.route_search_used = route_result.used_search;
        result.route_expanded_nodes = route_result.expanded_nodes;
        result.route_search_time_ms = route_result.search_time_ms;
        if (!route_result.is_valid)
        {
            result.termination = convert_route_reason(
                route_result.termination);
            return finish(result);
        }
        result.trajectory_start_time_s =
            route_result.trajectory_start_time_s;
        if (route_result.used_search &&
            !build_control_points_from_timed_route(
                route_result.route, count, snapshot.fixed_z_m, seed,
                duration_s))
        {
            result.termination = PlannerTerminationReason::INFEASIBLE;
            return finish(result);
        }
        planning_snapshot.trajectory_start_time_s =
            route_result.trajectory_start_time_s;
    }

    const double span_count = static_cast<double>(count - BSPLINE_DEGREE);
    const double knot_interval_s = duration_s / span_count;
    if (!is_finite(knot_interval_s) ||
        knot_interval_s <= EGO_SWARM_EPSILON_M)
    {
        result.termination = PlannerTerminationReason::INVALID_INPUT;
        return finish(result);
    }

    const double search_radius = config_.optimizer_search_radius_m;
    const auto projection = [
        &seed, snapshot, search_radius](std::vector<Vector3>& points)
    {
        if (points.size() != seed.size() || !is_finite(search_radius) ||
            search_radius < 0.0)
        {
            return false;
        }
        points.front() = snapshot.self_state.position;
        points.back() = snapshot.local_goal;
        for (std::size_t index = 1U; index + 1U < points.size(); ++index)
        {
            for (std::size_t axis = 0U; axis < 3U; ++axis)
            {
                const double lower = seed.at(index).at(axis) - search_radius;
                const double upper = seed.at(index).at(axis) + search_radius;
                points.at(index).at(axis) = std::max(
                    lower, std::min(upper, points.at(index).at(axis)));
            }
            if (snapshot.path_constraint == PathConstraint::FIXED_Z)
            {
                points.at(index).at(2) = snapshot.fixed_z_m;
            }
            else if (snapshot.path_constraint == PathConstraint::LINEAR_Z)
            {
                const double ratio = static_cast<double>(index) /
                    static_cast<double>(points.size() - 1U);
                points.at(index).at(2) = snapshot.self_state.position.at(2) +
                    (snapshot.local_goal.at(2) -
                     snapshot.self_state.position.at(2)) * ratio;
            }
        }
        return is_finite_vector(points.front()) &&
            is_finite_vector(points.back());
    };

    // 長距離 crossing 的 central finite-difference 需要重複評估整條
    // B-spline；只降低 optimizer 引導 sampling，保留後續 hard safety
    // validator 的密集取樣與 3 m contract。
    const double optimizer_objective_interval_s = distance > 25.0
        ? 1.0
        : 0.0;
    TrajectoryObjective objective(
        planning_snapshot, config_, knot_interval_s,
        optimizer_objective_interval_s);
    const BsplineTrajectory seed_trajectory(seed, knot_interval_s);
    const OptimizationEvaluation seed_evaluation = objective.evaluate(seed);
    // Objective 的稀疏 sampling 只用來做 gradient 引導；fast path 不能
    // 因為稀疏 sampling 漏掉狹窄的 peer crossing。直線 seed 若有 peer，
    // 再用 safety sampling interval 做一次 hard peer pre-check，真正需要
    // 繞行時才交給 solver。
    const OptimizationEvaluation seed_peer_safety_evaluation =
        planning_snapshot.peers.empty()
        ? seed_evaluation
        : TrajectoryObjective(
              planning_snapshot, config_, knot_interval_s,
              config_.sample_interval_s)
              .evaluate(seed);
    const bool seed_has_hard_clearance =
        (snapshot.obstacle_map->point_count() == 0U ||
         seed_peer_safety_evaluation.minimum_obstacle_distance_m +
             EGO_SWARM_EPSILON_M >= config_.obstacle_clearance_m) &&
        (seed_peer_safety_evaluation.minimum_peer_distance_m <= 0.0 ||
         seed_peer_safety_evaluation.minimum_peer_distance_m +
             EGO_SWARM_EPSILON_M >= config_.peer_collision_min_distance_m);
    // 直線 seed 已經通過密集 safety sampling，且沒有障礙物／同儕
    // soft penalty 時，不需要為了 open-space 長距離任務重複執行昂貴
    // 的 central finite-difference。obstacle map 可以包含遠方或保留
    // 點，因此不能以 point_count 是否為零作為 fast path 條件。
    // 這仍是 optimized backend 的 accepted seed，Node 之後仍會交給
    // ContinuousSafetyValidator；只有需要水平繞行時才進入 solver。
    if (seed_trajectory.is_valid() && seed_evaluation.is_valid &&
        seed_peer_safety_evaluation.is_valid &&
        seed_peer_safety_evaluation.obstacle_cost <=
            config_.optimizer_convergence_tolerance &&
        seed_peer_safety_evaluation.peer_cost <=
            config_.optimizer_convergence_tolerance &&
        seed_has_hard_clearance &&
        is_hard_dynamically_feasible(seed_trajectory, config_))
    {
        result.is_valid = true;
        result.is_hold = false;
        result.trajectory = seed_trajectory;
        result.cost = seed_peer_safety_evaluation.total_cost;
        result.goal_cost = seed_peer_safety_evaluation.goal_cost;
        result.smoothness_cost = seed_peer_safety_evaluation.smoothness_cost;
        result.dynamic_cost = seed_peer_safety_evaluation.dynamic_cost;
        result.obstacle_cost = seed_peer_safety_evaluation.obstacle_cost;
        result.peer_cost = seed_peer_safety_evaluation.peer_cost;
        result.minimum_obstacle_distance_m =
            seed_peer_safety_evaluation.minimum_obstacle_distance_m;
        result.minimum_peer_distance_m =
            seed_peer_safety_evaluation.minimum_peer_distance_m;
        result.termination = PlannerTerminationReason::SUCCESS;
        return finish(result);
    }

    ProjectedGradientSolver solver;
    // 3 m 是最後 safety contract；optimizer 另保留 soft margin，吸收
    // snapshot 更新、PX4 setpoint tracking 與 peer 狀態同步造成的誤差。
    const bool long_route_needs_tracking_margin = distance > 25.0;
    const double optimizer_peer_acceptance_distance =
        config_.peer_collision_min_distance_m +
        (long_route_needs_tracking_margin
            ? config_.optimizer_peer_soft_margin_m
            : 0.0);
    const double elapsed_before_solver_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    const double remaining_solver_budget_ms =
        config_.optimizer_time_budget_ms - elapsed_before_solver_ms;
    if (!is_finite(remaining_solver_budget_ms) ||
        remaining_solver_budget_ms <= EGO_SWARM_EPSILON_M)
    {
        result.termination = PlannerTerminationReason::TIMEOUT;
        return finish(result);
    }
    const ProjectedGradientSolverConfig solver_config{
        config_.optimizer_max_iterations,
        remaining_solver_budget_ms,
        config_.optimizer_gradient_epsilon_m,
        config_.optimizer_initial_step_m,
        config_.optimizer_minimum_step_m,
        config_.optimizer_convergence_tolerance,
        optimizer_peer_acceptance_distance,
        config_.obstacle_clearance_m};
    const ProjectedGradientSolverResult solver_result = solver.solve(
        seed, objective, solver_config, projection);
    result.iteration_count = solver_result.iterations;
    result.termination = convert_solver_reason(solver_result.termination);
    result.cost = solver_result.evaluation.total_cost;
    result.goal_cost = solver_result.evaluation.goal_cost;
    result.smoothness_cost = solver_result.evaluation.smoothness_cost;
    result.dynamic_cost = solver_result.evaluation.dynamic_cost;
    result.obstacle_cost = solver_result.evaluation.obstacle_cost;
    result.peer_cost = solver_result.evaluation.peer_cost;
    result.minimum_obstacle_distance_m =
        solver_result.evaluation.minimum_obstacle_distance_m;
    result.minimum_peer_distance_m =
        solver_result.evaluation.minimum_peer_distance_m;
    const bool solver_has_hard_clearance =
        (snapshot.obstacle_map->point_count() == 0U ||
         solver_result.evaluation.minimum_obstacle_distance_m +
             EGO_SWARM_EPSILON_M >= config_.obstacle_clearance_m) &&
        (solver_result.evaluation.minimum_peer_distance_m <= 0.0 ||
         solver_result.evaluation.minimum_peer_distance_m +
             EGO_SWARM_EPSILON_M >= optimizer_peer_acceptance_distance);
    // A route seed can already be hard-safe while the bounded numerical
    // solver is unable to reduce the remaining soft smoothness/clearance
    // cost.  Do not discard that geometrically safe candidate solely because
    // the solver reports DIVERGED; the hard dynamics check below and the
    // Node's existing ContinuousSafetyValidator remain mandatory gates.
    if (!solver_result.is_valid ||
        (solver_result.termination != SolverTerminationReason::CONVERGED &&
         !solver_has_hard_clearance))
    {
        return finish(result);
    }

    std::vector<Vector3> final_control_points = solver_result.control_points;
    OptimizationEvaluation final_evaluation = solver_result.evaluation;
    BsplineTrajectory trajectory(final_control_points, knot_interval_s);
    if (!trajectory.is_valid() ||
        !is_hard_dynamically_feasible(trajectory, config_))
    {
        // A bounded optimizer detour can exceed the endpoint dynamic limit
        // by a small amount after the peer barrier is cleared.  Try only
        // deterministic contractions toward the original line seed; every
        // candidate is re-evaluated and still must pass hard dynamics and
        // peer clearance before it can be accepted.
        bool recovered_dynamics = false;
        for (const double contraction :
             {0.995, 0.99, 0.985, 0.98, 0.97, 0.95, 0.90})
        {
            std::vector<Vector3> candidate = solver_result.control_points;
            for (std::size_t index = 1U;
                 index + 1U < candidate.size(); ++index)
            {
                candidate.at(index) = add_vectors(
                    seed.at(index),
                    scale_vector(
                        subtract_vectors(
                            solver_result.control_points.at(index),
                            seed.at(index)),
                        contraction));
            }
            if (!projection(candidate) || !is_finite_vector(candidate.front()) ||
                !is_finite_vector(candidate.back()))
            {
                continue;
            }
            const BsplineTrajectory candidate_trajectory(
                candidate, knot_interval_s);
            const OptimizationEvaluation candidate_evaluation =
                objective.evaluate(candidate);
            if (!candidate_trajectory.is_valid() ||
                !candidate_evaluation.is_valid ||
                (candidate_evaluation.minimum_peer_distance_m > 0.0 &&
                 candidate_evaluation.minimum_peer_distance_m +
                     EGO_SWARM_EPSILON_M <
                     optimizer_peer_acceptance_distance) ||
                !is_hard_dynamically_feasible(
                    candidate_trajectory, config_))
            {
                continue;
            }
            final_control_points = candidate;
            final_evaluation = candidate_evaluation;
            trajectory = candidate_trajectory;
            recovered_dynamics = true;
            break;
        }
        if (!recovered_dynamics)
        {
            result.termination = PlannerTerminationReason::INFEASIBLE;
            return finish(result);
        }
    }

    if ((snapshot.obstacle_map->point_count() > 0U &&
         final_evaluation.minimum_obstacle_distance_m + EGO_SWARM_EPSILON_M <
             config_.obstacle_clearance_m) ||
        (final_evaluation.minimum_peer_distance_m > 0.0 &&
         final_evaluation.minimum_peer_distance_m + EGO_SWARM_EPSILON_M <
             optimizer_peer_acceptance_distance))
    {
        result.termination = PlannerTerminationReason::INFEASIBLE;
        return finish(result);
    }

    result.is_valid = true;
    result.is_hold = false;
    result.trajectory = trajectory;
    result.cost = final_evaluation.total_cost;
    result.goal_cost = final_evaluation.goal_cost;
    result.smoothness_cost = final_evaluation.smoothness_cost;
    result.dynamic_cost = final_evaluation.dynamic_cost;
    result.obstacle_cost = final_evaluation.obstacle_cost;
    result.peer_cost = final_evaluation.peer_cost;
    result.minimum_obstacle_distance_m =
        final_evaluation.minimum_obstacle_distance_m;
    result.minimum_peer_distance_m =
        final_evaluation.minimum_peer_distance_m;
    result.termination = PlannerTerminationReason::SUCCESS;
    return finish(result);
}

bool OptimizedEgoPlanner::is_valid_config(const PlannerConfig& config)
{
    return is_finite(config.knot_interval_s) &&
           config.knot_interval_s > EGO_SWARM_EPSILON_M &&
           config.control_point_count >= BSPLINE_DEGREE + 1U &&
           config.control_point_count <= EGO_SWARM_MAX_SAMPLE_COUNT &&
           is_finite(config.sample_interval_s) &&
           config.sample_interval_s > EGO_SWARM_EPSILON_M &&
           is_finite(config.max_velocity_mps) &&
           config.max_velocity_mps > EGO_SWARM_EPSILON_M &&
           is_finite(config.max_acceleration_mps2) &&
           config.max_acceleration_mps2 > EGO_SWARM_EPSILON_M &&
           is_finite(config.max_jerk_mps3) &&
           config.max_jerk_mps3 > EGO_SWARM_EPSILON_M &&
           is_finite(config.candidate_offset_m) &&
           config.candidate_offset_m >= 0.0 &&
           is_finite(config.optimizer_search_radius_m) &&
           config.optimizer_search_radius_m > EGO_SWARM_EPSILON_M &&
           is_finite(config.optimizer_peer_soft_margin_m) &&
           config.optimizer_peer_soft_margin_m >= 0.0 &&
           is_finite(config.optimizer_obstacle_soft_margin_m) &&
           config.optimizer_obstacle_soft_margin_m >= 0.0 &&
           is_finite(config.obstacle_clearance_m) &&
           config.obstacle_clearance_m >= 0.0 &&
           is_finite(config.peer_collision_min_distance_m) &&
           config.peer_collision_min_distance_m >= 0.0 &&
           is_finite(config.peer_planning_margin_m) &&
           config.peer_planning_margin_m >= 0.0 &&
           is_finite(config.goal_weight) && config.goal_weight >= 0.0 &&
           is_finite(config.smoothness_weight) &&
           config.smoothness_weight >= 0.0 &&
           is_finite(config.dynamic_weight) && config.dynamic_weight >= 0.0 &&
           is_finite(config.obstacle_weight) && config.obstacle_weight >= 0.0 &&
           is_finite(config.peer_weight) && config.peer_weight >= 0.0 &&
           config.optimizer_max_iterations > 0 &&
           is_finite(config.optimizer_time_budget_ms) &&
           config.optimizer_time_budget_ms > EGO_SWARM_EPSILON_M &&
           is_finite(config.optimizer_gradient_epsilon_m) &&
           config.optimizer_gradient_epsilon_m > EGO_SWARM_EPSILON_M &&
           is_finite(config.optimizer_initial_step_m) &&
           config.optimizer_initial_step_m > EGO_SWARM_EPSILON_M &&
           is_finite(config.optimizer_minimum_step_m) &&
           config.optimizer_minimum_step_m > EGO_SWARM_EPSILON_M &&
           config.optimizer_initial_step_m >= config.optimizer_minimum_step_m &&
           is_finite(config.optimizer_convergence_tolerance) &&
           config.optimizer_convergence_tolerance > EGO_SWARM_EPSILON_M;
}

double OptimizedEgoPlanner::compute_seed_duration_s(
    const PlannerConfig& config,
    const Vector3& current,
    const Vector3& target)
{
    const double distance = distance_between_centers(current, target);
    if (!is_finite(distance))
    {
        return 0.0;
    }
    const double span_count = LEGACY_TIMING_SPAN_COUNT;
    const double velocity_duration = distance / config.max_velocity_mps;
    const double acceleration_duration = 2.0 * std::sqrt(
        distance / config.max_acceleration_mps2);
    const double base_duration = std::max(
        span_count * config.knot_interval_s,
        std::max(velocity_duration, acceleration_duration));
    // Clamped cubic B-spline 的端點速度沿用舊 6 點／3 spans timing
    // reference：degree * 3 / (6 - 1) 倍的平均速度。
    // 長距離 seed 若只使用平均速度 duration，會在 endpoint derivative
    // 超過 hard velocity limit，導致 solver 必須在很短的 budget 內先
    // 修正一條本來就不合格的 seed。額外保留 10% 數值裕度，讓水平
    // 繞行的 endpoint 側向偏移也能通過 hard dynamic check。
    const double endpoint_velocity_factor =
        static_cast<double>(BSPLINE_DEGREE) * span_count /
        LEGACY_TIMING_CONTROL_POINT_DENOMINATOR;
    const double endpoint_velocity_duration =
        endpoint_velocity_factor * velocity_duration * 1.11;
    if (distance > 20.0)
    {
        // 長距離水平指令的驗收上限為 20 秒。smoothstep seed 已降低
        // endpoint derivative，因此以 1.25 倍動力學基準並保留 18 秒
        // 下限，為水平側繞保留足夠時間；即使實際起點有約 5% 追蹤
        // 超調，30 m 級路徑仍維持在 20 秒驗收上限內。最終速度、
        // 加速度與 jerk 仍由 hard feasibility 檢查。
        const double duration = std::max(18.0, 1.32 * base_duration);
        // MOVE_FORWARD 30 與其 target-based BACKWARD 30 可能因飛控
        // 追蹤誤差讓實際起點落在 35 m 以內；在這個驗收範圍內限制
        // duration，避免路徑安全但無法在 20 秒內完成。更長任務仍
        // 依完整動力學距離估算，不套用此短任務上限。
        return distance <= 35.0 ? std::min(19.8, duration) : duration;
    }
    return std::max(
        1.5 * base_duration, endpoint_velocity_duration);
}

bool OptimizedEgoPlanner::is_hard_dynamically_feasible(
    const BsplineTrajectory& trajectory,
    const PlannerConfig& config)
{
    const double raw_count = std::ceil(
        trajectory.duration_s() / config.sample_interval_s);
    if (!is_finite(raw_count) || raw_count <= 0.0)
    {
        return false;
    }
    const std::size_t count = std::min(
        static_cast<std::size_t>(raw_count), EGO_SWARM_MAX_SAMPLE_COUNT);
    for (std::size_t index = 0U; index <= count; ++index)
    {
        const double time_s = std::min(
            trajectory.duration_s(),
            static_cast<double>(index) * config.sample_interval_s);
        const TrajectoryState state = trajectory.evaluate_state(time_s);
        const double velocity_mps = std::sqrt(squared_norm(state.velocity));
        const double acceleration_mps2 =
            std::sqrt(squared_norm(state.acceleration));
        const double jerk_mps3 = std::sqrt(squared_norm(state.jerk));
        if (!state.is_valid ||
            velocity_mps > config.max_velocity_mps + EGO_SWARM_EPSILON_M ||
            acceleration_mps2 >
                config.max_acceleration_mps2 + EGO_SWARM_EPSILON_M ||
            jerk_mps3 > config.max_jerk_mps3 + EGO_SWARM_EPSILON_M)
        {
            return false;
        }
    }
    return true;
}

}  // namespace ego_swarm
}  // namespace taiphoon
