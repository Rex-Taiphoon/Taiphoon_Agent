/**
 * @file LocalGoalSelector.hpp
 * @brief EGO-Swarm Local Goal 純 C++ 計算 helper
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只負責依自身位置、Final Target、路徑高度約束與距離 horizon
 * 計算單一 planning segment 的局部終點，不管理 ROS2、Formation 或
 * trajectory lifecycle。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "ego_swarm/PlannerTypes.hpp"
#include "ego_swarm/TrajectoryPriority.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief Local Goal 計算設定
 */
struct LocalGoalConfig
{
    /// 單一 planning segment 的最大空間長度，單位為公尺。
    double horizon_m{10.0};
    /// 目標受阻時，沿 Final Target 方向回退的最小距離，單位為公尺。
    double minimum_horizon_m{1.0};
    /// Local Goal 候選的最小水平側向繞行偏移，單位為公尺。
    double detour_offset_m{4.5};
};

/**
 * @brief Local Goal 選擇失敗原因
 */
enum class LocalGoalFailureReason
{
    NONE = 0,
    INVALID_INPUT,
    STALE_PEER_DATA,
    INVALID_OBSTACLE_MAP,
    NO_SAFE_CANDIDATE
};

inline std::string local_goal_failure_reason_string(
    const LocalGoalFailureReason reason)
{
    switch (reason)
    {
        case LocalGoalFailureReason::NONE:
            return "none";
        case LocalGoalFailureReason::INVALID_INPUT:
            return "invalid_input";
        case LocalGoalFailureReason::STALE_PEER_DATA:
            return "stale_peer_data";
        case LocalGoalFailureReason::INVALID_OBSTACLE_MAP:
            return "invalid_obstacle_map";
        case LocalGoalFailureReason::NO_SAFE_CANDIDATE:
            return "no_safe_candidate";
    }
    return "unknown";
}

/**
 * @brief Safe Local Goal 選擇所需的 immutable environment snapshot
 *
 * 此資料由 ROS2 Node 在單一 planner cycle 建立後傳入 pure C++ module。
 * Local Goal 只做低成本的終點佔位檢查與候選產生；完整連續路徑仍由
 * optimized planner 與 ContinuousSafetyValidator 負責。
 */
struct LocalGoalEnvironment
{
    double now_s{0.0};
    double candidate_start_time_s{0.0};
    int self_agent_id{0};
    bool peers_are_fresh{false};
    std::vector<PeerTrajectory> peers{};
    const perception::IObstacleMap* obstacle_map{nullptr};
    double peer_collision_min_distance_m{3.0};
    double obstacle_clearance_m{1.0};
    double maximum_velocity_mps{2.5};
};

/**
 * @brief Local Goal 計算結果
 */
struct LocalGoalResult
{
    bool is_valid{false};
    bool is_final_goal{false};
    Vector3 goal{0.0, 0.0, 0.0};
    LocalGoalFailureReason failure_reason{LocalGoalFailureReason::INVALID_INPUT};
    std::size_t candidate_count{0U};
    double minimum_peer_distance_m{0.0};
    double minimum_obstacle_distance_m{0.0};
};

/**
 * @brief 依 Final Target 計算單一 Local Goal
 *
 * FIXED_Z 會先將 planning current 的 Z 投影到 Final Target Z，再計算
 * horizon；如此水平移動不會因原始三維高度差而縮短可規劃的水平距離。
 * LINEAR_Z 與 FREE_3D 則沿 current 到 Final Target 的完整向量做比例插值。
 *
 * @param current 目前位置，使用共同 planning frame，單位為公尺
 * @param final_target 任務真正終點，使用共同 planning frame，單位為公尺
 * @param path_constraint 路徑高度約束
 * @param config Local Goal 計算設定
 * @return Local Goal 計算結果；輸入非法時回傳無效結果
 */
inline LocalGoalResult compute_local_goal(
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint,
    const LocalGoalConfig& config)
{
    LocalGoalResult result;
    if (!is_finite_vector(current) || !is_finite_vector(final_target) ||
        !std::isfinite(config.horizon_m) || config.horizon_m <= 0.0)
    {
        return result;
    }

    Vector3 planning_current = current;
    switch (path_constraint)
    {
        case PathConstraint::FREE_3D:
        case PathConstraint::LINEAR_Z:
            break;
        case PathConstraint::FIXED_Z:
            planning_current.at(2) = final_target.at(2);
            break;
        default:
            return result;
    }

    const Vector3 difference = subtract_vectors(final_target, planning_current);
    if (!is_finite_vector(difference))
    {
        return result;
    }
    const double distance = std::sqrt(squared_norm(difference));
    if (!std::isfinite(distance))
    {
        return result;
    }

    if (distance <= config.horizon_m)
    {
        result.goal = final_target;
        result.is_valid = true;
        result.is_final_goal = true;
        result.failure_reason = LocalGoalFailureReason::NONE;
        return result;
    }

    const double ratio = config.horizon_m / distance;
    if (!std::isfinite(ratio) || ratio <= 0.0 || ratio >= 1.0 ||
        !is_finite_vector(planning_current))
    {
        return result;
    }

    result.goal = add_vectors(
        planning_current, scale_vector(difference, ratio));
    result.is_valid = is_finite_vector(result.goal);
    result.failure_reason = result.is_valid
        ? LocalGoalFailureReason::NONE : LocalGoalFailureReason::INVALID_INPUT;
    return result;
}

namespace detail
{

inline bool local_goal_peer_endpoint_safe(
    const Vector3& position,
    const double absolute_time_s,
    const double candidate_start_time_s,
    const int self_agent_id,
    const std::vector<PeerTrajectory>& peers,
    const double minimum_distance_m,
    double& minimum_peer_distance_m)
{
    if (!is_finite_vector(position) || !is_finite(absolute_time_s) ||
        !is_finite(candidate_start_time_s) || self_agent_id <= 0 ||
        !is_finite(minimum_distance_m) || minimum_distance_m < 0.0)
    {
        return false;
    }

    for (const PeerTrajectory& peer : peers)
    {
        if (peer.agent_id == self_agent_id)
        {
            continue;
        }
        if (!peer.terminal_hold &&
            absolute_time_s > peer.end_time_s + EGO_SWARM_EPSILON_M)
        {
            continue;
        }
        if (!should_check_peer_occupancy(
                peer, absolute_time_s, candidate_start_time_s,
                self_agent_id))
        {
            continue;
        }

        const double peer_time_s = std::max(
            0.0, absolute_time_s - peer.start_time_s);
        const TrajectoryState peer_state = peer.trajectory.evaluate_state(
            peer_time_s);
        if (!peer_state.is_valid)
        {
            return false;
        }
        const double distance = distance_between_centers(
            position, peer_state.position);
        if (!is_finite(distance))
        {
            return false;
        }
        minimum_peer_distance_m = std::min(
            minimum_peer_distance_m, distance);
        if (distance + EGO_SWARM_EPSILON_M < minimum_distance_m)
        {
            return false;
        }
    }
    return true;
}

inline bool local_goal_endpoint_safe(
    const Vector3& position,
    const double arrival_time_s,
    const LocalGoalEnvironment& environment,
    double& minimum_peer_distance_m,
    double& minimum_obstacle_distance_m)
{
    if (environment.obstacle_map == nullptr ||
        !environment.obstacle_map->is_valid() ||
        !environment.obstacle_map->is_fresh(environment.now_s) ||
        !environment.obstacle_map->is_position_safe(
            position, environment.obstacle_clearance_m))
    {
        return false;
    }

    if (environment.obstacle_map->point_count() > 0U)
    {
        const double obstacle_distance =
            environment.obstacle_map->nearest_distance(position);
        if (!is_finite(obstacle_distance))
        {
            return false;
        }
        minimum_obstacle_distance_m = std::min(
            minimum_obstacle_distance_m, obstacle_distance);
    }

    return local_goal_peer_endpoint_safe(
               position, environment.candidate_start_time_s,
               environment.candidate_start_time_s,
               environment.self_agent_id, environment.peers,
               environment.peer_collision_min_distance_m,
               minimum_peer_distance_m) &&
           local_goal_peer_endpoint_safe(
               position, arrival_time_s, environment.candidate_start_time_s,
               environment.self_agent_id, environment.peers,
               environment.peer_collision_min_distance_m,
               minimum_peer_distance_m);
}

inline Vector3 local_goal_planning_current(
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint)
{
    Vector3 planning_current = current;
    if (path_constraint == PathConstraint::FIXED_Z)
    {
        planning_current.at(2) = final_target.at(2);
    }
    return planning_current;
}

inline Vector3 local_goal_lateral_offset(
    const Vector3& direction,
    const double offset_m,
    const double sign)
{
    const double horizontal_norm = std::sqrt(
        direction.at(0) * direction.at(0) +
        direction.at(1) * direction.at(1));
    if (!is_finite(horizontal_norm) || horizontal_norm <= EGO_SWARM_EPSILON_M)
    {
        return Vector3{0.0, 0.0, 0.0};
    }
    return Vector3{
        sign * (-direction.at(1) / horizontal_norm) * offset_m,
        sign * (direction.at(0) / horizontal_norm) * offset_m,
        0.0};
}

inline void append_local_goal_candidate(
    std::vector<Vector3>& candidates,
    const Vector3& candidate)
{
    if (!is_finite_vector(candidate))
    {
        return;
    }
    for (const Vector3& existing : candidates)
    {
        if (distance_between_centers(existing, candidate) <=
            EGO_SWARM_EPSILON_M)
        {
            return;
        }
    }
    candidates.push_back(candidate);
}

}  // namespace detail

/**
 * @brief 產生並篩選具同儕／障礙物終點安全性的 Local Goal
 *
 * nominal Local Goal 若落在同儕 terminal hold 或局部障礙物 clearance 內，
 * 會依序嘗試固定高度左右繞行候選與較短的沿線候選。候選只作為完整
 * trajectory planner 的目標；不取代 trajectory-level safety validation。
 */
inline LocalGoalResult compute_safe_local_goal(
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint,
    const LocalGoalConfig& config,
    const LocalGoalEnvironment& environment)
{
    LocalGoalResult result;
    if (!is_finite_vector(current) || !is_finite_vector(final_target) ||
        !std::isfinite(config.horizon_m) || config.horizon_m <= 0.0 ||
        !std::isfinite(config.minimum_horizon_m) ||
        config.minimum_horizon_m <= 0.0 ||
        !std::isfinite(config.detour_offset_m) ||
        config.detour_offset_m <= 0.0 ||
        !std::isfinite(environment.now_s) ||
        !std::isfinite(environment.candidate_start_time_s) ||
        environment.self_agent_id <= 0 ||
        !std::isfinite(environment.peer_collision_min_distance_m) ||
        environment.peer_collision_min_distance_m < 0.0 ||
        !std::isfinite(environment.obstacle_clearance_m) ||
        environment.obstacle_clearance_m < 0.0 ||
        !std::isfinite(environment.maximum_velocity_mps) ||
        environment.maximum_velocity_mps <= EGO_SWARM_EPSILON_M)
    {
        return result;
    }
    if (!environment.peers_are_fresh)
    {
        result.failure_reason = LocalGoalFailureReason::STALE_PEER_DATA;
        return result;
    }
    if (environment.obstacle_map == nullptr ||
        !environment.obstacle_map->is_valid() ||
        !environment.obstacle_map->is_fresh(environment.now_s))
    {
        result.failure_reason = LocalGoalFailureReason::INVALID_OBSTACLE_MAP;
        return result;
    }

    const Vector3 planning_current = detail::local_goal_planning_current(
        current, final_target, path_constraint);
    const Vector3 difference = subtract_vectors(final_target, planning_current);
    const double distance = std::sqrt(squared_norm(difference));
    if (!is_finite_vector(planning_current) || !is_finite_vector(difference) ||
        !std::isfinite(distance))
    {
        return result;
    }

    if (distance <= EGO_SWARM_EPSILON_M)
    {
        result.candidate_count = 1U;
        double minimum_peer_distance_m = std::numeric_limits<double>::max();
        double minimum_obstacle_distance_m =
            environment.obstacle_map->point_count() == 0U
            ? 0.0 : std::numeric_limits<double>::max();
        if (detail::local_goal_endpoint_safe(
                final_target, environment.candidate_start_time_s,
                environment, minimum_peer_distance_m,
                minimum_obstacle_distance_m))
        {
            result.goal = final_target;
            result.is_valid = true;
            result.is_final_goal = true;
            result.failure_reason = LocalGoalFailureReason::NONE;
            result.minimum_peer_distance_m = minimum_peer_distance_m ==
                std::numeric_limits<double>::max()
                ? 0.0 : minimum_peer_distance_m;
            result.minimum_obstacle_distance_m = minimum_obstacle_distance_m;
        }
        else
        {
            result.failure_reason = LocalGoalFailureReason::NO_SAFE_CANDIDATE;
        }
        return result;
    }

    const double nominal_distance = std::min(config.horizon_m, distance);
    const Vector3 direction = scale_vector(difference, 1.0 / distance);
    const Vector3 nominal = add_vectors(
        planning_current, scale_vector(direction, nominal_distance));
    std::vector<Vector3> candidates;
    detail::append_local_goal_candidate(candidates, nominal);

    const double detour_offset = std::max(
        config.detour_offset_m,
        environment.peer_collision_min_distance_m + 1.5);
    if (path_constraint != PathConstraint::FREE_3D ||
        std::sqrt(direction.at(0) * direction.at(0) +
                  direction.at(1) * direction.at(1)) > EGO_SWARM_EPSILON_M)
    {
        const Vector3 lateral_offset = detail::local_goal_lateral_offset(
            direction, detour_offset, 1.0);
        detail::append_local_goal_candidate(
            candidates, add_vectors(nominal, lateral_offset));
        detail::append_local_goal_candidate(
            candidates, add_vectors(nominal,
                                    detail::local_goal_lateral_offset(
                                        direction, detour_offset, -1.0)));
    }

    const double minimum_distance = std::min(
        config.minimum_horizon_m, nominal_distance);
    const double step_m = std::max(
        1.0, std::min(2.0, nominal_distance * 0.25));
    for (double candidate_distance = nominal_distance - step_m;
         candidate_distance + EGO_SWARM_EPSILON_M >= minimum_distance;
         candidate_distance -= step_m)
    {
        const Vector3 shortened = add_vectors(
            planning_current, scale_vector(direction, candidate_distance));
        detail::append_local_goal_candidate(candidates, shortened);
        if (candidate_distance > EGO_SWARM_EPSILON_M &&
            (path_constraint != PathConstraint::FREE_3D ||
             std::sqrt(direction.at(0) * direction.at(0) +
                       direction.at(1) * direction.at(1)) >
                 EGO_SWARM_EPSILON_M))
        {
            detail::append_local_goal_candidate(
                candidates, add_vectors(
                    shortened, detail::local_goal_lateral_offset(
                        direction, detour_offset, 1.0)));
            detail::append_local_goal_candidate(
                candidates, add_vectors(
                    shortened, detail::local_goal_lateral_offset(
                        direction, detour_offset, -1.0)));
        }
    }

    result.candidate_count = candidates.size();
    double best_progress = -std::numeric_limits<double>::max();
    double best_lateral_distance = std::numeric_limits<double>::max();
    for (const Vector3& candidate : candidates)
    {
        double minimum_peer_distance_m = std::numeric_limits<double>::max();
        double minimum_obstacle_distance_m =
            environment.obstacle_map->point_count() == 0U
            ? 0.0 : std::numeric_limits<double>::max();
        const double candidate_distance = distance_between_centers(
            planning_current, candidate);
        const double arrival_time_s = environment.candidate_start_time_s +
            candidate_distance / environment.maximum_velocity_mps;
        if (!detail::local_goal_endpoint_safe(
                candidate, arrival_time_s, environment,
                minimum_peer_distance_m, minimum_obstacle_distance_m))
        {
            continue;
        }

        const Vector3 candidate_delta = subtract_vectors(
            candidate, planning_current);
        const double progress = candidate_delta.at(0) * direction.at(0) +
            candidate_delta.at(1) * direction.at(1) +
            candidate_delta.at(2) * direction.at(2);
        const double lateral_distance = std::sqrt(std::max(
            0.0, squared_norm(candidate_delta) - progress * progress));
        if (progress > best_progress + EGO_SWARM_EPSILON_M ||
            (std::abs(progress - best_progress) <= EGO_SWARM_EPSILON_M &&
             lateral_distance < best_lateral_distance))
        {
            best_progress = progress;
            best_lateral_distance = lateral_distance;
            result.goal = candidate;
            result.is_final_goal = distance_between_centers(
                candidate, final_target) <= EGO_SWARM_EPSILON_M;
            result.is_valid = true;
            result.failure_reason = LocalGoalFailureReason::NONE;
            result.minimum_peer_distance_m = minimum_peer_distance_m ==
                std::numeric_limits<double>::max()
                ? 0.0 : minimum_peer_distance_m;
            result.minimum_obstacle_distance_m = minimum_obstacle_distance_m;
        }
    }

    if (!result.is_valid)
    {
        result.failure_reason = LocalGoalFailureReason::NO_SAFE_CANDIDATE;
    }
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
