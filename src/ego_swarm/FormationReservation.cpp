/**
 * @file FormationReservation.cpp
 * @brief 編隊 trajectory reservation 實作
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#include "ego_swarm/FormationReservation.hpp"

#include <algorithm>
#include <cmath>

#include "ego_swarm/TrajectoryPriority.hpp"

namespace taiphoon
{
namespace ego_swarm
{

FormationReservationDecision FormationReservationPlanner::reserve(
    const BsplineTrajectory& candidate,
    const PlanningSnapshot& snapshot,
    const double minimum_distance_m,
    const double lead_time_s,
    const double step_s,
    const double horizon_s)
{
    FormationReservationDecision result;
    if (!candidate.is_valid() || !snapshot.is_valid() ||
        !is_finite(minimum_distance_m) || minimum_distance_m < 0.0 ||
        !is_finite(lead_time_s) || lead_time_s < 0.0 ||
        !is_finite(step_s) || step_s <= EGO_SWARM_EPSILON_M ||
        !is_finite(horizon_s) || horizon_s < 0.0)
    {
        return result;
    }

    const double first_start = std::max(
        snapshot.trajectory_start_time_s,
        snapshot.planning_time_s + lead_time_s);
    if (!is_finite(first_start))
    {
        return result;
    }
    const double horizon_remaining_s = horizon_s -
        (first_start - snapshot.planning_time_s);
    if (horizon_remaining_s < -EGO_SWARM_EPSILON_M)
    {
        return result;
    }
    const std::size_t attempt_count = static_cast<std::size_t>(std::floor(
        std::max(0.0, horizon_remaining_s) / step_s)) + 1U;
    for (std::size_t attempt = 0U; attempt < attempt_count; ++attempt)
    {
        const double start_time = first_start +
            static_cast<double>(attempt) * step_s;
        if (!is_finite(start_time))
        {
            return result;
        }
        std::size_t blockers = 0U;
        if (is_window_safe(
                candidate, start_time, snapshot, minimum_distance_m,
                blockers))
        {
            result.is_reserved = true;
            result.start_time_s = start_time;
            result.blocking_agent_count = blockers;
            return result;
        }
        result.blocking_agent_count = blockers;
    }
    return result;
}

bool FormationReservationPlanner::is_window_safe(
    const BsplineTrajectory& candidate,
    const double start_time_s,
    const PlanningSnapshot& snapshot,
    const double minimum_distance_m,
    std::size_t& blocking_agent_count)
{
    blocking_agent_count = 0U;
    const std::size_t sample_count = std::min(
        static_cast<std::size_t>(std::ceil(
            candidate.duration_s() / 0.1)), EGO_SWARM_MAX_SAMPLE_COUNT);
    for (std::size_t index = 0U; index <= sample_count; ++index)
    {
        const double relative_time = std::min(
            candidate.duration_s(), static_cast<double>(index) * 0.1);
        const TrajectoryState self_sample = candidate.evaluate_state(
            relative_time);
        if (!self_sample.is_valid)
        {
            return false;
        }
        const double absolute_time = start_time_s + relative_time;
        if (!is_finite(absolute_time))
        {
            return false;
        }
        for (const PeerTrajectory& peer : snapshot.peers)
        {
            if (peer.agent_id == snapshot.self_agent_id)
            {
                continue;
            }
            if (!peer.terminal_hold &&
                absolute_time > peer.end_time_s + EGO_SWARM_EPSILON_M)
            {
                continue;
            }
            const bool peer_started = absolute_time + EGO_SWARM_EPSILON_M >=
                peer.start_time_s;
            if (peer_started && has_trajectory_priority(
                    start_time_s, snapshot.self_agent_id,
                    peer.start_time_s, peer.agent_id))
            {
                continue;
            }
            const double peer_time = std::max(
                0.0, absolute_time - peer.start_time_s);
            const TrajectoryState peer_sample = peer.trajectory.evaluate_state(
                peer_time);
            if (!peer_sample.is_valid)
            {
                return false;
            }
            const double distance = distance_between_centers(
                self_sample.position, peer_sample.position);
            if (!is_finite(distance))
            {
                return false;
            }
            if (distance + EGO_SWARM_EPSILON_M < minimum_distance_m)
            {
                ++blocking_agent_count;
                break;
            }
        }
        if (blocking_agent_count > 0U)
        {
            return false;
        }
    }
    return true;
}

}  // namespace ego_swarm
}  // namespace taiphoon
