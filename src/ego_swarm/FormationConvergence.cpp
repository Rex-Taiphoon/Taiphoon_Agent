/**
 * @file FormationConvergence.cpp
 * @brief 編隊收斂條件 evaluator 實作
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#include "ego_swarm/FormationConvergence.hpp"

#include <cmath>
#include <set>

namespace taiphoon
{
namespace ego_swarm
{

FormationConvergenceResult FormationConvergenceEvaluator::evaluate(
    const std::vector<FormationConvergenceObservation>& observations,
    const double position_tolerance_m)
{
    FormationConvergenceResult result;
    result.member_count = observations.size();
    if (observations.empty() || !is_finite(position_tolerance_m) ||
        position_tolerance_m < 0.0)
    {
        return result;
    }

    std::set<int> ids;
    for (const FormationConvergenceObservation& observation : observations)
    {
        if (observation.agent_id <= 0 ||
            !is_finite_vector(observation.position) ||
            !is_finite_vector(observation.target) ||
            !is_finite_vector(observation.velocity) ||
            !ids.insert(observation.agent_id).second)
        {
            return result;
        }
        const double distance = distance_between_centers(
            observation.position, observation.target);
        if (observation.has_fresh_state &&
            observation.has_fresh_occupancy &&
            observation.has_same_epoch &&
            is_finite(distance) && distance <= position_tolerance_m +
                EGO_SWARM_EPSILON_M)
        {
            ++result.ready_count;
        }
    }
    result.is_converged = result.ready_count == result.member_count;
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
