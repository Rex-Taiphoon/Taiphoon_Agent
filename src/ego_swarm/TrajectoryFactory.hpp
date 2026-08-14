/**
 * @file TrajectoryFactory.hpp
 * @brief planner-independent trajectory construction helpers
 */

#pragma once

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 建立鎖定指定座標的 finite stationary hold trajectory。
 *
 * 這個 factory 不屬於任何 planner backend，讓 Node 在 optimized-only
 * runtime 中建立 HOLD 時不需要組合 deterministic planner。
 */
inline PlannerResult make_exact_hold_result(
    const Vector3& hold_position,
    const PlannerConfig& config)
{
    PlannerResult result;
    if (!is_finite_vector(hold_position) ||
        !is_finite(config.knot_interval_s) ||
        config.knot_interval_s <= EGO_SWARM_EPSILON_M ||
        config.control_point_count < BSPLINE_DEGREE + 1U ||
        config.control_point_count > EGO_SWARM_MAX_SAMPLE_COUNT)
    {
        return result;
    }

    result.trajectory = BsplineTrajectory(
        std::vector<Vector3>(config.control_point_count, hold_position),
        config.knot_interval_s);
    result.is_valid = result.trajectory.is_valid();
    result.is_hold = true;
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
