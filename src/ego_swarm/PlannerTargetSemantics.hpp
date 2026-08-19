/**
 * @file PlannerTargetSemantics.hpp
 * @brief Planner Final Target 與 active plan 語意的純 C++ helper
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只定義 target 語意之間的判斷，不依賴 ROS2，也不管理
 * Formation、reservation 或 trajectory lifecycle。
 */

#pragma once

#include <cmath>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

constexpr double ACTIVE_PLAN_FINAL_TARGET_EPSILON_M = 1.0e-3;

/**
 * @brief 判斷 active trajectory 是否仍屬於目前的 Final Target
 *
 * active trajectory 可能只對應 Local Goal／Planning Target 的中間 segment，
 * 因此不能用 active_plan_target 直接判定任務目標是否改變。只有 active
 * trajectory 所記錄的 Final Target 與目前任務 Final Target 相同，才能
 * 保留既有 active plan。
 *
 * @param has_active_plan 是否存在可繼續使用的 active plan
 * @param active_plan_final_target active trajectory 所屬任務終點
 * @param final_target 目前任務真正終點
 * @param epsilon 允許的 Final Target 距離誤差，單位為公尺
 * @return true 表示 active plan 仍對應目前任務
 */
inline bool active_plan_matches_final_target(
    const bool has_active_plan,
    const Vector3& active_plan_final_target,
    const Vector3& final_target,
    const double epsilon = ACTIVE_PLAN_FINAL_TARGET_EPSILON_M)
{
    return has_active_plan &&
        is_finite_vector(active_plan_final_target) &&
        is_finite_vector(final_target) &&
        std::isfinite(epsilon) && epsilon >= 0.0 &&
        distance_between_centers(
            active_plan_final_target, final_target) <= epsilon;
}

}  // namespace ego_swarm
}  // namespace taiphoon
