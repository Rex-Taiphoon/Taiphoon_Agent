/**
 * @file PlanningTargetSelector.hpp
 * @brief 共用 planning layer 的 Planning Target 選擇 helper
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只負責將 feature gate 與 Local Goal Selector 接在一起，不依賴
 * ROS2，也不管理 Formation、reservation 或 trajectory lifecycle。
 */

#pragma once

#include "ego_swarm/LocalGoalSelector.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 依 Local Goal feature gate 選擇本輪 Planning Target
 *
 * 功能關閉時直接回傳 Final Target，確保舊架構的 planning input 不受
 * segmented planning 影響；功能開啟時才委派給 Local Goal Selector。
 *
 * @param enable_local_goal_planning 是否啟用 segmented Local Goal planning
 * @param current 目前位置，使用本 Agent 的 planner-local NED，單位為公尺
 * @param final_target 任務真正終點，使用本 Agent 的 planner-local NED，單位為公尺
 * @param path_constraint 路徑高度約束
 * @param config Local Goal 計算設定
 * @return 本輪 Planning Target；輸入或計算非法時回傳無效結果
 */
inline LocalGoalResult select_planning_target(
    const bool enable_local_goal_planning,
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint,
    const LocalGoalConfig& config)
{
    if (!enable_local_goal_planning)
    {
        LocalGoalResult result;
        if (!is_finite_vector(final_target))
        {
            return result;
        }
        result.goal = final_target;
        result.is_valid = true;
        result.is_final_goal = true;
        return result;
    }

    return compute_local_goal(current, final_target, path_constraint, config);
}

/**
 * @brief 依同儕與局部障礙物 environment 選擇安全 Planning Target
 *
 * Local Goal 功能關閉時保留既有 Final Target 語意；功能開啟時，候選
 * Local Goal 會先通過低成本終點安全檢查，再交給完整 trajectory planner。
 */
inline LocalGoalResult select_planning_target(
    const bool enable_local_goal_planning,
    const Vector3& current,
    const Vector3& final_target,
    const PathConstraint path_constraint,
    const LocalGoalConfig& config,
    const LocalGoalEnvironment& environment)
{
    if (!enable_local_goal_planning)
    {
        LocalGoalResult result;
        if (!is_finite_vector(final_target))
        {
            return result;
        }
        result.goal = final_target;
        result.is_valid = true;
        result.is_final_goal = true;
        result.failure_reason = LocalGoalFailureReason::NONE;
        return result;
    }

    return compute_safe_local_goal(
        current, final_target, path_constraint, config, environment);
}

}  // namespace ego_swarm
}  // namespace taiphoon
