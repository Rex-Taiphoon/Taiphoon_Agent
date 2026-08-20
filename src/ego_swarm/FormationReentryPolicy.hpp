/**
 * @file FormationReentryPolicy.hpp
 * @brief 既有 FormationEpoch 重入的純 C++ 狀態策略
 */

#pragma once

namespace taiphoon
{
namespace ego_swarm
{

/** @brief Planner 中與單機目標／編隊重入相關的旗標快照。 */
struct FormationReentryState
{
    bool has_formation{false};
    bool has_goal{false};
    bool has_position_target{false};
    bool preserve_altitude{false};
    bool formation_command_pending{false};
    bool formation_membership_change_pending{false};
    bool formation_initial_plan_pending{false};
};

/**
 * @brief 重新啟用相同 config／member snapshot 的既有 FormationEpoch。
 *
 * 只負責可測試的旗標轉移；Epoch identity、center 與 assignment 由呼叫端
 * 保留，active trajectory 的 invalidate 也由 ROS2 Node 執行。
 */
inline bool reactivate_existing_formation(
    FormationReentryState& state,
    const bool accepted_epoch_matches,
    const bool candidate_epoch_matches)
{
    if (state.has_formation ||
        (!accepted_epoch_matches && !candidate_epoch_matches))
    {
        return false;
    }

    state.has_goal = false;
    state.has_position_target = false;
    state.preserve_altitude = false;
    state.has_formation = true;
    state.formation_command_pending = false;
    state.formation_membership_change_pending = false;
    state.formation_initial_plan_pending = true;
    return true;
}

}  // namespace ego_swarm
}  // namespace taiphoon
