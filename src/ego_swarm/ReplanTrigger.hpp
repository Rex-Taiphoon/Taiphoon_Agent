/**
 * @file ReplanTrigger.hpp
 * @brief Local Goal segment 重規劃條件的純 C++ policy seam
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只判斷重規劃 trigger，不管理 ROS2、trajectory geometry、
 * reservation 或 active plan 的生命週期。
 */

#pragma once

#include <cmath>

namespace taiphoon
{
namespace ego_swarm
{

constexpr double DEFAULT_REPLAN_TRIGGER_RATIO = 0.70;
constexpr double DEFAULT_REPLAN_TRIGGER_INTERVAL_S = 0.5;

/**
 * @brief 重規劃的觸發來源
 */
enum class ReplanTriggerReason
{
    KEEP_ACTIVE = 0,
    INITIAL_PLAN,
    FINAL_TARGET_CHANGED,
    ACTIVE_TRAJECTORY_UNSAFE,
    SEGMENT_ENDED,
    SEGMENT_PROGRESS,
    INVALID_TIMING
};

/**
 * @brief 單一 planner tick 用於判斷 replan 的 immutable input
 */
struct ReplanTriggerInput
{
    bool local_goal_planning_enabled{false};
    bool has_active_plan{false};
    bool final_target_unchanged{false};
    bool active_trajectory_safe{true};
    double elapsed_s{0.0};
    double duration_s{0.0};
    double now_s{0.0};
    bool has_last_replan_time{false};
    double last_replan_time_s{0.0};
    double replan_ratio{DEFAULT_REPLAN_TRIGGER_RATIO};
    double minimum_replan_interval_s{
        DEFAULT_REPLAN_TRIGGER_INTERVAL_S};
};

/**
 * @brief 重規劃判斷結果
 */
struct ReplanTriggerDecision
{
    bool should_replan{false};
    ReplanTriggerReason reason{ReplanTriggerReason::KEEP_ACTIVE};
};

/**
 * @brief 依照 Step 7 規格判斷目前 active plan 是否需要重規劃
 *
 * Final Target 改變、active trajectory 不安全與 segment 結束不受
 * minimum interval 限制。只有正常的 progress-based replan
 * 受該限制；只有 Local Goal 功能開啟時才會由 progress 觸發。
 *
 * @param input 單一 planner tick 的判斷輸入
 * @return 是否需要重規劃及其原因
 */
inline ReplanTriggerDecision evaluate_replan_trigger(
    const ReplanTriggerInput& input)
{
    if (!input.has_active_plan)
    {
        return ReplanTriggerDecision{
            true, ReplanTriggerReason::INITIAL_PLAN};
    }
    if (!input.final_target_unchanged)
    {
        return ReplanTriggerDecision{
            true, ReplanTriggerReason::FINAL_TARGET_CHANGED};
    }
    if (!input.active_trajectory_safe)
    {
        return ReplanTriggerDecision{
            true, ReplanTriggerReason::ACTIVE_TRAJECTORY_UNSAFE};
    }
    if (!std::isfinite(input.elapsed_s) ||
        !std::isfinite(input.duration_s) || input.duration_s <= 0.0 ||
        !std::isfinite(input.now_s) ||
        !std::isfinite(input.replan_ratio) ||
        input.replan_ratio <= 0.0 || input.replan_ratio > 1.0 ||
        !std::isfinite(input.minimum_replan_interval_s) ||
        input.minimum_replan_interval_s < 0.0 ||
        (input.has_last_replan_time &&
         !std::isfinite(input.last_replan_time_s)))
    {
        return ReplanTriggerDecision{
            true, ReplanTriggerReason::INVALID_TIMING};
    }
    if (input.elapsed_s < 0.0)
    {
        return ReplanTriggerDecision{
            false, ReplanTriggerReason::KEEP_ACTIVE};
    }
    if (input.elapsed_s >= input.duration_s)
    {
        return ReplanTriggerDecision{
            true, ReplanTriggerReason::SEGMENT_ENDED};
    }
    if (!input.local_goal_planning_enabled)
    {
        return ReplanTriggerDecision{
            false, ReplanTriggerReason::KEEP_ACTIVE};
    }

    const double progress = input.elapsed_s / input.duration_s;
    if (!std::isfinite(progress) || progress < input.replan_ratio)
    {
        return ReplanTriggerDecision{
            false, ReplanTriggerReason::KEEP_ACTIVE};
    }

    if (input.has_last_replan_time &&
        input.now_s - input.last_replan_time_s <
            input.minimum_replan_interval_s)
    {
        return ReplanTriggerDecision{
            false, ReplanTriggerReason::KEEP_ACTIVE};
    }

    return ReplanTriggerDecision{
        true, ReplanTriggerReason::SEGMENT_PROGRESS};
}

}  // namespace ego_swarm
}  // namespace taiphoon
