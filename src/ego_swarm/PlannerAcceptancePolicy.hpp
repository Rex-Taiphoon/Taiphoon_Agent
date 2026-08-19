/**
 * @file PlannerAcceptancePolicy.hpp
 * @brief planner candidate acceptance chain 的純 C++ policy seam
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只判斷 candidate 是否可以進入 active plan，
 * 不依賴 ROS2，也不修改 trajectory geometry、
 * reservation 或 safety validator 的實作。
 */

#pragma once

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief candidate 被拒絕或接受的原因
 */
enum class CandidateAcceptanceReason
{
    ACCEPTED = 0,
    PLANNER_FAILED,
    PLANNER_OUTPUT_NONFINITE,
    RESERVATION_REJECTED,
    SAFETY_REJECTED
};

/**
 * @brief acceptance chain 的 immutable input
 *
 * `planner_succeeded` 應由呼叫端確認 planner result 與 trajectory
 * 有效；
 * `planner_output_finite` 則對應
 * PlannerResult::has_finite_output()。
 */
struct CandidateAcceptanceInput
{
    bool planner_succeeded{false};
    bool planner_output_finite{false};
    bool reservation_required{false};
    bool reservation_accepted{true};
    bool safety_passed{false};
};

/**
 * @brief acceptance chain 的判斷結果
 */
struct CandidateAcceptanceDecision
{
    bool should_accept{false};
    CandidateAcceptanceReason reason{
        CandidateAcceptanceReason::PLANNER_FAILED};
};

/**
 * @brief 確認 candidate 通過完整 acceptance chain
 *
 * 唯一接受順序為 planner success、finite output、reservation
 * （若需要）
 * 與 continuous safety validation；任一階段失敗都不得更新 active
 * state。
 *
 * @param input 各階段結果
 * @return candidate 是否可被接受及其原因
 */
inline CandidateAcceptanceDecision evaluate_candidate_acceptance(
    const CandidateAcceptanceInput& input)
{
    if (!input.planner_succeeded)
    {
        return CandidateAcceptanceDecision{
            false, CandidateAcceptanceReason::PLANNER_FAILED};
    }
    if (!input.planner_output_finite)
    {
        return CandidateAcceptanceDecision{
            false, CandidateAcceptanceReason::PLANNER_OUTPUT_NONFINITE};
    }
    if (input.reservation_required && !input.reservation_accepted)
    {
        return CandidateAcceptanceDecision{
            false, CandidateAcceptanceReason::RESERVATION_REJECTED};
    }
    if (!input.safety_passed)
    {
        return CandidateAcceptanceDecision{
            false, CandidateAcceptanceReason::SAFETY_REJECTED};
    }
    return CandidateAcceptanceDecision{
        true, CandidateAcceptanceReason::ACCEPTED};
}

}  // namespace ego_swarm
}  // namespace taiphoon
