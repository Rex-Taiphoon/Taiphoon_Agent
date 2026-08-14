/**
 * @file FormationReservation.hpp
 * @brief 編隊 trajectory 的空間／時間 reservation 純 C++ 模組
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

struct FormationReservationDecision
{
    bool is_reserved{false};
    double start_time_s{0.0};
    std::size_t blocking_agent_count{0U};
};

/**
 * @brief 在已發布／已保留 peer path 中搜尋最早可執行時間。
 *
 * 這個 module 不改變 control points，只決定 future start time。無衝突的
 * Agent 可以共享同一 start time；衝突時沿用 start time／Agent ID priority。
 */
class FormationReservationPlanner
{
public:
    /**
     * @brief 為 candidate 搜尋安全的 future start time。
     * @param candidate 候選 B-spline
     * @param snapshot immutable planning snapshot
     * @param minimum_distance_m peer 中心最小距離
     * @param lead_time_s reservation 提前時間
     * @param step_s 延後搜尋步進
     * @param horizon_s 最遠搜尋範圍
     * @return reservation decision
     */
    static FormationReservationDecision reserve(
        const BsplineTrajectory& candidate,
        const PlanningSnapshot& snapshot,
        double minimum_distance_m,
        double lead_time_s,
        double step_s,
        double horizon_s);

private:
    static bool is_window_safe(
        const BsplineTrajectory& candidate,
        double start_time_s,
        const PlanningSnapshot& snapshot,
        double minimum_distance_m,
        std::size_t& blocking_agent_count);
};

}  // namespace ego_swarm
}  // namespace taiphoon
