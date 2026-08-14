/**
 * @file ITrajectoryPlanner.hpp
 * @brief EGO-Swarm trajectory planner backend 的替換 seam
 * @author Taiphoon Agent Team
 * @date 2026-08-14
 */

#pragma once

#include "ego_swarm/PlannerTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 產生單一 planning snapshot 對應 trajectory 的深層介面。
 *
 * Backend 不得直接讀取 ROS2 callback state；所有資料均由 immutable
 * PlanningSnapshot 提供，輸出仍須交給共用 ContinuousSafetyValidator。
 */
class ITrajectoryPlanner
{
public:
    virtual ~ITrajectoryPlanner() = default;

    /**
     * @brief 依 snapshot 產生一條候選 trajectory。
     * @param snapshot 單一 planner cycle 的唯讀資料
     * @return planner 結果；失敗不得宣稱 trajectory 可安全執行
     */
    virtual PlannerResult plan(
        const PlanningSnapshot& snapshot) const = 0;
};

}  // namespace ego_swarm
}  // namespace taiphoon
