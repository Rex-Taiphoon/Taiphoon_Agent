/**
 * @file FormationCenterPolicy.hpp
 * @brief 初次 FormationEpoch 中心的純 C++ 選擇策略
 * @author Taiphoon Agent Team
 * @date 2026-10-15
 */

#pragma once

#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 編隊中心高度參考：planner-local NED 的 Home（z = 0 m）。
constexpr double FORMATION_CENTER_HOME_DOWN_M = 0.0;

/**
 * @brief 以成員目標平均建立初次編隊中心。
 *
 * 編隊高度參數（vertical_offset_m）以 Home 為基準；若中心高度直接沿用
 * 飛行中目標的平均高度，槽位會再疊加一次編隊高度。因此只取目標的水平
 * 平均，高度固定為 Home，與「成員目前位置」路徑的語意一致。
 *
 * @param member_targets 所有成員在 planner-local NED 的目標位置
 * @param center 輸出中心；失敗時為零向量
 * @return true 表示所有目標有限且至少有一個成員
 */
inline bool make_formation_center_from_targets(
    const std::vector<Vector3>& member_targets,
    Vector3& center)
{
    center = Vector3{0.0, 0.0, 0.0};
    if (member_targets.empty())
    {
        return false;
    }

    double north_sum_m = 0.0;
    double east_sum_m = 0.0;
    for (const Vector3& target : member_targets)
    {
        if (!is_finite_vector(target))
        {
            return false;
        }
        north_sum_m += target.at(0);
        east_sum_m += target.at(1);
    }

    const double count = static_cast<double>(member_targets.size());
    const Vector3 candidate{
        north_sum_m / count, east_sum_m / count, FORMATION_CENTER_HOME_DOWN_M};
    if (!is_finite_vector(candidate))
    {
        return false;
    }
    center = candidate;
    return true;
}

}  // namespace ego_swarm
}  // namespace taiphoon
