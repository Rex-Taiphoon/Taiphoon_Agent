/**
 * @file TrajectoryReservationPolicy.hpp
 * @brief moving trajectory reservation 的純 C++ policy seam
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只決定何時套用既有 reservation，以及該次仲裁使用的 peer
 * separation；不實作 reservation 搜尋，也不修改 trajectory geometry。
 */

#pragma once

#include <algorithm>

#include "ego_swarm/PlannerFeatureFlags.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 判斷 moving trajectory 是否需要進入 reservation pipeline
 *
 * Formation 維持既有 reservation 行為；generic reservation 旗標開啟後，
 * 一般 MOVE 等 moving trajectory 也進入相同 pipeline。
 *
 * @param has_formation 目前是否有有效 Formation task
 * @param feature_flags planner feature flags
 * @return true 表示必須執行 reservation
 */
inline bool should_reserve_moving_trajectory(
    const bool has_formation,
    const PlannerFeatureFlags& feature_flags)
{
    return has_formation ||
           feature_flags.enable_generic_trajectory_reservation;
}

/**
 * @brief 取得 reservation 使用的 peer separation
 *
 * MOVE 只使用一般 peer collision distance；Formation 則保留既有的
 * `max(formation_min_distance_m, peer_collision_min_distance_m)` 規則。
 * 呼叫端必須先完成兩個距離參數的 finite／range validation。
 *
 * @param has_formation 目前是否有有效 Formation task
 * @param formation_min_distance_m Formation peer separation
 * @param peer_collision_min_distance_m 一般 peer collision separation
 * @return 本次 reservation 的中心距離門檻
 */
inline double required_peer_separation_m(
    const bool has_formation,
    const double formation_min_distance_m,
    const double peer_collision_min_distance_m)
{
    if (!has_formation)
    {
        return peer_collision_min_distance_m;
    }
    return std::max(
        formation_min_distance_m,
        peer_collision_min_distance_m);
}

}  // namespace ego_swarm
}  // namespace taiphoon
