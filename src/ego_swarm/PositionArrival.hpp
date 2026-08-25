/**
 * @file PositionArrival.hpp
 * @brief 使用 PX4 local NED 與 planner-local NED 判斷目標是否已到達
 * @author Taiphoon Agent Team
 * @date 2026-08-12
 */

#pragma once

#include <cmath>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 雙座標來源的到達判斷結果
 */
struct PositionArrivalResult
{
    bool local_valid{false};
    bool planner_valid{false};
    bool reached{false};
    double local_error_m{0.0};
    double planner_error_m{0.0};
};

/**
 * @brief 同時以 PX4 local NED 與 planner-local NED 判斷是否到達
 *
 * 兩個位置來源都必須有效且落在各自容許誤差內，才回傳 reached。這樣
 * GPS 短暫量化或 local odometry 漂移時，不會單一來源提早結束任務。
 *
 * @param current_local_ned 目前 PX4 local NED 位置
 * @param target_local_ned 轉換至 PX4 local NED 的目標
 * @param current_planner_ned 由 WGS84 轉換出的本機 planner NED 位置
 * @param target_planner_ned 本機 planner NED 目標
 * @param local_tolerance_m local NED 到達容許誤差
 * @param planner_tolerance_m planner NED 到達容許誤差
 * @return PX4 local／planner-local 誤差與到達狀態
 */
inline PositionArrivalResult evaluate_position_arrival(
    const Vector3& current_local_ned,
    const Vector3& target_local_ned,
    const Vector3& current_planner_ned,
    const Vector3& target_planner_ned,
    const double local_tolerance_m,
    const double planner_tolerance_m)
{
    PositionArrivalResult result;
    if (!is_finite_vector(current_local_ned) ||
        !is_finite_vector(target_local_ned) ||
        !is_finite(local_tolerance_m) || local_tolerance_m < 0.0)
    {
        return result;
    }

    if (!is_finite_vector(current_planner_ned) ||
        !is_finite_vector(target_planner_ned) ||
        !is_finite(planner_tolerance_m) || planner_tolerance_m < 0.0)
    {
        result.local_error_m = distance_between_centers(
            current_local_ned, target_local_ned);
        result.local_valid = is_finite(result.local_error_m);
        return result;
    }

    result.local_error_m = distance_between_centers(
        current_local_ned, target_local_ned);
    result.planner_error_m = distance_between_centers(
        current_planner_ned, target_planner_ned);
    result.local_valid = is_finite(result.local_error_m);
    result.planner_valid = is_finite(result.planner_error_m);
    result.reached = result.local_valid && result.planner_valid &&
        result.local_error_m <= local_tolerance_m &&
        result.planner_error_m <= planner_tolerance_m;
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
