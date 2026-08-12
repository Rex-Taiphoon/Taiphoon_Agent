/**
 * @file PositionArrival.hpp
 * @brief 使用 local NED 與共同 global NED 判斷目標是否已到達
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
    bool global_valid{false};
    bool reached{false};
    double local_error_m{0.0};
    double global_error_m{0.0};
};

/**
 * @brief 同時以 PX4 local NED 與共同 global NED 判斷是否到達
 *
 * 兩個位置來源都必須有效且落在各自容許誤差內，才回傳 reached。這樣
 * GPS 短暫量化或 local odometry 漂移時，不會單一來源提早結束任務。
 *
 * @param current_local_ned 目前 PX4 local NED 位置
 * @param target_local_ned 轉換至 PX4 local NED 的目標
 * @param current_global_ned 由 GPS 轉換出的共同 NED 位置
 * @param target_global_ned 共同 NED 目標
 * @param local_tolerance_m local NED 到達容許誤差
 * @param global_tolerance_m global NED 到達容許誤差
 * @return local/global 誤差與到達狀態
 */
inline PositionArrivalResult evaluate_position_arrival(
    const Vector3& current_local_ned,
    const Vector3& target_local_ned,
    const Vector3& current_global_ned,
    const Vector3& target_global_ned,
    const double local_tolerance_m,
    const double global_tolerance_m)
{
    PositionArrivalResult result;
    if (!is_finite_vector(current_local_ned) ||
        !is_finite_vector(target_local_ned) ||
        !is_finite(local_tolerance_m) || local_tolerance_m < 0.0)
    {
        return result;
    }

    if (!is_finite_vector(current_global_ned) ||
        !is_finite_vector(target_global_ned) ||
        !is_finite(global_tolerance_m) || global_tolerance_m < 0.0)
    {
        result.local_error_m = distance_between_centers(
            current_local_ned, target_local_ned);
        result.local_valid = is_finite(result.local_error_m);
        return result;
    }

    result.local_error_m = distance_between_centers(
        current_local_ned, target_local_ned);
    result.global_error_m = distance_between_centers(
        current_global_ned, target_global_ned);
    result.local_valid = is_finite(result.local_error_m);
    result.global_valid = is_finite(result.global_error_m);
    result.reached = result.local_valid && result.global_valid &&
        result.local_error_m <= local_tolerance_m &&
        result.global_error_m <= global_tolerance_m;
    return result;
}

}  // namespace ego_swarm
}  // namespace taiphoon
