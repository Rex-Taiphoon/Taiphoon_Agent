/**
 * @file LivePeerDistancePolicy.hpp
 * @brief 即時同儕距離的空集合與安全判定語意
 * @author Taiphoon Agent Team
 * @date 2026-08-19
 *
 * 本檔案只提供 pure C++ policy，讓 ROS2 node 區分「沒有 peer」與
 * 「peer 資料無效」，避免空集合被誤當成零距離碰撞。
 */

#pragma once

#include <cmath>

namespace taiphoon
{
namespace ego_swarm
{

/**
 * @brief 一次即時 peer 距離查詢的結果
 *
 * `has_peer == false` 表示目前沒有需要比較的 peer，
 * 不代表距離為零。
 * `data_valid == false` 代表資料無效，呼叫端必須維持 fail-safe。
 */
struct LivePeerDistanceResult
{
    bool has_peer{false};
    bool data_valid{true};
    double minimum_distance_m{0.0};
};

/**
 * @brief 判定即時 peer 是否已進入碰撞距離
 *
 * 空 peer 集合不會觸發碰撞；有 peer 但資料無效時則 fail-safe
 * 地視為需要安全處置。
 *
 * @param result 即時 peer 距離查詢結果
 * @param collision_distance_m 碰撞距離門檻
 * @return true 表示應觸發碰撞安全處置
 */
inline bool live_peer_collision_detected(
    const LivePeerDistanceResult& result,
    const double collision_distance_m)
{
    if (!result.data_valid)
    {
        return true;
    }
    if (!result.has_peer)
    {
        return false;
    }
    return !std::isfinite(result.minimum_distance_m) ||
           !std::isfinite(collision_distance_m) ||
           result.minimum_distance_m < collision_distance_m;
}

}  // namespace ego_swarm
}  // namespace taiphoon
