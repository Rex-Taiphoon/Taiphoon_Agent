/**
 * @file TrajectoryPriority.hpp
 * @brief 同儕路線的時間優先權判斷
 * @author Taiphoon Agent Team
 * @date 2026-08-13
 *
 * 多台 Agent 只在路線有衝突時需要互相讓路。此檔案提供單一、純 C++
 * 的優先權規則，讓 planner 與 safety validator 對同一組路線得到相同
 * 結果，不需要新增 ROS2 訊息欄位。
 */

#pragma once

#include <cmath>

#include "ego_swarm/EgoSwarmTypes.hpp"

namespace taiphoon
{
namespace ego_swarm
{

constexpr double TRAJECTORY_PRIORITY_TIME_EPSILON_S = 1.0e-6;

/**
 * @brief 判斷自身路線是否擁有同儕路線的通行優先權
 *
 * 開始時間較早的路線優先；開始時間相同時，使用較小的 Agent ID
 * 作為固定的最後判斷，避免 DDS 訊息抵達順序影響結果。
 *
 * @param self_start_time_s 自身路線開始時間
 * @param self_agent_id 自身 Agent ID
 * @param peer_start_time_s 同儕路線開始時間
 * @param peer_agent_id 同儕 Agent ID
 * @return true 表示自身路線優先
 */
inline bool has_trajectory_priority(
    const double self_start_time_s,
    const int self_agent_id,
    const double peer_start_time_s,
    const int peer_agent_id)
{
    if (!is_finite(self_start_time_s) ||
        !is_finite(peer_start_time_s) ||
        self_agent_id <= 0 || peer_agent_id <= 0)
    {
        return false;
    }
    if (self_start_time_s < peer_start_time_s -
            TRAJECTORY_PRIORITY_TIME_EPSILON_S)
    {
        return true;
    }
    if (std::abs(self_start_time_s - peer_start_time_s) <=
            TRAJECTORY_PRIORITY_TIME_EPSILON_S)
    {
        return self_agent_id < peer_agent_id;
    }
    return false;
}

/**
 * @brief 判斷是否仍須檢查 peer 的空間佔位。
 *
 * terminal hold 與 reservation 尚未開始前的 peer 都是實際佔位，不能
 * 因 priority 直接略過。priority 只可仲裁已進入時間窗的 moving path。
 */
inline bool should_check_peer_occupancy(
    const PeerTrajectory& peer,
    const double absolute_time_s,
    const double self_start_time_s,
    const int self_agent_id)
{
    if (peer.terminal_hold ||
        !is_finite(absolute_time_s) ||
        !is_finite(peer.start_time_s) ||
        absolute_time_s + EGO_SWARM_EPSILON_M < peer.start_time_s)
    {
        return true;
    }
    return !has_trajectory_priority(
        self_start_time_s, self_agent_id,
        peer.start_time_s, peer.agent_id);
}

}  // namespace ego_swarm
}  // namespace taiphoon
