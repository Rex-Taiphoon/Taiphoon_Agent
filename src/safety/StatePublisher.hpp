/**
 * @file StatePublisher.hpp
 * @brief Agent 狀態發布輔助類別 — 將 PX4 odometry 轉譯為 AgentState/AgentAttitude
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 純邏輯類別，負責資料轉譯，不持有 ROS2 publisher。
 * 由 BasicControllerNode 在控制迴圈中呼叫。
 */

#pragma once

#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "controllers/BasicController.hpp"

namespace taiphoon
{
namespace safety
{

/**
 * @brief Agent 狀態發布輔助類別
 *
 * 提供靜態方法，將 PX4 VehicleOdometry 與 FlightState 轉譯為
 * AgentState 與 AgentAttitude 訊息。
 * 不持有任何狀態，所有方法均為靜態。
 */
class StatePublisher
{
public:
    /**
     * @brief 從 PX4 里程計與飛行狀態填充 AgentState 訊息
     *
     * @param[out] state         Agent 狀態訊息
     * @param[in]  odom          PX4 里程計訊息
     * @param[in]  flight_state  目前飛行狀態枚舉
     * @param[in]  online        Agent 是否在線
     */
    static void fill_agent_state(
        taiphoon_agent::msg::AgentState& state,
        const px4_msgs::msg::VehicleOdometry& odom,
        controllers::FlightState flight_state,
        bool online)
    {
        // 時間戳記
        state.stamp.sec = static_cast<int32_t>(odom.timestamp / 1000000ULL);
        state.stamp.nanosec = static_cast<uint32_t>(
            (odom.timestamp % 1000000ULL) * 1000ULL);

        // NED 位置
        state.position_ned[0] = odom.position[0];
        state.position_ned[1] = odom.position[1];
        state.position_ned[2] = odom.position[2];

        // 飛行模式
        state.flight_mode = static_cast<uint8_t>(flight_state);

        // 電量 (PX4 odometry 不含電量，預設 1.0)
        state.battery_remaining = 1.0F;

        // 在線旗標
        state.online = online;
    }

    /**
     * @brief 從 PX4 里程計填充 AgentAttitude 訊息
     *
     * @param[out] attitude Agent 姿態訊息
     * @param[in]  odom     PX4 里程計訊息
     */
    static void fill_agent_attitude(
        taiphoon_agent::msg::AgentAttitude& attitude,
        const px4_msgs::msg::VehicleOdometry& odom)
    {
        // 時間戳記
        attitude.stamp.sec = static_cast<int32_t>(odom.timestamp / 1000000ULL);
        attitude.stamp.nanosec = static_cast<uint32_t>(
            (odom.timestamp % 1000000ULL) * 1000ULL);

        // 四元數姿態 (w, x, y, z)
        attitude.q[0] = odom.q[0];
        attitude.q[1] = odom.q[1];
        attitude.q[2] = odom.q[2];
        attitude.q[3] = odom.q[3];
    }
};

}  // namespace safety
}  // namespace taiphoon
