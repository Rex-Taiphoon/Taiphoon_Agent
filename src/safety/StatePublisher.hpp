/**
 * @file StatePublisher.hpp
 * @brief Agent 狀態發布輔助類別 — 將 PX4 odometry / GPS / Battery 轉譯為 AgentState/AgentAttitude
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 純邏輯類別，負責資料轉譯，不持有 ROS2 publisher。
 * 由 BasicControllerNode 在控制迴圈中呼叫。
 */

#pragma once

#include <cmath>
#include <limits>

#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"
#include "controllers/FlightStateResolver.hpp"

namespace taiphoon
{
namespace safety
{

/**
 * @brief Agent 狀態發布輔助類別
 *
 * 提供靜態方法，將 PX4 VehicleOdometry、VehicleGlobalPosition 與 BatteryStatus
 * 轉譯為 AgentState 與 AgentAttitude 訊息。
 * 不持有任何狀態，所有方法均為靜態。
 */
class StatePublisher
{
public:
    /**
     * @brief 將狀態發布頻率限制為合法且不低於介面保證的值
     *
     * @param requested_rate_hz 使用者請求的狀態發布頻率（Hz）
     * @return 有限且至少為 10 Hz 的狀態發布頻率
     */
    static double normalize_publish_rate_hz(double requested_rate_hz)
    {
        constexpr double MIN_STATE_PUBLISH_RATE_HZ = 10.0;
        if (!std::isfinite(requested_rate_hz) ||
            requested_rate_hz < MIN_STATE_PUBLISH_RATE_HZ)
        {
            return MIN_STATE_PUBLISH_RATE_HZ;
        }

        return requested_rate_hz;
    }

    /**
     * @brief 從 PX4 里程計、GPS 與電池狀態填充 AgentState 訊息
     *
     * @param[out] state         Agent 狀態訊息
     * @param[in]  odom          PX4 里程計訊息
     * @param[in]  flight_state  目前飛行狀態枚舉
     * @param[in]  online        Agent 是否在線
     * @param[in]  gps           PX4 全域位置（可為 nullptr）
     * @param[in]  battery       PX4 電池狀態（可為 nullptr）
     * @param[in]  observation   PX4 飛行狀態觀測資料（可為 nullptr）
     */
    static void fill_agent_state(
        taiphoon_agent::msg::AgentState& state,
        const px4_msgs::msg::VehicleOdometry& odom,
        controllers::FlightState flight_state,
        bool online,
        const px4_msgs::msg::VehicleGlobalPosition* gps = nullptr,
        const px4_msgs::msg::BatteryStatus* battery = nullptr,
        const controllers::FlightStateObservation* observation = nullptr)
    {
        // 時間戳記
        state.stamp.sec = static_cast<int32_t>(odom.timestamp / 1000000ULL);
        state.stamp.nanosec = static_cast<uint32_t>(
            (odom.timestamp % 1000000ULL) * 1000ULL);

        // NED 位置
        state.position_ned[0] = odom.position[0];
        state.position_ned[1] = odom.position[1];
        state.position_ned[2] = odom.position[2];

        // 全域 GPS 座標
        if (gps != nullptr)
        {
            state.position_global[0] = gps->lat;
            state.position_global[1] = gps->lon;
            state.position_global[2] = static_cast<double>(gps->alt);
        }
        else
        {
            state.position_global[0] =
                std::numeric_limits<double>::quiet_NaN();
            state.position_global[1] =
                std::numeric_limits<double>::quiet_NaN();
            state.position_global[2] =
                std::numeric_limits<double>::quiet_NaN();
        }

        // PX4 飛行狀態與校正後的任務階段
        if (observation != nullptr)
        {
            auto resolution = controllers::FlightStateResolver::resolve(
                *observation, flight_state);
            state.flight_mode = static_cast<uint8_t>(resolution.effective_state);
            state.armed = observation->is_armed;
            state.landed = observation->is_landed;
            state.px4_nav_state = observation->px4_nav_state;
            state.px4_state_valid = observation->is_px4_state_valid;
        }
        else
        {
            state.flight_mode = static_cast<uint8_t>(flight_state);
            state.armed = false;
            state.landed = false;
            state.px4_nav_state = 0U;
            state.px4_state_valid = false;
        }

        // 電池狀態
        if (battery != nullptr)
        {
            state.battery_remaining = battery->remaining;
            state.battery_voltage_v = battery->voltage_v;
            state.battery_current_a = battery->current_a;
        }
        else
        {
            state.battery_remaining = 1.0F;
            state.battery_voltage_v =
                std::numeric_limits<float>::quiet_NaN();
            state.battery_current_a =
                std::numeric_limits<float>::quiet_NaN();
        }

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
