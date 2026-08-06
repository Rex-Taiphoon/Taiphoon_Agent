/**
 * @file FlightStateResolver.hpp
 * @brief PX4 觀測資料與 Agent 任務階段的純邏輯解析器
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "controllers/FlightState.hpp"

namespace taiphoon
{
namespace controllers
{

/// 目標高度達成比例（95% 視為已達起飛高度）
constexpr float TAKEOFF_ARRIVAL_RATIO = 0.95F;

/// 起飛意圖逾時時間（秒）
constexpr double TAKEOFF_TIMEOUT_S = 30.0;

/// NED 三維目標位置到達容差（公尺）
constexpr float MOVEMENT_ARRIVAL_TOLERANCE_M = 0.3F;

/**
 * @brief 用於解析對外飛行狀態的 PX4 觀測資料
 */
struct FlightStateObservation
{
    bool is_px4_state_valid{false};
    bool is_armed{false};
    bool is_landed{false};
    uint8_t px4_nav_state{0};
    float current_altitude_m{0.0F};
    float target_altitude_m{0.0F};
    float target_distance_m{0.0F};
    double takeoff_elapsed_s{0.0};
};

/**
 * @brief 有效飛行狀態與控制器後續轉移建議
 */
struct FlightStateResolution
{
    FlightState effective_state{FlightState::ERROR};
    bool should_transition_to_hover{false};
    bool should_hold_current_position{false};
};

/**
 * @brief 將命令意圖與 PX4 實際觀測資料解析為對外飛行狀態
 */
class FlightStateResolver
{
public:
    /// PX4 VehicleStatus NAVIGATION_STATE_AUTO_RTL 的原始值
    static constexpr uint8_t PX4_NAV_STATE_AUTO_RTL = 5U;

    /// PX4 VehicleStatus NAVIGATION_STATE_AUTO_LAND 的原始值
    static constexpr uint8_t PX4_NAV_STATE_AUTO_LAND = 18U;

    /**
     * @brief 解析有效飛行狀態
     *
     * @param observation PX4 觀測資料
     * @param command_intent Agent 目前命令意圖
     * @return 供對外發布與控制器轉移使用的解析結果
     */
    static FlightStateResolution resolve(
        const FlightStateObservation& observation,
        FlightState command_intent)
    {
        if (!observation.is_px4_state_valid)
        {
            return {FlightState::ERROR, false, false};
        }

        if (!observation.is_armed)
        {
            return {FlightState::IDLE, false, false};
        }

        if (observation.is_landed)
        {
            return {FlightState::ARMED, false, false};
        }

        if (observation.px4_nav_state == PX4_NAV_STATE_AUTO_RTL)
        {
            return {FlightState::RTL, false, false};
        }

        if (observation.px4_nav_state == PX4_NAV_STATE_AUTO_LAND)
        {
            return {FlightState::LAND, false, false};
        }

        if (command_intent == FlightState::TAKEOFF)
        {
            bool has_reached_takeoff_altitude =
                std::isfinite(observation.current_altitude_m) &&
                std::isfinite(observation.target_altitude_m) &&
                observation.target_altitude_m > 0.0F &&
                observation.current_altitude_m >=
                    observation.target_altitude_m * TAKEOFF_ARRIVAL_RATIO;

            if (has_reached_takeoff_altitude)
            {
                return {FlightState::HOVER, true, false};
            }

            if (observation.takeoff_elapsed_s >= TAKEOFF_TIMEOUT_S)
            {
                return {FlightState::HOVER, true, true};
            }

            return {FlightState::TAKEOFF, false, false};
        }

        if (command_intent == FlightState::MOVING)
        {
            bool has_reached_target =
                std::isfinite(observation.target_distance_m) &&
                observation.target_distance_m <= MOVEMENT_ARRIVAL_TOLERANCE_M;

            if (has_reached_target)
            {
                return {FlightState::HOVER, true, false};
            }

            return {FlightState::MOVING, false, false};
        }

        if (command_intent == FlightState::ERROR)
        {
            return {FlightState::ERROR, false, false};
        }

        return {FlightState::HOVER, false, false};
    }
};

}  // namespace controllers
}  // namespace taiphoon
