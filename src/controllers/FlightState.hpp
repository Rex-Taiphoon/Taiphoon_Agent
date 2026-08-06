/**
 * @file FlightState.hpp
 * @brief 基礎飛行控制器使用的任務階段列舉
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#pragma once

#include <cstdint>

namespace taiphoon
{
namespace controllers
{

/// Agent 任務階段
enum class FlightState : uint8_t
{
    IDLE = 0,
    OFFBOARD_INIT,
    OFFBOARD_ACTIVE,
    ARMED,
    TAKEOFF,
    HOVER,
    RTL,
    LAND,
    MOVING,
    POST_RTL,
    ERROR
};

}  // namespace controllers
}  // namespace taiphoon
