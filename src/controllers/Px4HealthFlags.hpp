/**
 * @file Px4HealthFlags.hpp
 * @brief PX4 FailsafeFlags 轉換為 AgentHealth.failsafe_flags 位元
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 *
 * 位元值必須與 msg/AgentHealth.msg 的 FAILSAFE_* 常數一致。
 * 轉換以樣板實作，不依賴 px4_msgs，方便以假結構做單元測試。
 */

#pragma once

#include <cstdint>

namespace taiphoon
{
namespace controllers
{

constexpr uint32_t FAILSAFE_BIT_LOCAL_POSITION_INVALID = 1U;
constexpr uint32_t FAILSAFE_BIT_GLOBAL_POSITION_INVALID = 2U;
constexpr uint32_t FAILSAFE_BIT_HOME_POSITION_INVALID = 4U;
constexpr uint32_t FAILSAFE_BIT_ATTITUDE_INVALID = 8U;
constexpr uint32_t FAILSAFE_BIT_MANUAL_CONTROL_LOST = 16U;
constexpr uint32_t FAILSAFE_BIT_OFFBOARD_SIGNAL_LOST = 32U;
constexpr uint32_t FAILSAFE_BIT_BATTERY_WARNING = 64U;
constexpr uint32_t FAILSAFE_BIT_BATTERY_UNHEALTHY = 128U;
constexpr uint32_t FAILSAFE_BIT_GEOFENCE_BREACHED = 256U;
constexpr uint32_t FAILSAFE_BIT_FD_CRITICAL_FAILURE = 512U;
constexpr uint32_t FAILSAFE_BIT_FD_ESC_ARMING_FAILURE = 1024U;
constexpr uint32_t FAILSAFE_BIT_LOCAL_POSITION_ACCURACY_LOW = 2048U;

/**
 * @brief 將 PX4 FailsafeFlags 整理為 AgentHealth.failsafe_flags 位元
 *
 * `battery_warning` 為 BatteryStatus 的警告等級，非 0（NONE）即設定位元；
 * `position_accuracy_low` 對應 FAILSAFE_LOCAL_POSITION_ACCURACY_LOW。
 *
 * @tparam FailsafeFlagsT 具備 px4_msgs/FailsafeFlags 對應欄位的型別
 * @param flags PX4 FailsafeFlags
 * @return 位元組合
 */
template <typename FailsafeFlagsT>
uint32_t make_failsafe_flag_bits(const FailsafeFlagsT& flags)
{
    uint32_t bits = 0U;
    const auto set_bit = [&bits](const bool is_set, const uint32_t bit)
        {
            if (is_set)
            {
                bits |= bit;
            }
        };

    set_bit(flags.local_position_invalid, FAILSAFE_BIT_LOCAL_POSITION_INVALID);
    set_bit(flags.global_position_invalid, FAILSAFE_BIT_GLOBAL_POSITION_INVALID);
    set_bit(flags.home_position_invalid, FAILSAFE_BIT_HOME_POSITION_INVALID);
    set_bit(flags.attitude_invalid, FAILSAFE_BIT_ATTITUDE_INVALID);
    set_bit(flags.manual_control_signal_lost, FAILSAFE_BIT_MANUAL_CONTROL_LOST);
    set_bit(flags.offboard_control_signal_lost, FAILSAFE_BIT_OFFBOARD_SIGNAL_LOST);
    set_bit(flags.battery_warning != 0U, FAILSAFE_BIT_BATTERY_WARNING);
    set_bit(flags.battery_unhealthy, FAILSAFE_BIT_BATTERY_UNHEALTHY);
    set_bit(flags.geofence_breached, FAILSAFE_BIT_GEOFENCE_BREACHED);
    set_bit(flags.fd_critical_failure, FAILSAFE_BIT_FD_CRITICAL_FAILURE);
    set_bit(flags.fd_esc_arming_failure, FAILSAFE_BIT_FD_ESC_ARMING_FAILURE);
    set_bit(flags.position_accuracy_low, FAILSAFE_BIT_LOCAL_POSITION_ACCURACY_LOW);
    return bits;
}

}  // namespace controllers
}  // namespace taiphoon
