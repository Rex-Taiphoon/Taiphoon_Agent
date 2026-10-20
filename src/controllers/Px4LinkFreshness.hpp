/**
 * @file Px4LinkFreshness.hpp
 * @brief PX4 狀態訊息新鮮度與 Offboard 輸出持續條件的純邏輯判定
 * @author Taiphoon Agent Team
 * @date 2026-10-20
 *
 * 所有時間皆為單調時鐘（steady clock）奈秒，不受系統時間調整影響。
 * 區分「PX4 明確離開 Offboard」與「狀態訊息暫時未到達」：前者立即停止
 * 輸出，後者只在超過逾時門檻時才停止。設計見 docs/basic_controller_manual.md。
 */

#pragma once

#include <cstdint>

#include "controllers/FlightStateResolver.hpp"

namespace taiphoon
{
namespace controllers
{

/// PX4 飛行狀態資料逾時時間（毫秒）；vehicle_status 約 2 Hz，可容忍連續遺失約 9 筆
constexpr int PX4_STATE_TIMEOUT_MS = 5000;

/// VehicleStatus 間隔超過此值時輸出連線品質警告（毫秒）
constexpr int PX4_STATUS_GAP_WARN_MS = 1500;

/// 毫秒換算奈秒
constexpr int64_t NANOSECONDS_PER_MILLISECOND = 1000000LL;

/// PX4 飛行狀態資料逾時時間（奈秒）
constexpr int64_t PX4_STATE_TIMEOUT_NS =
    static_cast<int64_t>(PX4_STATE_TIMEOUT_MS) * NANOSECONDS_PER_MILLISECOND;

/// VehicleStatus 間隔警告門檻（奈秒）
constexpr int64_t PX4_STATUS_GAP_WARN_NS =
    static_cast<int64_t>(PX4_STATUS_GAP_WARN_MS) * NANOSECONDS_PER_MILLISECOND;

/**
 * @brief Offboard session 期間每個控制週期的輸出決策
 */
enum class OffboardOutputDecision : uint8_t
{
    CONTINUE,            ///< 狀態新鮮且仍在 Offboard，繼續輸出
    STOP_LEFT_OFFBOARD,  ///< 新鮮狀態顯示 PX4 已離開 Offboard，立即停止
    STOP_STATUS_STALE    ///< 狀態超過逾時門檻未到達，停止輸出
};

/**
 * @brief 判定 PX4 訊息是否仍在新鮮期間內
 *
 * 尚未收到（0）、收到時間晚於目前時間或逾時門檻為負時皆視為不新鮮。
 *
 * @param received_ns 最近一次收到訊息的單調時鐘奈秒
 * @param now_ns 目前單調時鐘奈秒
 * @param timeout_ns 逾時門檻（奈秒），等於門檻時仍視為新鮮
 * @return true 表示訊息新鮮
 */
inline bool is_px4_message_fresh(
    const int64_t received_ns,
    const int64_t now_ns,
    const int64_t timeout_ns)
{
    return received_ns > 0 && timeout_ns >= 0 && now_ns >= received_ns &&
        now_ns - received_ns <= timeout_ns;
}

/**
 * @brief 判定 Offboard session 是否應繼續輸出
 *
 * @param status_received_ns 最近一次收到 VehicleStatus 的單調時鐘奈秒
 * @param nav_state 最近一次 VehicleStatus 的 nav_state
 * @param now_ns 目前單調時鐘奈秒
 * @param timeout_ns 狀態逾時門檻（奈秒）
 * @return 輸出決策
 */
inline OffboardOutputDecision evaluate_offboard_output(
    const int64_t status_received_ns,
    const uint8_t nav_state,
    const int64_t now_ns,
    const int64_t timeout_ns)
{
    if (!is_px4_message_fresh(status_received_ns, now_ns, timeout_ns))
    {
        return OffboardOutputDecision::STOP_STATUS_STALE;
    }
    if (nav_state != FlightStateResolver::PX4_NAV_STATE_OFFBOARD)
    {
        return OffboardOutputDecision::STOP_LEFT_OFFBOARD;
    }
    return OffboardOutputDecision::CONTINUE;
}

/**
 * @brief 判定兩筆 VehicleStatus 之間的間隔是否值得警告
 *
 * @param previous_received_ns 前一筆收到時間（0 表示第一筆）
 * @param received_ns 本筆收到時間
 * @param warn_ns 警告門檻（奈秒）
 * @return true 表示間隔超過門檻
 */
inline bool is_px4_status_gap_notable(
    const int64_t previous_received_ns,
    const int64_t received_ns,
    const int64_t warn_ns)
{
    return previous_received_ns > 0 && received_ns >= previous_received_ns &&
        received_ns - previous_received_ns > warn_ns;
}

}  // namespace controllers
}  // namespace taiphoon
