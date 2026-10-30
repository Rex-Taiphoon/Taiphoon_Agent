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

/// 位置資料（odometry、global position）沉默門檻（毫秒），與 CONTEXT.md 的沉默失效定義一致
constexpr int PX4_POSITION_TIMEOUT_MS = 1000;

/// 啟動後超過此時間仍未收到任何資料，才視為「從未收到」（毫秒）
constexpr int PX4_STARTUP_GRACE_MS = 5000;

/// 位置資料沉默門檻（奈秒）
constexpr int64_t PX4_POSITION_TIMEOUT_NS =
    static_cast<int64_t>(PX4_POSITION_TIMEOUT_MS) * NANOSECONDS_PER_MILLISECOND;

/// 啟動寬限時間（奈秒）
constexpr int64_t PX4_STARTUP_GRACE_NS =
    static_cast<int64_t>(PX4_STARTUP_GRACE_MS) * NANOSECONDS_PER_MILLISECOND;

/// 奈秒換算秒
constexpr double SECONDS_PER_NANOSECOND = 1.0e-9;

/**
 * @brief 計算訊息年齡（秒），供 AgentHealth 的 *_age_s 欄位使用
 *
 * @param received_ns 最近一次收到訊息的單調時鐘奈秒（0 表示從未收到）
 * @param now_ns 目前單調時鐘奈秒
 * @return 年齡秒數；從未收到回傳 -1
 */
inline double px4_message_age_s(const int64_t received_ns, const int64_t now_ns)
{
    if (received_ns <= 0)
    {
        return -1.0;
    }
    if (now_ns <= received_ns)
    {
        return 0.0;
    }
    return static_cast<double>(now_ns - received_ns) * SECONDS_PER_NANOSECOND;
}

/**
 * @brief PX4 資料連線狀態轉換
 */
enum class Px4LinkTransition : uint8_t
{
    NONE = 0U,            ///< 沒有轉換
    BECAME_STALE = 1U,    ///< 曾經新鮮，現在逾時
    NEVER_RECEIVED = 2U,  ///< 啟動寬限期過後仍未收到任何資料
    RECOVERED = 3U        ///< 逾時（含從未收到）後恢復新鮮
};

/**
 * @brief PX4 單一 topic 的新鮮度轉換監控
 *
 * 在控制迴圈每週期輸入最近接收時間，只在狀態轉換時回報一次，
 * 讓事件不會在逾時期間持續洗版。第一次 update() 的時間視為監控起點。
 */
class Px4LinkMonitor
{
public:
    /**
     * @brief 建構子
     * @param timeout_ns 新鮮度門檻（奈秒）
     * @param startup_grace_ns 從未收到資料時的啟動寬限（奈秒）
     */
    Px4LinkMonitor(const int64_t timeout_ns, const int64_t startup_grace_ns)
        : timeout_ns_(timeout_ns),
          startup_grace_ns_(startup_grace_ns)
    {
    }

    /**
     * @brief 輸入最近接收時間並取得轉換
     *
     * @param received_ns 最近一次收到訊息的單調時鐘奈秒（0 表示從未收到）
     * @param now_ns 目前單調時鐘奈秒
     * @return 本次的狀態轉換
     */
    Px4LinkTransition update(const int64_t received_ns, const int64_t now_ns)
    {
        if (monitor_start_ns_ <= 0)
        {
            monitor_start_ns_ = now_ns;
        }

        if (is_px4_message_fresh(received_ns, now_ns, timeout_ns_))
        {
            const LinkState previous_state = state_;
            state_ = LinkState::FRESH;
            return previous_state == LinkState::STALE ?
                Px4LinkTransition::RECOVERED : Px4LinkTransition::NONE;
        }

        if (state_ == LinkState::FRESH)
        {
            state_ = LinkState::STALE;
            return Px4LinkTransition::BECAME_STALE;
        }

        if (state_ == LinkState::UNKNOWN)
        {
            if (received_ns > 0)
            {
                // 監控開始前曾收到但已逾時，等同曾經新鮮後逾時。
                state_ = LinkState::STALE;
                return Px4LinkTransition::BECAME_STALE;
            }
            if (now_ns - monitor_start_ns_ >= startup_grace_ns_)
            {
                state_ = LinkState::STALE;
                return Px4LinkTransition::NEVER_RECEIVED;
            }
        }
        return Px4LinkTransition::NONE;
    }

    /** @brief 目前是否處於逾時（含從未收到） */
    bool is_stale() const
    {
        return state_ == LinkState::STALE;
    }

private:
    enum class LinkState : uint8_t
    {
        UNKNOWN = 0U,
        FRESH = 1U,
        STALE = 2U
    };

    int64_t timeout_ns_{PX4_STATE_TIMEOUT_NS};
    int64_t startup_grace_ns_{PX4_STARTUP_GRACE_NS};
    int64_t monitor_start_ns_{0};
    LinkState state_{LinkState::UNKNOWN};
};

}  // namespace controllers
}  // namespace taiphoon
