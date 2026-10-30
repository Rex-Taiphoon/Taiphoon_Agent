/**
 * @file FlightControllerReboot.hpp
 * @brief REBOOT_FC 安全條件與 PX4 重新啟動偵測的純邏輯
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 *
 * 所有時間皆為單調時鐘奈秒。安全條件對應
 * docs/codex/agent-waypoint-mission-plan.md 的 C2 表格；
 * 重新啟動偵測對應 C3。
 */

#pragma once

#include <cstdint>

#include "controllers/Px4LinkFreshness.hpp"

namespace taiphoon
{
namespace controllers
{

/// VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN
constexpr uint32_t PX4_VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246U;

/// param1：重新啟動飛控
constexpr float PX4_REBOOT_AUTOPILOT = 1.0F;

/// param2：不影響機上電腦
constexpr float PX4_REBOOT_ONBOARD_COMPUTER_NONE = 0.0F;

/// REBOOT_FC 送出後的冷卻時間預設值（秒）
constexpr double DEFAULT_REBOOT_COOLDOWN_S = 15.0;

/// REBOOT_FC 已被接受後，等待 PX4 重新連線的上限（奈秒）
constexpr int64_t PX4_REBOOT_PENDING_TIMEOUT_NS = 60000000000LL;

/**
 * @brief REBOOT_FC 安全判斷結果
 */
enum class FlightControllerRebootDecision : uint8_t
{
    SEND = 0U,                  ///< 送出重新啟動命令
    REJECT_COOLDOWN = 1U,       ///< 冷卻期間
    REJECT_ARMED = 2U,          ///< PX4 已解鎖（force 也拒絕）
    REJECT_NOT_LANDED = 3U,     ///< 已上鎖但未確認落地（force 也拒絕）
    REJECT_STATE_UNKNOWN = 4U   ///< VehicleStatus 從未收到或逾時，需 force
};

/**
 * @brief REBOOT_FC 判斷輸入
 */
struct FlightControllerRebootConditions
{
    bool is_status_fresh{false};        ///< VehicleStatus 是否新鮮
    bool is_armed{false};               ///< 最新 VehicleStatus 是否為已解鎖
    bool is_land_detected_fresh{false}; ///< VehicleLandDetected 是否新鮮
    bool is_landed{false};              ///< 最新 VehicleLandDetected.landed
    bool is_forced{false};              ///< REBOOT_FC 1
    bool is_cooldown_active{false};     ///< 上一次送出仍在冷卻期間
};

/**
 * @brief 依 C2 表格判斷是否送出 REBOOT_FC
 *
 * 狀態不明時 force 允許送出，前提是 PX4 Commander 在解鎖時會以
 * DENIED 拒絕重新啟動；這必須在 SITL 與實機各驗證一次。
 *
 * @param conditions 判斷輸入
 * @return 判斷結果
 */
inline FlightControllerRebootDecision evaluate_flight_controller_reboot(
    const FlightControllerRebootConditions& conditions)
{
    if (conditions.is_cooldown_active)
    {
        return FlightControllerRebootDecision::REJECT_COOLDOWN;
    }
    if (!conditions.is_status_fresh)
    {
        return conditions.is_forced ?
            FlightControllerRebootDecision::SEND :
            FlightControllerRebootDecision::REJECT_STATE_UNKNOWN;
    }
    if (conditions.is_armed)
    {
        return FlightControllerRebootDecision::REJECT_ARMED;
    }
    if (!conditions.is_land_detected_fresh || !conditions.is_landed)
    {
        return FlightControllerRebootDecision::REJECT_NOT_LANDED;
    }
    return FlightControllerRebootDecision::SEND;
}

/**
 * @brief 取得拒絕原因的英文說明（填入 AgentEvent.detail）
 *
 * @param decision 判斷結果
 * @return 說明字串；SEND 回傳空字串
 */
inline const char* flight_controller_reboot_decision_detail(
    const FlightControllerRebootDecision decision)
{
    switch (decision)
    {
        case FlightControllerRebootDecision::REJECT_COOLDOWN:
            return "REBOOT_FC rejected: reboot cooldown is active";
        case FlightControllerRebootDecision::REJECT_ARMED:
            return "REBOOT_FC rejected: PX4 is armed";
        case FlightControllerRebootDecision::REJECT_NOT_LANDED:
            return "REBOOT_FC rejected: PX4 is disarmed but landing is not confirmed";
        case FlightControllerRebootDecision::REJECT_STATE_UNKNOWN:
            return "REBOOT_FC rejected: PX4 state is unknown; send REBOOT_FC 1 to force";
        case FlightControllerRebootDecision::SEND:
        default:
            return "";
    }
}

/**
 * @brief PX4 重新啟動偵測
 *
 * 任一條件成立即視為重新啟動：
 * 1. VehicleStatus 逾時（含從未收到）後恢復，且 PX4 為上鎖。
 * 2. REBOOT_FC 已被接受後，VehicleStatus 出現超過 PX4_STATUS_GAP_WARN_MS
 *    的中斷並恢復；接受後 PX4_REBOOT_PENDING_TIMEOUT_NS 內未恢復即放棄等待。
 */
class Px4RestartDetector
{
public:
    /**
     * @brief 記錄 REBOOT_FC 已被 PX4 接受
     * @param now_ns 目前單調時鐘奈秒
     */
    void on_reboot_accepted(const int64_t now_ns)
    {
        is_reboot_pending_ = true;
        is_reboot_gap_seen_ = false;
        reboot_accepted_ns_ = now_ns;
    }

    /**
     * @brief 每個控制週期輸入 VehicleStatus 狀態
     *
     * @param status_transition 本週期 VehicleStatus 的連線轉換
     * @param status_received_ns 最近一次收到 VehicleStatus 的單調時鐘奈秒
     * @param is_armed 最新 VehicleStatus 是否為已解鎖
     * @param now_ns 目前單調時鐘奈秒
     * @return true 表示本週期偵測到 PX4 重新啟動
     */
    bool update(const Px4LinkTransition status_transition,
                const int64_t status_received_ns,
                const bool is_armed,
                const int64_t now_ns)
    {
        bool is_restarted = false;
        if (is_reboot_pending_)
        {
            if (now_ns - reboot_accepted_ns_ > PX4_REBOOT_PENDING_TIMEOUT_NS)
            {
                is_reboot_pending_ = false;
                is_reboot_gap_seen_ = false;
            }
            else
            {
                const bool is_status_gap = status_received_ns <= 0 ||
                    now_ns - status_received_ns > PX4_STATUS_GAP_WARN_NS;
                if (is_status_gap)
                {
                    is_reboot_gap_seen_ = true;
                }
                else if (is_reboot_gap_seen_ &&
                         status_received_ns > reboot_accepted_ns_)
                {
                    is_restarted = true;
                }
            }
        }

        if (status_transition == Px4LinkTransition::RECOVERED && !is_armed)
        {
            is_restarted = true;
        }

        if (is_restarted)
        {
            is_reboot_pending_ = false;
            is_reboot_gap_seen_ = false;
        }
        return is_restarted;
    }

    /** @brief 是否仍在等待 REBOOT_FC 後的重新連線 */
    bool is_reboot_pending() const
    {
        return is_reboot_pending_;
    }

private:
    bool is_reboot_pending_{false};
    bool is_reboot_gap_seen_{false};
    int64_t reboot_accepted_ns_{0};
};

}  // namespace controllers
}  // namespace taiphoon
