/**
 * @file AgentEventCodes.hpp
 * @brief Agent 對外事件代碼、嚴重度、發布節流與狀態轉換偵測
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 *
 * 事件代碼是 Agent 與 GCS 的對外契約（docs/taiphoon-agent-interface.md），只增不改。
 * 本檔不依賴 ROS2：節點以 AgentEventGate 決定是否發布，再填入 AgentEvent 訊息。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>

namespace taiphoon
{
namespace safety
{

/**
 * @brief 事件代碼（對應 AgentEvent.code）
 */
enum class AgentEventCode : uint16_t
{
    COMMAND_UNKNOWN = 100U,             ///< 無法解析的命令字串
    COMMAND_INVALID_PARAMS = 101U,      ///< 參數缺少或不合法
    COMMAND_REJECTED = 102U,            ///< Agent 狀態不允許執行
    COMMAND_FAILED = 103U,              ///< 命令處理失敗
    PX4_COMMAND_REJECTED = 200U,        ///< VehicleCommandAck 不是 ACCEPTED／IN_PROGRESS
    PX4_COMMAND_TIMEOUT = 201U,         ///< 未在時限內收到 ack 或確認
    PX4_COMMAND_ACCEPTED = 202U,        ///< ARM、DISARM、REBOOT_FC 已被 PX4 接受
    TAKEOFF_CANCELLED = 203U,           ///< 排隊中的 TAKEOFF 等待解鎖逾時
    OFFBOARD_LOST = 204U,               ///< PX4 離開 Offboard
    PX4_STATUS_STALE = 300U,            ///< VehicleStatus 從未收到或逾時
    PX4_STATUS_RECOVERED = 301U,        ///< VehicleStatus 恢復
    PX4_POSITION_STALE = 302U,          ///< odometry 或 global position 逾時
    PX4_PREFLIGHT_FAILED = 303U,        ///< 預檢由通過轉為未通過，或 ARM 時未通過
    PX4_FAILSAFE = 304U,                ///< PX4 進入 failsafe
    PX4_RESTART_DETECTED = 305U,        ///< 偵測到 PX4 重新啟動或重新連線
    PLANNER_COMMAND_REJECTED = 400U,    ///< planner 拒絕命令
    PLANNER_EMERGENCY_BRAKE = 401U,     ///< planner 啟動緊急煞停
    PLANNER_FAILURE_PERSISTENT = 402U,  ///< 連續規劃失敗超過門檻
    SAFETY_POSITION_DEVIATION = 500U    ///< 位置偏差安全介入（保留，暫不發布）
};

/**
 * @brief 事件嚴重度（對應 AgentEvent.SEVERITY_*）
 */
enum class AgentEventSeverity : uint8_t
{
    INFO = 0U,
    WARNING = 1U,
    ERROR = 2U
};

/**
 * @brief 事件來源節點（對應 AgentEvent.SOURCE_*）
 */
enum class AgentEventSource : uint8_t
{
    BASIC_CONTROLLER = 1U,
    PLANNER = 2U,
    SUPERVISOR = 3U
};

/// AgentEvent.px4_result 沒有對應 ack 時的值
constexpr int16_t PX4_RESULT_NONE = -1;

/// 同一代碼與節流鍵的最短發布間隔（奈秒）
constexpr int64_t AGENT_EVENT_THROTTLE_NS = 5000000000LL;

/// /{agent_namespace}/events 的 QoS depth（reliable、volatile）；所有發布者與訂閱者共用
constexpr std::size_t AGENT_EVENT_QOS_DEPTH = 20U;

/**
 * @brief 待發布的事件內容
 */
struct AgentEventRecord
{
    AgentEventCode code{AgentEventCode::COMMAND_UNKNOWN};
    std::string command;                 ///< 相關命令名稱，沒有時為空字串
    int16_t px4_result{PX4_RESULT_NONE}; ///< 相關 VehicleCommandAck.result
    std::string detail;                  ///< 英文補充說明
};

/**
 * @brief 取得事件代碼的固定嚴重度
 *
 * @param code 事件代碼
 * @return 嚴重度
 */
inline AgentEventSeverity agent_event_severity(const AgentEventCode code)
{
    switch (code)
    {
        case AgentEventCode::PX4_COMMAND_ACCEPTED:
        case AgentEventCode::PX4_STATUS_RECOVERED:
        case AgentEventCode::PX4_RESTART_DETECTED:
            return AgentEventSeverity::INFO;
        case AgentEventCode::COMMAND_FAILED:
        case AgentEventCode::PX4_COMMAND_REJECTED:
        case AgentEventCode::PX4_COMMAND_TIMEOUT:
        case AgentEventCode::PX4_STATUS_STALE:
        case AgentEventCode::PLANNER_FAILURE_PERSISTENT:
        case AgentEventCode::SAFETY_POSITION_DEVIATION:
            return AgentEventSeverity::ERROR;
        case AgentEventCode::COMMAND_UNKNOWN:
        case AgentEventCode::COMMAND_INVALID_PARAMS:
        case AgentEventCode::COMMAND_REJECTED:
        case AgentEventCode::TAKEOFF_CANCELLED:
        case AgentEventCode::OFFBOARD_LOST:
        case AgentEventCode::PX4_POSITION_STALE:
        case AgentEventCode::PX4_PREFLIGHT_FAILED:
        case AgentEventCode::PX4_FAILSAFE:
        case AgentEventCode::PLANNER_COMMAND_REJECTED:
        case AgentEventCode::PLANNER_EMERGENCY_BRAKE:
        default:
            return AgentEventSeverity::WARNING;
    }
}

/**
 * @brief 取得事件代碼名稱（用於日誌）
 *
 * @param code 事件代碼
 * @return 代碼名稱
 */
inline const char* agent_event_code_name(const AgentEventCode code)
{
    switch (code)
    {
        case AgentEventCode::COMMAND_UNKNOWN:            return "COMMAND_UNKNOWN";
        case AgentEventCode::COMMAND_INVALID_PARAMS:     return "COMMAND_INVALID_PARAMS";
        case AgentEventCode::COMMAND_REJECTED:           return "COMMAND_REJECTED";
        case AgentEventCode::COMMAND_FAILED:             return "COMMAND_FAILED";
        case AgentEventCode::PX4_COMMAND_REJECTED:       return "PX4_COMMAND_REJECTED";
        case AgentEventCode::PX4_COMMAND_TIMEOUT:        return "PX4_COMMAND_TIMEOUT";
        case AgentEventCode::PX4_COMMAND_ACCEPTED:       return "PX4_COMMAND_ACCEPTED";
        case AgentEventCode::TAKEOFF_CANCELLED:          return "TAKEOFF_CANCELLED";
        case AgentEventCode::OFFBOARD_LOST:              return "OFFBOARD_LOST";
        case AgentEventCode::PX4_STATUS_STALE:           return "PX4_STATUS_STALE";
        case AgentEventCode::PX4_STATUS_RECOVERED:       return "PX4_STATUS_RECOVERED";
        case AgentEventCode::PX4_POSITION_STALE:         return "PX4_POSITION_STALE";
        case AgentEventCode::PX4_PREFLIGHT_FAILED:       return "PX4_PREFLIGHT_FAILED";
        case AgentEventCode::PX4_FAILSAFE:               return "PX4_FAILSAFE";
        case AgentEventCode::PX4_RESTART_DETECTED:       return "PX4_RESTART_DETECTED";
        case AgentEventCode::PLANNER_COMMAND_REJECTED:   return "PLANNER_COMMAND_REJECTED";
        case AgentEventCode::PLANNER_EMERGENCY_BRAKE:    return "PLANNER_EMERGENCY_BRAKE";
        case AgentEventCode::PLANNER_FAILURE_PERSISTENT: return "PLANNER_FAILURE_PERSISTENT";
        case AgentEventCode::SAFETY_POSITION_DEVIATION:  return "SAFETY_POSITION_DEVIATION";
        default:                                          return "UNKNOWN";
    }
}

/**
 * @brief 判斷是否為只在狀態轉換時發布的事件
 *
 * 這類事件由呼叫端的轉換偵測決定時機，不再套用 5 秒節流，
 * 否則快速恢復後再次逾時的轉換會被吞掉。
 *
 * @param code 事件代碼
 * @return true 表示 300、301、303、304
 */
inline bool is_state_transition_event(const AgentEventCode code)
{
    return code == AgentEventCode::PX4_STATUS_STALE ||
        code == AgentEventCode::PX4_STATUS_RECOVERED ||
        code == AgentEventCode::PX4_PREFLIGHT_FAILED ||
        code == AgentEventCode::PX4_FAILSAFE;
}

/**
 * @brief 將 VehicleCommandAck.result 轉為 AgentEvent.px4_result
 *
 * PX4 目前的 result 為 uint8；保留轉換函式避免日後型別放寬時溢位。
 *
 * @param result ack 結果
 * @return 介於 0～32767 的結果值
 */
inline int16_t to_px4_result_field(const uint32_t result)
{
    constexpr uint32_t MAX_RESULT =
        static_cast<uint32_t>(std::numeric_limits<int16_t>::max());
    return static_cast<int16_t>(result > MAX_RESULT ? MAX_RESULT : result);
}

/**
 * @brief 取得 PX4 VehicleCommandAck 結果名稱（MAV_RESULT）
 *
 * @param result ack 結果
 * @return 名稱；未知值回傳 "UNKNOWN_RESULT"
 */
inline const char* px4_command_result_name(const uint32_t result)
{
    switch (result)
    {
        case 0U:  return "ACCEPTED";
        case 1U:  return "TEMPORARILY_REJECTED";
        case 2U:  return "DENIED";
        case 3U:  return "UNSUPPORTED";
        case 4U:  return "FAILED";
        case 5U:  return "IN_PROGRESS";
        case 6U:  return "CANCELLED";
        case 7U:  return "COMMAND_LONG_ONLY";
        case 8U:  return "COMMAND_INT_ONLY";
        case 9U:  return "UNSUPPORTED_MAV_FRAME";
        default:  return "UNKNOWN_RESULT";
    }
}

/**
 * @brief 事件發布閘門：序號與同代碼節流
 *
 * 同一代碼與節流鍵（通常是命令名稱）在節流時間內只發布一次；
 * 狀態轉換事件不節流。序號在每次實際發布時遞增，GCS 用來偵測遺失與去重。
 */
class AgentEventGate
{
public:
    /**
     * @brief 建構子
     * @param throttle_ns 同代碼與節流鍵的最短間隔（奈秒），負值視為 0
     */
    explicit AgentEventGate(const int64_t throttle_ns = AGENT_EVENT_THROTTLE_NS)
        : throttle_ns_(throttle_ns > 0 ? throttle_ns : 0)
    {
    }

    /**
     * @brief 判斷事件是否應發布，並記錄發布時間
     *
     * @param code 事件代碼
     * @param key 節流鍵（例如命令名稱或資料來源）
     * @param now_ns 目前單調時鐘奈秒
     * @return true 表示應發布
     */
    bool should_publish(const AgentEventCode code, const std::string& key,
                        const int64_t now_ns)
    {
        if (is_state_transition_event(code))
        {
            return true;
        }

        const auto map_key = std::make_pair(static_cast<uint16_t>(code), key);
        const auto found = last_published_ns_.find(map_key);
        if (found != last_published_ns_.end() && now_ns >= found->second &&
            now_ns - found->second < throttle_ns_)
        {
            return false;
        }
        last_published_ns_[map_key] = now_ns;
        return true;
    }

    /**
     * @brief 取得下一個事件序號（從 1 開始，溢位後回到 0 繼續遞增）
     * @return 事件序號
     */
    uint32_t next_sequence()
    {
        ++sequence_;
        return sequence_;
    }

private:
    int64_t throttle_ns_{AGENT_EVENT_THROTTLE_NS};
    std::map<std::pair<uint16_t, std::string>, int64_t> last_published_ns_;
    uint32_t sequence_{0U};
};

/**
 * @brief 布林狀態轉換
 */
enum class BooleanTransition : uint8_t
{
    NONE = 0U,
    BECAME_TRUE = 1U,
    BECAME_FALSE = 2U
};

/**
 * @brief 布林狀態轉換偵測
 *
 * 第一筆觀測只建立基準、不算轉換，避免 Agent 啟動或 PX4 重新啟動時
 * 把初始值誤報成事件；持續性的狀態由 health 週期呈現。
 */
class BooleanTransitionDetector
{
public:
    /**
     * @brief 輸入最新觀測值
     * @param value 最新值
     * @return 與前一筆相比的轉換
     */
    BooleanTransition update(const bool value)
    {
        if (!has_value_)
        {
            has_value_ = true;
            value_ = value;
            return BooleanTransition::NONE;
        }
        if (value == value_)
        {
            return BooleanTransition::NONE;
        }
        value_ = value;
        return value ? BooleanTransition::BECAME_TRUE :
            BooleanTransition::BECAME_FALSE;
    }

    /** @brief 清除基準，下一筆觀測重新建立 */
    void reset()
    {
        has_value_ = false;
        value_ = false;
    }

private:
    bool has_value_{false};
    bool value_{false};
};

}  // namespace safety
}  // namespace taiphoon
