/**
 * @file WaypointMissionExecutor.hpp
 * @brief Agent 端航點任務的驗證、座標換算與狀態機（純 C++）
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 *
 * executor 只處理任務資料、狀態轉換、停留與逾時計時；
 * 目標設定、到達判斷與停在目前位置由 ego_swarm_planner_node 依回傳結果執行。
 * 狀態與原因碼數值與 msg/WaypointMissionStatus.msg 一致。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/LocalNedFrame.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// WaypointMission.ALTITUDE_PX4_LOCAL：高度為 PX4 local NED 的 -z
constexpr uint8_t WAYPOINT_ALTITUDE_PX4_LOCAL = 1U;

/// 判斷到達事件的目標是否為本段目標的容差（公尺）
constexpr double WAYPOINT_TARGET_MATCH_TOLERANCE_M = 1.0e-3;

/**
 * @brief 任務狀態（WaypointMissionStatus.STATE_*）
 */
enum class WaypointMissionState : uint8_t
{
    IDLE = 0U,
    LOADED = 1U,
    RUNNING = 2U,
    DWELLING = 3U,
    COMPLETED = 4U,
    ABORTED = 5U,
    REJECTED = 6U
};

/**
 * @brief 原因碼（WaypointMissionStatus.REASON_*）
 */
enum class WaypointMissionReason : uint8_t
{
    NONE = 0U,
    INVALID_PLAN = 1U,
    OUT_OF_RANGE = 2U,
    NOT_READY = 3U,
    NOT_HOVERING = 4U,
    BUSY = 5U,
    ID_MISMATCH = 6U,
    OPERATOR_STOP = 10U,
    COMMAND_OVERRIDE = 11U,
    FLIGHT_STATE = 12U,
    LANDED = 13U,
    LEG_TIMEOUT = 14U
};

/**
 * @brief 任務限制（對應 ego_swarm_planner 的 mission_* 參數）
 */
struct WaypointMissionLimits
{
    std::size_t max_waypoints{50U};
    double min_altitude_m{1.0};
    double max_altitude_m{120.0};
    double max_range_m{1000.0};
    double max_dwell_s{60.0};
    double leg_base_timeout_s{30.0};
    double min_expected_speed_mps{0.3};
    double load_timeout_s{10.0};
};

/**
 * @brief 上傳的航點（與 MissionWaypoint.msg 相同欄位，不依賴 ROS2 訊息型別）
 */
struct MissionWaypointInput
{
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    double dwell_s{0.0};
    double yaw_deg{0.0};
};

/**
 * @brief 換算後的航段：planner-local NED 目標與停留時間
 */
struct WaypointMissionLeg
{
    Vector3 target_planner_ned{0.0, 0.0, 0.0};
    double dwell_s{0.0};
};

/**
 * @brief 驗證、載入或開始的判斷結果
 *
 * 不接受時，reason 與 index 用於發布一次 REJECTED 狀態；
 * index 指向第一個不合法的航點（與航點無關時為 0）。
 */
struct WaypointMissionDecision
{
    bool is_accepted{false};
    WaypointMissionReason reason{WaypointMissionReason::NONE};
    uint16_t index{0U};
    std::string detail;
};

/**
 * @brief MISSION_START 的開始條件
 */
struct WaypointMissionStartConditions
{
    bool can_plan{false};          ///< FlightMissionLifecycle::can_plan()
    bool is_hovering{false};       ///< AgentState 有效且 flight_mode == HOVER
    bool is_px4_offboard{false};   ///< PX4 nav_state == OFFBOARD
    bool has_formation{false};     ///< 是否在編隊中
};

/**
 * @brief 對外狀態快照（填入 WaypointMissionStatus）
 */
struct WaypointMissionStatusSnapshot
{
    uint32_t mission_id{0U};
    WaypointMissionState state{WaypointMissionState::IDLE};
    WaypointMissionReason reason{WaypointMissionReason::NONE};
    uint16_t current_index{0U};
    uint16_t waypoint_count{0U};
    std::string detail;
};

/**
 * @brief 驗證上傳的任務格式（不含座標換算與距離檢查）
 *
 * @param mission_id 任務識別碼
 * @param altitude_frame 高度基準
 * @param waypoints 航點
 * @param limits 任務限制
 * @return 接受或 INVALID_PLAN 與第一個不合法的航點索引
 */
WaypointMissionDecision validate_waypoint_mission_plan(
    uint32_t mission_id,
    uint8_t altitude_frame,
    const std::vector<MissionWaypointInput>& waypoints,
    const WaypointMissionLimits& limits);

/**
 * @brief 將航點換算為 planner-local NED 航段
 *
 * 水平以 planner frame 由 WGS84 直接換算；垂直以 PX4 local 高度
 * 換算：z_planner = -altitude_m - offset.z（PX4 local = planner-local + offset）。
 *
 * @param waypoints 已驗證的航點
 * @param planner_frame 已設定原點的 planner frame
 * @param planner_to_px4_ned_offset planner-local 至 PX4 local NED 的平移
 * @param[out] legs 換算結果；失敗時清空
 * @return true 表示全部換算成功
 */
bool convert_waypoints_to_planner_legs(
    const std::vector<MissionWaypointInput>& waypoints,
    const LocalNedFrame& planner_frame,
    const Vector3& planner_to_px4_ned_offset,
    std::vector<WaypointMissionLeg>& legs);

/**
 * @brief 檢查每個航段與目前位置的水平距離
 *
 * @param legs 航段
 * @param current_planner_ned 目前 planner-local 位置
 * @param max_range_m 水平距離上限
 * @return 接受或 OUT_OF_RANGE 與第一個超出範圍的航點索引
 */
WaypointMissionDecision check_waypoint_mission_range(
    const std::vector<WaypointMissionLeg>& legs,
    const Vector3& current_planner_ned,
    double max_range_m);

/**
 * @brief 航點任務狀態機
 *
 * 時間輸入為節點提供的單調秒數。目標由 due_target() 交給節點設定，
 * 到達由節點以 on_target_reached() 回報；executor 不直接改變 planner 狀態。
 */
class WaypointMissionExecutor
{
public:
    /**
     * @brief 建構子
     * @param limits 任務限制
     */
    explicit WaypointMissionExecutor(
        const WaypointMissionLimits& limits = WaypointMissionLimits{});

    /** @brief 設定任務限制（只影響之後載入與開始的任務） */
    void set_limits(const WaypointMissionLimits& limits);

    /** @brief 取得任務限制 */
    const WaypointMissionLimits& limits() const;

    /**
     * @brief 載入已換算的任務
     *
     * 執行中或編隊中拒絕（BUSY），目前狀態不變；接受時覆蓋已載入、
     * 已完成或已中止的任務並轉為 LOADED。
     *
     * @param mission_id 任務識別碼
     * @param legs 航段
     * @param has_formation 是否在編隊中
     * @param now_s 目前時間（秒）
     * @return 判斷結果
     */
    WaypointMissionDecision load(uint32_t mission_id,
                                 std::vector<WaypointMissionLeg> legs,
                                 bool has_formation,
                                 double now_s);

    /**
     * @brief 開始已載入的任務
     *
     * 依序檢查：識別碼（含載入逾時）、懸停與 Offboard、編隊、距離上限。
     *
     * @param mission_id 任務識別碼
     * @param conditions 開始條件
     * @param current_planner_ned 目前 planner-local 位置
     * @param now_s 目前時間（秒）
     * @return 判斷結果
     */
    WaypointMissionDecision start(uint32_t mission_id,
                                  const WaypointMissionStartConditions& conditions,
                                  const Vector3& current_planner_ned,
                                  double now_s);

    /**
     * @brief 停止任務
     *
     * RUNNING／DWELLING 轉為 ABORTED；LOADED 清除為 IDLE；其他狀態不變。
     *
     * @param reason 原因碼
     * @param detail 英文補充說明
     * @return true 表示中止了執行中的任務，呼叫端需依原因決定是否停在目前位置
     */
    bool stop(WaypointMissionReason reason, const std::string& detail);

    /**
     * @brief LOADED 超過載入時限即回到 IDLE
     * @param now_s 目前時間（秒）
     * @return true 表示本次清除了已載入的任務
     */
    bool expire_load_if_due(double now_s);

    /**
     * @brief 取得需要設定的下一段目標
     *
     * 開始後第一次呼叫回傳第一段；停留結束時前進到下一段並回傳。
     *
     * @param now_s 目前時間（秒）
     * @return 需要設定的 planner-local 目標；沒有時為空
     */
    std::optional<Vector3> due_target(double now_s);

    /**
     * @brief 回報 planner 已到達 Final Target
     *
     * 只在 RUNNING、目標已交給節點且與本段目標相同時有效；
     * 非最後一點轉 DWELLING，最後一點轉 COMPLETED（不停留）。
     *
     * @param final_target 已到達的 planner Final Target
     * @param now_s 目前時間（秒）
     * @return true 表示本段已完成
     */
    bool on_target_reached(const Vector3& final_target, double now_s);

    /**
     * @brief 本段是否逾時（只在 RUNNING 計算，停留不計）
     * @param now_s 目前時間（秒）
     * @return true 表示逾時
     */
    bool is_leg_timed_out(double now_s) const;

    /** @brief LOADED、RUNNING 或 DWELLING */
    bool is_active() const;

    /** @brief RUNNING 或 DWELLING */
    bool is_running() const;

    /** @brief 取得目前狀態快照 */
    WaypointMissionStatusSnapshot snapshot() const;

private:
    /**
     * @brief 開始指定索引的航段
     * @param index 航段索引
     * @param leg_origin 本段起點（開始位置或上一個航點）
     * @param now_s 目前時間（秒）
     */
    void begin_leg(std::size_t index, const Vector3& leg_origin, double now_s);

    WaypointMissionLimits limits_{};
    WaypointMissionState state_{WaypointMissionState::IDLE};
    WaypointMissionReason reason_{WaypointMissionReason::NONE};
    uint32_t mission_id_{0U};
    std::vector<WaypointMissionLeg> legs_;
    std::size_t current_index_{0U};
    std::string detail_;

    double loaded_at_s_{0.0};
    double leg_started_s_{0.0};
    double leg_timeout_s_{0.0};
    double dwell_until_s_{0.0};

    /// 本段目標尚未交給節點設定
    bool is_target_pending_{false};
};

}  // namespace ego_swarm
}  // namespace taiphoon
