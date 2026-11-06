/**
 * @file GeofenceUpdate.hpp
 * @brief 地面站執行期設定地理圍籬的更新規則（純 C++）
 * @author Taiphoon Agent Team
 * @date 2026-11-06
 *
 * 圍籬只由地面站在執行期設定，飛行中可隨時替換或清除。本檔負責：
 *
 * 1. 檢查命令本身（版本號、動作、margin、頂點數量與座標）。
 * 2. 以 planner Home 建立 planner NED 圍籬；Home 尚未凍結時暫存為 PENDING。
 * 3. 拒絕時保留原本的圍籬，不 fail closed；內容相同的重送不重新套用。
 * 4. 提供圍籬更新後判斷目標是否需要取消的查詢。
 *
 * 本模組不處理 ROS2 msg、編隊、任務或煞停；套用後的處理由 Node 決定。
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ego_swarm/EgoSwarmTypes.hpp"
#include "ego_swarm/Geofence.hpp"
#include "ego_swarm/SwarmFrame.hpp"

namespace taiphoon
{
namespace ego_swarm
{

/// 地面站可設定的最大 margin（公尺）；避免誤填使整個場域不可用。
constexpr double GEOFENCE_MAX_MARGIN_M = 10.0;

/**
 * @brief 圍籬命令動作（對應 Geofence.ACTION_*）
 */
enum class GeofenceAction : uint8_t
{
    SET = 1U,
    CLEAR = 2U
};

/**
 * @brief 圍籬狀態（對應 GeofenceStatus.STATE_*）
 */
enum class GeofenceState : uint8_t
{
    DISABLED = 0U,
    PENDING = 1U,
    ACTIVE = 2U
};

/**
 * @brief 命令處理結果（對應 GeofenceStatus.RESULT_*）
 */
enum class GeofenceUpdateResult : uint8_t
{
    NONE = 0U,
    ACCEPTED = 1U,
    REJECTED_INVALID = 2U,
    REJECTED_TOO_FAR = 3U
};

/**
 * @brief 地面站的圍籬命令（已從 ROS2 msg 轉出）
 */
struct GeofenceRequest
{
    uint32_t fence_id{0U};
    uint8_t action{0U};
    double margin_m{0.0};
    std::vector<GeoCoordinate> vertices{};
};

/**
 * @brief 目前生效或等待中的圍籬內容（WGS84）
 */
struct GeofenceDefinition
{
    uint32_t fence_id{0U};
    double margin_m{0.0};
    std::vector<GeoCoordinate> vertices{};
};

/**
 * @brief 一次處理的結果
 */
struct GeofenceUpdateOutcome
{
    /// 生效中的圍籬是否改變（替換、清除或由 PENDING 轉 ACTIVE）；Node 需重新檢查目前工作
    bool is_fence_changed{false};
    /// 回報給地面站的狀態是否改變
    bool is_status_changed{false};
    GeofenceUpdateResult result{GeofenceUpdateResult::NONE};
    std::string detail{};
};

/**
 * @brief 檢查命令本身是否合法（不需 planner Home）
 * @param request 地面站命令
 * @param[out] out_detail 不合法時的英文說明
 * @return true 表示合法
 */
inline bool is_geofence_request_valid(
    const GeofenceRequest& request,
    std::string& out_detail)
{
    if (request.fence_id == 0U)
    {
        out_detail = "fence_id must not be 0";
        return false;
    }
    if (request.action == static_cast<uint8_t>(GeofenceAction::CLEAR))
    {
        return true;
    }
    if (request.action != static_cast<uint8_t>(GeofenceAction::SET))
    {
        out_detail = "unknown action";
        return false;
    }
    if (!std::isfinite(request.margin_m) || request.margin_m < 0.0 ||
        request.margin_m > GEOFENCE_MAX_MARGIN_M)
    {
        out_detail = "margin_m must be within 0 to 10 m";
        return false;
    }
    if (request.vertices.size() < 3U ||
        request.vertices.size() > GEOFENCE_MAX_VERTEX_COUNT)
    {
        out_detail = "vertex count must be within 3 to 64";
        return false;
    }
    for (const GeoCoordinate& vertex : request.vertices)
    {
        GeoCoordinate horizontal = vertex;
        horizontal.altitude_m = 0.0;
        if (!LocalNedFrame::is_valid_geodetic(horizontal))
        {
            out_detail = "vertex latitude or longitude is invalid";
            return false;
        }
    }
    return true;
}

/**
 * @brief 以 planner Home 建立 planner NED 圍籬
 *
 * 與 Geofence::from_geodetic 相同，但區分「距 Home 過遠」與其他幾何錯誤，
 * 讓地面站能顯示正確原因。
 *
 * @param frame 已凍結 Home 的 planner frame
 * @param definition 圍籬內容
 * @param[out] out_fence 建立結果；失敗時為無效圍籬
 * @param[out] out_detail 失敗時的英文說明
 * @return ACCEPTED 或拒絕原因
 */
inline GeofenceUpdateResult build_planner_geofence(
    const LocalNedFrame& frame,
    const GeofenceDefinition& definition,
    Geofence& out_fence,
    std::string& out_detail)
{
    out_fence = Geofence{};
    std::vector<Vector3> vertices_ned;
    vertices_ned.reserve(definition.vertices.size());
    for (const GeoCoordinate& vertex : definition.vertices)
    {
        GeoCoordinate horizontal = vertex;
        horizontal.altitude_m = 0.0;
        Vector3 ned{};
        if (!frame.to_local_ned(horizontal, ned))
        {
            out_detail = "planner Home unavailable or vertex conversion failed";
            return GeofenceUpdateResult::REJECTED_INVALID;
        }
        if (std::hypot(ned.at(0), ned.at(1)) > GEOFENCE_MAX_VERTEX_DISTANCE_M)
        {
            out_detail = "vertex farther than 5 km from planner Home";
            return GeofenceUpdateResult::REJECTED_TOO_FAR;
        }
        vertices_ned.push_back(ned);
    }
    if (!out_fence.set_polygon(vertices_ned, definition.margin_m))
    {
        out_detail = "polygon is self-intersecting or degenerate";
        return GeofenceUpdateResult::REJECTED_INVALID;
    }
    return GeofenceUpdateResult::ACCEPTED;
}

/**
 * @brief 兩份圍籬內容是否相同（版本號、margin 與頂點完全一致）
 * @param first 第一份
 * @param second 第二份
 * @return true 表示相同
 */
inline bool is_same_geofence_definition(
    const GeofenceDefinition& first,
    const GeofenceDefinition& second)
{
    if (first.fence_id != second.fence_id ||
        first.margin_m != second.margin_m ||
        first.vertices.size() != second.vertices.size())
    {
        return false;
    }
    for (std::size_t index = 0U; index < first.vertices.size(); ++index)
    {
        if (first.vertices.at(index).latitude_deg !=
                second.vertices.at(index).latitude_deg ||
            first.vertices.at(index).longitude_deg !=
                second.vertices.at(index).longitude_deg)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 圍籬更新後，目標是否落在圍籬（含 margin）外
 * @param fence 生效中的圍籬；nullptr 表示沒有圍籬
 * @param target planner NED 目標
 * @return true 表示需要取消
 */
inline bool is_target_outside_geofence(
    const Geofence* const fence,
    const Vector3& target)
{
    return fence != nullptr && !fence->contains_with_margin(target);
}

/**
 * @brief 找出第一個落在圍籬外的待執行航點
 * @param fence 生效中的圍籬；nullptr 表示沒有圍籬
 * @param targets （航點索引, planner NED 目標）
 * @return 第一個越界航點的索引；全部在內時為空
 */
inline std::optional<std::size_t> first_target_outside_geofence(
    const Geofence* const fence,
    const std::vector<std::pair<std::size_t, Vector3>>& targets)
{
    for (const auto& target : targets)
    {
        if (is_target_outside_geofence(fence, target.second))
        {
            return target.first;
        }
    }
    return std::nullopt;
}

/**
 * @brief 管理地面站設定的圍籬生命週期
 *
 * 狀態只在命令或 Home 凍結時改變；Node 依 Outcome 決定是否重新檢查目前工作
 * 與發布 GeofenceStatus。
 */
class GeofenceUpdatePolicy
{
public:
    /**
     * @brief 處理地面站命令
     * @param request 地面站命令
     * @param home_frame 已凍結 Home 的 planner frame；尚未凍結時為 nullptr
     * @return 處理結果
     */
    GeofenceUpdateOutcome handle_request(
        const GeofenceRequest& request,
        const LocalNedFrame* const home_frame)
    {
        GeofenceUpdateOutcome outcome;
        const GeofenceState previous_state = state_;
        const bool is_new_request = request.fence_id != last_request_id_;
        last_request_id_ = request.fence_id;

        std::string detail;
        if (!is_geofence_request_valid(request, detail))
        {
            return reject(GeofenceUpdateResult::REJECTED_INVALID, detail,
                          is_new_request);
        }

        if (request.action == static_cast<uint8_t>(GeofenceAction::CLEAR))
        {
            const bool was_enabled = state_ != GeofenceState::DISABLED;
            state_ = GeofenceState::DISABLED;
            definition_ = GeofenceDefinition{};
            fence_ = Geofence{};
            outcome.is_fence_changed = was_enabled &&
                previous_state == GeofenceState::ACTIVE;
            return accept(outcome, "geofence cleared", was_enabled || is_new_request);
        }

        GeofenceDefinition definition;
        definition.fence_id = request.fence_id;
        definition.margin_m = request.margin_m;
        definition.vertices = request.vertices;
        if (state_ != GeofenceState::DISABLED &&
            is_same_geofence_definition(definition, definition_))
        {
            // transient_local 重送或地面站重複發布：不重新套用；
            // 只有版本號或上次結果不同時才需要更新回報。
            if (!is_new_request &&
                last_result_ == GeofenceUpdateResult::ACCEPTED)
            {
                outcome.result = GeofenceUpdateResult::ACCEPTED;
                outcome.detail = last_detail_;
                return outcome;
            }
            return accept(outcome, "geofence unchanged", true);
        }

        if (home_frame == nullptr || !home_frame->has_origin())
        {
            // Home 尚未凍結：舊圍籬也不可能生效，直接改為等待新內容。
            state_ = GeofenceState::PENDING;
            definition_ = std::move(definition);
            fence_ = Geofence{};
            return accept(outcome, "geofence pending planner Home", true);
        }

        Geofence candidate;
        const GeofenceUpdateResult build_result = build_planner_geofence(
            *home_frame, definition, candidate, detail);
        if (build_result != GeofenceUpdateResult::ACCEPTED)
        {
            return reject(build_result, detail, true);
        }
        state_ = GeofenceState::ACTIVE;
        definition_ = std::move(definition);
        fence_ = candidate;
        outcome.is_fence_changed = true;
        return accept(outcome, "geofence active", true);
    }

    /**
     * @brief planner Home 凍結時建立等待中的圍籬
     * @param home_frame 已凍結 Home 的 planner frame
     * @return 處理結果；沒有等待中的圍籬時兩個 changed 旗標皆為 false
     */
    GeofenceUpdateOutcome on_home_ready(const LocalNedFrame& home_frame)
    {
        GeofenceUpdateOutcome outcome;
        if (state_ != GeofenceState::PENDING)
        {
            return outcome;
        }
        std::string detail;
        Geofence candidate;
        const GeofenceUpdateResult build_result = build_planner_geofence(
            home_frame, definition_, candidate, detail);
        if (build_result != GeofenceUpdateResult::ACCEPTED)
        {
            // 等待中的內容無法建立：沒有舊圍籬可保留，回到無圍籬。
            state_ = GeofenceState::DISABLED;
            definition_ = GeofenceDefinition{};
            fence_ = Geofence{};
            return reject(build_result, detail, true);
        }
        state_ = GeofenceState::ACTIVE;
        fence_ = candidate;
        outcome.is_fence_changed = true;
        return accept(outcome, "geofence active", true);
    }

    /**
     * @brief 取得生效中的圍籬
     * @return ACTIVE 時為圍籬指標，其他狀態為 nullptr
     */
    const Geofence* active_fence() const
    {
        return state_ == GeofenceState::ACTIVE ? &fence_ : nullptr;
    }

    /** @brief 目前狀態 */
    GeofenceState state() const
    {
        return state_;
    }

    /** @brief 目前生效或等待中的圍籬內容；DISABLED 時為空 */
    const GeofenceDefinition& definition() const
    {
        return definition_;
    }

    /** @brief 最近一次處理的命令版本號 */
    uint32_t last_request_id() const
    {
        return last_request_id_;
    }

    /** @brief 最近一次處理的結果 */
    GeofenceUpdateResult last_result() const
    {
        return last_result_;
    }

    /** @brief 最近一次處理的英文說明 */
    const std::string& last_detail() const
    {
        return last_detail_;
    }

private:
    GeofenceUpdateOutcome accept(
        GeofenceUpdateOutcome outcome,
        const std::string& detail,
        const bool is_status_changed)
    {
        const bool is_result_changed =
            last_result_ != GeofenceUpdateResult::ACCEPTED ||
            last_detail_ != detail;
        last_result_ = GeofenceUpdateResult::ACCEPTED;
        last_detail_ = detail;
        outcome.result = GeofenceUpdateResult::ACCEPTED;
        outcome.detail = detail;
        outcome.is_status_changed = is_status_changed || is_result_changed ||
            outcome.is_fence_changed;
        return outcome;
    }

    GeofenceUpdateOutcome reject(
        const GeofenceUpdateResult result,
        const std::string& detail,
        const bool is_status_changed)
    {
        const bool is_result_changed =
            last_result_ != result || last_detail_ != detail;
        last_result_ = result;
        last_detail_ = detail;
        GeofenceUpdateOutcome outcome;
        outcome.result = result;
        outcome.detail = detail;
        outcome.is_status_changed = is_status_changed || is_result_changed;
        return outcome;
    }

    GeofenceState state_{GeofenceState::DISABLED};
    GeofenceDefinition definition_{};
    Geofence fence_{};
    uint32_t last_request_id_{0U};
    GeofenceUpdateResult last_result_{GeofenceUpdateResult::NONE};
    std::string last_detail_{};
};

}  // namespace ego_swarm
}  // namespace taiphoon
