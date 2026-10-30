/**
 * @file WaypointMissionExecutor.cpp
 * @brief 航點任務驗證、座標換算與狀態機實作
 * @author Taiphoon Agent Team
 * @date 2026-10-30
 */

#include "ego_swarm/WaypointMissionExecutor.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace taiphoon
{
namespace ego_swarm
{

namespace
{
/**
 * @brief 建立拒絕結果
 */
WaypointMissionDecision make_rejection(const WaypointMissionReason reason,
                                       const std::size_t index,
                                       const std::string& detail)
{
    WaypointMissionDecision decision;
    decision.is_accepted = false;
    decision.reason = reason;
    decision.index = static_cast<uint16_t>(
        index > std::numeric_limits<uint16_t>::max() ?
            std::numeric_limits<uint16_t>::max() : index);
    decision.detail = detail;
    return decision;
}

/**
 * @brief 建立接受結果
 */
WaypointMissionDecision make_acceptance()
{
    WaypointMissionDecision decision;
    decision.is_accepted = true;
    return decision;
}

/**
 * @brief 兩點三維距離
 */
double distance_3d(const Vector3& from, const Vector3& to)
{
    const double delta_north = to.at(0) - from.at(0);
    const double delta_east = to.at(1) - from.at(1);
    const double delta_down = to.at(2) - from.at(2);
    return std::sqrt(delta_north * delta_north + delta_east * delta_east +
                     delta_down * delta_down);
}

/**
 * @brief 兩點水平距離
 */
double horizontal_distance(const Vector3& from, const Vector3& to)
{
    const double delta_north = to.at(0) - from.at(0);
    const double delta_east = to.at(1) - from.at(1);
    return std::sqrt(delta_north * delta_north + delta_east * delta_east);
}
}  // namespace

WaypointMissionDecision validate_waypoint_mission_plan(
    const uint32_t mission_id,
    const uint8_t altitude_frame,
    const std::vector<MissionWaypointInput>& waypoints,
    const WaypointMissionLimits& limits)
{
    if (mission_id == 0U)
    {
        return make_rejection(WaypointMissionReason::INVALID_PLAN, 0U,
                              "mission_id must not be 0");
    }
    if (altitude_frame != WAYPOINT_ALTITUDE_PX4_LOCAL)
    {
        return make_rejection(WaypointMissionReason::INVALID_PLAN, 0U,
                              "unsupported altitude_frame");
    }
    if (waypoints.empty() || waypoints.size() > limits.max_waypoints)
    {
        return make_rejection(WaypointMissionReason::INVALID_PLAN, 0U,
                              "waypoint count out of range");
    }

    constexpr double MAX_LATITUDE_DEG = 90.0;
    constexpr double MAX_LONGITUDE_DEG = 180.0;
    for (std::size_t index = 0U; index < waypoints.size(); ++index)
    {
        const MissionWaypointInput& waypoint = waypoints.at(index);
        if (!std::isfinite(waypoint.latitude_deg) ||
            !std::isfinite(waypoint.longitude_deg) ||
            std::abs(waypoint.latitude_deg) > MAX_LATITUDE_DEG ||
            std::abs(waypoint.longitude_deg) > MAX_LONGITUDE_DEG)
        {
            return make_rejection(WaypointMissionReason::INVALID_PLAN, index,
                                  "invalid latitude or longitude");
        }
        if (!std::isfinite(waypoint.altitude_m) ||
            waypoint.altitude_m < limits.min_altitude_m ||
            waypoint.altitude_m > limits.max_altitude_m)
        {
            return make_rejection(WaypointMissionReason::INVALID_PLAN, index,
                                  "altitude out of range");
        }
        if (!std::isfinite(waypoint.dwell_s) || waypoint.dwell_s < 0.0 ||
            waypoint.dwell_s > limits.max_dwell_s)
        {
            return make_rejection(WaypointMissionReason::INVALID_PLAN, index,
                                  "dwell_s out of range");
        }
        // 本版不控制航點航向；有值代表 GCS 預期 Agent 會轉向，不能默默忽略。
        if (!std::isnan(waypoint.yaw_deg))
        {
            return make_rejection(WaypointMissionReason::INVALID_PLAN, index,
                                  "yaw_deg must be NaN");
        }
    }
    return make_acceptance();
}

bool convert_waypoints_to_planner_legs(
    const std::vector<MissionWaypointInput>& waypoints,
    const LocalNedFrame& planner_frame,
    const Vector3& planner_to_px4_ned_offset,
    std::vector<WaypointMissionLeg>& legs)
{
    legs.clear();
    if (!planner_frame.has_origin() ||
        !is_finite_vector(planner_to_px4_ned_offset))
    {
        return false;
    }

    std::vector<WaypointMissionLeg> converted;
    converted.reserve(waypoints.size());
    for (const MissionWaypointInput& waypoint : waypoints)
    {
        // 水平只取 North／East；高度以 PX4 local 換算，不使用 WGS84 高度。
        const GeoCoordinate coordinate{
            waypoint.latitude_deg, waypoint.longitude_deg,
            planner_frame.origin().altitude_m};
        Vector3 horizontal_ned{0.0, 0.0, 0.0};
        if (!planner_frame.to_local_ned(coordinate, horizontal_ned))
        {
            return false;
        }

        WaypointMissionLeg leg;
        leg.target_planner_ned = Vector3{
            horizontal_ned.at(0),
            horizontal_ned.at(1),
            -waypoint.altitude_m - planner_to_px4_ned_offset.at(2)};
        leg.dwell_s = waypoint.dwell_s;
        if (!is_finite_vector(leg.target_planner_ned))
        {
            return false;
        }
        converted.push_back(leg);
    }
    legs = std::move(converted);
    return true;
}

WaypointMissionDecision check_waypoint_mission_range(
    const std::vector<WaypointMissionLeg>& legs,
    const Vector3& current_planner_ned,
    const double max_range_m)
{
    if (!is_finite_vector(current_planner_ned) || !std::isfinite(max_range_m))
    {
        return make_rejection(WaypointMissionReason::NOT_READY, 0U,
                              "current position unavailable");
    }
    for (std::size_t index = 0U; index < legs.size(); ++index)
    {
        if (horizontal_distance(current_planner_ned,
                                legs.at(index).target_planner_ned) > max_range_m)
        {
            return make_rejection(WaypointMissionReason::OUT_OF_RANGE, index,
                                  "waypoint beyond mission_max_range_m");
        }
    }
    return make_acceptance();
}

WaypointMissionExecutor::WaypointMissionExecutor(
    const WaypointMissionLimits& limits)
    : limits_(limits)
{
}

void WaypointMissionExecutor::set_limits(const WaypointMissionLimits& limits)
{
    limits_ = limits;
}

const WaypointMissionLimits& WaypointMissionExecutor::limits() const
{
    return limits_;
}

WaypointMissionDecision WaypointMissionExecutor::load(
    const uint32_t mission_id,
    std::vector<WaypointMissionLeg> legs,
    const bool has_formation,
    const double now_s)
{
    if (is_running())
    {
        return make_rejection(WaypointMissionReason::BUSY, 0U,
                              "another mission is running");
    }
    if (has_formation)
    {
        return make_rejection(WaypointMissionReason::BUSY, 0U,
                              "formation is active");
    }
    if (mission_id == 0U || legs.empty())
    {
        return make_rejection(WaypointMissionReason::INVALID_PLAN, 0U,
                              "empty mission");
    }

    state_ = WaypointMissionState::LOADED;
    reason_ = WaypointMissionReason::NONE;
    mission_id_ = mission_id;
    legs_ = std::move(legs);
    current_index_ = 0U;
    detail_.clear();
    loaded_at_s_ = now_s;
    is_target_pending_ = false;
    return make_acceptance();
}

WaypointMissionDecision WaypointMissionExecutor::start(
    const uint32_t mission_id,
    const WaypointMissionStartConditions& conditions,
    const Vector3& current_planner_ned,
    const double now_s)
{
    expire_load_if_due(now_s);
    if (state_ != WaypointMissionState::LOADED || mission_id != mission_id_)
    {
        return make_rejection(WaypointMissionReason::ID_MISMATCH, 0U,
                              "no loaded mission with this mission_id");
    }
    if (!conditions.can_plan || !conditions.is_hovering ||
        !conditions.is_px4_offboard)
    {
        return make_rejection(WaypointMissionReason::NOT_HOVERING, 0U,
                              "Agent is not hovering in Offboard");
    }
    if (conditions.has_formation)
    {
        return make_rejection(WaypointMissionReason::BUSY, 0U,
                              "formation is active");
    }
    // 上傳到開始之間可能已移動，重新檢查距離上限。
    const WaypointMissionDecision range_decision = check_waypoint_mission_range(
        legs_, current_planner_ned, limits_.max_range_m);
    if (!range_decision.is_accepted)
    {
        return range_decision;
    }

    state_ = WaypointMissionState::RUNNING;
    reason_ = WaypointMissionReason::NONE;
    detail_.clear();
    begin_leg(0U, current_planner_ned, now_s);
    return make_acceptance();
}

bool WaypointMissionExecutor::stop(const WaypointMissionReason reason,
                                   const std::string& detail)
{
    if (is_running())
    {
        state_ = WaypointMissionState::ABORTED;
        reason_ = reason;
        detail_ = detail;
        is_target_pending_ = false;
        return true;
    }
    if (state_ == WaypointMissionState::LOADED)
    {
        // IDLE 依 WaypointMissionStatus 定義不帶任務與原因碼，只在 detail 留下說明。
        (void)reason;
        state_ = WaypointMissionState::IDLE;
        reason_ = WaypointMissionReason::NONE;
        mission_id_ = 0U;
        detail_ = detail;
        legs_.clear();
        current_index_ = 0U;
    }
    return false;
}

bool WaypointMissionExecutor::expire_load_if_due(const double now_s)
{
    if (state_ != WaypointMissionState::LOADED ||
        now_s - loaded_at_s_ < limits_.load_timeout_s)
    {
        return false;
    }
    state_ = WaypointMissionState::IDLE;
    reason_ = WaypointMissionReason::NONE;
    mission_id_ = 0U;
    detail_ = "load timeout";
    legs_.clear();
    current_index_ = 0U;
    return true;
}

std::optional<Vector3> WaypointMissionExecutor::due_target(const double now_s)
{
    if (state_ == WaypointMissionState::RUNNING && is_target_pending_ &&
        current_index_ < legs_.size())
    {
        is_target_pending_ = false;
        return legs_.at(current_index_).target_planner_ned;
    }

    if (state_ == WaypointMissionState::DWELLING && now_s >= dwell_until_s_ &&
        current_index_ + 1U < legs_.size())
    {
        const Vector3 previous_target = legs_.at(current_index_).target_planner_ned;
        state_ = WaypointMissionState::RUNNING;
        begin_leg(current_index_ + 1U, previous_target, now_s);
        is_target_pending_ = false;
        return legs_.at(current_index_).target_planner_ned;
    }
    return std::nullopt;
}

bool WaypointMissionExecutor::on_target_reached(const Vector3& final_target,
                                                const double now_s)
{
    if (state_ != WaypointMissionState::RUNNING || is_target_pending_ ||
        current_index_ >= legs_.size() || !is_finite_vector(final_target))
    {
        return false;
    }
    // 只接受本段目標的到達，避免其他來源設定的 Final Target 被誤算成航點。
    if (distance_3d(final_target, legs_.at(current_index_).target_planner_ned) >
        WAYPOINT_TARGET_MATCH_TOLERANCE_M)
    {
        return false;
    }

    if (current_index_ + 1U >= legs_.size())
    {
        state_ = WaypointMissionState::COMPLETED;
        reason_ = WaypointMissionReason::NONE;
        detail_.clear();
        return true;
    }
    state_ = WaypointMissionState::DWELLING;
    dwell_until_s_ = now_s + legs_.at(current_index_).dwell_s;
    return true;
}

bool WaypointMissionExecutor::is_leg_timed_out(const double now_s) const
{
    return state_ == WaypointMissionState::RUNNING &&
        now_s - leg_started_s_ > leg_timeout_s_;
}

bool WaypointMissionExecutor::is_active() const
{
    return state_ == WaypointMissionState::LOADED || is_running();
}

bool WaypointMissionExecutor::is_running() const
{
    return state_ == WaypointMissionState::RUNNING ||
        state_ == WaypointMissionState::DWELLING;
}

WaypointMissionStatusSnapshot WaypointMissionExecutor::snapshot() const
{
    WaypointMissionStatusSnapshot status;
    status.mission_id = mission_id_;
    status.state = state_;
    status.reason = reason_;
    status.current_index = static_cast<uint16_t>(current_index_);
    status.waypoint_count = static_cast<uint16_t>(legs_.size());
    status.detail = detail_;
    return status;
}

void WaypointMissionExecutor::begin_leg(const std::size_t index,
                                        const Vector3& leg_origin,
                                        const double now_s)
{
    current_index_ = index;
    leg_started_s_ = now_s;
    is_target_pending_ = true;
    double leg_length_m = 0.0;
    if (index < legs_.size() && is_finite_vector(leg_origin))
    {
        leg_length_m = distance_3d(leg_origin, legs_.at(index).target_planner_ned);
    }
    const double expected_speed_mps = limits_.min_expected_speed_mps > 0.0 ?
        limits_.min_expected_speed_mps : WaypointMissionLimits{}.min_expected_speed_mps;
    // 同儕仲裁、煞停與規劃失敗的等待時間都計入，逾時即停在目前位置。
    leg_timeout_s_ = limits_.leg_base_timeout_s + leg_length_m / expected_speed_mps;
}

}  // namespace ego_swarm
}  // namespace taiphoon
