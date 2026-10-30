
/**
 * @file BasicController.cpp
 * @brief 基礎飛行控制器實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 *
 * 實作 IBasicFlightController 介面，
 * 並重構為遵循 SOLID 單一職責原則，
 * 將每個 Publisher/Subscriber 初始化與狀態機處理分離為獨立函式。
 */

#include "controllers/BasicController.hpp"
#include "controllers/Px4HealthFlags.hpp"
#include "ego_swarm/BsplineTrajectory.hpp"
#include "utils/math_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <limits>

namespace taiphoon
{
namespace controllers
{

namespace
{
/// PX4 狀態新鮮度使用單調時鐘，避免對時或手動改時間造成誤判。
int64_t steady_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

constexpr uint32_t VEHICLE_CMD_DO_SET_MODE = 176U;
constexpr uint32_t VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20U;
constexpr uint32_t VEHICLE_CMD_NAV_LAND = 21U;
constexpr uint32_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400U;

/// 事件 detail 的格式化緩衝區大小
constexpr std::size_t EVENT_DETAIL_BUFFER_SIZE = 160U;

/// 秒換算奈秒
constexpr double NANOSECONDS_PER_SECOND = 1.0e9;
}  // namespace

// =============================================================================
// 建構子
// =============================================================================

BasicController::BasicController(rclcpp::Node* node, uint8_t target_system)
    : node_(node),
      target_system_(target_system)
{
    // L1-02：檢查指標是否為 nullptr
    if (node_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Node pointer is nullptr");
    }
}


// =============================================================================
// IBasicFlightController 介面實作
// =============================================================================

bool BasicController::initialize()
{
    // L1-02：檢查指標是否為 nullptr
    if (node_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot init: Node pointer is nullptr");
        return false;
    }


    // --- 依序建立各 Publisher (單一職責) ---
    if (!create_offboard_control_mode_publisher() ||
        !create_trajectory_setpoint_publisher() ||
        !create_vehicle_command_publisher())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Publisher creation failed");
        return false;
    }


    // --- 依序建立各 Subscriber (單一職責) ---
    if (!create_vehicle_command_ack_subscriber() ||
        !create_vehicle_status_subscriber() ||
        !create_vehicle_land_detected_subscriber())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Subscriber creation failed");
        return false;
    }

    // FailsafeFlags 只用於 AgentHealth，建立失敗不影響飛行控制。
    if (!create_failsafe_flags_subscriber())
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "FailsafeFlags subscriber unavailable; "
                    "health failsafe_flags_valid stays false");
    }

    is_initialized_.store(true);
    flight_state_.store(FlightState::IDLE);

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "BasicController initialized (target_system=%d)",
                target_system_);
    return true;
}

void BasicController::shutdown()
{
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "BasicController shutting down");

    stop_offboard_session();
    {
        std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
        has_frame_revision_ = false;
        timed_trajectory_frame_revision_ = 0U;
        timed_trajectory_epoch_start_ = rclcpp::Time(
            static_cast<int64_t>(0), RCL_ROS_TIME);
        has_timed_trajectory_epoch_ = false;
    }

    offboard_control_mode_pub_.reset();
    trajectory_setpoint_pub_.reset();
    vehicle_command_pub_.reset();
    vehicle_command_ack_sub_.reset();
    vehicle_status_sub_.reset();
    vehicle_land_detected_sub_.reset();
    failsafe_flags_sub_.reset();
    pending_px4_commands_.clear();

    is_initialized_.store(false);
    flight_state_.store(FlightState::IDLE);
}

bool BasicController::is_initialized() const
{
    return is_initialized_.load();
}

void BasicController::clear_timed_trajectory()
{
    std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
    timed_trajectory_ = TimedTrajectory{};
    has_timed_trajectory_ = false;
    trajectory_setpoint_active_.store(false);
}

bool BasicController::complete_timed_trajectory(const uint32_t trajectory_id)
{
    std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
    if (!has_timed_trajectory_ ||
        timed_trajectory_.trajectory_id != trajectory_id)
    {
        return false;
    }

    last_completed_trajectory_id_ = trajectory_id;
    has_completed_trajectory_ = true;
    timed_trajectory_ = TimedTrajectory{};
    has_timed_trajectory_ = false;
    trajectory_setpoint_active_.store(false);
    return true;
}

void BasicController::stop_offboard_session()
{
    offboard_prestream_count_.store(0U);
    offboard_mode_request_sent_.store(false);
    offboard_session_active_.store(false);
    offboard_cmd_echo_received_.store(false);
    offboard_cmd_rejected_.store(false);
    offboard_mode_request_time_ = rclcpp::Time(
        static_cast<int64_t>(0), RCL_ROS_TIME);
    // 排隊命令只屬於這一次 session；session 結束或進入失敗時一併取消。
    is_arm_queued_.store(false);
    is_takeoff_queued_.store(false);
    is_queued_takeoff_waiting_arm_ = false;
    trajectory_setpoint_pending_.store(false);
    // Offboard 中斷期間機頭可能被手動或 failsafe 改變；重新進入後從實際
    // 航向逼近目標，不可沿用中斷前的送出值造成瞬間跳轉。
    commanded_yaw_rad_ = std::numeric_limits<float>::quiet_NaN();
    has_last_setpoint_position_ = false;
    clear_timed_trajectory();
}

void BasicController::begin_offboard_session()
{
    stop_offboard_session();
    offboard_entry_failed_.store(false);
    offboard_start_time_ = node_->now();
    flight_state_.store(FlightState::OFFBOARD_INIT);
}

void BasicController::fail_offboard_entry(const char* reason)
{
    stop_offboard_session();
    offboard_entry_failed_.store(true);
    flight_state_.store(FlightState::ERROR);
    RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                 "Offboard entry failed: %s", reason);
}


// =============================================================================
// 命令介面實作
// =============================================================================

bool BasicController::arm()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot arm: not initialized");
        return false;
    }

    // 預檢未通過時 PX4 必定拒絕 ARM；先讓 GCS 知道原因，仍照常送出以取得 ack 結果。
    if (is_vehicle_status_fresh() && !pre_flight_checks_pass_.load())
    {
        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail),
                      "ARM requested while PX4 preflight checks fail "
                      "(latest_arming_reason=%u)",
                      static_cast<unsigned int>(latest_arming_reason_.load()));
        emit_event(safety::AgentEventCode::PX4_PREFLIGHT_FAILED, "ARM",
                   safety::PX4_RESULT_NONE, detail);
    }

    if (flight_state_.load() == FlightState::OFFBOARD_INIT)
    {
        // 直接送出 ARM 會讓狀態離開 OFFBOARD_INIT 而中斷 prestream。
        is_arm_queued_.store(true);
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "ARM queued until PX4 confirms Offboard (target_system=%d)",
                    target_system_);
        return true;
    }

    const bool is_px4_armed = arming_state_.load() ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    if (!offboard_session_active_.load() && !is_px4_armed)
    {
        // PX4 在地面停在 Hold 或缺少心跳的 Offboard 時無法直接解鎖；
        // 先建立 Offboard session，確認後再送 ARM。
        begin_offboard_session();
        is_arm_queued_.store(true);
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "ARM requested without Offboard session; entering "
                    "Offboard first (target_system=%d)", target_system_);
        return true;
    }

    return send_arm_now();
}

bool BasicController::send_arm_now()
{
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Sending arm command (target_system=%d)", target_system_);
    is_arm_rejected_.store(false);
    is_arm_ack_pending_.store(true);
    bool result = send_arm_command(true);
    if (result)
    {
        offboard_start_time_ = node_->now();
        flight_state_.store(FlightState::ARMED);
    }
    else
    {
        is_arm_ack_pending_.store(false);
    }
    return result;
}

bool BasicController::disarm()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot disarm: not initialized");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Sending disarm command (target_system=%d)", target_system_);
    // DISARM 與 ARM 共用 command 400；避免把 DISARM 的回應誤判為 ARM。
    is_arm_ack_pending_.store(false);
    bool result = send_arm_command(false);
    if (result)
    {
        stop_offboard_session();
        flight_state_.store(FlightState::IDLE);
        rtl_cmd_sent_ = false;
        land_cmd_sent_ = false;
    }
    return result;
}

bool BasicController::takeoff(float altitude_m)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot takeoff: not initialized");
        return false;
    }
    if (!utils::is_valid_float(altitude_m))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Takeoff altitude invalid (NaN or Inf)");
        return false;
    }
    if (altitude_m <= 1e-6F)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Takeoff altitude must be positive");
        return false;
    }

    const bool is_entering_offboard =
        flight_state_.load() == FlightState::OFFBOARD_INIT;
    const bool needs_ground_offboard_entry =
        !offboard_session_active_.load() && is_landed_.load();
    if (is_entering_offboard || needs_ground_offboard_entry)
    {
        if (needs_ground_offboard_entry && !is_entering_offboard)
        {
            begin_offboard_session();
        }
        queued_takeoff_altitude_m_ = altitude_m;
        is_queued_takeoff_waiting_arm_ = false;
        is_takeoff_queued_.store(true);
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "TAKEOFF %.2f m queued until PX4 confirms Offboard and "
                    "is armed (target_system=%d)", altitude_m, target_system_);
        return true;
    }

    return start_takeoff(altitude_m);
}

bool BasicController::start_takeoff(float altitude_m)
{
    target_altitude_m_.store(altitude_m);
    target_position_ned_ = current_position_ned_;
    target_position_ned_.at(2) = -altitude_m;
    // 地面起飛鎖定當下航向：舊行為在目標航向未設定時送出 yaw=0，起飛後
    // 機頭會轉向正北。空中重送 TAKEOFF 則保留目標航向，與 planner 一致。
    if (!utils::is_valid_float(target_yaw_rad_) || is_landed_.load())
    {
        lock_heading_to_current_yaw();
    }
    const auto takeoff_epoch_start = node_->now();
    flight_state_.store(FlightState::TAKEOFF);
    takeoff_start_time_ = takeoff_epoch_start;
    {
        std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
        // TAKEOFF 取得本機 PX4 setpoint 控制權。peer occupancy heartbeat
        // 仍會在 ROS2 peer table 中流通，但不能把起飛前的地面 hold
        // trajectory 帶進 HOVER 後重新接管本機控制。
        timed_trajectory_ = TimedTrajectory{};
        has_timed_trajectory_ = false;
        trajectory_setpoint_active_.store(false);
        timed_trajectory_epoch_start_ = takeoff_epoch_start;
        has_timed_trajectory_epoch_ = true;
    }
    trajectory_setpoint_pending_.store(true);
    altitude_log_counter_ = 0;
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Takeoff to %.2f meters, heading %.1f deg (target_system=%d)",
                altitude_m,
                static_cast<double>(target_yaw_rad_ / YAW_DEG_TO_RAD),
                target_system_);
    return true;
}

void BasicController::lock_heading_to_current_yaw()
{
    float heading_rad = current_yaw_rad_.load();
    if (!utils::is_valid_float(heading_rad))
    {
        heading_rad = 0.0F;
    }
    target_yaw_rad_ = wrap_yaw_rad(heading_rad);
    commanded_yaw_rad_ = target_yaw_rad_;
}

bool BasicController::hold_position(float duration_s)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot hold: not initialized");
        return false;
    }
    if (!utils::is_valid_float(duration_s))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Hold duration invalid (NaN or Inf)");
        return false;
    }
    if (duration_s <= 0.0F)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Hold duration must be positive");
        return false;
    }
    hover_duration_s_.store(duration_s);
    target_position_ned_ = current_position_ned_;
    clear_timed_trajectory();
    flight_state_.store(FlightState::HOVER);
    trajectory_setpoint_pending_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Hovering for %.2f seconds", duration_s);
    return true;
}

bool BasicController::land()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot land: not initialized");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Landing (target_system=%d)", target_system_);
    stop_offboard_session();
    flight_state_.store(FlightState::LAND);
    land_cmd_sent_ = false;
    return true;
}

bool BasicController::execute_takeoff_land_sequence(float altitude_m, float hover_duration_s)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot execute sequence: not initialized");
        return false;
    }
    if (!utils::is_valid_float(altitude_m) || !utils::is_valid_float(hover_duration_s))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Sequence parameter invalid (NaN or Inf)");
        return false;
    }
    if (altitude_m <= 0.0F || hover_duration_s < 0.0F)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Sequence parameters invalid");
        return false;
    }
    target_altitude_m_.store(altitude_m);
    hover_duration_s_.store(hover_duration_s);
    begin_offboard_session();
    arm_cmd_echo_received_.store(false);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Takeoff-land sequence started: alt=%.2f m, hover=%.2f s",
                altitude_m, hover_duration_s);
    return true;
}

const char* BasicController::get_state() const
{
    switch (flight_state_.load())
    {
        case FlightState::IDLE:          return "IDLE";
        case FlightState::OFFBOARD_INIT: return "OFFBOARD_INIT";
        case FlightState::ARMED:         return "ARMED";
        case FlightState::TAKEOFF:       return "TAKEOFF";
        case FlightState::HOVER:         return "HOVER";
        case FlightState::RTL:           return "RTL";
        case FlightState::LAND:          return "LAND";
        case FlightState::OFFBOARD_ACTIVE: return "OFFBOARD_ACTIVE";
        case FlightState::MOVING:        return "MOVING";
        case FlightState::POST_RTL:       return "POST_RTL";
        case FlightState::ERROR:         return "ERROR";
        default:                         return "UNKNOWN";
    }
}

FlightState BasicController::get_flight_state() const
{
    return flight_state_.load();
}

FlightStateObservation BasicController::get_flight_state_observation() const
{
    FlightStateObservation observation;
    if (node_ == nullptr)
    {
        return observation;
    }

    const int64_t now_ns = steady_now_ns();
    const bool has_recent_status = is_px4_message_fresh(
        last_vehicle_status_ns_.load(), now_ns, PX4_STATE_TIMEOUT_NS);
    const bool has_recent_land_detection = is_px4_message_fresh(
        last_land_detected_ns_.load(), now_ns, PX4_STATE_TIMEOUT_NS);

    observation.is_px4_state_valid =
        has_recent_status && has_recent_land_detection;
    observation.is_armed = arming_state_.load() ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    observation.is_landed = is_landed_.load();
    observation.px4_nav_state = px4_nav_state_.load();
    observation.current_altitude_m = current_altitude_m_.load();
    observation.target_altitude_m = target_altitude_m_.load();

    float delta_x = current_position_ned_[0] - target_position_ned_[0];
    float delta_y = current_position_ned_[1] - target_position_ned_[1];
    float delta_z = current_position_ned_[2] - target_position_ned_[2];
    observation.target_distance_m = std::sqrt(
        delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    if (flight_state_.load() == FlightState::TAKEOFF)
    {
        observation.takeoff_elapsed_s =
            (node_->now() - takeoff_start_time_).seconds();
    }

    return observation;
}


// =============================================================================
// 高階命令方法實作
// =============================================================================

bool BasicController::switch_to_rtl_mode()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot switch to RTL: not initialized");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Switching to RTL mode (target_system=%d)", target_system_);
    stop_offboard_session();
    flight_state_.store(FlightState::RTL);
    rtl_cmd_sent_ = false;
    return true;
}

bool BasicController::switch_to_land_mode()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot switch to LAND: not initialized");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Switching to LAND mode (target_system=%d)", target_system_);
    stop_offboard_session();
    flight_state_.store(FlightState::LAND);
    land_cmd_sent_ = false;
    return true;
}

bool BasicController::switch_to_offboard_mode()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot switch to OFFBOARD: not initialized");
        return false;
    }
    begin_offboard_session();
    arm_cmd_echo_received_.store(false);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Switching to OFFBOARD mode (target_system=%d)", target_system_);
    return true;
}

bool BasicController::switch_to_hold_mode()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot switch to HOLD: not initialized");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Switching to HOLD mode (target_system=%d)", target_system_);
    bool result = send_hold_mode_command();
    stop_offboard_session();
    if (result)
    {
        flight_state_.store(FlightState::IDLE);
    }
    return result;
}

bool BasicController::move_forward(float distance_m)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot move: not initialized");
        return false;
    }
    if (!utils::is_valid_float(distance_m))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Move distance invalid (NaN or Inf)");
        return false;
    }
    // Body → NED: nose direction
    float yaw = current_yaw_rad_.load();
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    target_position_ned_[0] = current_position_ned_[0] + distance_m * cos_yaw;
    target_position_ned_[1] = current_position_ned_[1] + distance_m * sin_yaw;
    target_position_ned_[2] = current_position_ned_[2];
    flight_state_.store(FlightState::MOVING);
    trajectory_setpoint_pending_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Moving forward %.2f m (target_system=%d)", distance_m, target_system_);
    return true;
}

bool BasicController::move_right(float distance_m)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot move: not initialized");
        return false;
    }
    if (!utils::is_valid_float(distance_m))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Move distance invalid (NaN or Inf)");
        return false;
    }
    // Body → NED: right direction
    float yaw = current_yaw_rad_.load();
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    target_position_ned_[0] = current_position_ned_[0] + distance_m * (-sin_yaw);
    target_position_ned_[1] = current_position_ned_[1] + distance_m * cos_yaw;
    target_position_ned_[2] = current_position_ned_[2];
    flight_state_.store(FlightState::MOVING);
    trajectory_setpoint_pending_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Moving right %.2f m (target_system=%d)", distance_m, target_system_);
    return true;
}


// =============================================================================

bool BasicController::move_vertical(float delta_m)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot move vertically: not initialized");
        return false;
    }
    if (!utils::is_valid_float(delta_m))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Vertical move delta invalid (NaN or Inf)");
        return false;
    }

    FlightState current_state = flight_state_.load();
    if (current_state != FlightState::HOVER &&
        current_state != FlightState::TAKEOFF &&
        current_state != FlightState::MOVING)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Cannot move vertically: not in flight state (current=%d)",
                    static_cast<int>(current_state));
        return false;
    }

    // target_position_ned_[2] is NED Z (positive = down)
    // delta_m > 0 = move up → NED Z decreases
    float new_z = target_position_ned_[2] - delta_m;

    // Clamp: don't go below minimum altitude (1m above ground)
    float new_alt = -new_z;
    if (new_alt < ALTITUDE_MIN_M)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Vertical move clamped: target %.2f m below minimum %.2f m, using minimum",
                    new_alt, ALTITUDE_MIN_M);
        new_z = -ALTITUDE_MIN_M;
    }

    target_position_ned_[2] = new_z;
    trajectory_setpoint_pending_.store(true);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Vertical move %.2f m, new target NED Z: %.2f (target_system=%d)",
                static_cast<double>(delta_m), static_cast<double>(new_z), target_system_);
    return true;
}


bool BasicController::rotate_yaw(float degrees)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot rotate: not initialized");
        return false;
    }
    if (!utils::is_valid_float(degrees))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Yaw angle invalid (NaN or Inf)");
        return false;
    }

    FlightState current_state = flight_state_.load();
    if (current_state != FlightState::HOVER &&
        current_state != FlightState::TAKEOFF &&
        current_state != FlightState::MOVING)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Cannot rotate: not in flight state (current=%d)",
                    static_cast<int>(current_state));
        return false;
    }

    const float delta_rad = degrees * YAW_DEG_TO_RAD;

    // YAW 為相對目前航向的命令。PX4 NED yaw 的正方向是順時針，
    // 與 move_forward/move_right 使用的 NED 幾何一致，因此使用 +=。
    // 累加在最終目標而非逼近中的送出值，與 planner 的 target yaw 同基準。
    if (!utils::is_valid_float(target_yaw_rad_))
    {
        target_yaw_rad_ = current_yaw_rad_.load();
        if (!utils::is_valid_float(target_yaw_rad_))
        {
            target_yaw_rad_ = 0.0F;
        }
    }
    target_yaw_rad_ = wrap_yaw_rad(target_yaw_rad_ + delta_rad);
    // 不設定 pending：未到達目標時 should_publish_trajectory_setpoint()
    // 會逐週期重送，位置沿用上一筆已送出的 setpoint。

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Yaw rotation %.1f deg, target yaw: %.1f deg at %.1f deg/s "
                "(target_system=%d)",
                degrees, target_yaw_rad_ / YAW_DEG_TO_RAD,
                get_yaw_rate_limit_deg_s(), target_system_);
    return true;
}

bool BasicController::set_absolute_yaw(float degrees)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot set absolute yaw: not initialized");
        return false;
    }
    if (!utils::is_valid_float(degrees))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Absolute yaw angle invalid (NaN or Inf)");
        return false;
    }

    target_yaw_rad_ = wrap_yaw_rad(degrees * YAW_DEG_TO_RAD);

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Absolute yaw target %.1f deg at %.1f deg/s (target_system=%d)",
                target_yaw_rad_ / YAW_DEG_TO_RAD,
                get_yaw_rate_limit_deg_s(), target_system_);
    return true;
}

bool BasicController::set_yaw_rate_limit(
    const float rate_deg_s, const float control_period_s)
{
    if (!utils::is_valid_float(rate_deg_s) ||
        !utils::is_valid_float(control_period_s) ||
        static_cast<double>(rate_deg_s) < MIN_YAW_RATE_LIMIT_DEG_S ||
        static_cast<double>(rate_deg_s) > MAX_YAW_RATE_LIMIT_DEG_S ||
        control_period_s <= 0.0F)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Yaw rate limit %.3f deg/s or control period %.3f s invalid; "
                     "keeping %.1f deg/s",
                     static_cast<double>(rate_deg_s),
                     static_cast<double>(control_period_s),
                     static_cast<double>(get_yaw_rate_limit_deg_s()));
        return false;
    }

    yaw_rate_limit_rad_s_ = rate_deg_s * YAW_DEG_TO_RAD;
    yaw_slew_period_s_ = control_period_s;
    return true;
}

float BasicController::get_yaw_rate_limit_deg_s() const
{
    return yaw_rate_limit_rad_s_ / YAW_DEG_TO_RAD;
}

bool BasicController::is_yaw_slew_pending() const
{
    if (!utils::is_valid_float(target_yaw_rad_))
    {
        return false;
    }
    if (!utils::is_valid_float(commanded_yaw_rad_))
    {
        return true;
    }
    return is_yaw_short_of_target(commanded_yaw_rad_, target_yaw_rad_);
}

YawSlewStep BasicController::advance_commanded_yaw()
{
    if (!utils::is_valid_float(commanded_yaw_rad_))
    {
        const float current_yaw = current_yaw_rad_.load();
        commanded_yaw_rad_ = utils::is_valid_float(current_yaw)
            ? wrap_yaw_rad(current_yaw)
            : 0.0F;
    }
    // 目標未設定時鎖定目前送出航向；舊行為退回 0 會讓機頭轉向正北。
    if (!utils::is_valid_float(target_yaw_rad_))
    {
        target_yaw_rad_ = commanded_yaw_rad_;
    }

    YawSlewStep step = step_yaw_toward(
        commanded_yaw_rad_, target_yaw_rad_,
        yaw_rate_limit_rad_s_, yaw_slew_period_s_);
    if (!step.is_valid)
    {
        // 正常情況不會發生（輸入皆已檢查）；保持目前送出航向且不給前饋。
        step.yaw_rad = commanded_yaw_rad_;
        step.yawspeed_rad_s = 0.0F;
        step.is_converged = true;
        step.is_valid = true;
    }
    commanded_yaw_rad_ = step.yaw_rad;
    return step;
}

bool BasicController::set_ned_target(float x, float y, float z, float yaw_deg)
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot set NED target: not initialized");
        return false;
    }
    if (!utils::is_valid_float(x) || !utils::is_valid_float(y) ||
        !utils::is_valid_float(z))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "NED target values invalid (NaN or Inf)");
        return false;
    }

    target_position_ned_[0] = x;
    target_position_ned_[1] = y;
    target_position_ned_[2] = z;

    if (!std::isnan(yaw_deg) && utils::is_valid_float(yaw_deg))
    {
        (void)set_absolute_yaw(yaw_deg);
    }

    flight_state_.store(FlightState::MOVING);
    trajectory_setpoint_pending_.store(true);

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "NED target set: (%.2f, %.2f, %.2f) yaw=%.1f deg (target_system=%d)",
                static_cast<double>(x), static_cast<double>(y),
                static_cast<double>(z), static_cast<double>(yaw_deg),
                target_system_);
    return true;
}

// 公開輔助方法
// =============================================================================

void BasicController::update_odometry(const px4_msgs::msg::VehicleOdometry& msg)
{
    if (utils::is_valid_float(msg.position[0]) &&
        utils::is_valid_float(msg.position[1]) &&
        utils::is_valid_float(msg.position[2]))
    {
        current_altitude_m_.store(-msg.position[2]);
        current_position_ned_[0] = msg.position[0];
        current_position_ned_[1] = msg.position[1];
        current_position_ned_[2] = msg.position[2];
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Ignoring odometry position containing NaN or Inf");
    }

    // Extract yaw from orientation quaternion (NED convention)
    // q = [w, x, y, z], yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²))
    float w = msg.q[0], x = msg.q[1], y = msg.q[2], z = msg.q[3];
    if (utils::is_valid_float(w) && utils::is_valid_float(x) &&
        utils::is_valid_float(y) && utils::is_valid_float(z))
    {
        float yaw_rad = std::atan2(2.0F * (w * z + x * y),
                                   1.0F - 2.0F * (y * y + z * z));
        if (utils::is_valid_float(yaw_rad))
        {
            current_yaw_rad_.store(yaw_rad);
        }
    }
}
// =============================================================================
// Publisher 初始化 (單一職責原則)
// =============================================================================

bool BasicController::create_offboard_control_mode_publisher()
{
    offboard_control_mode_pub_ = node_->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort());
    if (offboard_control_mode_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create OffboardControlMode publisher");
        return false;
    }
    return true;
}

bool BasicController::create_trajectory_setpoint_publisher()
{
    trajectory_setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort());
    if (trajectory_setpoint_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create TrajectorySetpoint publisher");
        return false;
    }
    return true;
}

bool BasicController::create_vehicle_command_publisher()
{
    vehicle_command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
        "fmu/in/vehicle_command", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort());
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create VehicleCommand publisher");
        return false;
    }
    return true;
}


// =============================================================================
// Subscriber 初始化 (單一職責原則)
// =============================================================================

bool BasicController::create_vehicle_command_ack_subscriber()
{
    vehicle_command_ack_sub_ = node_->create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "fmu/out/vehicle_command_ack_v1", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort(),
        std::bind(&BasicController::vehicle_command_ack_cb, this, std::placeholders::_1));
    if (vehicle_command_ack_sub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create VehicleCommandAck subscriber");
        return false;
    }
    return true;
}

bool BasicController::create_vehicle_status_subscriber()
{
    vehicle_status_sub_ = node_->create_subscription<px4_msgs::msg::VehicleStatus>(
        "fmu/out/vehicle_status_v4", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort(),
        std::bind(&BasicController::vehicle_status_cb, this, std::placeholders::_1));
    if (vehicle_status_sub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create VehicleStatus subscriber");
        return false;
    }
    return true;
}

bool BasicController::create_vehicle_land_detected_subscriber()
{
    vehicle_land_detected_sub_ =
        node_->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "fmu/out/vehicle_land_detected", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort(),
            std::bind(&BasicController::vehicle_land_detected_cb, this,
                      std::placeholders::_1));
    if (vehicle_land_detected_sub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Failed to create VehicleLandDetected subscriber");
        return false;
    }
    return true;
}

bool BasicController::create_failsafe_flags_subscriber()
{
    failsafe_flags_sub_ =
        node_->create_subscription<px4_msgs::msg::FailsafeFlags>(
            "fmu/out/failsafe_flags", rclcpp::QoS(DEFAULT_QOS_DEPTH).best_effort(),
            std::bind(&BasicController::failsafe_flags_cb, this,
                      std::placeholders::_1));
    return failsafe_flags_sub_ != nullptr;
}


// =============================================================================
// 控制命令發布 (主要計時器回呼)
// =============================================================================

void BasicController::publish_control_commands()
{
    if (!is_initialized_.load())
    {
        return;
    }

    // 事件處理放在控制迴圈：PX4 訂閱回呼只暫存資料，不做發布。
    process_command_acks();
    expire_pending_px4_commands();
    update_px4_link_events();

    // Offboard session 成功後，任何外部模式切換都必須先關閉兩個 PX4
    // setpoint publisher。OFFBOARD_INIT 期間的非 Offboard nav_state 是
    // 預期狀態，不能在尚未送出模式命令前誤判為外部離開。
    // 狀態只是暫時未到達時不立即停止：無線連線偶發遺失會讓 PX4 觸發
    // Offboard 遺失 failsafe，比繼續輸出更危險。
    if (offboard_session_active_.load())
    {
        const auto decision = evaluate_offboard_output(
            last_vehicle_status_ns_.load(), px4_nav_state_.load(),
            steady_now_ns(), PX4_STATE_TIMEOUT_NS);
        if (decision != OffboardOutputDecision::CONTINUE)
        {
            if (decision == OffboardOutputDecision::STOP_LEFT_OFFBOARD)
            {
                RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                            "PX4 left Offboard (nav_state=%u); "
                            "stopping Offboard output",
                            static_cast<unsigned int>(px4_nav_state_.load()));
                char detail[EVENT_DETAIL_BUFFER_SIZE];
                std::snprintf(detail, sizeof(detail),
                              "PX4 left Offboard (nav_state=%u); "
                              "Agent stopped Offboard output",
                              static_cast<unsigned int>(px4_nav_state_.load()));
                emit_event(safety::AgentEventCode::OFFBOARD_LOST, "",
                           safety::PX4_RESULT_NONE, detail);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                            "VehicleStatus not received for more than %d ms; "
                            "stopping Offboard output",
                            PX4_STATE_TIMEOUT_MS);
            }
            stop_offboard_session();
            flight_state_.store(FlightState::IDLE);
            has_offboard_stopped_ = true;
        }
    }

    handle_arm_rejection();
    update_command_state_from_observation();

    if (flight_state_.load() == FlightState::OFFBOARD_INIT)
    {
        handle_offboard_init_state();
    }
    execute_queued_commands();

    // 這些狀態的 VehicleCommand 必須在輸出閘門 return 前處理，確保
    // RTL/LAND/HOLD 的模式命令不會因停止 setpoint 而被跳過。
    auto current_state = flight_state_.load();
    if (current_state == FlightState::RTL)
    {
        handle_rtl_state();
    }
    else if (current_state == FlightState::LAND)
    {
        handle_land_state();
    }
    else if (current_state == FlightState::POST_RTL)
    {
        handle_post_rtl_state();
    }
    park_in_hold_when_disarmed_in_offboard();

    current_state = flight_state_.load();
    if (!should_publish_offboard_control_mode() ||
        offboard_control_mode_pub_ == nullptr)
    {
        return;
    }

    // 由 PX4 以接收時刻蓋章，心跳新鮮度只取決於 PX4 是否收到，
    // 不受 Agent 與 PX4 時鐘偏移估計影響。
    auto mode_msg = px4_msgs::msg::OffboardControlMode();
    mode_msg.timestamp = PX4_INBOUND_TIMESTAMP_US;
    mode_msg.position = true;

    // OffboardControlMode 是獨立的 proof-of-life 心跳；TAKEOFF 前、ARM 後
    // 以及 HOVER 狀態都只發布這一筆訊息。
    offboard_control_mode_pub_->publish(mode_msg);

    if (current_state == FlightState::OFFBOARD_INIT)
    {
        const uint32_t completed_heartbeats =
            offboard_prestream_count_.fetch_add(1U) + 1U;
        if (completed_heartbeats >= OFFBOARD_PRESTREAM_COUNT &&
            !offboard_mode_request_sent_.load())
        {
            // 先建立 request 狀態再發布 command，避免多執行緒 executor 在
            // publish 期間收到 ACK 時被誤判為非本次 Offboard request。
            offboard_mode_request_sent_.store(true);
            offboard_mode_request_time_ = node_->now();
            if (!send_offboard_mode_command())
            {
                offboard_mode_request_sent_.store(false);
                fail_offboard_entry("unable to publish DO_SET_MODE");
                emit_event(safety::AgentEventCode::COMMAND_FAILED,
                           "MODE_OFFBOARD", safety::PX4_RESULT_NONE,
                           "Offboard entry failed: unable to publish DO_SET_MODE");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                        "Published %u Offboard prestream heartbeats; "
                        "sent one mode request",
                        completed_heartbeats);
        }
    }

    if (!should_publish_trajectory_setpoint() ||
        trajectory_setpoint_pub_ == nullptr)
    {
        return;
    }

    // 只因偏航逼近而重送時，位置必須維持上一筆已送出的 setpoint；
    // 狀態處理函式使用的 target_position_ned_ 在軌跡完成後可能仍是舊目標。
    const bool is_yaw_only_refresh =
        !trajectory_setpoint_pending_.load() &&
        !trajectory_setpoint_active_.load() &&
        has_last_setpoint_position_;

    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = PX4_INBOUND_TIMESTAMP_US;
    const auto hold_position = get_safe_hold_position();
    msg.position[0] = hold_position[0];
    msg.position[1] = hold_position[1];
    msg.position[2] = hold_position[2];
    for (std::size_t index = 0U; index < 3U; ++index)
    {
        msg.velocity[index] = 0.0F;
        msg.acceleration[index] = 0.0F;
        msg.jerk[index] = 0.0F;
    }
    msg.yawspeed = 0.0F;

    switch (current_state)
    {
        case FlightState::TAKEOFF:
            handle_takeoff_state(msg);
            break;
        case FlightState::HOVER:
            handle_hover_state(msg);
            break;
        case FlightState::MOVING:
            handle_moving_state(msg);
            break;
        case FlightState::IDLE:
        case FlightState::OFFBOARD_INIT:
        case FlightState::OFFBOARD_ACTIVE:
        case FlightState::ARMED:
        case FlightState::RTL:
        case FlightState::LAND:
        case FlightState::POST_RTL:
        case FlightState::ERROR:
        default:
            break;
    }

    bool trajectory_completed = false;
    uint32_t sampled_trajectory_id = 0U;
    if (trajectory_setpoint_active_.load())
    {
        {
            std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
            if (has_timed_trajectory_)
            {
                sampled_trajectory_id = timed_trajectory_.trajectory_id;
            }
        }

        std::array<float, 3> sampled_position = get_safe_hold_position();
        std::array<float, 3> sampled_velocity{0.0F, 0.0F, 0.0F};
        std::array<float, 3> sampled_acceleration{0.0F, 0.0F, 0.0F};
        const auto sample_result = sample_timed_trajectory(
            node_->now(), sampled_position, sampled_velocity,
            sampled_acceleration);

        if (sample_result != TimedTrajectorySampleResult::VALID &&
            sample_result != TimedTrajectorySampleResult::COMPLETED)
        {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("BasicController"), *node_->get_clock(),
                2000,
                "Timed trajectory unavailable (result=%d); "
                "holding the safe trajectory fallback",
                static_cast<int>(sample_result));
        }

        msg.position[0] = sampled_position[0];
        msg.position[1] = sampled_position[1];
        msg.position[2] = sampled_position[2];
        for (std::size_t index = 0U; index < 3U; ++index)
        {
            msg.velocity[index] = sampled_velocity[index];
            msg.acceleration[index] = sampled_acceleration[index];
        }
        trajectory_completed =
            sample_result == TimedTrajectorySampleResult::COMPLETED;
    }

    if (is_yaw_only_refresh)
    {
        msg.position[0] = last_setpoint_position_ned_[0];
        msg.position[1] = last_setpoint_position_ned_[1];
        msg.position[2] = last_setpoint_position_ned_[2];
    }

    // PX4 Offboard 不套用 MPC_YAWRAUTO_MAX，因此由此處限速逼近並給前饋。
    const YawSlewStep yaw_step = advance_commanded_yaw();
    msg.yaw = yaw_step.yaw_rad;
    msg.yawspeed = yaw_step.yawspeed_rad_s;

    for (std::size_t index = 0U; index < 3U; ++index)
    {
        if (!utils::is_valid_float(msg.position[index]))
        {
            msg.position[index] = hold_position[index];
        }
        if (!utils::is_valid_float(msg.velocity[index]))
        {
            msg.velocity[index] = 0.0F;
        }
        if (!utils::is_valid_float(msg.acceleration[index]))
        {
            msg.acceleration[index] = 0.0F;
        }
        if (!utils::is_valid_float(msg.jerk[index]))
        {
            msg.jerk[index] = 0.0F;
        }
    }
    if (!utils::is_valid_float(msg.yaw))
    {
        // 防呆值使用目前航向而非 0，避免機頭被拉向正北。
        msg.yaw = current_yaw_rad_.load();
        if (!utils::is_valid_float(msg.yaw))
        {
            msg.yaw = 0.0F;
        }
        msg.yawspeed = 0.0F;
        commanded_yaw_rad_ = msg.yaw;
    }
    if (!utils::is_valid_float(msg.yawspeed))
    {
        msg.yawspeed = 0.0F;
    }

    trajectory_setpoint_pub_->publish(msg);
    trajectory_setpoint_pending_.store(false);
    last_setpoint_position_ned_ = {msg.position[0], msg.position[1], msg.position[2]};
    has_last_setpoint_position_ = true;

    if (trajectory_completed &&
        complete_timed_trajectory(sampled_trajectory_id))
    {
        flight_state_.store(FlightState::HOVER);
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "Timed trajectory completed at endpoint; entering HOVER");
    }
}

float BasicController::get_current_altitude() const
{
    return current_altitude_m_.load();
}

std::array<float, 3> BasicController::get_current_position_ned() const
{
    return current_position_ned_;
}

std::array<float, 3> BasicController::get_target_position_ned() const
{
    return target_position_ned_;
}

bool BasicController::validate_timed_trajectory(
    const TimedTrajectory& trajectory)
{
    if (!trajectory.valid ||
        !std::isfinite(trajectory.knot_interval_s) ||
        trajectory.knot_interval_s <= 0.0F ||
        !std::isfinite(trajectory.duration_s) ||
        trajectory.duration_s <= 0.0F)
    {
        return false;
    }

    const std::size_t degree = static_cast<std::size_t>(trajectory.order);
    if (degree == 0U || degree >= trajectory.control_points.size())
    {
        return false;
    }

    for (const auto& point : trajectory.control_points)
    {
        if (!std::isfinite(point[0]) ||
            !std::isfinite(point[1]) ||
            !std::isfinite(point[2]))
        {
            return false;
        }
    }

    return true;
}

bool BasicController::is_timed_trajectory_allowed_in_current_epoch(
    const TimedTrajectory& trajectory) const
{
    if (!has_timed_trajectory_epoch_ ||
        trajectory.start_time.get_clock_type() !=
            timed_trajectory_epoch_start_.get_clock_type())
    {
        return true;
    }

    // 起飛前的軌跡即使在 TAKEOFF 結束後才被重送，也不能重新取得
    // BasicController 的本機控制權。
    return trajectory.start_time >= timed_trajectory_epoch_start_;
}

bool BasicController::update_timed_trajectory(
    const TimedTrajectory& trajectory)
{
    if (!validate_timed_trajectory(trajectory))
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Rejecting invalid timed trajectory %u; keeping last valid trajectory",
                    trajectory.trajectory_id);
        return false;
    }

    std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);

    if (!is_timed_trajectory_allowed_in_current_epoch(trajectory))
    {
        RCLCPP_DEBUG(
            rclcpp::get_logger("BasicController"),
            "Ignoring timed trajectory %u: TAKEOFF owns local control or "
            "trajectory predates the current TAKEOFF epoch",
            trajectory.trajectory_id);
        return false;
    }

    if (has_completed_trajectory_ &&
        trajectory.trajectory_id <= last_completed_trajectory_id_)
    {
        RCLCPP_DEBUG(
            rclcpp::get_logger("BasicController"),
            "Ignoring completed or stale timed trajectory %u; "
            "last completed trajectory is %u",
            trajectory.trajectory_id, last_completed_trajectory_id_);
        return false;
    }

    // frame_revision 是座標系生命週期 token。版本切換時先清除舊座標的
    // 軌跡，並要求下一筆相同版本資料才啟用，避免跨 frame 跳接。
    if (has_frame_revision_ &&
        trajectory.frame_revision != timed_trajectory_frame_revision_)
    {
        const uint32_t previous_frame_revision =
            timed_trajectory_frame_revision_;
        if (trajectory.frame_revision < timed_trajectory_frame_revision_)
        {
            RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                        "Rejecting stale timed trajectory frame revision %u; "
                        "current revision is %u",
                        trajectory.frame_revision,
                        timed_trajectory_frame_revision_);
            return false;
        }

        if (trajectory.frame_revision > timed_trajectory_frame_revision_)
        {
            timed_trajectory_frame_revision_ = trajectory.frame_revision;
        }
        has_timed_trajectory_ = false;
        trajectory_setpoint_active_.store(false);
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Timed trajectory frame revision changed from %u to %u; "
                    "holding until the next trajectory in the new frame",
                    previous_frame_revision,
                    trajectory.frame_revision);
        return false;
    }

    if (!has_frame_revision_)
    {
        timed_trajectory_frame_revision_ = trajectory.frame_revision;
        has_frame_revision_ = true;
    }

    timed_trajectory_ = trajectory;
    has_timed_trajectory_ = true;
    trajectory_setpoint_active_.store(true);
    return true;
}

TimedTrajectorySampleResult BasicController::sample_timed_trajectory(
    const rclcpp::Time& sample_time,
    std::array<float, 3>& position) const
{
    std::array<float, 3> velocity{0.0F, 0.0F, 0.0F};
    std::array<float, 3> acceleration{0.0F, 0.0F, 0.0F};
    return sample_timed_trajectory(
        sample_time, position, velocity, acceleration);
}

TimedTrajectorySampleResult BasicController::sample_timed_trajectory(
    const rclcpp::Time& sample_time,
    std::array<float, 3>& position,
    std::array<float, 3>& velocity,
    std::array<float, 3>& acceleration) const
{
    position = {0.0F, 0.0F, 0.0F};
    velocity = {0.0F, 0.0F, 0.0F};
    acceleration = {0.0F, 0.0F, 0.0F};

    std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
    if (!has_timed_trajectory_)
    {
        return TimedTrajectorySampleResult::NO_TRAJECTORY;
    }

    const auto& trajectory = timed_trajectory_;
    if (!validate_timed_trajectory(trajectory))
    {
        return TimedTrajectorySampleResult::INVALID;
    }

    if (sample_time.get_clock_type() != trajectory.start_time.get_clock_type())
    {
        return TimedTrajectorySampleResult::INVALID;
    }

    const double elapsed_s = (sample_time - trajectory.start_time).seconds();
    if (!std::isfinite(elapsed_s))
    {
        return TimedTrajectorySampleResult::INVALID;
    }
    if (elapsed_s < 0.0)
    {
        return TimedTrajectorySampleResult::NOT_STARTED;
    }
    if (elapsed_s >= static_cast<double>(trajectory.duration_s))
    {
        // 完成後不可回退到 current_position_ned_，否則到達目標後的
        // setpoint 會追著 odometry 漂移。以最後控制點作為完成 endpoint。
        position = trajectory.control_points.back();
        return TimedTrajectorySampleResult::COMPLETED;
    }

    const std::size_t degree = static_cast<std::size_t>(trajectory.order);
    const double span_count = static_cast<double>(
        trajectory.control_points.size() - degree);
    const double effective_knot_interval_s =
        static_cast<double>(trajectory.duration_s) / span_count;
    if (!std::isfinite(effective_knot_interval_s) ||
        effective_knot_interval_s <= 0.0)
    {
        return TimedTrajectorySampleResult::INVALID;
    }

    std::vector<ego_swarm::Vector3> control_points;
    control_points.reserve(trajectory.control_points.size());
    for (const auto& point : trajectory.control_points)
    {
        control_points.push_back(ego_swarm::Vector3{
            static_cast<double>(point[0]),
            static_cast<double>(point[1]),
            static_cast<double>(point[2])});
    }

    const ego_swarm::BsplineTrajectory spline(
        control_points, effective_knot_interval_s, degree);
    if (!spline.is_valid())
    {
        return TimedTrajectorySampleResult::INVALID;
    }

    const auto sample = spline.evaluate(elapsed_s);
    if (!sample.is_valid)
    {
        return TimedTrajectorySampleResult::INVALID;
    }

    for (std::size_t index = 0U; index < 3U; ++index)
    {
        position[index] = static_cast<float>(sample.position.at(index));
        velocity[index] = static_cast<float>(sample.velocity.at(index));
        acceleration[index] = static_cast<float>(
            sample.acceleration.at(index));
    }
    if (!std::isfinite(position[0]) || !std::isfinite(position[1]) ||
        !std::isfinite(position[2]) || !std::isfinite(velocity[0]) ||
        !std::isfinite(velocity[1]) || !std::isfinite(velocity[2]) ||
        !std::isfinite(acceleration[0]) ||
        !std::isfinite(acceleration[1]) ||
        !std::isfinite(acceleration[2]))
    {
        position = {0.0F, 0.0F, 0.0F};
        velocity = {0.0F, 0.0F, 0.0F};
        acceleration = {0.0F, 0.0F, 0.0F};
        return TimedTrajectorySampleResult::INVALID;
    }

    return TimedTrajectorySampleResult::VALID;
}

bool BasicController::has_timed_trajectory() const
{
    std::lock_guard<std::mutex> lock(timed_trajectory_mutex_);
    return has_timed_trajectory_;
}

std::array<float, 3> BasicController::get_safe_hold_position() const
{
    if (validate_setpoint(
            current_position_ned_[0], current_position_ned_[1],
            current_position_ned_[2]))
    {
        return current_position_ned_;
    }

    if (validate_setpoint(
            target_position_ned_[0], target_position_ned_[1],
            target_position_ned_[2]))
    {
        return target_position_ned_;
    }

    return {0.0F, 0.0F, 0.0F};
}

bool BasicController::hold_current_position()
{
    if (!is_initialized_.load())
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Cannot hold current position: not initialized");
        return false;
    }

    target_position_ned_ = current_position_ned_;
    clear_timed_trajectory();
    flight_state_.store(FlightState::HOVER);
    trajectory_setpoint_pending_.store(true);
    RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                "Collision safety hold at current NED position "
                "(%.2f, %.2f, %.2f)",
                static_cast<double>(target_position_ned_.at(0)),
                static_cast<double>(target_position_ned_.at(1)),
                static_cast<double>(target_position_ned_.at(2)));
    return true;
}


// =============================================================================
// 狀態機處理函式 (單一職責原則)
// =============================================================================

void BasicController::handle_offboard_init_state()
{
    if (offboard_entry_failed_.load())
    {
        return;
    }

    if (!is_vehicle_status_fresh())
    {
        fail_offboard_entry("VehicleStatus is unavailable or stale");
        emit_event(safety::AgentEventCode::COMMAND_FAILED, "MODE_OFFBOARD",
                   safety::PX4_RESULT_NONE,
                   "Offboard entry failed: VehicleStatus is unavailable or stale");
        return;
    }

    if (offboard_cmd_rejected_.load())
    {
        // ack 結果已由 process_command_acks() 以 PX4_COMMAND_REJECTED 回報。
        fail_offboard_entry("PX4 rejected DO_SET_MODE");
        return;
    }

    if (offboard_mode_request_sent_.load() && is_px4_offboard_confirmed())
    {
        offboard_session_active_.store(true);
        offboard_entry_failed_.store(false);
        flight_state_.store(FlightState::OFFBOARD_ACTIVE);
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "VehicleStatus confirmed PX4 is in Offboard mode");
        return;
    }

    if (offboard_mode_request_sent_.load())
    {
        const auto elapsed = node_->now() - offboard_mode_request_time_;
        const double elapsed_ms = elapsed.seconds() * 1000.0;
        if (elapsed_ms >= static_cast<double>(VEHICLE_CMD_ECHO_TIMEOUT_MS))
        {
            fail_offboard_entry(
                "VehicleStatus did not confirm Offboard before timeout");
            emit_event(safety::AgentEventCode::PX4_COMMAND_TIMEOUT,
                       "MODE_OFFBOARD", safety::PX4_RESULT_NONE,
                       "VehicleStatus did not confirm Offboard before timeout");
        }
    }
}

void BasicController::handle_offboard_active_state()
{
    // Offboard 已啟用；心跳由 publish_control_commands() 統一發布。
    return;
}

void BasicController::handle_takeoff_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    msg.position[0] = target_position_ned_.at(0);
    msg.position[1] = target_position_ned_.at(1);
    msg.position[2] = target_position_ned_.at(2);
    float current_alt = current_altitude_m_.load();
    float target_alt = -target_position_ned_.at(2);

    altitude_log_counter_++;
    if (altitude_log_counter_ >= ALTITUDE_LOG_INTERVAL)
    {
        altitude_log_counter_ = 0;
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "Takeoff progress: %.2f / %.2f m", current_alt, target_alt);
    }
    // 避免自動進入RTL模式，先註解掉自動進入HOVER或RTL的邏輯
    // if (current_alt >= target_alt * ALTITUDE_ARRIVAL_RATIO)
    // {
    //     float hold_dur = hover_duration_s_.load();
    //     if (hold_dur > 0.0F)
    //     {
    //         flight_state_.store(FlightState::HOVER);
    //         hover_start_time_ = node_->now();
    //         is_hover_timer_active_ = true;
    //     }
    //     else
    //     {
    //         flight_state_.store(FlightState::RTL);
    //         rtl_cmd_sent_ = false;
    //     }
    // }
    // else
    // {
    //     auto elapsed = node_->now() - takeoff_start_time_;
    //     if (elapsed.seconds() * 1000.0 >= static_cast<double>(TAKEOFF_TIMEOUT_MS))
    //     {
    //         RCLCPP_WARN(rclcpp::get_logger("BasicController"), "Takeoff timeout, aborting to RTL");
    //         flight_state_.store(FlightState::RTL);
    //         rtl_cmd_sent_ = false;
    //     }
    // }
}

void BasicController::handle_hover_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    msg.position[0] = target_position_ned_.at(0);
    msg.position[1] = target_position_ned_.at(1);
    msg.position[2] = target_position_ned_.at(2);
    // 避免自動進入RTL模式，先註解掉自動進入HOVER或RTL的邏輯
    // if (is_hover_timer_active_)
    // {
    //     auto elapsed = node_->now() - hover_start_time_;
    //     if (elapsed.seconds() >= static_cast<double>(hover_duration_s_.load()))
    //     {
    //         flight_state_.store(FlightState::RTL);
    //         rtl_cmd_sent_ = false;
    //         is_hover_timer_active_ = false;
    //     }
    // }
}

void BasicController::handle_rtl_state()
{
    if (!rtl_cmd_sent_)
    {
        rtl_cmd_sent_ = true;
        takeoff_start_time_ = node_->now();
        send_rtl_command();
    }
    if (arming_state_.load() == 1)
    {
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "RTL landing detected, transitioning to POST_RTL");
        flight_state_.store(FlightState::POST_RTL);
        rtl_cmd_sent_ = false;
        hold_cmd_sent_ = false;
    }
    else
    {
        auto elapsed = node_->now() - takeoff_start_time_;
        if (elapsed.seconds() * 1000.0 >= static_cast<double>(RTL_TIMEOUT_MS))
        {
            flight_state_.store(FlightState::IDLE);
            rtl_cmd_sent_ = false;
        }
    }
}

void BasicController::handle_land_state()
{
    if (!land_cmd_sent_)
    {
        land_cmd_sent_ = true;
        takeoff_start_time_ = node_->now();
        send_land_command();
    }
    if (arming_state_.load() == 1)
    {
        flight_state_.store(FlightState::IDLE);
        land_cmd_sent_ = false;
    }
    else
    {
        auto elapsed = node_->now() - takeoff_start_time_;
        if (elapsed.seconds() * 1000.0 >= static_cast<double>(RTL_TIMEOUT_MS))
        {
            flight_state_.store(FlightState::IDLE);
            land_cmd_sent_ = false;
        }
    }
}

void BasicController::handle_post_rtl_state()
{
    // RTL 降落完成，停止 Offboard 控制；position/yaw 仍維持有限 hold，
    // 實際模式轉換交由下方的 PX4 HOLD 命令處理。

    if (!hold_cmd_sent_)
    {
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "RTL complete, sending HOLD mode command and stopping Offboard");
        send_hold_mode_command();
        hold_cmd_sent_ = true;
        has_offboard_stopped_ = true;
    }

    // 等待一段時間確認命令發出後轉入 IDLE
    auto elapsed = node_->now() - takeoff_start_time_;
    if (elapsed.seconds() >= 2.0)
    {
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "POST_RTL complete, entering IDLE");
        flight_state_.store(FlightState::IDLE);
    }
}

void BasicController::execute_queued_commands()
{
    if (!offboard_session_active_.load())
    {
        return;
    }

    if (is_arm_queued_.exchange(false))
    {
        send_arm_now();
        // TAKEOFF 從 ARM 送出後開始等待 PX4 解鎖。
        return;
    }

    if (!is_takeoff_queued_.load())
    {
        return;
    }

    if (arming_state_.load() ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        is_takeoff_queued_.store(false);
        is_queued_takeoff_waiting_arm_ = false;
        start_takeoff(queued_takeoff_altitude_m_);
        return;
    }

    if (!is_queued_takeoff_waiting_arm_)
    {
        is_queued_takeoff_waiting_arm_ = true;
        queued_takeoff_wait_start_ = node_->now();
        return;
    }

    const double waited_ms =
        (node_->now() - queued_takeoff_wait_start_).seconds() * 1000.0;
    if (waited_ms >= static_cast<double>(QUEUED_TAKEOFF_ARM_TIMEOUT_MS))
    {
        is_takeoff_queued_.store(false);
        is_queued_takeoff_waiting_arm_ = false;
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "Queued TAKEOFF cancelled: PX4 not armed within %d ms; "
                    "send ARM before TAKEOFF", QUEUED_TAKEOFF_ARM_TIMEOUT_MS);
        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail),
                      "Queued TAKEOFF cancelled: PX4 not armed within %d ms; "
                      "send ARM before TAKEOFF", QUEUED_TAKEOFF_ARM_TIMEOUT_MS);
        emit_event(safety::AgentEventCode::TAKEOFF_CANCELLED, "TAKEOFF",
                   safety::PX4_RESULT_NONE, detail);
    }
}

void BasicController::handle_arm_rejection()
{
    if (!is_arm_rejected_.exchange(false))
    {
        return;
    }

    RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                "PX4 rejected ARM (result=%u); check PX4 preflight and health "
                "status. Stopping Offboard session",
                static_cast<unsigned int>(arm_reject_result_.load()));

    // 只回收解鎖前後的地面狀態；使用者若已改下 RTL/LAND 等命令則保留。
    const FlightState state = flight_state_.load();
    if (state == FlightState::ARMED || state == FlightState::TAKEOFF ||
        state == FlightState::OFFBOARD_ACTIVE)
    {
        stop_offboard_session();
        flight_state_.store(FlightState::IDLE);
    }
}

void BasicController::park_in_hold_when_disarmed_in_offboard()
{
    const FlightState state = flight_state_.load();
    const bool is_agent_idle =
        state == FlightState::IDLE || state == FlightState::ERROR;
    const bool is_px4_disarmed_in_offboard =
        is_vehicle_status_fresh() &&
        arming_state_.load() !=
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
        px4_nav_state_.load() == FlightStateResolver::PX4_NAV_STATE_OFFBOARD;
    if (!is_agent_idle || offboard_session_active_.load() ||
        !is_px4_disarmed_in_offboard)
    {
        has_parking_hold_sent_ = false;
        return;
    }

    const rclcpp::Time now = node_->now();
    if (has_parking_hold_sent_)
    {
        const double since_last_ms =
            (now - last_parking_hold_time_).seconds() * 1000.0;
        if (since_last_ms < static_cast<double>(PARKING_HOLD_RETRY_MS))
        {
            return;
        }
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "PX4 is still in Offboard without an Agent session; "
                    "retrying Hold mode command");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "PX4 is disarmed in Offboard without an Agent session; "
                    "switching PX4 to Hold");
    }
    send_hold_mode_command();
    has_parking_hold_sent_ = true;
    last_parking_hold_time_ = now;
}

void BasicController::handle_moving_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    // 發布完整 3D NED 目標位置，支援水平與垂直移動
    msg.position[0] = target_position_ned_[0];
    msg.position[1] = target_position_ned_[1];
    msg.position[2] = target_position_ned_[2];

    // 不進行達成判斷：由外部節點（如 formation_node）決定何時切換狀態
}


// =============================================================================
// VehicleCommand 發送輔助方法
// =============================================================================

bool BasicController::send_arm_command(bool do_arm)
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"), "VehicleCommand Publisher is nullptr");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = do_arm ? 1.0F : 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, do_arm ? "ARM" : "DISARM");
    RCLCPP_INFO(rclcpp::get_logger("BasicController"), "%s command sent (target_system=%d)",
                do_arm ? "ARM" : "DISARM", target_system_);
    return true;
}

bool BasicController::send_offboard_mode_command()
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "VehicleCommand Publisher is nullptr in send_offboard_mode_command");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0F;
    cmd.param2 = 6.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(VEHICLE_CMD_DO_SET_MODE, "MODE_OFFBOARD");
    return true;
}

bool BasicController::send_rtl_command()
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "VehicleCommand Publisher is nullptr in send_rtl_command");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.param1 = 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, "MODE_RTL");
    RCLCPP_INFO(rclcpp::get_logger("BasicController"), "RTL command sent (target_system=%d)", target_system_);
    return true;
}

bool BasicController::send_land_command()
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "VehicleCommand Publisher is nullptr in send_land_command");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = VEHICLE_CMD_NAV_LAND;
    cmd.param1 = 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(VEHICLE_CMD_NAV_LAND, "MODE_LAND");
    RCLCPP_INFO(rclcpp::get_logger("BasicController"), "LAND command sent (target_system=%d)", target_system_);
    return true;
}

bool BasicController::send_hold_mode_command()
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "VehicleCommand Publisher is nullptr in send_hold_mode_command");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = PX4_MODE_FLAG_CUSTOM_MODE_ENABLED;
    cmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
    cmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(VEHICLE_CMD_DO_SET_MODE, "HOLD");
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "HOLD mode command sent (target_system=%d)", target_system_);
    return true;
}

bool BasicController::send_reboot_command()
{
    if (vehicle_command_pub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "VehicleCommand Publisher is nullptr in send_reboot_command");
        return false;
    }
    auto cmd = px4_msgs::msg::VehicleCommand();
    cmd.timestamp = PX4_INBOUND_TIMESTAMP_US;
    cmd.command = PX4_VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    cmd.param1 = PX4_REBOOT_AUTOPILOT;
    cmd.param2 = PX4_REBOOT_ONBOARD_COMPUTER_NONE;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    track_px4_command(PX4_VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN, "REBOOT_FC");
    RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                "REBOOT_FC command sent (target_system=%d)", target_system_);
    return true;
}

bool BasicController::validate_setpoint(float x, float y, float z) const
{
    if (!utils::is_valid_float(x) || !utils::is_valid_float(y) || !utils::is_valid_float(z))
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"), "Setpoint invalid (NaN/Inf), rejected");
        return false;
    }
    return true;
}


// =============================================================================
// ROS2 回呼函式
// =============================================================================

void BasicController::vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"), "vehicle_command_ack_cb received nullptr");
        return;
    }
    last_command_ack_ns_.store(steady_now_ns());
    {
        // 事件由控制迴圈的 process_command_acks() 統一發布。
        std::lock_guard<std::mutex> lock(command_ack_mutex_);
        received_command_acks_.push_back(
            Px4CommandAckRecord{msg->command, msg->result});
    }
    if (msg->command == 176)
    {
        if (!offboard_mode_request_sent_.load() ||
            flight_state_.load() != FlightState::OFFBOARD_INIT)
        {
            // Offboard 進入流程以外的 DO_SET_MODE 只有 Hold 模式命令。
            if (msg->result !=
                    px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED &&
                msg->result !=
                    px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                            "PX4 rejected Hold mode command (result=%u)",
                            static_cast<unsigned int>(msg->result));
            }
            return;
        }

        if (msg->result ==
            px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {
            offboard_cmd_echo_received_.store(true);
        }
        else if (msg->result !=
                     px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)
        {
            offboard_cmd_rejected_.store(true);
            RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                        "PX4 rejected Offboard mode command (result=%u)",
                        static_cast<unsigned int>(msg->result));
        }
    }
    else if (msg->command == 400)
    {
        if (msg->result ==
            px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {
            arm_cmd_echo_received_.store(true);
            is_arm_ack_pending_.store(false);
        }
        else if (msg->result !=
                     px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS &&
                 is_arm_ack_pending_.exchange(false))
        {
            // 由控制迴圈統一處理，避免在 callback 中改動 session 狀態。
            arm_reject_result_.store(msg->result);
            is_arm_rejected_.store(true);
        }
    }
}

void BasicController::vehicle_status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"), "vehicle_status_cb received nullptr");
        return;
    }
    arming_state_.store(msg->arming_state);
    if (msg->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
        clear_timed_trajectory();
    }
    px4_nav_state_.store(msg->nav_state);
    pre_flight_checks_pass_.store(msg->pre_flight_checks_pass);
    px4_failsafe_.store(msg->failsafe);
    latest_arming_reason_.store(msg->latest_arming_reason);
    const int64_t received_ns = steady_now_ns();
    const int64_t previous_ns = last_vehicle_status_ns_.exchange(received_ns);
    if (is_px4_status_gap_notable(previous_ns, received_ns,
                                  PX4_STATUS_GAP_WARN_NS))
    {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("BasicController"), *node_->get_clock(), 5000,
            "VehicleStatus gap %.2f s (stop above %.1f s); check PX4 link quality",
            static_cast<double>(received_ns - previous_ns) * 1.0e-9,
            static_cast<double>(PX4_STATE_TIMEOUT_MS) * 1.0e-3);
    }
}

void BasicController::vehicle_land_detected_cb(
    const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "vehicle_land_detected_cb received nullptr");
        return;
    }
    is_landed_.store(msg->landed);
    if (msg->landed)
    {
        clear_timed_trajectory();
    }
    last_land_detected_ns_.store(steady_now_ns());
}

void BasicController::failsafe_flags_cb(
    const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "failsafe_flags_cb received nullptr");
        return;
    }
    failsafe_flag_bits_.store(make_failsafe_flag_bits(*msg));
    last_failsafe_flags_ns_.store(steady_now_ns());
}

// =============================================================================
// 事件、健康狀態與飛控重新啟動
// =============================================================================

void BasicController::set_event_callback(EventCallback callback)
{
    event_callback_ = std::move(callback);
}

bool BasicController::set_command_ack_timeout_s(const double timeout_s)
{
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0)
    {
        return false;
    }
    command_ack_timeout_ns_ =
        static_cast<int64_t>(timeout_s * NANOSECONDS_PER_SECOND);
    return true;
}

bool BasicController::set_reboot_cooldown_s(const double cooldown_s)
{
    if (!std::isfinite(cooldown_s) || cooldown_s < 0.0)
    {
        return false;
    }
    reboot_cooldown_ns_ =
        static_cast<int64_t>(cooldown_s * NANOSECONDS_PER_SECOND);
    return true;
}

FlightControllerRebootResult BasicController::reboot_flight_controller(
    const bool is_forced)
{
    FlightControllerRebootResult result;
    if (!is_initialized_.load())
    {
        result.detail = "REBOOT_FC rejected: controller is not initialized";
        return result;
    }

    const int64_t now_ns = steady_now_ns();
    FlightControllerRebootConditions conditions;
    conditions.is_status_fresh = is_vehicle_status_fresh();
    conditions.is_armed = arming_state_.load() ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    conditions.is_land_detected_fresh = is_px4_message_fresh(
        last_land_detected_ns_.load(), now_ns, PX4_STATE_TIMEOUT_NS);
    conditions.is_landed = is_landed_.load();
    conditions.is_forced = is_forced;
    conditions.is_cooldown_active = last_reboot_request_ns_ > 0 &&
        now_ns >= last_reboot_request_ns_ &&
        now_ns - last_reboot_request_ns_ < reboot_cooldown_ns_;

    const auto decision = evaluate_flight_controller_reboot(conditions);
    if (decision != FlightControllerRebootDecision::SEND)
    {
        result.detail = flight_controller_reboot_decision_detail(decision);
        RCLCPP_WARN(rclcpp::get_logger("BasicController"), "%s",
                    result.detail.c_str());
        return result;
    }

    // 重新啟動後 PX4 local NED 可能改變，舊 session、排隊命令與軌跡都不可沿用。
    stop_offboard_session();
    flight_state_.store(FlightState::IDLE);
    rtl_cmd_sent_ = false;
    land_cmd_sent_ = false;
    is_arm_ack_pending_.store(false);

    if (!send_reboot_command())
    {
        result.detail = "REBOOT_FC failed: unable to publish VehicleCommand";
        return result;
    }
    last_reboot_request_ns_ = now_ns;
    result.is_sent = true;
    return result;
}

Px4HealthSnapshot BasicController::get_px4_health_snapshot() const
{
    Px4HealthSnapshot snapshot;
    snapshot.status_received_ns = last_vehicle_status_ns_.load();
    snapshot.land_detected_received_ns = last_land_detected_ns_.load();
    snapshot.command_ack_received_ns = last_command_ack_ns_.load();
    snapshot.failsafe_flags_received_ns = last_failsafe_flags_ns_.load();
    snapshot.pre_flight_checks_pass = pre_flight_checks_pass_.load();
    snapshot.failsafe = px4_failsafe_.load();
    snapshot.arming_state = arming_state_.load();
    snapshot.latest_arming_reason = latest_arming_reason_.load();
    snapshot.failsafe_flag_bits = failsafe_flag_bits_.load();
    snapshot.offboard_session_active = offboard_session_active_.load();
    return snapshot;
}

void BasicController::track_px4_command(const uint32_t px4_command,
                                        const char* command_name)
{
    pending_px4_commands_.erase(
        std::remove_if(pending_px4_commands_.begin(), pending_px4_commands_.end(),
                       [px4_command](const PendingPx4Command& pending)
                       {
                           return pending.px4_command == px4_command;
                       }),
        pending_px4_commands_.end());
    pending_px4_commands_.push_back(
        PendingPx4Command{px4_command, command_name, steady_now_ns()});
}

void BasicController::process_command_acks()
{
    std::vector<Px4CommandAckRecord> acks;
    {
        std::lock_guard<std::mutex> lock(command_ack_mutex_);
        acks.swap(received_command_acks_);
    }

    for (const auto& ack : acks)
    {
        const auto pending = std::find_if(
            pending_px4_commands_.begin(), pending_px4_commands_.end(),
            [&ack](const PendingPx4Command& candidate)
            {
                return candidate.px4_command == ack.command;
            });
        if (pending == pending_px4_commands_.end())
        {
            // 非 Agent 送出（例如其他 GCS）或已逾時的命令不回報。
            continue;
        }

        if (ack.result ==
            px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)
        {
            // 執行中：重新計時，等待最終結果。
            pending->sent_ns = steady_now_ns();
            continue;
        }

        const std::string command_name = pending->name;
        pending_px4_commands_.erase(pending);

        if (ack.result ==
            px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
        {
            if (command_name == "REBOOT_FC")
            {
                restart_detector_.on_reboot_accepted(steady_now_ns());
            }
            if (command_name == "ARM" || command_name == "DISARM" ||
                command_name == "REBOOT_FC")
            {
                emit_event(safety::AgentEventCode::PX4_COMMAND_ACCEPTED,
                           command_name,
                           safety::to_px4_result_field(ack.result),
                           command_name + " accepted by PX4");
            }
            continue;
        }

        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail), "PX4 rejected %s (result=%u %s)",
                      command_name.c_str(), static_cast<unsigned int>(ack.result),
                      safety::px4_command_result_name(ack.result));
        emit_event(safety::AgentEventCode::PX4_COMMAND_REJECTED, command_name,
                   safety::to_px4_result_field(ack.result), detail);
    }
}

void BasicController::expire_pending_px4_commands()
{
    const int64_t now_ns = steady_now_ns();
    for (auto pending = pending_px4_commands_.begin();
         pending != pending_px4_commands_.end();)
    {
        if (now_ns - pending->sent_ns < command_ack_timeout_ns_)
        {
            ++pending;
            continue;
        }

        const std::string detail = pending->name == "REBOOT_FC" ?
            "No ack for REBOOT_FC; PX4 may be restarting or the link is down, "
            "watch health to confirm recovery" :
            "No ack from PX4 for " + pending->name;
        RCLCPP_WARN(rclcpp::get_logger("BasicController"), "%s",
                    detail.c_str());
        emit_event(safety::AgentEventCode::PX4_COMMAND_TIMEOUT, pending->name,
                   safety::PX4_RESULT_NONE, detail);
        pending = pending_px4_commands_.erase(pending);
    }
}

void BasicController::update_px4_link_events()
{
    const int64_t now_ns = steady_now_ns();
    const int64_t status_received_ns = last_vehicle_status_ns_.load();
    const auto status_transition =
        status_link_monitor_.update(status_received_ns, now_ns);

    if (status_transition == Px4LinkTransition::NEVER_RECEIVED)
    {
        emit_event(safety::AgentEventCode::PX4_STATUS_STALE, "",
                   safety::PX4_RESULT_NONE, "VehicleStatus never received");
    }
    else if (status_transition == Px4LinkTransition::BECAME_STALE)
    {
        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail),
                      "VehicleStatus not received for more than %d ms",
                      PX4_STATE_TIMEOUT_MS);
        emit_event(safety::AgentEventCode::PX4_STATUS_STALE, "",
                   safety::PX4_RESULT_NONE, detail);
    }
    else if (status_transition == Px4LinkTransition::RECOVERED)
    {
        emit_event(safety::AgentEventCode::PX4_STATUS_RECOVERED, "",
                   safety::PX4_RESULT_NONE, "VehicleStatus recovered");
    }

    const bool is_armed = arming_state_.load() ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    if (restart_detector_.update(status_transition, status_received_ns,
                                 is_armed, now_ns))
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "PX4 restart detected; resetting Offboard session and "
                    "locked heading");
        // 先重設再通知，讓節點收到事件時控制器已回到 IDLE。
        reset_after_px4_restart();
        emit_event(safety::AgentEventCode::PX4_RESTART_DETECTED, "",
                   safety::PX4_RESULT_NONE,
                   "PX4 restart or reconnection detected; Agent state reset");
    }

    if (!is_vehicle_status_fresh())
    {
        return;
    }

    if (preflight_transition_.update(pre_flight_checks_pass_.load()) ==
        safety::BooleanTransition::BECAME_FALSE)
    {
        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail),
                      "PX4 preflight checks no longer pass "
                      "(latest_arming_reason=%u)",
                      static_cast<unsigned int>(latest_arming_reason_.load()));
        emit_event(safety::AgentEventCode::PX4_PREFLIGHT_FAILED, "",
                   safety::PX4_RESULT_NONE, detail);
    }

    if (failsafe_transition_.update(px4_failsafe_.load()) ==
        safety::BooleanTransition::BECAME_TRUE)
    {
        char detail[EVENT_DETAIL_BUFFER_SIZE];
        std::snprintf(detail, sizeof(detail),
                      "PX4 entered failsafe (nav_state=%u)",
                      static_cast<unsigned int>(px4_nav_state_.load()));
        emit_event(safety::AgentEventCode::PX4_FAILSAFE, "",
                   safety::PX4_RESULT_NONE, detail);
    }
}

void BasicController::reset_after_px4_restart()
{
    stop_offboard_session();
    offboard_entry_failed_.store(false);
    flight_state_.store(FlightState::IDLE);
    rtl_cmd_sent_ = false;
    land_cmd_sent_ = false;
    hold_cmd_sent_ = false;
    has_parking_hold_sent_ = false;
    is_arm_ack_pending_.store(false);
    is_arm_rejected_.store(false);
    // 新的 PX4 instance 航向基準可能不同；下次起飛重新鎖定目前航向。
    target_yaw_rad_ = std::numeric_limits<float>::quiet_NaN();
    pending_px4_commands_.clear();
    preflight_transition_.reset();
    failsafe_transition_.reset();
}

void BasicController::emit_event(const safety::AgentEventCode code,
                                 const std::string& command,
                                 const int16_t px4_result,
                                 const std::string& detail)
{
    if (!event_callback_)
    {
        return;
    }
    safety::AgentEventRecord record;
    record.code = code;
    record.command = command;
    record.px4_result = px4_result;
    record.detail = detail;
    event_callback_(record);
}

void BasicController::update_command_state_from_observation()
{
    const FlightState command_intent = flight_state_.load();
    if (command_intent != FlightState::TAKEOFF)
    {
        return;
    }

    auto resolution = FlightStateResolver::resolve(
        get_flight_state_observation(), command_intent);

    if (!resolution.should_transition_to_hover)
    {
        return;
    }

    if (resolution.should_hold_current_position)
    {
        target_position_ned_ = current_position_ned_;
        target_altitude_m_.store(-target_position_ned_[2]);
    }

    flight_state_.store(FlightState::HOVER);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "PX4 observation completed command intent, transitioning to HOVER");
}

bool BasicController::is_vehicle_status_fresh() const
{
    if (node_ == nullptr)
    {
        return false;
    }

    return is_px4_message_fresh(
        last_vehicle_status_ns_.load(), steady_now_ns(), PX4_STATE_TIMEOUT_NS);
}

bool BasicController::is_px4_offboard_confirmed() const
{
    return is_vehicle_status_fresh() &&
        px4_nav_state_.load() == FlightStateResolver::PX4_NAV_STATE_OFFBOARD;
}

bool BasicController::should_publish_offboard_control_mode() const
{
    if (!is_vehicle_status_fresh() || offboard_entry_failed_.load())
    {
        return false;
    }

    switch (flight_state_.load())
    {
        case FlightState::OFFBOARD_INIT:
            // PX4 尚未進入 Offboard 時仍須完成 prestream；此狀態只在
            // 明確開始一次新的 session 後成立。
            return true;

        case FlightState::OFFBOARD_ACTIVE:
        case FlightState::ARMED:
        case FlightState::TAKEOFF:
        case FlightState::HOVER:
        case FlightState::MOVING:
            return offboard_session_active_.load() &&
                is_px4_offboard_confirmed();

        case FlightState::IDLE:
        case FlightState::RTL:
        case FlightState::LAND:
        case FlightState::POST_RTL:
        case FlightState::ERROR:
        default:
            return false;
    }
}

bool BasicController::should_publish_trajectory_setpoint() const
{
    const FlightState state = flight_state_.load();
    if (state != FlightState::TAKEOFF &&
        state != FlightState::HOVER &&
        state != FlightState::MOVING)
    {
        return false;
    }

    // 偏航未到達目標時逐週期重送，讓 PX4 收到限速後的中間航向。
    return trajectory_setpoint_pending_.load() ||
        trajectory_setpoint_active_.load() ||
        is_yaw_slew_pending();
}

}  // namespace controllers
}  // namespace taiphoon
