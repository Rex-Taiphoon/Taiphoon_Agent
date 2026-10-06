
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
#include "ego_swarm/BsplineTrajectory.hpp"
#include "utils/math_utils.hpp"
#include <cmath>

#include <limits>

namespace taiphoon
{
namespace controllers
{

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
    trajectory_setpoint_pending_.store(false);
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

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Sending arm command (target_system=%d)", target_system_);
    bool result = send_arm_command(true);
    if (result)
    {
        offboard_start_time_ = node_->now();
        flight_state_.store(FlightState::ARMED);
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
    target_altitude_m_.store(altitude_m);
    target_position_ned_ = current_position_ned_;
    target_position_ned_.at(2) = -altitude_m;
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
                "Takeoff to %.2f meters (target_system=%d)",
                altitude_m, target_system_);
    return true;
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

    int64_t now_ns = node_->now().nanoseconds();
    int64_t status_ns = last_vehicle_status_ns_.load();
    int64_t landed_ns = last_land_detected_ns_.load();
    constexpr int64_t PX4_STATE_TIMEOUT_NS =
        static_cast<int64_t>(PX4_STATE_TIMEOUT_MS) * 1000000LL;

    bool has_recent_status = status_ns > 0 && now_ns >= status_ns &&
        now_ns - status_ns <= PX4_STATE_TIMEOUT_NS;
    bool has_recent_land_detection = landed_ns > 0 && now_ns >= landed_ns &&
        now_ns - landed_ns <= PX4_STATE_TIMEOUT_NS;

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

    constexpr float PI = 3.14159265358979323846F;
    constexpr float TWO_PI = 2.0F * PI;
    constexpr float DEG_TO_RAD = PI / 180.0F;
    const float delta_rad = degrees * DEG_TO_RAD;

    // YAW 為相對目前航向的命令。PX4 NED yaw 的正方向是順時針，
    // 與 move_forward/move_right 使用的 NED 幾何一致，因此使用 +=。
    if (!utils::is_valid_float(target_yaw_rad_))
    {
        target_yaw_rad_ = current_yaw_rad_.load();
        if (!utils::is_valid_float(target_yaw_rad_))
        {
            target_yaw_rad_ = 0.0F;
        }
    }
    target_yaw_rad_ += delta_rad;
    target_yaw_rad_ = std::fmod(target_yaw_rad_ + PI, TWO_PI);
    if (target_yaw_rad_ < 0.0F)
    {
        target_yaw_rad_ += TWO_PI;
    }
    target_yaw_rad_ -= PI;
    trajectory_setpoint_pending_.store(true);

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Yaw rotation %.1f deg, target yaw: %.1f deg (target_system=%d)",
                degrees, target_yaw_rad_ / DEG_TO_RAD, target_system_);
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

    constexpr float PI = 3.14159265358979323846F;
    constexpr float TWO_PI = 2.0F * PI;
    constexpr float DEG_TO_RAD = PI / 180.0F;
    target_yaw_rad_ = std::fmod(degrees * DEG_TO_RAD + PI, TWO_PI);
    if (target_yaw_rad_ < 0.0F)
    {
        target_yaw_rad_ += TWO_PI;
    }
    target_yaw_rad_ -= PI;

    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Absolute yaw target %.1f deg (target_system=%d)",
                target_yaw_rad_ / DEG_TO_RAD, target_system_);
    return true;
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


// =============================================================================
// 控制命令發布 (主要計時器回呼)
// =============================================================================

void BasicController::publish_control_commands()
{
    if (!is_initialized_.load())
    {
        return;
    }

    // Offboard session 成功後，任何外部模式切換都必須先關閉兩個 PX4
    // setpoint publisher。OFFBOARD_INIT 期間的非 Offboard nav_state 是
    // 預期狀態，不能在尚未送出模式命令前誤判為外部離開。
    if (offboard_session_active_.load() && !is_px4_offboard_confirmed())
    {
        RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                    "PX4 left Offboard or VehicleStatus became stale; "
                    "stopping Offboard output");
        stop_offboard_session();
        flight_state_.store(FlightState::IDLE);
        has_offboard_stopped_ = true;
    }

    update_command_state_from_observation();

    if (flight_state_.load() == FlightState::OFFBOARD_INIT)
    {
        handle_offboard_init_state();
    }

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

    current_state = flight_state_.load();
    if (!should_publish_offboard_control_mode() ||
        offboard_control_mode_pub_ == nullptr)
    {
        return;
    }

    const auto timestamp = node_->now().nanoseconds() / 1000ULL;
    auto mode_msg = px4_msgs::msg::OffboardControlMode();
    mode_msg.timestamp = timestamp;
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

    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = timestamp;
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
    msg.yaw = 0.0F;
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

    if (utils::is_valid_float(target_yaw_rad_))
    {
        msg.yaw = target_yaw_rad_;
    }

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
        msg.yaw = 0.0F;
    }
    if (!utils::is_valid_float(msg.yawspeed))
    {
        msg.yawspeed = 0.0F;
    }

    trajectory_setpoint_pub_->publish(msg);
    trajectory_setpoint_pending_.store(false);

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
        return;
    }

    if (offboard_cmd_rejected_.load())
    {
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
    cmd.timestamp = node_->now().nanoseconds() / 1000ULL;
    constexpr uint32_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
    cmd.command = VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = do_arm ? 1.0F : 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
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
    cmd.timestamp = node_->now().nanoseconds() / 1000ULL;
    constexpr uint32_t VEHICLE_CMD_DO_SET_MODE = 176;
    cmd.command = VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0F;
    cmd.param2 = 6.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
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
    cmd.timestamp = node_->now().nanoseconds() / 1000ULL;
    constexpr uint32_t VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20;
    cmd.command = VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
    cmd.param1 = 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
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
    cmd.timestamp = node_->now().nanoseconds() / 1000ULL;
    constexpr uint32_t VEHICLE_CMD_NAV_LAND = 21;
    cmd.command = VEHICLE_CMD_NAV_LAND;
    cmd.param1 = 0.0F;
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
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
    cmd.timestamp = node_->now().nanoseconds() / 1000ULL;
    constexpr uint32_t VEHICLE_CMD_DO_SET_MODE = 176;
    cmd.command = VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1.0F;  // custom mode
    cmd.param2 = 8.0F;  // Position Hold mode for multicopter
    cmd.target_system = target_system_;
    cmd.target_component = 1;
    cmd.source_system = target_system_;
    cmd.source_component = 1;
    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "HOLD mode command sent (target_system=%d)", target_system_);
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
    if (msg->command == 176)
    {
        if (!offboard_mode_request_sent_.load() ||
            flight_state_.load() != FlightState::OFFBOARD_INIT)
        {
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
    last_vehicle_status_ns_.store(node_->now().nanoseconds());
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
    last_land_detected_ns_.store(node_->now().nanoseconds());
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

    const int64_t status_ns = last_vehicle_status_ns_.load();
    const int64_t now_ns = node_->now().nanoseconds();
    const int64_t timeout_ns =
        static_cast<int64_t>(PX4_STATE_TIMEOUT_MS) * 1000000LL;
    return status_ns > 0 && now_ns >= status_ns &&
        now_ns - status_ns <= timeout_ns;
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

    return trajectory_setpoint_pending_.load() ||
        trajectory_setpoint_active_.load();
}

}  // namespace controllers
}  // namespace taiphoon
