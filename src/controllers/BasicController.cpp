
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
#include "utils/math_utils.hpp"

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
        !create_vehicle_status_subscriber())
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

    offboard_control_mode_pub_.reset();
    trajectory_setpoint_pub_.reset();
    vehicle_command_pub_.reset();
    vehicle_command_ack_sub_.reset();
    vehicle_status_sub_.reset();

    is_initialized_.store(false);
    flight_state_.store(FlightState::IDLE);
}

bool BasicController::is_initialized() const
{
    return is_initialized_.load();
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
    if (result) flight_state_.store(FlightState::ARMED);
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
    if (altitude_m <= 0.0F)
    {
        RCLCPP_ERROR(rclcpp::get_logger("BasicController"),
                     "Takeoff altitude must be positive");
        return false;
    }
    target_altitude_m_.store(altitude_m);
    flight_state_.store(FlightState::TAKEOFF);
    takeoff_start_time_ = node_->now();
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
    flight_state_.store(FlightState::HOVER);
    hover_start_time_ = node_->now();
    is_hover_timer_active_ = true;
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
    is_offboard_timer_active_ = true;
    is_hover_timer_active_ = false;
    offboard_cmd_sending_ = false;
    offboard_cmd_echo_received_.store(false);
    arm_cmd_echo_received_.store(false);
    offboard_start_time_ = node_->now();
    flight_state_.store(FlightState::OFFBOARD_INIT);
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
        case FlightState::MOVING:        return "MOVING";
        case FlightState::ERROR:         return "ERROR";
        default:                         return "UNKNOWN";
    }
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
    is_offboard_timer_active_ = true;
    offboard_cmd_sending_ = false;
    offboard_cmd_echo_received_.store(false);
    arm_cmd_echo_received_.store(false);
    offboard_start_time_ = node_->now();
    flight_state_.store(FlightState::OFFBOARD_INIT);
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Switching to OFFBOARD mode (target_system=%d)", target_system_);
    return true;
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
    target_position_ned_ = current_position_ned_;
    target_position_ned_[0] += distance_m;
    flight_state_.store(FlightState::MOVING);
    move_start_time_ = node_->now();
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
    target_position_ned_ = current_position_ned_;
    target_position_ned_[1] += distance_m;
    flight_state_.store(FlightState::MOVING);
    move_start_time_ = node_->now();
    RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                "Moving right %.2f m (target_system=%d)", distance_m, target_system_);
    return true;
}

// =============================================================================
// 公開輔助方法
// =============================================================================

void BasicController::update_odometry(const px4_msgs::msg::VehicleOdometry& msg)
{
    current_altitude_m_.store(-msg.position[2]);
    current_position_ned_[0] = msg.position[0];
    current_position_ned_[1] = msg.position[1];
    current_position_ned_[2] = msg.position[2];
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

// =============================================================================
// 控制命令發布 (主要計時器回呼)
// =============================================================================

void BasicController::publish_control_commands()
{
    if (!is_initialized_.load()) return;
    if (offboard_control_mode_pub_ == nullptr || trajectory_setpoint_pub_ == nullptr) return;

    auto mode_msg = px4_msgs::msg::OffboardControlMode();
    mode_msg.timestamp = node_->now().nanoseconds() / 1000ULL;
    mode_msg.position = true;
    offboard_control_mode_pub_->publish(mode_msg);

    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.timestamp = node_->now().nanoseconds() / 1000ULL;
    msg.position[0] = std::numeric_limits<float>::quiet_NaN();
    msg.position[1] = std::numeric_limits<float>::quiet_NaN();
    msg.position[2] = std::numeric_limits<float>::quiet_NaN();
    msg.yaw = std::numeric_limits<float>::quiet_NaN();

    switch (flight_state_.load())
    {
        case FlightState::OFFBOARD_INIT: handle_offboard_init_state(mode_msg); break;
        case FlightState::ARMED:         handle_armed_state(mode_msg); break;
        case FlightState::TAKEOFF:       handle_takeoff_state(msg); break;
        case FlightState::HOVER:         handle_hover_state(msg); break;
        case FlightState::RTL:           handle_rtl_state(msg); break;
        case FlightState::LAND:          handle_land_state(msg); break;
        case FlightState::MOVING:        handle_moving_state(msg); break;
        case FlightState::IDLE:
        case FlightState::ERROR:
        default: break;
    }
    trajectory_setpoint_pub_->publish(msg);
}

float BasicController::get_current_altitude() const
{
    return current_altitude_m_.load();
}

std::array<float, 3> BasicController::get_current_position_ned() const
{
    return current_position_ned_;
}

// =============================================================================
// 狀態機處理函式 (單一職責原則)
// =============================================================================

void BasicController::handle_offboard_init_state(px4_msgs::msg::OffboardControlMode& /*mode_msg*/)
{
    if (!is_offboard_timer_active_) return;
    auto elapsed = node_->now() - offboard_start_time_;
    double elapsed_ms = elapsed.seconds() * 1000.0;

    if (!offboard_cmd_sending_ && elapsed_ms < static_cast<double>(HEARTBEAT_WARMUP_MS))
    {
        return;
    }

    if (!offboard_cmd_sending_)
    {
        offboard_cmd_sending_ = true;
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "Heartbeat warmup complete, sending mode switch");
    }

    if (vehicle_command_pub_ != nullptr) send_offboard_mode_command();

    bool echoed = offboard_cmd_echo_received_.load();
    bool timeout = (elapsed_ms >= static_cast<double>(HEARTBEAT_WARMUP_MS + VEHICLE_CMD_ECHO_TIMEOUT_MS));

    if (echoed || timeout)
    {
        if (!echoed)
            RCLCPP_WARN(rclcpp::get_logger("BasicController"),
                       "Offboard echo timeout after %.0f ms, proceeding", elapsed_ms);
        is_offboard_timer_active_ = false;
        offboard_cmd_sending_ = false;
        if (!arm())
        {
            RCLCPP_ERROR(rclcpp::get_logger("BasicController"), "Arm failed");
            flight_state_.store(FlightState::ERROR);
        }
    }
}

void BasicController::handle_armed_state(px4_msgs::msg::OffboardControlMode& /*mode_msg*/)
{
    bool echoed = arm_cmd_echo_received_.load();
    auto elapsed = node_->now() - offboard_start_time_;
    double elapsed_ms = elapsed.seconds() * 1000.0;
    bool timeout = (!echoed && elapsed_ms >= static_cast<double>(
        HEARTBEAT_WARMUP_MS + VEHICLE_CMD_ECHO_TIMEOUT_MS + 5000.0));
    if (echoed || timeout)
    {
        float alt = target_altitude_m_.load();
        if (alt > 0.0F) takeoff(alt);
    }
}

void BasicController::handle_takeoff_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    float target_z = -target_altitude_m_.load();
    msg.position[2] = target_z;
    float current_alt = current_altitude_m_.load();
    float target_alt = target_altitude_m_.load();

    altitude_log_counter_++;
    if (altitude_log_counter_ >= ALTITUDE_LOG_INTERVAL)
    {
        altitude_log_counter_ = 0;
        RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                    "Takeoff progress: %.2f / %.2f m", current_alt, target_alt);
    }

    if (current_alt >= target_alt * ALTITUDE_ARRIVAL_RATIO)
    {
        float hold_dur = hover_duration_s_.load();
        if (hold_dur > 0.0F)
        {
            flight_state_.store(FlightState::HOVER);
            hover_start_time_ = node_->now();
            is_hover_timer_active_ = true;
        }
        else
        {
            flight_state_.store(FlightState::RTL);
            rtl_cmd_sent_ = false;
        }
    }
    else
    {
        auto elapsed = node_->now() - takeoff_start_time_;
        if (elapsed.seconds() * 1000.0 >= static_cast<double>(TAKEOFF_TIMEOUT_MS))
        {
            RCLCPP_WARN(rclcpp::get_logger("BasicController"), "Takeoff timeout, aborting to RTL");
            flight_state_.store(FlightState::RTL);
            rtl_cmd_sent_ = false;
        }
    }
}

void BasicController::handle_hover_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    msg.position[2] = -target_altitude_m_.load();
    if (is_hover_timer_active_)
    {
        auto elapsed = node_->now() - hover_start_time_;
        if (elapsed.seconds() >= static_cast<double>(hover_duration_s_.load()))
        {
            flight_state_.store(FlightState::RTL);
            rtl_cmd_sent_ = false;
            is_hover_timer_active_ = false;
        }
    }
}

void BasicController::handle_rtl_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    if (!rtl_cmd_sent_)
    {
        rtl_cmd_sent_ = true;
        takeoff_start_time_ = node_->now();
        send_rtl_command();
    }
    msg.position[0] = std::numeric_limits<float>::quiet_NaN();
    msg.position[1] = std::numeric_limits<float>::quiet_NaN();
    msg.position[2] = std::numeric_limits<float>::quiet_NaN();
    if (arming_state_.load() == 1)
    {
        flight_state_.store(FlightState::IDLE);
        rtl_cmd_sent_ = false;
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

void BasicController::handle_land_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    if (!land_cmd_sent_)
    {
        land_cmd_sent_ = true;
        takeoff_start_time_ = node_->now();
        send_land_command();
    }
    msg.position[0] = std::numeric_limits<float>::quiet_NaN();
    msg.position[1] = std::numeric_limits<float>::quiet_NaN();
    msg.position[2] = std::numeric_limits<float>::quiet_NaN();
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

void BasicController::handle_moving_state(px4_msgs::msg::TrajectorySetpoint& msg)
{
    msg.position[0] = target_position_ned_[0];
    msg.position[1] = target_position_ned_[1];
    msg.position[2] = current_position_ned_[2];
    float dx = target_position_ned_[0] - current_position_ned_[0];
    float dy = target_position_ned_[1] - current_position_ned_[1];
    float dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= move_arrival_threshold_m_)
    {
        flight_state_.store(FlightState::HOVER);
        hover_start_time_ = node_->now();
        is_hover_timer_active_ = true;
        hover_duration_s_.store(3.0F);
    }
    else
    {
        auto elapsed = node_->now() - move_start_time_;
        if (static_cast<int>(elapsed.seconds()) % 2 == 0 && elapsed.nanoseconds() < 100000000LL)
        {
            RCLCPP_INFO(rclcpp::get_logger("BasicController"),
                       "Moving: %.2f m remaining", static_cast<double>(dist));
        }
    }
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
    if (vehicle_command_pub_ == nullptr) return false;
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
    if (vehicle_command_pub_ == nullptr) return false;
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
    if (vehicle_command_pub_ == nullptr) return false;
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
        offboard_cmd_echo_received_.store(true);
    }
    else if (msg->command == 400)
    {
        arm_cmd_echo_received_.store(true);
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
}

}  // namespace controllers
}  // namespace taiphoon
