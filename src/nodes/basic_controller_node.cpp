/**
 * @file basic_controller_node.cpp
 * @brief BasicController ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include "nodes/basic_controller_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace taiphoon
{
namespace nodes
{

using taiphoon::controllers::FlightState;

namespace
{
constexpr double RATE_COMPARISON_EPSILON = 1.0E-6;
constexpr char VEHICLE_GLOBAL_POSITION_TOPIC[] =
    "fmu/out/vehicle_global_position";
}  // namespace

BasicControllerNode::BasicControllerNode()
    : rclcpp::Node("basic_controller_node")
{
    // L2-19：宣告 ROS2 參數並指定預設值
    this->declare_parameter<double>("target_altitude_m", DEFAULT_TARGET_ALTITUDE_M);
    this->declare_parameter<double>("hover_duration_s", DEFAULT_HOVER_DURATION_S);
    this->declare_parameter<double>("startup_delay_s", 0.0);
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<std::string>("agent_namespace", "fmu1");
    this->declare_parameter<bool>("auto_start_sequence", true);
    this->declare_parameter<double>("state_publish_rate_hz", DEFAULT_STATE_PUBLISH_RATE_HZ);
    this->declare_parameter<double>("position_deviation_threshold_m",
                                    DEFAULT_POSITION_DEVIATION_THRESHOLD_M);
    this->declare_parameter<double>("collision_safety_margin_m",
                                    DEFAULT_COLLISION_SAFETY_MARGIN_M);
    this->declare_parameter<double>("collision_peer_timeout_s",
                                    DEFAULT_COLLISION_PEER_TIMEOUT_S);

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    hover_duration_s_ = this->get_parameter("hover_duration_s").as_double();
    startup_delay_s_ = this->get_parameter("startup_delay_s").as_double();
    target_system_ = this->get_parameter("target_system").as_int();
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    auto_start_sequence_ = this->get_parameter("auto_start_sequence").as_bool();
    const double requested_state_publish_rate_hz =
        this->get_parameter("state_publish_rate_hz").as_double();
    state_publish_rate_hz_ = safety::StatePublisher::normalize_publish_rate_hz(
        requested_state_publish_rate_hz);
    if (!std::isfinite(requested_state_publish_rate_hz) ||
        std::abs(requested_state_publish_rate_hz - state_publish_rate_hz_) >
            RATE_COMPARISON_EPSILON)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Requested state publish rate %.3f Hz is invalid or below "
                    "10 Hz; using %.1f Hz",
                    requested_state_publish_rate_hz, state_publish_rate_hz_);
    }
    position_deviation_threshold_m_ =
        this->get_parameter("position_deviation_threshold_m").as_double();
    const double requested_collision_margin_m =
        this->get_parameter("collision_safety_margin_m").as_double();
    collision_safety_margin_m_ = std::isfinite(requested_collision_margin_m)
        ? std::max(requested_collision_margin_m, DEFAULT_COLLISION_SAFETY_MARGIN_M)
        : DEFAULT_COLLISION_SAFETY_MARGIN_M;
    collision_peer_timeout_s_ =
        this->get_parameter("collision_peer_timeout_s").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Parameters: target_altitude=%.2f m, hover_duration=%.2f s, "
                "startup_delay=%.1f s, target_system=%d, agent_namespace=%s, "
                "state_publish_rate=%.1f Hz, deviation_threshold=%.2f m, "
                "collision_margin=%.2f m, collision_peer_timeout=%.2f s",
                target_altitude_m_, hover_duration_s_,
                startup_delay_s_, target_system_,
                agent_namespace_.c_str(),
                state_publish_rate_hz_, position_deviation_threshold_m_,
                collision_safety_margin_m_, collision_peer_timeout_s_);

    // --- 建立飛行控制器 ---
    controller_ = std::make_unique<controllers::BasicController>(
        this, static_cast<uint8_t>(target_system_));

    if (controller_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller creation failed");
        return;
    }

    if (!controller_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(), "Controller init failed");
        return;
    }

    // --- 初始化位置安全監控器 ---
    position_safety_monitor_ = safety::PositionSafetyMonitor(
        static_cast<float>(position_deviation_threshold_m_));
    collision_target_gate_ = safety::CollisionTargetGate(
        static_cast<float>(collision_safety_margin_m_));

    // --- 初始化里程計訂閱者 ---
    if (!initialize_odometry_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry subscriber init failed");
        return;
    }

    // --- 初始化 GPS 訂閱者 ---
    if (!initialize_gps_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "GPS subscriber init failed");
        return;
    }

    // --- 初始化電池狀態訂閱者 ---
    if (!initialize_battery_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Battery subscriber init failed");
        return;
    }

    // --- 初始化 NED 目標訂閱者 ---
    if (!initialize_ned_target_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "NED target subscriber init failed");
        return;
    }

    // --- 初始化命令處理器 ---
    if (!initialize_agent_command_handler())
    {
        RCLCPP_ERROR(this->get_logger(), "Agent command handler init failed");
        return;
    }

    // --- 初始化 Agent 狀態發布者 ---
    if (!initialize_state_publishers())
    {
        RCLCPP_ERROR(this->get_logger(), "State publishers init failed");
        return;
    }

    // --- 初始化防撞同儕發現 ---
    scan_collision_peers_cb();
    auto peer_scan_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 * DEFAULT_COLLISION_PEER_SCAN_INTERVAL_S));
    collision_peer_scan_timer_ = this->create_wall_timer(
        peer_scan_period_ms,
        std::bind(&BasicControllerNode::scan_collision_peers_cb, this));

    // --- 建立控制迴圈計時器 ---
    auto ctrl_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / CONTROL_LOOP_RATE_HZ));

    control_loop_timer_ = this->create_wall_timer(
        ctrl_period_ms,
        std::bind(&BasicControllerNode::control_loop_cb, this));

    // --- 建立狀態發布計時器 ---
    auto state_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / state_publish_rate_hz_));

    state_publish_timer_ = this->create_wall_timer(
        state_period_ms,
        std::bind(&BasicControllerNode::state_publish_cb, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "BasicControllerNode initialized, control rate %.0f Hz, "
                "state publish rate %.1f Hz",
                CONTROL_LOOP_RATE_HZ, state_publish_rate_hz_);
}

bool BasicControllerNode::initialize_odometry_subscriber()
{
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "fmu/out/vehicle_odometry",
        rclcpp::QoS(10).best_effort(),
        std::bind(&BasicControllerNode::odometry_cb, this,
                  std::placeholders::_1));

    if (odometry_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create odometry subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_gps_subscriber()
{
    // 節點 namespace 為 /fmu{N}，此相對名稱解析成
    // /fmu{N}/fmu/out/vehicle_global_position。
    gps_sub_ =
        this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            VEHICLE_GLOBAL_POSITION_TOPIC,
            rclcpp::QoS(10).best_effort(),
            std::bind(&BasicControllerNode::gps_cb, this,
                      std::placeholders::_1));

    if (gps_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create GPS subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_battery_subscriber()
{
    battery_sub_ =
        this->create_subscription<px4_msgs::msg::BatteryStatus>(
            "fmu/out/battery_status_v1",
            rclcpp::QoS(10).best_effort(),
            std::bind(&BasicControllerNode::battery_cb, this,
                      std::placeholders::_1));

    if (battery_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create battery subscriber");
        return false;
    }

    return true;
}

bool BasicControllerNode::initialize_ned_target_subscriber()
{
    std::string ned_topic = "/" + agent_namespace_ + "/ned_target";
    ned_target_sub_ = this->create_subscription<taiphoon_agent::msg::NedTarget>(
        ned_topic,
        rclcpp::QoS(5).best_effort(),
        std::bind(&BasicControllerNode::ned_target_cb, this,
                  std::placeholders::_1));

    if (ned_target_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create NED target subscriber on '%s'",
                     ned_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Subscribed to NED target topic: %s", ned_topic.c_str());
    return true;
}

bool BasicControllerNode::initialize_agent_command_handler()
{
    agent_cmd_handler_ =
        std::make_unique<controllers::AgentCommandHandler>(
            this, agent_namespace_);

    if (agent_cmd_handler_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentCommandHandler");
        return false;
    }

    agent_cmd_handler_->set_command_callback(
        std::bind(&BasicControllerNode::on_agent_command, this,
                  std::placeholders::_1));

    // 初始化命令處理器 (建立 Subscriber)
    if (!agent_cmd_handler_->initialize())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "AgentCommandHandler initialization failed");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "AgentCommandHandler initialized for namespace '%s'",
                agent_namespace_.c_str());
    return true;
}

bool BasicControllerNode::initialize_state_publishers()
{
    std::string state_topic = "/" + agent_namespace_ + "/state";
    std::string attitude_topic = "/" + agent_namespace_ + "/attitude";

    agent_state_pub_ =
        this->create_publisher<taiphoon_agent::msg::AgentState>(
            state_topic, rclcpp::QoS(10));

    if (agent_state_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentState publisher on '%s'",
                     state_topic.c_str());
        return false;
    }

    agent_attitude_pub_ =
        this->create_publisher<taiphoon_agent::msg::AgentAttitude>(
            attitude_topic, rclcpp::QoS(10));

    if (agent_attitude_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create AgentAttitude publisher on '%s'",
                     attitude_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "State publishers created: %s, %s",
                state_topic.c_str(), attitude_topic.c_str());
    return true;
}

// =============================================================================
// 回呼實作
// =============================================================================

void BasicControllerNode::odometry_cb(
    const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "odometry_cb received nullptr message, ignoring");
        return;
    }

    latest_odometry_ = *msg;
    has_odometry_data_ = true;

    if (controller_ != nullptr)
    {
        controller_->update_odometry(*msg);
    }
}

void BasicControllerNode::gps_cb(
    const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "gps_cb received nullptr message, ignoring");
        return;
    }

    latest_gps_ = *msg;
    has_gps_data_ = true;
}

void BasicControllerNode::battery_cb(
    const px4_msgs::msg::BatteryStatus::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "battery_cb received nullptr message, ignoring");
        return;
    }

    latest_battery_ = *msg;
    has_battery_data_ = true;
}

void BasicControllerNode::control_loop_cb()
{
    if (controller_ == nullptr)
    {
        return;
    }

    // 防撞關卡在每次 PX4 setpoint 發布前重新評估最新同儕狀態。
    evaluate_collision_target();

//     // --- 位置安全監控：檢查實際位置與目標位置偏差 ---
//     {
//         auto current_pos = controller_->get_current_position_ned();
//         auto target_pos = controller_->get_target_position_ned();
//
//         // TAKEOFF 期間不檢查位置偏差（爬升為有意行為）
//         if (controller_->get_flight_state() != FlightState::TAKEOFF &&
//             position_safety_monitor_.is_deviation_exceeded(
//                 current_pos, target_pos))
//         {
//             float dist = safety::PositionSafetyMonitor::compute_distance(
//                 current_pos, target_pos);
//
//             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
//                                  3000,
//                                  "Position deviation %.2f m exceeds threshold "
//                                  "%.2f m — triggering hover intervention",
//                                  static_cast<double>(dist),
//                                  position_deviation_threshold_m_);
//
//             // 觸發 hover 介入：重設目標為目前位置
//             controller_->set_ned_target(
//                 current_pos[0], current_pos[1], current_pos[2],
//                 std::numeric_limits<float>::quiet_NaN());
//         }
//     }
//
    // 驅動控制命令發布
    controller_->publish_control_commands();

    // Auto-start Offboard mode after connection + startup_delay (if enabled)
    if (auto_start_sequence_ && !sequence_started_)
    {
        auto elapsed = this->now() - start_time_;
        if (elapsed.seconds() >= startup_delay_s_)
        {
            sequence_started_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Auto-starting Offboard mode after %.1f s delay",
                        startup_delay_s_);
            controller_->switch_to_offboard_mode();
        }
    }
}

void BasicControllerNode::state_publish_cb()
{
    if (!has_odometry_data_)
    {
        return;
    }

    if (controller_ == nullptr)
    {
        return;
    }

    const px4_msgs::msg::VehicleGlobalPosition* gps_ptr =
        has_gps_data_ ? &latest_gps_ : nullptr;

    const px4_msgs::msg::BatteryStatus* battery_ptr =
        has_battery_data_ ? &latest_battery_ : nullptr;

    // --- 發布 AgentState ---
    {
        auto observation = controller_->get_flight_state_observation();
        taiphoon_agent::msg::AgentState state_msg;
        safety::StatePublisher::fill_agent_state(
            state_msg, latest_odometry_,
            controller_->get_flight_state(), true,
            gps_ptr, battery_ptr, &observation);
        agent_state_pub_->publish(state_msg);
    }

    // --- 發布 AgentAttitude ---
    {
        taiphoon_agent::msg::AgentAttitude attitude_msg;
        safety::StatePublisher::fill_agent_attitude(
            attitude_msg, latest_odometry_);
        agent_attitude_pub_->publish(attitude_msg);
    }
}

void BasicControllerNode::ned_target_cb(
    const taiphoon_agent::msg::NedTarget::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "ned_target_cb received nullptr, ignoring");
        return;
    }

    if (controller_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Controller not available, NED target ignored");
        return;
    }

    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
        !std::isfinite(msg->z) || std::isinf(msg->yaw))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "NED target contains NaN or Inf, rejecting");
        return;
    }

    collision_target_gate_.set_requested_target(
        {msg->x, msg->y, msg->z, msg->yaw});
    evaluate_collision_target();
}

void BasicControllerNode::on_agent_command(
    const utils::ParsedCommand& cmd)
{
    if (controller_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Controller not available, command ignored");
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Executing command: %s (param=%.2f, has_param=%d)",
                utils::agent_command_to_string(cmd.command),
                cmd.param_value, cmd.has_param ? 1 : 0);

    bool result = false;

    switch (cmd.command)
    {
        case utils::AgentCommand::MODE_RTL:
            result = controller_->switch_to_rtl_mode();
            break;

        case utils::AgentCommand::MODE_LAND:
            result = controller_->switch_to_land_mode();
            break;

        case utils::AgentCommand::MODE_OFFBOARD:
            result = controller_->switch_to_offboard_mode();
            break;

        case utils::AgentCommand::MOVE_FORWARD:
            if (cmd.has_param)
                result = controller_->move_forward(cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "MOVE_FORWARD requires a distance parameter");
            break;

        case utils::AgentCommand::MOVE_BACKWARD:
            if (cmd.has_param)
                result = controller_->move_forward(-cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "MOVE_BACKWARD requires a distance parameter");
            break;

        case utils::AgentCommand::MOVE_LEFT:
            if (cmd.has_param)
                result = controller_->move_right(-cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "MOVE_LEFT requires a distance parameter");
            break;

        case utils::AgentCommand::MOVE_RIGHT:
            if (cmd.has_param)
                result = controller_->move_right(cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "MOVE_RIGHT requires a distance parameter");
            break;

        case utils::AgentCommand::ARM:
            result = controller_->arm();
            break;

        case utils::AgentCommand::DISARM:
            result = controller_->disarm();
            break;

        case utils::AgentCommand::TAKEOFF:
            {
                float alt = cmd.has_param
                    ? cmd.param_value
                    : static_cast<float>(target_altitude_m_);
                result = controller_->takeoff(alt);
            }
            break;

        case utils::AgentCommand::MOVE_UP:
            if (cmd.has_param)
                result = controller_->move_vertical(cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "UP requires a distance parameter");
            break;

        case utils::AgentCommand::MOVE_DOWN:
            if (cmd.has_param)
                result = controller_->move_vertical(-cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "DOWN requires a distance parameter");
            break;

        case utils::AgentCommand::YAW:
            if (cmd.has_param)
                result = controller_->rotate_yaw(cmd.param_value);
            else
                RCLCPP_WARN(this->get_logger(),
                           "YAW requires an angle parameter");
            break;

        case utils::AgentCommand::MOVE_POINT:
            if (cmd.has_multi_params)
                result = controller_->set_ned_target(
                    cmd.param_value, cmd.param2, cmd.param3, cmd.param4);
            else
                RCLCPP_WARN(this->get_logger(),
                           "MOVE_POINT requires x y z yaw parameters");
            break;

        default:
            RCLCPP_WARN(this->get_logger(),
                       "Unhandled command type");
            break;
    }

    if (result)
    {
        if (is_position_motion_command(cmd.command))
        {
            register_controller_motion_target(
                cmd.command == utils::AgentCommand::MOVE_POINT
                    ? cmd.param4
                    : std::numeric_limits<float>::quiet_NaN());
        }
        else if (is_collision_bypass_command(cmd.command))
        {
            // 安全中止命令不得被舊移動目標自動恢復覆蓋。
            collision_target_gate_.clear_target();
        }

        RCLCPP_INFO(this->get_logger(),
                    "Command '%s' executed successfully",
                    utils::agent_command_to_string(cmd.command));
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "Command '%s' execution failed",
                    utils::agent_command_to_string(cmd.command));
    }
}

void BasicControllerNode::collision_peer_state_cb(
    int agent_index,
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "collision_peer_state_cb received nullptr, ignoring");
        return;
    }

    auto& peer = collision_peer_states_[agent_index];
    peer.state = *msg;
    peer.last_arrival = std::chrono::steady_clock::now();
    peer.has_state = true;
    peer.is_discovered = true;
}

void BasicControllerNode::scan_collision_peers_cb()
{
    const auto node_names = this->get_node_names();
    const auto found_indices = formation::PeerMatcher::filter_agent_nodes(
        node_names, agent_namespace_);
    std::set<int> discovered_indices;

    for (const int agent_index : found_indices)
    {
        discovered_indices.insert(agent_index);
        collision_peer_states_[agent_index].is_discovered = true;
        if (collision_peer_subs_.find(agent_index) != collision_peer_subs_.end())
        {
            continue;
        }

        const std::string topic = "/fmu" + std::to_string(agent_index) +
            "_agent/state";
        auto subscription =
            this->create_subscription<taiphoon_agent::msg::AgentState>(
                topic, rclcpp::QoS(10).best_effort(),
                [this, agent_index](
                    const taiphoon_agent::msg::AgentState::SharedPtr msg)
                {
                    this->collision_peer_state_cb(agent_index, msg);
                });
        if (subscription != nullptr)
        {
            collision_peer_subs_[agent_index] = subscription;
            RCLCPP_INFO(this->get_logger(),
                        "Collision guard discovered peer %d at %s",
                        agent_index, topic.c_str());
        }
    }

    std::set<int> known_indices;
    for (const auto& peer : collision_peer_states_)
    {
        known_indices.insert(peer.first);
    }

    const auto removed_indices = formation::PeerMatcher::find_removed_indices(
        known_indices, discovered_indices);
    for (const int agent_index : removed_indices)
    {
        collision_peer_subs_.erase(agent_index);
        collision_peer_states_.erase(agent_index);
        RCLCPP_INFO(this->get_logger(),
                    "Collision guard removed disappeared peer %d",
                    agent_index);
    }
}

std::vector<safety::PeerCollisionObservation>
BasicControllerNode::build_collision_peer_observations() const
{
    std::vector<safety::PeerCollisionObservation> observations;
    observations.reserve(collision_peer_states_.size());
    const auto current_ned = controller_->get_current_position_ned();
    const auto now = std::chrono::steady_clock::now();

    const std::array<float, 3> own_global{
        static_cast<float>(latest_gps_.lat),
        static_cast<float>(latest_gps_.lon),
        latest_gps_.alt};
    const bool is_own_global_valid = has_gps_data_ &&
        std::isfinite(own_global.at(0)) &&
        std::isfinite(own_global.at(1)) &&
        std::isfinite(own_global.at(2));

    for (const auto& entry : collision_peer_states_)
    {
        safety::PeerCollisionObservation observation;
        observation.agent_index = entry.first;
        const auto& peer = entry.second;
        const double age_s = peer.has_state
            ? std::chrono::duration<double>(now - peer.last_arrival).count()
            : std::numeric_limits<double>::infinity();

        const std::array<float, 3> peer_global{
            peer.state.position_global.at(0),
            peer.state.position_global.at(1),
            peer.state.position_global.at(2)};
        const bool is_peer_global_valid =
            std::isfinite(peer_global.at(0)) &&
            std::isfinite(peer_global.at(1)) &&
            std::isfinite(peer_global.at(2));
        const bool is_state_valid = peer.is_discovered && peer.has_state &&
            peer.state.online && peer.state.px4_state_valid &&
            age_s <= collision_peer_timeout_s_ &&
            is_own_global_valid && is_peer_global_valid;

        if (is_state_valid)
        {
            observation.is_valid =
                formation::FormationCalculator::compute_common_frame_position(
                    current_ned, own_global, peer_global,
                    observation.position_ned);
        }
        observations.push_back(observation);
    }
    return observations;
}

void BasicControllerNode::evaluate_collision_target()
{
    if (controller_ == nullptr || !has_odometry_data_)
    {
        return;
    }

    const auto evaluation = collision_target_gate_.evaluate(
        controller_->get_current_position_ned(),
        build_collision_peer_observations());

    if (evaluation.action == safety::CollisionTargetAction::APPLY_TARGET)
    {
        if (controller_->set_ned_target(
                evaluation.target_ned.at(0), evaluation.target_ned.at(1),
                evaluation.target_ned.at(2), evaluation.yaw_deg))
        {
            RCLCPP_INFO(this->get_logger(),
                        "Collision-checked target applied; minimum distance %.2f m",
                        static_cast<double>(evaluation.decision.minimum_distance_m));
        }
    }
    else if (evaluation.action == safety::CollisionTargetAction::HOLD_POSITION)
    {
        controller_->hold_current_position();
        RCLCPP_WARN(this->get_logger(),
                    "Collision guard holding position; peer=%d, minimum distance=%.2f m",
                    evaluation.decision.conflicting_agent_index,
                    static_cast<double>(evaluation.decision.minimum_distance_m));
    }
}

void BasicControllerNode::register_controller_motion_target(float yaw_deg)
{
    if (controller_ == nullptr)
    {
        return;
    }
    const auto target = controller_->get_target_position_ned();
    collision_target_gate_.set_requested_target(
        {target.at(0), target.at(1), target.at(2), yaw_deg});
    evaluate_collision_target();
}

bool BasicControllerNode::is_position_motion_command(
    utils::AgentCommand command)
{
    switch (command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
        case utils::AgentCommand::MOVE_BACKWARD:
        case utils::AgentCommand::MOVE_LEFT:
        case utils::AgentCommand::MOVE_RIGHT:
        case utils::AgentCommand::MOVE_UP:
        case utils::AgentCommand::MOVE_DOWN:
        case utils::AgentCommand::MOVE_POINT:
            return true;
        default:
            return false;
    }
}

bool BasicControllerNode::is_collision_bypass_command(
    utils::AgentCommand command)
{
    return command == utils::AgentCommand::MODE_RTL ||
        command == utils::AgentCommand::MODE_LAND ||
        command == utils::AgentCommand::DISARM;
}

}  // namespace nodes
}  // namespace taiphoon
