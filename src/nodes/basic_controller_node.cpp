/**
 * @file basic_controller_node.cpp
 * @brief BasicController ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include "nodes/basic_controller_node.hpp"

#include <cmath>

namespace taiphoon
{
namespace nodes
{

using taiphoon::controllers::FlightState;

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

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    hover_duration_s_ = this->get_parameter("hover_duration_s").as_double();
    startup_delay_s_ = this->get_parameter("startup_delay_s").as_double();
    target_system_ = this->get_parameter("target_system").as_int();
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    auto_start_sequence_ = this->get_parameter("auto_start_sequence").as_bool();
    state_publish_rate_hz_ = this->get_parameter("state_publish_rate_hz").as_double();
    position_deviation_threshold_m_ =
        this->get_parameter("position_deviation_threshold_m").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Parameters: target_altitude=%.2f m, hover_duration=%.2f s, "
                "startup_delay=%.1f s, target_system=%d, agent_namespace=%s, "
                "state_publish_rate=%.1f Hz, deviation_threshold=%.2f m",
                target_altitude_m_, hover_duration_s_,
                startup_delay_s_, target_system_,
                agent_namespace_.c_str(),
                state_publish_rate_hz_, position_deviation_threshold_m_);

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

    // --- 初始化里程計訂閱者 ---
    if (!initialize_odometry_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry subscriber init failed");
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

    // L1-12：檢查 Subscriber 是否建立成功
    if (odometry_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create odometry subscriber");
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
    // 建立命令處理器，傳入 Agent 命名空間
    agent_cmd_handler_ = std::make_unique<controllers::AgentCommandHandler>(
        this, agent_namespace_);

    if (agent_cmd_handler_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "AgentCommandHandler creation failed");
        return false;
    }

    // 註冊命令執行回呼
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
    // L1-03：ROS2 回呼必須檢查訊息指標
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "odometry_cb received nullptr message, ignoring");
        return;
    }

    // 快取最新里程計資料（用於狀態發布）
    latest_odometry_ = *msg;
    has_odometry_data_ = true;

    if (controller_ != nullptr)
    {
        controller_->update_odometry(*msg);
    }
}

void BasicControllerNode::control_loop_cb()
{
    if (controller_ == nullptr)
    {
        return;
    }

    // --- 位置安全監控：檢查實際位置與目標位置偏差 ---
    {
        auto current_pos = controller_->get_current_position_ned();
        auto target_pos = controller_->get_target_position_ned();

        // TAKEOFF 期間不檢查位置偏差（爬升為有意行為）
        if (controller_->get_flight_state() != FlightState::TAKEOFF &&
            position_safety_monitor_.is_deviation_exceeded(
                current_pos, target_pos))
        {
            float dist = safety::PositionSafetyMonitor::compute_distance(
                current_pos, target_pos);

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                                 3000,
                                 "Position deviation %.2f m exceeds threshold "
                                 "%.2f m — triggering hover intervention",
                                 static_cast<double>(dist),
                                 position_deviation_threshold_m_);

            // 觸發 hover 介入：重設目標為目前位置
            controller_->set_ned_target(
                current_pos[0], current_pos[1], current_pos[2],
                std::numeric_limits<float>::quiet_NaN());
        }
    }

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
    // 尚未收到里程計資料時不發布
    if (!has_odometry_data_)
    {
        return;
    }

    if (controller_ == nullptr)
    {
        return;
    }

    // --- 發布 AgentState ---
    {
        taiphoon_agent::msg::AgentState state_msg;
        safety::StatePublisher::fill_agent_state(
            state_msg, latest_odometry_,
            controller_->get_flight_state(), true);
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

    controller_->set_ned_target(msg->x, msg->y, msg->z, msg->yaw);
}

void BasicControllerNode::on_agent_command(
    const utils::ParsedCommand& cmd)
{
    // L1-02：檢查控制器是否有效
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

    // 將高階命令轉譯為 BasicController 的呼叫
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
                // 有命令參數時使用命令指定的高度，否則使用 YAML 設定值
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

}  // namespace nodes
}  // namespace taiphoon
