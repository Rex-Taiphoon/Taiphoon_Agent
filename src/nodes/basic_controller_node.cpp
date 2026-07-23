/**
 * @file basic_controller_node.cpp
 * @brief BasicController ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include "nodes/basic_controller_node.hpp"

namespace taiphoon
{
namespace nodes
{

BasicControllerNode::BasicControllerNode()
    : rclcpp::Node("basic_controller_node")
{
    // L2-19：宣告 ROS2 參數並指定預設值
    this->declare_parameter<double>("target_altitude_m", DEFAULT_TARGET_ALTITUDE_M);
    this->declare_parameter<double>("hover_duration_s", DEFAULT_HOVER_DURATION_S);
    this->declare_parameter<double>("startup_delay_s", 0.0);
    this->declare_parameter<int>("target_system", 1);
    this->declare_parameter<std::string>("agent_namespace", "fmu1");

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    hover_duration_s_ = this->get_parameter("hover_duration_s").as_double();
    startup_delay_s_ = this->get_parameter("startup_delay_s").as_double();
    target_system_ = this->get_parameter("target_system").as_int();
    agent_namespace_ = this->get_parameter("agent_namespace").as_string();

    RCLCPP_INFO(this->get_logger(),
                "Parameters: target_altitude=%.2f m, hover_duration=%.2f s, "
                "startup_delay=%.1f s, target_system=%d, agent_namespace=%s",
                target_altitude_m_, hover_duration_s_,
                startup_delay_s_, target_system_,
                agent_namespace_.c_str());

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

    // --- 初始化里程計訂閱者 ---
    if (!initialize_odometry_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Odometry subscriber init failed");
        return;
    }

    // --- 初始化命令處理器 ---
    if (!initialize_agent_command_handler())
    {
        RCLCPP_ERROR(this->get_logger(), "Agent command handler init failed");
        return;
    }

    // --- 建立控制迴圈計時器 ---
    auto period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / CONTROL_LOOP_RATE_HZ));

    control_loop_timer_ = this->create_wall_timer(
        period_ms,
        std::bind(&BasicControllerNode::control_loop_cb, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
                "BasicControllerNode initialized, control rate %.0f Hz",
                CONTROL_LOOP_RATE_HZ);
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

    // 驅動控制命令發布
    controller_->publish_control_commands();

    // 如果任務序列尚未啟動，檢查是否應啟動
    if (!sequence_started_)
    {
        auto elapsed = this->now() - start_time_;
        if (elapsed.seconds() >= startup_delay_s_)
        {
            sequence_started_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Startup delay (%.1f s) elapsed, triggering sequence",
                        startup_delay_s_);
            trigger_takeoff_land_sequence();
        }
    }
}

void BasicControllerNode::trigger_takeoff_land_sequence()
{
    RCLCPP_INFO(this->get_logger(),
                "============================================");
    RCLCPP_INFO(this->get_logger(),
                "  Triggering takeoff-land sequence");
    RCLCPP_INFO(this->get_logger(),
                "  Target altitude: %.2f m", target_altitude_m_);
    RCLCPP_INFO(this->get_logger(),
                "  Hover duration: %.2f s", hover_duration_s_);
    RCLCPP_INFO(this->get_logger(),
                "============================================");

    if (controller_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller is nullptr");
        return;
    }

    bool success = controller_->execute_takeoff_land_sequence(
        static_cast<float>(target_altitude_m_),
        static_cast<float>(hover_duration_s_));

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Takeoff-land sequence failed");
    }
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
            result = controller_->takeoff(
                static_cast<float>(target_altitude_m_));
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
