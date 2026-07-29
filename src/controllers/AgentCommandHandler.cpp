/**
 * @file AgentCommandHandler.cpp
 * @brief 高階命令處理器實作
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include "controllers/AgentCommandHandler.hpp"

namespace taiphoon
{
namespace controllers
{

AgentCommandHandler::AgentCommandHandler(
    rclcpp::Node* node,
    const std::string& agent_namespace)
    : node_(node),
      agent_namespace_(agent_namespace)
{
    // L1-02：檢查指標是否為 nullptr
    if (node_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Node pointer is nullptr");
    }
}

bool AgentCommandHandler::initialize()
{
    // L1-02：檢查指標是否為 nullptr
    if (node_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Cannot init: Node pointer is nullptr");
        return false;
    }

    // 建立特定 Agent 命令訂閱者
    if (!create_agent_command_subscriber())
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Failed to create agent command subscriber");
        return false;
    }

    // 建立全體 Agent 命令訂閱者
    if (!create_all_agent_command_subscriber())
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Failed to create all-agent command subscriber");
        return false;
    }

    initialized_ = true;

    RCLCPP_INFO(rclcpp::get_logger("AgentCommandHandler"),
                "AgentCommandHandler initialized for namespace '%s'",
                agent_namespace_.c_str());

    return true;
}

void AgentCommandHandler::set_command_callback(CommandCallback callback)
{
    command_callback_ = callback;
}

const std::string& AgentCommandHandler::get_agent_namespace() const
{
    return agent_namespace_;
}

bool AgentCommandHandler::create_agent_command_subscriber()
{
    // L2-19：使用 ROS2 參數讀取命令主題名稱
    std::string agent_cmd_topic = "/" + agent_namespace_ + "/command";

    agent_command_sub_ =
        node_->create_subscription<std_msgs::msg::String>(
            agent_cmd_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(&AgentCommandHandler::agent_command_cb, this,
                      std::placeholders::_1));

    // L1-12：檢查 Publisher/Subscriber 是否建立成功
    if (agent_command_sub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Failed to create subscriber on topic '%s'",
                     agent_cmd_topic.c_str());
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AgentCommandHandler"),
                "Subscribed to '%s' for agent-specific commands",
                agent_cmd_topic.c_str());

    return true;
}

bool AgentCommandHandler::create_all_agent_command_subscriber()
{
    // 全體 Agent 命令主題固定為 /fmu_all_agent/command
    const std::string all_cmd_topic = "/fmu_all_agent/command";

    all_agent_command_sub_ =
        node_->create_subscription<std_msgs::msg::String>(
            all_cmd_topic,
            rclcpp::QoS(10).reliable(),
            std::bind(&AgentCommandHandler::all_agent_command_cb, this,
                      std::placeholders::_1));

    // L1-12：檢查 Subscriber 是否建立成功
    if (all_agent_command_sub_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AgentCommandHandler"),
                     "Failed to create subscriber on topic '%s'",
                     all_cmd_topic.c_str());
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AgentCommandHandler"),
                "Subscribed to '%s' for all-agent broadcast commands",
                all_cmd_topic.c_str());

    return true;
}

void AgentCommandHandler::agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    // L1-03：ROS2 回呼必須檢查訊息指標
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "agent_command_cb received nullptr message, ignoring");
        return;
    }

    const std::string& cmd_str = msg->data;

    RCLCPP_INFO(rclcpp::get_logger("AgentCommandHandler"),
                "Received agent command on '%s': '%s'",
                agent_namespace_.c_str(), cmd_str.c_str());

    // 解析命令字串
    utils::ParsedCommand parsed = utils::parse_command_string(cmd_str);

    // L1-03：檢查解析結果是否有效
    if (parsed.command == utils::AgentCommand::UNKNOWN)
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "Unknown command received: '%s'",
                    cmd_str.c_str());
        return;
    }

    // 如果有註冊回呼則執行
    if (command_callback_)
    {
        command_callback_(parsed);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "No command callback registered, command '%s' ignored",
                    cmd_str.c_str());
    }
}

void AgentCommandHandler::all_agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    // L1-03：ROS2 回呼必須檢查訊息指標
    if (msg == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "all_agent_command_cb received nullptr message, ignoring");
        return;
    }

    const std::string& cmd_str = msg->data;

    RCLCPP_INFO(rclcpp::get_logger("AgentCommandHandler"),
                "Received broadcast command for agent '%s': '%s'",
                agent_namespace_.c_str(), cmd_str.c_str());

    // 全體命令與個別命令使用相同的解析邏輯
    utils::ParsedCommand parsed = utils::parse_command_string(cmd_str);

    // L1-03：檢查解析結果是否有效
    if (parsed.command == utils::AgentCommand::UNKNOWN)
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "Unknown broadcast command received: '%s'",
                    cmd_str.c_str());
        return;
    }

    // 如果有註冊回呼則執行
    if (command_callback_)
    {
        command_callback_(parsed);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("AgentCommandHandler"),
                    "No command callback registered, broadcast command '%s' ignored",
                    cmd_str.c_str());
    }
}

}  // namespace controllers
}  // namespace taiphoon
