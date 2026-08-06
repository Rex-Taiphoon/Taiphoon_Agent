/**
 * @file main_agent_supervisor.cpp
 * @brief Taiphoon_Agent 容器生命週期管理節點進入點
 * @author Taiphoon Agent Team
 * @date 2026-08-06
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nodes/AgentSupervisor.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<taiphoon::nodes::AgentSupervisor>();
    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main_agent_supervisor"),
                     "Failed to create AgentSupervisor");
        rclcpp::shutdown();
        return 1;
    }

    if (!node->initialize())
    {
        RCLCPP_ERROR(rclcpp::get_logger("main_agent_supervisor"),
                     "Failed to initialize AgentSupervisor");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main_agent_supervisor"),
                "Taiphoon Agent supervisor starting...");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
