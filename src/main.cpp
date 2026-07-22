/**
 * @file main.cpp
 * @brief Taiphoon Agent BasicController 進入點
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nodes/basic_controller_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<taiphoon::nodes::BasicControllerNode>();

    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Failed to create BasicControllerNode");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Taiphoon Agent BasicController starting...");

    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main"),
                "Taiphoon Agent BasicController shutting down");

    rclcpp::shutdown();
    return 0;
}