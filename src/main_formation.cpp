/**
 * @file main_formation.cpp
 * @brief Taiphoon Agent FormationNode 進入點
 * @author Taiphoon Agent Team
 * @date 2026-07-28
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nodes/formation_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<taiphoon::nodes::FormationNode>();

    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main_formation"),
                     "Failed to create FormationNode");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main_formation"),
                "Taiphoon Agent FormationNode starting...");

    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main_formation"),
                "Taiphoon Agent FormationNode shutting down");

    rclcpp::shutdown();
    return 0;
}
