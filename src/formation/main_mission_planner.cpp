/**
 * @file main_mission_planner.cpp
 * @brief Taiphoon Agent MissionPlanner 進入點
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "formation/MissionPlanner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<taiphoon::nodes::MissionPlanner>();

    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main_mission_planner"),
                     "Failed to create MissionPlanner");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main_mission_planner"),
                "Taiphoon Agent MissionPlanner starting...");

    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main_mission_planner"),
                "Taiphoon Agent MissionPlanner shutting down");

    rclcpp::shutdown();
    return 0;
}
