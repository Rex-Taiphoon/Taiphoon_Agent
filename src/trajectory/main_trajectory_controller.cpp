/**
 * @file main_trajectory_controller.cpp
 * @brief Taiphoon Agent TrajectoryController 進入點
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "trajectory/TrajectoryController.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<taiphoon::nodes::TrajectoryController>();

    if (node == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main_trajectory_controller"),
                     "Failed to create TrajectoryController");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(rclcpp::get_logger("main_trajectory_controller"),
                "Taiphoon Agent TrajectoryController starting...");

    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("main_trajectory_controller"),
                "Taiphoon Agent TrajectoryController shutting down");

    rclcpp::shutdown();
    return 0;
}
