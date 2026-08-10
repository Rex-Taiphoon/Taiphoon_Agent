/**
 * @file main_ego_swarm_planner.cpp
 * @brief EGO-Swarm Planner ROS2 節點進入點
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "rclcpp/rclcpp.hpp"

#include "nodes/ego_swarm_planner_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<taiphoon::nodes::EgoSwarmPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
