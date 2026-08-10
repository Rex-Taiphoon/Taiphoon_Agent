/**
 * @file main_obstacle_cloud_simulator.cpp
 * @brief PointCloud2 障礙物模擬節點進入點
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "rclcpp/rclcpp.hpp"

#include "nodes/obstacle_cloud_simulator_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<taiphoon::nodes::ObstacleCloudSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
