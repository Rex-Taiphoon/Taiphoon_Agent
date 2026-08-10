/**
 * @file obstacle_cloud_simulator_node.hpp
 * @brief PointCloud2 障礙物模擬 ROS2 節點
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "perception/ObstacleCloudGenerator.hpp"

namespace taiphoon
{
namespace nodes
{

/**
 * @brief 讀取簡化 YAML 場景並發布 PointCloud2 的模擬節點
 */
class ObstacleCloudSimulatorNode : public rclcpp::Node
{
public:
    /** @brief 建立障礙物點雲模擬節點。 */
    ObstacleCloudSimulatorNode();
    ~ObstacleCloudSimulatorNode() override = default;

private:
    bool load_scene_file(const std::string& path);
    void publish_cloud_cb();

    std::string agent_namespace_{"fmu1_agent"};
    std::string config_file_{};
    double publish_rate_hz_{15.0};
    perception::GenerationConfig generation_config_{};
    perception::SensorPose sensor_pose_{};
    std::vector<perception::ObstacleDefinition> obstacles_{};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
