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
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"

#include "ego_swarm/SwarmFrame.hpp"
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
    bool initialize_pose_subscribers();
    void own_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void origin_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void attitude_cb(const taiphoon_agent::msg::AgentAttitude::SharedPtr msg);
    void update_sensor_pose();
    void publish_cloud_cb();
    int compute_agent_index() const;

    std::string agent_namespace_{"fmu1_agent"};
    std::string config_file_{};
    int agent_index_{1};
    int swarm_origin_agent_index_{1};
    double publish_rate_hz_{15.0};
    perception::GenerationConfig generation_config_{};
    perception::SensorPose sensor_pose_{};
    std::vector<perception::ObstacleDefinition> obstacles_{};
    ego_swarm::SwarmFrame swarm_frame_{};
    taiphoon_agent::msg::AgentState own_state_{};
    taiphoon_agent::msg::AgentState origin_state_{};
    bool has_own_state_{false};
    bool has_origin_state_{false};
    bool has_sensor_pose_{false};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr
        own_state_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr
        origin_state_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentAttitude>::SharedPtr
        attitude_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
