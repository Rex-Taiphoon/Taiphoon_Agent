/**
 * @file ego_swarm_planner_node.hpp
 * @brief EGO-Swarm ROS2 節點，組合定位、編隊、障礙物與同儕軌跡資料
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_trajectory.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/planner_status.hpp"

#include "ego_swarm/EgoSwarmPlanner.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/PositionArrival.hpp"
#include "ego_swarm/SwarmFrame.hpp"
#include "formation/PeerMatcher.hpp"
#include "perception/RawPointObstacleMap.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_EGO_PLANNER_RATE_HZ = 10.0;
constexpr double DEFAULT_FORMATION_MIN_DISTANCE_M = 3.0;
constexpr double DEFAULT_PEER_COLLISION_MIN_DISTANCE_M = 3.0;
constexpr double DEFAULT_OBSTACLE_CLEARANCE_M = 1.0;
constexpr double DEFAULT_FORMATION_RADIUS_M = 5.0;
constexpr double DEFAULT_FORMATION_SPACING_M = 3.0;
constexpr double DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M = 0.5;
constexpr double DEFAULT_GLOBAL_ARRIVAL_TOLERANCE_M = 1.0;
constexpr double DEFAULT_PEER_TRAJECTORY_TIMEOUT_S = 1.0;
constexpr double DEFAULT_MAX_PEER_TIME_SKEW_S = 0.5;
constexpr double DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S = 1.0;

/**
 * @brief EGO-Swarm Planner ROS2 節點
 *
 * 節點只負責 ROS2 資料收發與資料轉換；局部候選軌跡計算由
 * `ego_swarm::EgoSwarmPlanner` 純 C++ 模組完成。
 */
class EgoSwarmPlannerNode : public rclcpp::Node
{
public:
    /** @brief 建立 EGO-Swarm Planner 節點。 */
    EgoSwarmPlannerNode();
    ~EgoSwarmPlannerNode() override = default;

private:
    struct PeerState
    {
        taiphoon_agent::msg::AgentState state{};
        std::chrono::steady_clock::time_point last_arrival{};
        bool has_state{false};
    };

    void own_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void peer_state_cb(
        int agent_index,
        const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void ned_target_cb(const taiphoon_agent::msg::NedTarget::SharedPtr msg);
    void mission_intent_cb(const std_msgs::msg::String::SharedPtr msg);
    void agent_command_cb(const std_msgs::msg::String::SharedPtr msg);
    void all_agent_command_cb(const std_msgs::msg::String::SharedPtr msg);
    void obstacle_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void trajectory_cb(
        const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg);
    void planner_loop_cb();
    void scan_peers_cb();

    bool initialize_subscribers();
    bool initialize_publishers();
    bool update_swarm_frame();
    bool update_local_swarm_transform();
    bool get_position_in_swarm_frame(
        const taiphoon_agent::msg::AgentState& state,
        ego_swarm::Vector3& position) const;
    bool get_target_in_local_ned(
        const ego_swarm::Vector3& target,
        ego_swarm::Vector3& local_target) const;
    ego_swarm::PositionArrivalResult evaluate_target_arrival(
        const ego_swarm::Vector3& current_global_ned,
        const ego_swarm::Vector3& target) const;
    void invalidate_active_plan();
    bool get_current_target(ego_swarm::Vector3& target) const;
    void set_goal_from_command(const std::string& command);
    void publish_plan(
        const ego_swarm::PlannerResult& result,
        const ego_swarm::Vector3& origin_position,
        const rclcpp::Time& now,
        const rclcpp::Time& start_time,
        uint32_t trajectory_id);
    void publish_status(
        uint8_t state,
        const std::string& reason,
        const ego_swarm::PlannerResult* result,
        bool localization_valid,
        bool peer_data_valid,
        const rclcpp::Time& stamp);
    std::string obstacle_map_status_reason(double now_s) const;
    int compute_self_index() const;

    std::string agent_namespace_{"fmu1_agent"};
    int agent_index_{1};
    int swarm_origin_agent_index_{1};
    double planning_rate_hz_{DEFAULT_EGO_PLANNER_RATE_HZ};
    double peer_trajectory_timeout_s_{DEFAULT_PEER_TRAJECTORY_TIMEOUT_S};
    double max_peer_time_skew_s_{DEFAULT_MAX_PEER_TIME_SKEW_S};
    double obstacle_cloud_timeout_s_{DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S};
    bool require_obstacle_cloud_{true};
    double formation_min_distance_m_{DEFAULT_FORMATION_MIN_DISTANCE_M};
    double local_arrival_tolerance_m_{DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M};
    double global_arrival_tolerance_m_{DEFAULT_GLOBAL_ARRIVAL_TOLERANCE_M};

    ego_swarm::PlannerConfig planner_config_{};
    ego_swarm::EgoSwarmPlanner planner_{};
    ego_swarm::PeerTrajectoryTable peer_trajectories_{};
    ego_swarm::SwarmFrame swarm_frame_{};
    ego_swarm::GeoCoordinate frame_origin_{};
    /// 目前固定 swarm_ned 原點的 Agent ID；-1 表示尚未選定原點。
    int frame_origin_agent_index_{-1};
    uint32_t frame_revision_{0U};
    bool has_local_swarm_transform_{false};
    ego_swarm::Vector3 local_swarm_offset_{0.0, 0.0, 0.0};

    bool has_own_state_{false};
    bool has_goal_{false};
    bool has_formation_{false};
    bool preserve_altitude_{false};
    bool has_active_plan_{false};
    taiphoon_agent::msg::AgentState own_state_{};
    ego_swarm::Vector3 goal_position_{0.0, 0.0, 0.0};
    ego_swarm::PlannerResult active_plan_{};
    ego_swarm::Vector3 active_plan_target_{0.0, 0.0, 0.0};
    rclcpp::Time active_plan_start_time_{
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME};
    uint32_t active_plan_trajectory_id_{0U};
    perception::RawPointObstacleMap obstacle_map_{};
    ego_swarm::FormationConfig formation_config_{};
    ego_swarm::Vector3 formation_center_{0.0, 0.0, 0.0};

    std::map<int, PeerState> peers_{};
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        peer_subscriptions_{};

    uint32_t trajectory_id_{0U};
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr own_state_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr ned_target_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_intent_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agent_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr all_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_sub_;

    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        local_trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::PlannerStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr peer_scan_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
