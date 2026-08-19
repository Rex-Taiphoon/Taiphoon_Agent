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
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "taiphoon_agent/msg/agent_trajectory.hpp"
#include "taiphoon_agent/msg/formation_epoch.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/planner_status.hpp"

#include "ego_swarm/ContinuousSafetyValidator.hpp"
#include "ego_swarm/FormationConvergence.hpp"
#include "ego_swarm/FormationReservation.hpp"
#include "ego_swarm/EmergencyBrakePlanner.hpp"
#include "ego_swarm/FormationAssignment.hpp"
#include "ego_swarm/FormationSlotPlanner.hpp"
#include "ego_swarm/ITrajectoryPlanner.hpp"
#include "ego_swarm/MovementFrame.hpp"
#include "ego_swarm/OptimizedEgoPlanner.hpp"
#include "ego_swarm/PeerTrajectoryTable.hpp"
#include "ego_swarm/PlannerAcceptancePolicy.hpp"
#include "ego_swarm/PlannerFeatureFlags.hpp"
#include "ego_swarm/PlanningTargetSelector.hpp"
#include "ego_swarm/PlannerTargetSemantics.hpp"
#include "ego_swarm/PositionArrival.hpp"
#include "ego_swarm/ReplanTrigger.hpp"
#include "ego_swarm/SwarmFrame.hpp"
#include "ego_swarm/TrajectoryFactory.hpp"
#include "ego_swarm/TrajectoryReservationPolicy.hpp"
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
constexpr double DEFAULT_PEER_TRAJECTORY_TIMEOUT_S = 2.0;
constexpr double DEFAULT_MAX_PEER_TIME_SKEW_S = 1.0;
constexpr double DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S = 1.0;
constexpr double DEFAULT_MAX_VELOCITY_MPS = 2.5;
constexpr double DEFAULT_MAX_ACCELERATION_MPS2 = 1.5;
constexpr double DEFAULT_MAX_JERK_MPS3 = 3.0;
constexpr double DEFAULT_MOVE_ATTITUDE_TIMEOUT_S = 1.0;
constexpr double DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M = 2.0;
constexpr int DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT = 2;
constexpr double FORMATION_PLAN_START_DELAY_S = 0.5;
constexpr double PEER_REPLAN_RETRY_DELAY_S = 0.5;
// SITL 與實機 callback 負載下，central finite-difference 需要保留足夠
// 的 wall-time；仍遠低於 10 Hz planner cycle，避免移動中反覆 timeout。
constexpr double DEFAULT_OPTIMIZER_TIME_BUDGET_MS = 50.0;
constexpr int DEFAULT_OPTIMIZER_MAX_ITERATIONS = 40;
constexpr double DEFAULT_FORMATION_CROSSING_PENALTY_M = 10.0;
constexpr double DEFAULT_RESERVATION_LEAD_TIME_S = 0.5;
constexpr double DEFAULT_RESERVATION_STEP_S = 0.5;
constexpr double DEFAULT_RESERVATION_HORIZON_S = 5.0;

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
        ego_swarm::Vector3 previous_position_ned{0.0, 0.0, 0.0};
        ego_swarm::Vector3 velocity_ned{0.0, 0.0, 0.0};
        double previous_position_time_s{0.0};
        bool has_velocity{false};
        bool has_state{false};
    };

    void own_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void own_attitude_cb(
        const taiphoon_agent::msg::AgentAttitude::SharedPtr msg);
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
    void formation_epoch_cb(
        const taiphoon_agent::msg::FormationEpoch::SharedPtr msg);

    bool initialize_subscribers();
    bool initialize_publishers();
    bool attitude_is_fresh() const;
    bool make_formation_proposal(const std::vector<int>& member_ids);
    bool update_formation_assignment();
    bool get_formation_target_for_agent(
        int agent_id, ego_swarm::Vector3& target) const;
    ego_swarm::FormationConvergenceResult evaluate_formation_convergence(
        double now_s) const;
    bool epoch_is_valid(
        const taiphoon_agent::msg::FormationEpoch& epoch) const;
    bool epoch_is_same_snapshot(
        const taiphoon_agent::msg::FormationEpoch& first,
        const taiphoon_agent::msg::FormationEpoch& second) const;
    bool epoch_is_higher_priority(
        const taiphoon_agent::msg::FormationEpoch& first,
        const taiphoon_agent::msg::FormationEpoch& second) const;
    bool epoch_parent_matches_current(
        const taiphoon_agent::msg::FormationEpoch& epoch) const;
    void adopt_formation_epoch(
        const taiphoon_agent::msg::FormationEpoch& epoch);
    void maybe_activate_formation_epoch();
    void publish_formation_epoch(uint8_t status);
    std::vector<int> stable_member_ids() const;
    bool formation_config_matches(
        const ego_swarm::FormationConfig& first,
        const ego_swarm::FormationConfig& second) const;
    double live_peer_min_distance(
        const ego_swarm::Vector3& current) const;
    bool update_swarm_frame();
    bool update_local_swarm_transform();
    bool get_position_in_swarm_frame(
        const taiphoon_agent::msg::AgentState& state,
        ego_swarm::Vector3& position) const;
    bool get_target_in_local_ned(
        const ego_swarm::Vector3& target,
        ego_swarm::Vector3& local_target) const;
    /** @brief 清除不再代表目前控制狀態的 terminal hold heartbeat。 */
    void clear_terminal_hold();
    ego_swarm::PositionArrivalResult evaluate_target_arrival(
        const ego_swarm::Vector3& current_global_ned,
        const ego_swarm::Vector3& final_target) const;
    void invalidate_active_plan();
    /** @brief 取得目前任務真正的 Final Target。 */
    bool get_current_final_target(ego_swarm::Vector3& final_target) const;
    void update_vehicle_motion_state(
        const taiphoon_agent::msg::AgentState& state);
    ego_swarm::VehiclePlanningState make_planning_state(
        const ego_swarm::Vector3& position,
        double now_s) const;
    ego_swarm::SafetyValidationResult validate_plan(
        const ego_swarm::BsplineTrajectory& trajectory,
        const ego_swarm::VehiclePlanningState& state,
        double now_s,
        double trajectory_start_time_s,
        const std::vector<ego_swarm::PeerTrajectory>* peer_snapshot = nullptr) const;
    bool publish_emergency_brake(
        const ego_swarm::VehiclePlanningState& state,
        const ego_swarm::Vector3& active_target,
        const std::string& reason,
        const rclcpp::Time& now,
        bool peer_data_valid);
    bool publish_waiting_hold(
        const ego_swarm::Vector3& current,
        const ego_swarm::Vector3& target,
        const std::string& reason,
        const rclcpp::Time& now,
        bool peer_data_valid);
    void set_goal_from_command(const std::string& command);
    void publish_plan(
        const ego_swarm::PlannerResult& result,
        const ego_swarm::Vector3& origin_position,
        const rclcpp::Time& now,
        const rclcpp::Time& start_time,
        uint32_t trajectory_id,
        bool terminal_hold = false);
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
    double move_attitude_timeout_s_{DEFAULT_MOVE_ATTITUDE_TIMEOUT_S};
    double formation_slot_switch_penalty_m_ =
        DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M;
    int formation_member_stable_scan_count_ =
        DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT;
    std::string planner_backend_{"optimized"};
    int optimizer_max_iterations_{DEFAULT_OPTIMIZER_MAX_ITERATIONS};
    double optimizer_time_budget_ms_{DEFAULT_OPTIMIZER_TIME_BUDGET_MS};
    double formation_crossing_penalty_m_ =
        DEFAULT_FORMATION_CROSSING_PENALTY_M;
    double reservation_lead_time_s_{DEFAULT_RESERVATION_LEAD_TIME_S};
    double reservation_step_s_{DEFAULT_RESERVATION_STEP_S};
    double reservation_horizon_s_{DEFAULT_RESERVATION_HORIZON_S};
    ego_swarm::PlannerFeatureFlags planner_feature_flags_{};

    ego_swarm::PlannerConfig planner_config_{};
    std::unique_ptr<ego_swarm::ITrajectoryPlanner> selected_planner_{};
    ego_swarm::ContinuousSafetyValidator safety_validator_{};
    ego_swarm::EmergencyBrakePlanner emergency_brake_planner_{};
    ego_swarm::PeerTrajectoryTable peer_trajectories_{};
    ego_swarm::SwarmFrame swarm_frame_{};
    ego_swarm::GeoCoordinate frame_origin_{};
    /// 目前 local Home 所屬的 Agent ID（本機自身）；-1 表示尚未建立 Home。
    int frame_origin_agent_index_{-1};
    uint32_t frame_revision_{0U};
    bool has_local_swarm_transform_{false};
    ego_swarm::Vector3 local_swarm_offset_{0.0, 0.0, 0.0};

    bool has_own_state_{false};
    bool has_previous_state_sample_{false};
    bool has_vehicle_motion_state_{false};
    double previous_state_timestamp_s_{0.0};
    ego_swarm::Vector3 previous_position_ned_{0.0, 0.0, 0.0};
    ego_swarm::Vector3 previous_velocity_ned_{0.0, 0.0, 0.0};
    ego_swarm::VehiclePlanningState vehicle_motion_state_{};
    bool has_goal_{false};
    /// 目標到達後仍保留最後的任務目標，供下一個相對命令累加。
    bool has_position_target_{false};
    bool has_formation_{false};
    bool formation_command_pending_{false};
    bool formation_membership_change_pending_{false};
    bool formation_initial_plan_pending_{false};
    bool preserve_altitude_{false};
    bool has_active_plan_{false};
    bool emergency_brake_active_{false};
    bool waiting_for_replan_{false};
    double waiting_retry_time_s_{0.0};
    bool has_terminal_hold_{false};
    taiphoon_agent::msg::AgentState own_state_{};
    taiphoon_agent::msg::AgentAttitude own_attitude_{};
    bool has_own_attitude_{false};
    std::chrono::steady_clock::time_point attitude_last_arrival_{};
    /// 累積相對命令使用的目標 yaw；未設定前以第一筆有效 attitude 初始化。
    bool has_target_yaw_{false};
    double target_yaw_rad_{0.0};
    ego_swarm::Vector3 goal_position_{0.0, 0.0, 0.0};
    ego_swarm::PlannerResult active_plan_{};
    ego_swarm::Vector3 active_plan_target_{0.0, 0.0, 0.0};
    /// 目前 active trajectory 所屬任務的真正 Final Target。
    ego_swarm::Vector3 active_plan_final_target_{0.0, 0.0, 0.0};
    rclcpp::Time active_plan_start_time_{
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME};
    /// 最近一次接受正常 planner candidate 的時間，用於
    /// progress replan interval。
    bool has_last_replan_time_{false};
    double last_replan_time_s_{0.0};
    uint32_t active_plan_trajectory_id_{0U};
    ego_swarm::PlannerResult terminal_hold_plan_{};
    ego_swarm::Vector3 terminal_hold_position_{0.0, 0.0, 0.0};
    rclcpp::Time terminal_hold_start_time_{
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME};
    uint32_t terminal_hold_trajectory_id_{0U};
    perception::RawPointObstacleMap obstacle_map_{};
    ego_swarm::FormationConfig formation_config_{};
    ego_swarm::Vector3 formation_center_{0.0, 0.0, 0.0};
    /// 是否已有可供後續編隊命令沿用的 formation center target。
    bool has_formation_center_target_{false};
    std::map<int, std::size_t> formation_assignment_{};

    bool has_accepted_formation_epoch_{false};
    bool has_candidate_formation_epoch_{false};
    taiphoon_agent::msg::FormationEpoch accepted_formation_epoch_{};
    taiphoon_agent::msg::FormationEpoch candidate_formation_epoch_{};
    std::set<int> candidate_adopted_agents_{};
    double last_epoch_publish_s_{0.0};
    std::vector<int> discovery_candidate_members_{};
    std::vector<int> stable_members_{};
    int discovery_same_scan_count_{0};
    std::map<int, int> discovery_absent_scan_count_{};
    std::set<int> currently_discovered_peers_{};

    std::map<int, PeerState> peers_{};
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        peer_subscriptions_{};

    uint32_t trajectory_id_{0U};
    uint32_t last_published_trajectory_id_{0U};
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr own_state_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentAttitude>::SharedPtr
        own_attitude_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr ned_target_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_intent_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agent_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr all_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::FormationEpoch>::SharedPtr
        formation_epoch_sub_;

    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        local_trajectory_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::PlannerStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<taiphoon_agent::msg::FormationEpoch>::SharedPtr
        formation_epoch_pub_;
    rclcpp::TimerBase::SharedPtr planner_timer_;
    rclcpp::TimerBase::SharedPtr peer_scan_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
