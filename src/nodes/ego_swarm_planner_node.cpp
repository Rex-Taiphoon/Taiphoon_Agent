/**
 * @file ego_swarm_planner_node.cpp
 * @brief EGO-Swarm ROS2 節點實作
 * @author Taiphoon Agent Team
 * @date 2026-08-10
 */

#include "nodes/ego_swarm_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <cstring>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "utils/CommandTypes.hpp"

namespace taiphoon
{
namespace nodes
{

namespace
{
constexpr uint8_t PLANNER_STATE_IDLE = 0U;
constexpr uint8_t PLANNER_STATE_RUNNING = 1U;
constexpr uint8_t PLANNER_STATE_HOLD = 2U;
constexpr uint8_t PLANNER_STATE_ERROR = 3U;
constexpr double TRAJECTORY_DURATION_ABSOLUTE_EPSILON_S = 1.0e-4;
constexpr double TRAJECTORY_DURATION_RELATIVE_EPSILON = 1.0e-5;
constexpr double FORMATION_EPOCH_HEARTBEAT_PERIOD_S = 1.0;
// Live GPS/state callback 與 PX4 setpoint 反應之間仍有延遲；只用
// collision_distance 直接煞車會把剩餘動能帶進安全門檻內。此緩衝是
// 動態煞停距離以外的固定反應裕度，並非放寬碰撞門檻。
constexpr double LIVE_PEER_REACTION_BUFFER_M = 3.0;

bool normalize_yaw_rad(const double input, double& output)
{
    output = 0.0;
    if (!std::isfinite(input))
    {
        return false;
    }
    output = std::fmod(input + ego_swarm::EGO_SWARM_PI,
                       2.0 * ego_swarm::EGO_SWARM_PI);
    if (output < 0.0)
    {
        output += 2.0 * ego_swarm::EGO_SWARM_PI;
    }
    output -= ego_swarm::EGO_SWARM_PI;
    return std::isfinite(output);
}

std::string optimized_failure_reason(
    const ego_swarm::PlannerTerminationReason reason)
{
    switch (reason)
    {
        case ego_swarm::PlannerTerminationReason::NO_FEASIBLE_PATH:
            return "optimized_no_feasible_horizontal_path";
        case ego_swarm::PlannerTerminationReason::TIMEOUT:
            return "optimized_timeout";
        case ego_swarm::PlannerTerminationReason::NONFINITE_GRADIENT:
            return "optimized_nonfinite_gradient";
        case ego_swarm::PlannerTerminationReason::INFEASIBLE:
        case ego_swarm::PlannerTerminationReason::ITERATION_LIMIT:
        case ego_swarm::PlannerTerminationReason::DIVERGED:
        case ego_swarm::PlannerTerminationReason::INVALID_INPUT:
            return "optimized_infeasible";
        case ego_swarm::PlannerTerminationReason::SUCCESS:
            return "optimized_infeasible";
    }
    return "optimized_infeasible";
}

bool is_finite_state_position(const taiphoon_agent::msg::AgentState& state)
{
    return state.online && state.px4_state_valid &&
        std::isfinite(state.position_global[0]) &&
        std::isfinite(state.position_global[1]) &&
        std::isfinite(state.position_global[2]);
}

ego_swarm::GeoCoordinate to_geo(
    const taiphoon_agent::msg::AgentState& state)
{
    return ego_swarm::GeoCoordinate{
        static_cast<double>(state.position_global[0]),
        static_cast<double>(state.position_global[1]),
        static_cast<double>(state.position_global[2])};
}

std::string command_to_text(const taiphoon_agent::msg::NedTarget& target)
{
    std::ostringstream stream;
    stream << "MOVE_POINT " << target.x << " " << target.y << " "
           << target.z << " " << target.yaw;
    return stream.str();
}

/**
 * @brief 比較訊息 duration 與由控制點推導的 B-spline duration。
 * @param message_duration_s AgentTrajectory 內的 duration
 * @param computed_duration_s BsplineTrajectory 計算出的 duration
 * @return true 表示兩者在絕對或相對容差內一致
 */
bool is_consistent_duration(
    const double message_duration_s,
    const double computed_duration_s)
{
    if (!std::isfinite(message_duration_s) ||
        !std::isfinite(computed_duration_s) ||
        message_duration_s <= 0.0 || computed_duration_s <= 0.0)
    {
        return false;
    }

    const double scale = std::max(
        1.0, std::max(std::abs(message_duration_s),
                      std::abs(computed_duration_s)));
    const double tolerance = std::max(
        TRAJECTORY_DURATION_ABSOLUTE_EPSILON_S,
        TRAJECTORY_DURATION_RELATIVE_EPSILON * scale);
    return std::abs(message_duration_s - computed_duration_s) <= tolerance;
}

uint32_t hash_bytes(uint32_t hash, const void* data, const std::size_t size)
{
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0U; index < size; ++index)
    {
        hash ^= static_cast<uint32_t>(bytes[index]);
        hash *= 16777619U;
    }
    return hash;
}

template<typename T>
uint32_t hash_value(uint32_t hash, const T& value)
{
    return hash_bytes(hash, &value, sizeof(T));
}

uint32_t formation_epoch_hash(const taiphoon_agent::msg::FormationEpoch& epoch)
{
    uint32_t hash = 2166136261U;
    hash = hash_value(hash, epoch.sequence);
    hash = hash_value(hash, epoch.formation_type);
    hash = hash_value(hash, epoch.radius_m);
    hash = hash_value(hash, epoch.spacing_m);
    hash = hash_value(hash, epoch.vertical_offset_m);
    hash = hash_value(hash, epoch.center_latitude_deg);
    hash = hash_value(hash, epoch.center_longitude_deg);
    hash = hash_value(hash, epoch.center_altitude_m);
    for (std::size_t index = 0U; index < epoch.member_agent_ids.size(); ++index)
    {
        hash = hash_value(hash, epoch.member_agent_ids.at(index));
        hash = hash_value(hash, epoch.member_latitudes_deg.at(index));
        hash = hash_value(hash, epoch.member_longitudes_deg.at(index));
        hash = hash_value(hash, epoch.member_altitudes_m.at(index));
    }
    return hash;
}
}

EgoSwarmPlannerNode::EgoSwarmPlannerNode()
    : rclcpp::Node("ego_swarm_planner")
{
    declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    declare_parameter<int>("agent_index", 1);
    declare_parameter<int>("swarm_origin_agent_index", 1);
    declare_parameter<double>("planning_rate_hz", DEFAULT_EGO_PLANNER_RATE_HZ);
    declare_parameter<bool>("enable_local_goal_planning", true);
    declare_parameter<bool>("enable_generic_trajectory_reservation", false);
    declare_parameter<double>(
        "local_goal_horizon_m",
        ego_swarm::DEFAULT_LOCAL_GOAL_HORIZON_M);
    declare_parameter<double>(
        "local_goal_detour_margin_m",
        ego_swarm::DEFAULT_LOCAL_GOAL_DETOUR_MARGIN_M);
    declare_parameter<double>(
        "local_goal_replan_ratio",
        ego_swarm::DEFAULT_LOCAL_GOAL_REPLAN_RATIO);
    declare_parameter<double>(
        "minimum_replan_interval_s",
        ego_swarm::DEFAULT_MINIMUM_REPLAN_INTERVAL_S);
    declare_parameter<double>("peer_trajectory_timeout_s",
                             DEFAULT_PEER_TRAJECTORY_TIMEOUT_S);
    declare_parameter<double>("max_peer_time_skew_s",
                             DEFAULT_MAX_PEER_TIME_SKEW_S);
    declare_parameter<double>("obstacle_cloud_timeout_s",
                             DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S);
    declare_parameter<bool>("require_obstacle_cloud", true);
    declare_parameter<double>("formation_min_distance_m",
                             DEFAULT_FORMATION_MIN_DISTANCE_M);
    declare_parameter<double>("peer_collision_min_distance_m",
                             DEFAULT_PEER_COLLISION_MIN_DISTANCE_M);
    declare_parameter<double>("peer_planning_margin_m", 1.0);
    declare_parameter<double>("obstacle_clearance_m",
                             DEFAULT_OBSTACLE_CLEARANCE_M);
    declare_parameter<double>("knot_interval_s", 1.0);
    declare_parameter<double>("max_velocity_mps", DEFAULT_MAX_VELOCITY_MPS);
    declare_parameter<double>("max_acceleration_mps2",
                             DEFAULT_MAX_ACCELERATION_MPS2);
    declare_parameter<double>("max_jerk_mps3", DEFAULT_MAX_JERK_MPS3);
    // 保留舊參數名稱，既有 compose / launch 不需同步切換。
    declare_parameter<double>("max_velocity_m_s", DEFAULT_MAX_VELOCITY_MPS);
    declare_parameter<double>("max_acceleration_m_s2",
                             DEFAULT_MAX_ACCELERATION_MPS2);
    declare_parameter<double>("candidate_offset_m", 3.0);
    declare_parameter<double>("local_arrival_tolerance_m",
                             DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M);
    declare_parameter<double>("global_arrival_tolerance_m",
                             DEFAULT_GLOBAL_ARRIVAL_TOLERANCE_M);
    declare_parameter<double>("move_attitude_timeout_s",
                             DEFAULT_MOVE_ATTITUDE_TIMEOUT_S);
    declare_parameter<double>("formation_slot_switch_penalty_m",
                             DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M);
    declare_parameter<int>("formation_member_stable_scan_count",
                          DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT);
    declare_parameter<std::string>("planner_backend", "optimized");
    declare_parameter<int>("optimizer_max_iterations",
                           DEFAULT_OPTIMIZER_MAX_ITERATIONS);
    declare_parameter<double>("optimizer_time_budget_ms",
                             DEFAULT_OPTIMIZER_TIME_BUDGET_MS);
    declare_parameter<double>("formation_crossing_penalty_m",
                             DEFAULT_FORMATION_CROSSING_PENALTY_M);
    declare_parameter<double>("reservation_lead_time_s",
                             DEFAULT_RESERVATION_LEAD_TIME_S);
    declare_parameter<double>("reservation_step_s",
                             DEFAULT_RESERVATION_STEP_S);
    declare_parameter<double>("reservation_horizon_s",
                             DEFAULT_RESERVATION_HORIZON_S);
    declare_parameter<bool>("enable_peer_deconfliction", true);
    declare_parameter<double>(
        "peer_deconfliction_release_margin_m",
        DEFAULT_PEER_DECONFLICTION_RELEASE_MARGIN_M);
    declare_parameter<double>(
        "peer_deconfliction_max_escape_distance_m",
        DEFAULT_PEER_DECONFLICTION_MAX_ESCAPE_DISTANCE_M);

    agent_namespace_ = get_parameter("agent_namespace").as_string();
    agent_index_ = get_parameter("agent_index").as_int();
    if (agent_index_ <= 0)
    {
        agent_index_ = compute_self_index();
    }

    swarm_origin_agent_index_ = get_parameter(
        "swarm_origin_agent_index").as_int();
    if (swarm_origin_agent_index_ <= 0)
    {
        swarm_origin_agent_index_ = 1;
    }

    planning_rate_hz_ = get_parameter("planning_rate_hz").as_double();
    if (!std::isfinite(planning_rate_hz_) || planning_rate_hz_ <= 0.0)
    {
        planning_rate_hz_ = DEFAULT_EGO_PLANNER_RATE_HZ;
    }

    ego_swarm::PlannerFeatureFlags requested_feature_flags{};
    requested_feature_flags.enable_local_goal_planning = get_parameter(
        "enable_local_goal_planning").as_bool();
    requested_feature_flags.enable_generic_trajectory_reservation =
        get_parameter("enable_generic_trajectory_reservation").as_bool();
    requested_feature_flags.local_goal_horizon_m = get_parameter(
        "local_goal_horizon_m").as_double();
    requested_feature_flags.local_goal_detour_margin_m = get_parameter(
        "local_goal_detour_margin_m").as_double();
    requested_feature_flags.local_goal_replan_ratio = get_parameter(
        "local_goal_replan_ratio").as_double();
    requested_feature_flags.minimum_replan_interval_s = get_parameter(
        "minimum_replan_interval_s").as_double();
    planner_feature_flags_ = ego_swarm::normalize_planner_feature_flags(
        requested_feature_flags);

    peer_trajectory_timeout_s_ = get_parameter(
        "peer_trajectory_timeout_s").as_double();
    if (!std::isfinite(peer_trajectory_timeout_s_) ||
        peer_trajectory_timeout_s_ <= 0.0)
    {
        peer_trajectory_timeout_s_ = DEFAULT_PEER_TRAJECTORY_TIMEOUT_S;
    }

    max_peer_time_skew_s_ = get_parameter(
        "max_peer_time_skew_s").as_double();
    if (!std::isfinite(max_peer_time_skew_s_) ||
        max_peer_time_skew_s_ < 0.0)
    {
        max_peer_time_skew_s_ = DEFAULT_MAX_PEER_TIME_SKEW_S;
    }

    obstacle_cloud_timeout_s_ = get_parameter(
        "obstacle_cloud_timeout_s").as_double();
    if (!std::isfinite(obstacle_cloud_timeout_s_) ||
        obstacle_cloud_timeout_s_ <= 0.0)
    {
        obstacle_cloud_timeout_s_ = DEFAULT_OBSTACLE_CLOUD_TIMEOUT_S;
    }
    require_obstacle_cloud_ = get_parameter(
        "require_obstacle_cloud").as_bool();

    formation_min_distance_m_ = get_parameter(
        "formation_min_distance_m").as_double();
    if (!std::isfinite(formation_min_distance_m_) ||
        formation_min_distance_m_ < 0.0)
    {
        formation_min_distance_m_ = DEFAULT_FORMATION_MIN_DISTANCE_M;
    }

    planner_config_.peer_collision_min_distance_m = get_parameter(
        "peer_collision_min_distance_m").as_double();
    planner_config_.peer_planning_margin_m = get_parameter(
        "peer_planning_margin_m").as_double();
    planner_config_.obstacle_clearance_m = get_parameter(
        "obstacle_clearance_m").as_double();
    planner_config_.knot_interval_s = get_parameter(
        "knot_interval_s").as_double();
    planner_config_.max_velocity_mps = get_parameter(
        "max_velocity_mps").as_double();
    planner_config_.max_acceleration_mps2 = get_parameter(
        "max_acceleration_mps2").as_double();
    planner_config_.max_jerk_mps3 = get_parameter(
        "max_jerk_mps3").as_double();
    planner_config_.max_velocity_m_s = get_parameter(
        "max_velocity_m_s").as_double();
    planner_config_.max_acceleration_m_s2 = get_parameter(
        "max_acceleration_m_s2").as_double();
    planner_config_.candidate_offset_m = get_parameter(
        "candidate_offset_m").as_double();
    local_arrival_tolerance_m_ = get_parameter(
        "local_arrival_tolerance_m").as_double();
    global_arrival_tolerance_m_ = get_parameter(
        "global_arrival_tolerance_m").as_double();
    if (!std::isfinite(local_arrival_tolerance_m_) ||
        local_arrival_tolerance_m_ < 0.0)
    {
        local_arrival_tolerance_m_ = DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M;
    }
    if (!std::isfinite(global_arrival_tolerance_m_) ||
        global_arrival_tolerance_m_ < 0.0)
    {
        global_arrival_tolerance_m_ = DEFAULT_GLOBAL_ARRIVAL_TOLERANCE_M;
    }
    move_attitude_timeout_s_ = get_parameter(
        "move_attitude_timeout_s").as_double();
    if (!std::isfinite(move_attitude_timeout_s_) ||
        move_attitude_timeout_s_ <= 0.0)
    {
        move_attitude_timeout_s_ = DEFAULT_MOVE_ATTITUDE_TIMEOUT_S;
    }
    formation_slot_switch_penalty_m_ = get_parameter(
        "formation_slot_switch_penalty_m").as_double();
    if (!std::isfinite(formation_slot_switch_penalty_m_) ||
        formation_slot_switch_penalty_m_ < 0.0)
    {
        formation_slot_switch_penalty_m_ =
            DEFAULT_FORMATION_SLOT_SWITCH_PENALTY_M;
    }
    formation_member_stable_scan_count_ = get_parameter(
        "formation_member_stable_scan_count").as_int();
    if (formation_member_stable_scan_count_ < 2)
    {
        formation_member_stable_scan_count_ =
            DEFAULT_FORMATION_MEMBER_STABLE_SCAN_COUNT;
    }
    planner_backend_ = get_parameter("planner_backend").as_string();
    if (planner_backend_ != "optimized")
    {
        RCLCPP_ERROR(
            get_logger(),
            "Invalid planner_backend '%s'; optimized is the only supported runtime backend",
            planner_backend_.c_str());
        return;
    }
    optimizer_max_iterations_ = get_parameter(
        "optimizer_max_iterations").as_int();
    if (optimizer_max_iterations_ <= 0)
    {
        optimizer_max_iterations_ = DEFAULT_OPTIMIZER_MAX_ITERATIONS;
    }
    optimizer_time_budget_ms_ = get_parameter(
        "optimizer_time_budget_ms").as_double();
    if (!std::isfinite(optimizer_time_budget_ms_) ||
        optimizer_time_budget_ms_ <= 0.0)
    {
        optimizer_time_budget_ms_ = DEFAULT_OPTIMIZER_TIME_BUDGET_MS;
    }
    formation_crossing_penalty_m_ = get_parameter(
        "formation_crossing_penalty_m").as_double();
    if (!std::isfinite(formation_crossing_penalty_m_) ||
        formation_crossing_penalty_m_ < 0.0)
    {
        formation_crossing_penalty_m_ = DEFAULT_FORMATION_CROSSING_PENALTY_M;
    }
    reservation_lead_time_s_ = get_parameter(
        "reservation_lead_time_s").as_double();
    if (!std::isfinite(reservation_lead_time_s_) ||
        reservation_lead_time_s_ < 0.0)
    {
        reservation_lead_time_s_ = DEFAULT_RESERVATION_LEAD_TIME_S;
    }
    reservation_step_s_ = get_parameter("reservation_step_s").as_double();
    if (!std::isfinite(reservation_step_s_) || reservation_step_s_ <= 0.0)
    {
        reservation_step_s_ = DEFAULT_RESERVATION_STEP_S;
    }
    reservation_horizon_s_ = get_parameter(
        "reservation_horizon_s").as_double();
    if (!std::isfinite(reservation_horizon_s_) ||
        reservation_horizon_s_ < reservation_lead_time_s_)
    {
        reservation_horizon_s_ = DEFAULT_RESERVATION_HORIZON_S;
    }
    planner_config_.optimizer_max_iterations = optimizer_max_iterations_;
    planner_config_.optimizer_time_budget_ms = optimizer_time_budget_ms_;
    planner_config_.formation_crossing_penalty_m =
        formation_crossing_penalty_m_;
    planner_config_.reservation_lead_time_s = reservation_lead_time_s_;
    planner_config_.reservation_step_s = reservation_step_s_;
    planner_config_.reservation_horizon_s = reservation_horizon_s_;
    planner_config_.enable_spatiotemporal_route_planning =
        planner_feature_flags_.enable_local_goal_planning;
    peer_deconfliction_enabled_ = get_parameter(
        "enable_peer_deconfliction").as_bool();
    const double peer_deconfliction_release_margin_m = get_parameter(
        "peer_deconfliction_release_margin_m").as_double();
    const double peer_deconfliction_max_escape_distance_m = get_parameter(
        "peer_deconfliction_max_escape_distance_m").as_double();
    const double normalized_release_margin_m =
        std::isfinite(peer_deconfliction_release_margin_m) &&
        peer_deconfliction_release_margin_m > 0.0
        ? peer_deconfliction_release_margin_m
        : DEFAULT_PEER_DECONFLICTION_RELEASE_MARGIN_M;
    const double normalized_max_escape_distance_m =
        std::isfinite(peer_deconfliction_max_escape_distance_m) &&
        peer_deconfliction_max_escape_distance_m >
            planner_config_.peer_collision_min_distance_m
        ? peer_deconfliction_max_escape_distance_m
        : DEFAULT_PEER_DECONFLICTION_MAX_ESCAPE_DISTANCE_M;
    if (!peer_deconfliction_enabled_)
    {
        RCLCPP_INFO(get_logger(), "Peer deconfliction recovery disabled");
    }
    if (!peer_deconfliction_planner_.set_config(
        ego_swarm::PeerDeconflictionConfig{
            planner_config_.peer_collision_min_distance_m,
            normalized_release_margin_m,
            normalized_max_escape_distance_m,
            planner_config_.sample_interval_s,
            0.15,
            0.10}))
    {
        RCLCPP_ERROR(get_logger(),
                     "Peer deconfliction recovery parameters are invalid");
        return;
    }
    auto optimized_planner = std::make_unique<ego_swarm::OptimizedEgoPlanner>();
    if (!optimized_planner->set_config(planner_config_))
    {
        RCLCPP_ERROR(get_logger(), "EGO-Swarm planner parameters are invalid");
        return;
    }
    planner_config_ = optimized_planner->config();
    selected_planner_ = std::move(optimized_planner);
    if (!safety_validator_.set_config(planner_config_) ||
        !emergency_brake_planner_.set_config(planner_config_))
    {
        RCLCPP_ERROR(get_logger(),
                     "EGO-Swarm safety parameters are invalid");
        return;
    }
    if (!obstacle_map_.set_freshness_timeout(obstacle_cloud_timeout_s_))
    {
        RCLCPP_ERROR(get_logger(), "Obstacle map freshness parameters are invalid");
        return;
    }
    if (!peer_trajectories_.set_config(
        ego_swarm::PeerTrajectoryTableConfig{
                peer_trajectory_timeout_s_, max_peer_time_skew_s_,
                reservation_lead_time_s_ + reservation_horizon_s_ + 1.0}))
    {
        RCLCPP_ERROR(get_logger(),
                     "Peer trajectory freshness parameters are invalid");
        return;
    }

    if (!initialize_subscribers() || !initialize_publishers())
    {
        RCLCPP_ERROR(get_logger(), "EGO-Swarm Planner initialization failed");
        return;
    }

    const auto planning_period = std::chrono::duration_cast<
        std::chrono::milliseconds>(std::chrono::duration<double>(
            1.0 / planning_rate_hz_));
    planner_timer_ = create_wall_timer(
        std::max(planning_period, std::chrono::milliseconds(1)),
        std::bind(&EgoSwarmPlannerNode::planner_loop_cb, this));
    peer_scan_timer_ = create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&EgoSwarmPlannerNode::scan_peers_cb, this));
    scan_peers_cb();

    RCLCPP_INFO(get_logger(),
                "EGO-Swarm Planner ready: agent=%d namespace=%s rate=%.1f Hz "
                "backend=%s optimizer_iterations=%d optimizer_budget=%.1f ms "
                "reservation_lead=%.2f s reservation_step=%.2f s "
                "reservation_horizon=%.2f s crossing_penalty=%.2f m "
                "compat_origin_agent=%d peer_distance=%.2f m "
                "peer_planning_margin=%.2f m obstacle_clearance=%.2f m "
                "peer_timeout=%.2f s "
                "max_peer_time_skew=%.2f s obstacle_timeout=%.2f s "
                "require_obstacle_cloud=%s move_attitude_timeout=%.2f s "
                "slot_switch_penalty=%.2f m member_stable_scans=%d "
                "feature_local_goal=%s feature_generic_reservation=%s "
                "local_goal_horizon=%.2f m local_goal_detour_margin=%.2f m "
                "local_goal_replan_ratio=%.2f "
                "minimum_replan_interval=%.2f s",
                agent_index_, agent_namespace_.c_str(), planning_rate_hz_,
                planner_backend_.c_str(), optimizer_max_iterations_,
                optimizer_time_budget_ms_, reservation_lead_time_s_,
                reservation_step_s_, reservation_horizon_s_,
                formation_crossing_penalty_m_,
                swarm_origin_agent_index_,
                planner_config_.peer_collision_min_distance_m,
                planner_config_.peer_planning_margin_m,
                planner_config_.obstacle_clearance_m,
                peer_trajectory_timeout_s_, max_peer_time_skew_s_,
                obstacle_cloud_timeout_s_,
                require_obstacle_cloud_ ? "true" : "false",
                move_attitude_timeout_s_, formation_slot_switch_penalty_m_,
                formation_member_stable_scan_count_,
                planner_feature_flags_.enable_local_goal_planning ? "true" : "false",
                planner_feature_flags_.enable_generic_trajectory_reservation ?
                    "true" : "false",
                planner_feature_flags_.local_goal_horizon_m,
                planner_feature_flags_.local_goal_detour_margin_m,
                planner_feature_flags_.local_goal_replan_ratio,
                planner_feature_flags_.minimum_replan_interval_s);
}

bool EgoSwarmPlannerNode::initialize_subscribers()
{
    const std::string own_state_topic = "/" + agent_namespace_ + "/state";
    own_state_sub_ = create_subscription<taiphoon_agent::msg::AgentState>(
        own_state_topic, rclcpp::QoS(10).best_effort(),
        std::bind(&EgoSwarmPlannerNode::own_state_cb, this,
                  std::placeholders::_1));
    own_attitude_sub_ = create_subscription<taiphoon_agent::msg::AgentAttitude>(
        "/" + agent_namespace_ + "/attitude", rclcpp::QoS(10).best_effort(),
        std::bind(&EgoSwarmPlannerNode::own_attitude_cb, this,
                  std::placeholders::_1));
    ned_target_sub_ = create_subscription<taiphoon_agent::msg::NedTarget>(
        "/" + agent_namespace_ + "/ned_target", rclcpp::QoS(10).best_effort(),
        std::bind(&EgoSwarmPlannerNode::ned_target_cb, this,
                  std::placeholders::_1));
    mission_intent_sub_ = create_subscription<std_msgs::msg::String>(
        "/" + agent_namespace_ + "/planning/mission_intent",
        rclcpp::QoS(10),
        std::bind(&EgoSwarmPlannerNode::mission_intent_cb, this,
                  std::placeholders::_1));
    agent_command_sub_ = create_subscription<std_msgs::msg::String>(
        "/" + agent_namespace_ + "/command", rclcpp::QoS(10),
        std::bind(&EgoSwarmPlannerNode::agent_command_cb, this,
                  std::placeholders::_1));
    all_command_sub_ = create_subscription<std_msgs::msg::String>(
        "/fmu_all_agent/command", rclcpp::QoS(10),
        std::bind(&EgoSwarmPlannerNode::all_agent_command_cb, this,
                  std::placeholders::_1));
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/" + agent_namespace_ + "/perception/obstacles",
        rclcpp::QoS(10).best_effort(),
        std::bind(&EgoSwarmPlannerNode::obstacle_cb, this,
                  std::placeholders::_1));
    trajectory_sub_ = create_subscription<taiphoon_agent::msg::AgentTrajectory>(
        "/fmu_all_agent/swarm/agent_trajectories",
        rclcpp::QoS(10).best_effort(),
                  std::bind(&EgoSwarmPlannerNode::trajectory_cb, this,
                  std::placeholders::_1));
    formation_epoch_sub_ = create_subscription<taiphoon_agent::msg::FormationEpoch>(
        "/fmu_all_agent/swarm/formation_epochs",
        rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&EgoSwarmPlannerNode::formation_epoch_cb, this,
                  std::placeholders::_1));

    return own_state_sub_ != nullptr && own_attitude_sub_ != nullptr &&
        ned_target_sub_ != nullptr &&
        mission_intent_sub_ != nullptr && agent_command_sub_ != nullptr &&
        all_command_sub_ != nullptr &&
        obstacle_sub_ != nullptr && trajectory_sub_ != nullptr &&
        formation_epoch_sub_ != nullptr;
}

bool EgoSwarmPlannerNode::initialize_publishers()
{
    trajectory_pub_ = create_publisher<taiphoon_agent::msg::AgentTrajectory>(
        "/fmu_all_agent/swarm/agent_trajectories", rclcpp::QoS(10));
    local_trajectory_pub_ =
        create_publisher<taiphoon_agent::msg::AgentTrajectory>(
            "/" + agent_namespace_ + "/planning/trajectory",
            rclcpp::QoS(10));
    status_pub_ = create_publisher<taiphoon_agent::msg::PlannerStatus>(
            "/" + agent_namespace_ + "/planning/status", rclcpp::QoS(10));
    formation_epoch_pub_ = create_publisher<taiphoon_agent::msg::FormationEpoch>(
        "/fmu_all_agent/swarm/formation_epochs",
        rclcpp::QoS(10).reliable().transient_local());

    return trajectory_pub_ != nullptr && local_trajectory_pub_ != nullptr &&
        status_pub_ != nullptr && formation_epoch_pub_ != nullptr;
}

void EgoSwarmPlannerNode::own_state_cb(
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "own_state_cb received nullptr");
        return;
    }
    own_state_ = *msg;
    has_own_state_ = true;
    update_vehicle_motion_state(*msg);
}

void EgoSwarmPlannerNode::own_attitude_cb(
    const taiphoon_agent::msg::AgentAttitude::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "own_attitude_cb received nullptr");
        return;
    }
    double yaw_rad = 0.0;
    if (!ego_swarm::quaternion_to_ned_yaw(msg->q, yaw_rad))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Ignoring attitude with non-finite quaternion");
        return;
    }
    own_attitude_ = *msg;
    has_own_attitude_ = true;
    attitude_last_arrival_ = std::chrono::steady_clock::now();
}

void EgoSwarmPlannerNode::update_vehicle_motion_state(
    const taiphoon_agent::msg::AgentState& state)
{
    const ego_swarm::Vector3 position{
        static_cast<double>(state.position_ned.at(0)),
        static_cast<double>(state.position_ned.at(1)),
        static_cast<double>(state.position_ned.at(2))};
    const double timestamp_s = this->now().seconds();
    if (!ego_swarm::is_finite_vector(position) ||
        !std::isfinite(timestamp_s))
    {
        return;
    }

    ego_swarm::Vector3 velocity{0.0, 0.0, 0.0};
    ego_swarm::Vector3 acceleration{0.0, 0.0, 0.0};
    if (has_previous_state_sample_)
    {
        const double delta_time_s = timestamp_s - previous_state_timestamp_s_;
        if (delta_time_s > 1.0e-3 && delta_time_s <= 1.0)
        {
            velocity = ego_swarm::scale_vector(
                ego_swarm::subtract_vectors(position, previous_position_ned_),
                1.0 / delta_time_s);
            if (has_vehicle_motion_state_)
            {
                acceleration = ego_swarm::scale_vector(
                    ego_swarm::subtract_vectors(
                        velocity, previous_velocity_ned_),
                    1.0 / delta_time_s);
            }
        }
    }

    if (!ego_swarm::is_finite_vector(velocity) ||
        !ego_swarm::is_finite_vector(acceleration))
    {
        velocity = ego_swarm::Vector3{0.0, 0.0, 0.0};
        acceleration = ego_swarm::Vector3{0.0, 0.0, 0.0};
    }
    vehicle_motion_state_.position = position;
    vehicle_motion_state_.velocity = velocity;
    vehicle_motion_state_.acceleration = acceleration;
    vehicle_motion_state_.timestamp_s = timestamp_s;
    has_vehicle_motion_state_ = true;
    has_previous_state_sample_ = true;
    previous_state_timestamp_s_ = timestamp_s;
    previous_position_ned_ = position;
    previous_velocity_ned_ = velocity;
}

void EgoSwarmPlannerNode::peer_state_cb(
    const int agent_index,
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "peer_state_cb received nullptr for %d",
                    agent_index);
        return;
    }
    auto& peer = peers_[agent_index];
    peer.state = *msg;
    peer.last_arrival = std::chrono::steady_clock::now();
    peer.has_state = true;

    // peer 的 position_ned 屬於 peer 自己的 local frame，不能直接拿來和
    // 本機 swarm frame 比較。速度仍供 transition／動態安全邏輯使用，
    // 但不再作為 formation convergence 的到位條件。
    ego_swarm::Vector3 current_position{};
    const double sample_time_s = this->now().seconds();
    if (swarm_frame_.has_origin() &&
        get_position_in_swarm_frame(peer.state, current_position) &&
        std::isfinite(sample_time_s))
    {
        const double delta_time_s = sample_time_s - peer.previous_position_time_s;
        if (peer.has_state && delta_time_s > 1.0e-3 &&
            delta_time_s <= 1.0 &&
            ego_swarm::is_finite_vector(peer.previous_position_ned))
        {
            const ego_swarm::Vector3 velocity = ego_swarm::scale_vector(
                ego_swarm::subtract_vectors(
                    current_position, peer.previous_position_ned),
                1.0 / delta_time_s);
            if (ego_swarm::is_finite_vector(velocity))
            {
                peer.velocity_ned = velocity;
                peer.has_velocity = true;
            }
        }
        peer.previous_position_ned = current_position;
        peer.previous_position_time_s = sample_time_s;
    }
}

void EgoSwarmPlannerNode::ned_target_cb(
    const taiphoon_agent::msg::NedTarget::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "ned_target_cb received nullptr");
        return;
    }
    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
        !std::isfinite(msg->z) || std::isinf(msg->yaw))
    {
        RCLCPP_WARN(get_logger(), "Rejecting invalid NED target");
        return;
    }
    set_goal_from_command(command_to_text(*msg));
}

void EgoSwarmPlannerNode::mission_intent_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "mission_intent_cb received nullptr");
        return;
    }
    set_goal_from_command(msg->data);
}

void EgoSwarmPlannerNode::agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "agent_command_cb received nullptr");
        return;
    }

    const auto parsed = utils::parse_command_string(msg->data);
    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
        case utils::AgentCommand::MOVE_BACKWARD:
        case utils::AgentCommand::MOVE_LEFT:
        case utils::AgentCommand::MOVE_RIGHT:
        case utils::AgentCommand::MOVE_UP:
        case utils::AgentCommand::MOVE_DOWN:
        case utils::AgentCommand::MOVE_POINT:
        case utils::AgentCommand::YAW:
        case utils::AgentCommand::FORMATION_CANCEL:
            set_goal_from_command(msg->data);
            break;
        default:
            // MODE_*, ARM, TAKEOFF 與 LAND 由 BasicController 處理。
            break;
    }
}

void EgoSwarmPlannerNode::all_agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "all_agent_command_cb received nullptr");
        return;
    }

    const auto parsed = utils::parse_command_string(msg->data);
    if (parsed.command == utils::AgentCommand::FORMATION_CANCEL)
    {
        ego_swarm::Vector3 preserved_target{};
        if (has_formation_ && get_formation_target_for_agent(
                agent_index_, preserved_target))
        {
            // 取消編隊後仍保留原本的 slot target，讓下一個相對命令
            // 不會因 callback 時序改以漂移中的實際位置重新累加。
            goal_position_ = preserved_target;
            has_goal_ = true;
            has_position_target_ = true;
        }
        has_formation_ = false;
        formation_command_pending_ = false;
        formation_membership_change_pending_ = false;
        formation_initial_plan_pending_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        return;
    }

    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
        case utils::AgentCommand::MOVE_BACKWARD:
        case utils::AgentCommand::MOVE_LEFT:
        case utils::AgentCommand::MOVE_RIGHT:
        case utils::AgentCommand::MOVE_UP:
        case utils::AgentCommand::MOVE_DOWN:
        case utils::AgentCommand::MOVE_POINT:
            (void)handle_group_move_command(parsed);
            return;
        default:
            break;
    }

    if (parsed.command != utils::AgentCommand::FORMATION_CIRCLE &&
        parsed.command != utils::AgentCommand::FORMATION_HORI_LINEAR &&
        parsed.command != utils::AgentCommand::FORMATION_VERT_LINEAR)
    {
        return;
    }
    const bool is_vertical_linear =
        parsed.command == utils::AgentCommand::FORMATION_VERT_LINEAR;
    if ((!parsed.has_param || !std::isfinite(parsed.param_value)) ||
        (!is_vertical_linear &&
         (!parsed.has_multi_params || !std::isfinite(parsed.param2))))
    {
        RCLCPP_WARN(get_logger(), "Rejecting invalid formation command");
        return;
    }

    const ego_swarm::FormationConfig previous_config = formation_config_;
    ego_swarm::FormationConfig requested_config = formation_config_;
    if (parsed.command == utils::AgentCommand::FORMATION_CIRCLE)
    {
        requested_config.type = ego_swarm::FormationType::CIRCLE;
        requested_config.radius_m = parsed.param_value;
        requested_config.vertical_offset_m = -parsed.param2;
    }
    else if (parsed.command == utils::AgentCommand::FORMATION_HORI_LINEAR)
    {
        requested_config.type = ego_swarm::FormationType::HORI_LINEAR;
        requested_config.spacing_m = parsed.param_value;
        requested_config.vertical_offset_m = -parsed.param2;
    }
    else
    {
        requested_config.type = ego_swarm::FormationType::VERT_LINEAR;
        requested_config.spacing_m = parsed.param_value;
        requested_config.vertical_offset_m = 0.0;
    }
    if (!ego_swarm::is_finite(requested_config.radius_m) ||
        !ego_swarm::is_finite(requested_config.spacing_m) ||
        !ego_swarm::is_finite(requested_config.vertical_offset_m) ||
        requested_config.radius_m < 0.0 || requested_config.spacing_m < 0.0)
    {
        RCLCPP_WARN(get_logger(), "Rejecting formation command with invalid geometry");
        return;
    }

    preserve_altitude_ = false;
    std::vector<int> members = stable_member_ids();
    std::sort(members.begin(), members.end());
    members.erase(std::unique(members.begin(), members.end()), members.end());
    const bool accepted_epoch_matches = has_accepted_formation_epoch_ &&
        formation_config_matches(requested_config, previous_config) &&
        accepted_formation_epoch_.member_agent_ids == members;
    const bool candidate_epoch_matches = has_candidate_formation_epoch_ &&
        formation_config_matches(requested_config, previous_config) &&
        candidate_formation_epoch_.member_agent_ids == members;

    // 單機 MOVE 或 FORMATION_CANCEL 只暫停目前編隊，不摧毀既有 Epoch。
    // 相同編隊命令應重新啟用原 assignment／center，而不是被冪等檢查丟棄。
    ego_swarm::FormationReentryState reentry_state{
        has_formation_, has_goal_, has_position_target_, preserve_altitude_,
        formation_command_pending_, formation_membership_change_pending_,
        formation_initial_plan_pending_};
    if (ego_swarm::reactivate_existing_formation(
            reentry_state, accepted_epoch_matches, candidate_epoch_matches))
    {
        has_formation_ = reentry_state.has_formation;
        has_goal_ = reentry_state.has_goal;
        has_position_target_ = reentry_state.has_position_target;
        preserve_altitude_ = reentry_state.preserve_altitude;
        formation_command_pending_ = reentry_state.formation_command_pending;
        formation_membership_change_pending_ =
            reentry_state.formation_membership_change_pending;
        formation_initial_plan_pending_ =
            reentry_state.formation_initial_plan_pending;
        invalidate_active_plan();
        RCLCPP_INFO(
            get_logger(),
            "Reactivated existing FormationEpoch seq=%u hash=%u",
            has_accepted_formation_epoch_
                ? accepted_formation_epoch_.sequence
                : candidate_formation_epoch_.sequence,
            has_accepted_formation_epoch_
                ? accepted_formation_epoch_.snapshot_hash
                : candidate_formation_epoch_.snapshot_hash);
        return;
    }
    if (has_candidate_formation_epoch_ &&
        formation_config_matches(requested_config, previous_config) &&
        candidate_formation_epoch_.member_agent_ids == members)
    {
        // 相同 config 與成員 snapshot 的重複字串命令是 idempotent。
        return;
    }
    if (has_accepted_formation_epoch_ &&
        formation_config_matches(requested_config, previous_config) &&
        accepted_formation_epoch_.member_agent_ids == members &&
        has_formation_)
    {
        return;
    }
    formation_config_ = requested_config;
    formation_command_pending_ = true;
    formation_membership_change_pending_ = false;
    invalidate_active_plan();
    if (make_formation_proposal(members))
    {
        formation_command_pending_ = false;
    }
}

bool EgoSwarmPlannerNode::handle_group_move_command(
    const utils::ParsedCommand& parsed)
{
    if (!has_formation_ || !has_accepted_formation_epoch_)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting group MOVE: no active FormationEpoch is available");
        return false;
    }

    std::vector<int> members = stable_member_ids();
    std::sort(members.begin(), members.end());
    members.erase(std::unique(members.begin(), members.end()), members.end());
    if (members.empty() ||
        !std::binary_search(members.begin(), members.end(), agent_index_))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting group MOVE: stable formation membership is unavailable");
        return false;
    }

    // 全體命令只由穩定成員中 ID 最小者建立下一個 Epoch，避免各 Agent
    // 以自己的 local Home 轉換中心後提出略有差異的 proposal。
    if (agent_index_ != members.front())
    {
        RCLCPP_DEBUG(
            get_logger(),
            "Ignoring group MOVE proposal: Agent %d is waiting for proposer %d",
            agent_index_, members.front());
        return true;
    }

    if (has_candidate_formation_epoch_ &&
        !epoch_is_same_snapshot(
            candidate_formation_epoch_, accepted_formation_epoch_))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting group MOVE while a FormationEpoch proposal is pending");
        return false;
    }

    if (!has_own_state_ || !update_swarm_frame() ||
        !ego_swarm::is_finite_vector(formation_center_))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting group MOVE: Formation center frame is unavailable");
        return false;
    }

    ego_swarm::Vector3 requested_center{};
    if (parsed.command == utils::AgentCommand::MOVE_POINT)
    {
        if (!parsed.has_multi_params || !std::isfinite(parsed.param_value) ||
            !std::isfinite(parsed.param2) || !std::isfinite(parsed.param3) ||
            !std::isfinite(parsed.param4))
        {
            RCLCPP_WARN(get_logger(), "Rejecting invalid group MOVE_POINT");
            return false;
        }
        // 全體 MOVE_POINT 的 x/y/z 是共同 swarm_ned 中 Formation center
        // 的絕對目標；各 Agent 不會把它解讀成自己的 slot 目標。
        requested_center = ego_swarm::Vector3{
            static_cast<double>(parsed.param_value),
            static_cast<double>(parsed.param2),
            static_cast<double>(parsed.param3)};
    }
    else
    {
        if (!parsed.has_param || !std::isfinite(parsed.param_value))
        {
            RCLCPP_WARN(get_logger(), "Rejecting invalid group MOVE distance");
            return false;
        }

        ego_swarm::SwarmMoveDirection direction{};
        switch (parsed.command)
        {
            case utils::AgentCommand::MOVE_FORWARD:
                direction = ego_swarm::SwarmMoveDirection::FORWARD;
                break;
            case utils::AgentCommand::MOVE_BACKWARD:
                direction = ego_swarm::SwarmMoveDirection::BACKWARD;
                break;
            case utils::AgentCommand::MOVE_LEFT:
                direction = ego_swarm::SwarmMoveDirection::LEFT;
                break;
            case utils::AgentCommand::MOVE_RIGHT:
                direction = ego_swarm::SwarmMoveDirection::RIGHT;
                break;
            case utils::AgentCommand::MOVE_UP:
                direction = ego_swarm::SwarmMoveDirection::UP;
                break;
            case utils::AgentCommand::MOVE_DOWN:
                direction = ego_swarm::SwarmMoveDirection::DOWN;
                break;
            default:
                RCLCPP_WARN(get_logger(), "Unsupported group MOVE command");
                return false;
        }

        ego_swarm::Vector3 delta{};
        if (!ego_swarm::common_swarm_ned_offset(
                direction, static_cast<double>(parsed.param_value), delta))
        {
            RCLCPP_WARN(get_logger(), "Rejecting non-finite group MOVE delta");
            return false;
        }
        requested_center = ego_swarm::add_vectors(formation_center_, delta);
    }

    if (!ego_swarm::is_finite_vector(requested_center))
    {
        RCLCPP_WARN(get_logger(), "Rejecting non-finite Formation center target");
        return false;
    }

    // 驗證目標可以轉回 WGS84，避免產生無法被其他 Agent 驗證的 Epoch。
    ego_swarm::GeoCoordinate target_geo{};
    if (!swarm_frame_.from_swarm_ned(requested_center, target_geo) ||
        !ego_swarm::SwarmFrame::is_valid_geodetic(target_geo))
    {
        RCLCPP_WARN(get_logger(), "Rejecting group MOVE outside valid WGS84 range");
        return false;
    }

    formation_command_pending_ = true;
    formation_membership_change_pending_ =
        members != accepted_formation_epoch_.member_agent_ids;
    formation_initial_plan_pending_ = true;
    preserve_altitude_ = false;
    invalidate_active_plan();
    if (make_formation_proposal(members, &requested_center))
    {
        formation_command_pending_ = false;
        return true;
    }
    return false;
}

bool EgoSwarmPlannerNode::formation_config_matches(
    const ego_swarm::FormationConfig& first,
    const ego_swarm::FormationConfig& second) const
{
    return first.type == second.type &&
        std::abs(first.radius_m - second.radius_m) <= 1.0e-9 &&
        std::abs(first.spacing_m - second.spacing_m) <= 1.0e-9 &&
        std::abs(first.vertical_offset_m - second.vertical_offset_m) <= 1.0e-9;
}

ego_swarm::LivePeerDistanceResult
EgoSwarmPlannerNode::live_peer_min_distance(
    const ego_swarm::Vector3& current) const
{
    if (!ego_swarm::is_finite_vector(current))
    {
        return ego_swarm::LivePeerDistanceResult{true, false, 0.0};
    }

    std::vector<int> peer_ids;
    if (has_formation_ && has_accepted_formation_epoch_)
    {
        peer_ids = accepted_formation_epoch_.member_agent_ids;
    }
    else
    {
        peer_ids.reserve(peers_.size() + 1U);
        for (const auto& entry : peers_)
        {
            peer_ids.push_back(entry.first);
        }
        peer_ids.push_back(agent_index_);
    }

    double minimum_distance = std::numeric_limits<double>::infinity();
    std::size_t peer_count = 0U;
    for (const int peer_id : peer_ids)
    {
        if (peer_id == agent_index_)
        {
            continue;
        }
        ++peer_count;
        const auto peer_iterator = peers_.find(peer_id);
        if (peer_iterator == peers_.end() ||
            !peer_iterator->second.has_state ||
            !is_finite_state_position(peer_iterator->second.state))
        {
            return ego_swarm::LivePeerDistanceResult{true, false, 0.0};
        }
        ego_swarm::Vector3 peer_position{};
        if (!get_position_in_swarm_frame(
                peer_iterator->second.state, peer_position))
        {
            return ego_swarm::LivePeerDistanceResult{true, false, 0.0};
        }
        const double distance = ego_swarm::distance_between_centers(
            current, peer_position);
        if (!std::isfinite(distance))
        {
            return ego_swarm::LivePeerDistanceResult{true, false, 0.0};
        }
        minimum_distance = std::min(minimum_distance, distance);
    }
    if (peer_count == 0U)
    {
        return ego_swarm::LivePeerDistanceResult{false, true, 0.0};
    }
    return ego_swarm::LivePeerDistanceResult{
        true, std::isfinite(minimum_distance), minimum_distance};
}

ego_swarm::PeerDeconflictionInput
EgoSwarmPlannerNode::make_peer_deconfliction_input(
    const ego_swarm::Vector3& current,
    const ego_swarm::Vector3& final_target,
    const double now_s,
    const bool peer_data_valid) const
{
    ego_swarm::PeerDeconflictionInput input;
    input.self_agent_id = agent_index_;
    input.self_position = current;
    input.final_target = final_target;
    input.now_s = now_s;
    input.peers_are_fresh = peer_data_valid;
    input.obstacle_map = &obstacle_map_;
    input.planner_config = planner_config_;

    std::vector<int> peer_ids;
    if (has_formation_ && has_accepted_formation_epoch_)
    {
        peer_ids = accepted_formation_epoch_.member_agent_ids;
    }
    else
    {
        peer_ids.reserve(peers_.size());
        for (const auto& entry : peers_)
        {
            peer_ids.push_back(entry.first);
        }
    }
    std::sort(peer_ids.begin(), peer_ids.end());
    peer_ids.erase(std::unique(peer_ids.begin(), peer_ids.end()), peer_ids.end());

    for (const int peer_id : peer_ids)
    {
        if (peer_id <= 0 || peer_id == agent_index_)
        {
            continue;
        }
        const auto iterator = peers_.find(peer_id);
        if (iterator == peers_.end() || !iterator->second.has_state)
        {
            continue;
        }
        const PeerState& peer_state = iterator->second;
        ego_swarm::Vector3 position{};
        if (!get_position_in_swarm_frame(peer_state.state, position))
        {
            continue;
        }

        ego_swarm::PeerDeconflictionPeer peer;
        peer.agent_id = peer_id;
        peer.position = position;
        const double age_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - peer_state.last_arrival).count();
        peer.velocity_valid = peer_state.has_velocity &&
            std::isfinite(age_s) && age_s >= 0.0 &&
            age_s <= peer_trajectory_timeout_s_ &&
            ego_swarm::is_finite_vector(peer_state.velocity_ned);
        if (peer.velocity_valid)
        {
            peer.velocity = peer_state.velocity_ned;
        }

        ego_swarm::PeerTrajectory peer_trajectory;
        peer.can_move = true;
        if (peer_trajectories_.get_fresh(peer_id, now_s, peer_trajectory))
        {
            peer.can_move = !peer_trajectory.terminal_hold;
        }
        input.peers.push_back(peer);
    }
    return input;
}

bool EgoSwarmPlannerNode::peer_deconfliction_released(
    const ego_swarm::PeerDeconflictionInput& input) const
{
    if (peer_deconfliction_conflict_peer_ids_.empty())
    {
        return false;
    }
    const double release_distance =
        peer_deconfliction_planner_.release_distance_m();
    for (const int peer_id : peer_deconfliction_conflict_peer_ids_)
    {
        const auto iterator = std::find_if(
            input.peers.begin(), input.peers.end(),
            [peer_id](const ego_swarm::PeerDeconflictionPeer& peer)
            {
                return peer.agent_id == peer_id;
            });
        if (iterator == input.peers.end() ||
            ego_swarm::distance_between_centers(
                input.self_position, iterator->position) +
                ego_swarm::EGO_SWARM_EPSILON_M < release_distance)
        {
            return false;
        }
    }
    return true;
}

void EgoSwarmPlannerNode::clear_peer_deconfliction_state()
{
    peer_deconfliction_active_ = false;
    peer_deconfliction_owner_id_ = 0;
    peer_deconfliction_conflict_peer_ids_.clear();
}

bool EgoSwarmPlannerNode::publish_peer_deconfliction_hold(
    const ego_swarm::Vector3& current,
    const ego_swarm::Vector3& final_target,
    const int owner_agent_id,
    const std::vector<int>& conflict_peer_ids,
    const std::string& reason,
    const rclcpp::Time& now,
    const bool peer_data_valid)
{
    if (!ego_swarm::is_finite_vector(current) ||
        !ego_swarm::is_finite_vector(final_target))
    {
        return false;
    }

    const bool can_reuse_hold = peer_deconfliction_active_ &&
        peer_deconfliction_owner_id_ != agent_index_ && has_active_plan_ &&
        active_plan_.is_hold && active_plan_.trajectory.is_valid();
    if (!can_reuse_hold)
    {
        const auto hold_result = ego_swarm::make_exact_hold_result(
            current, planner_config_);
        if (!hold_result.is_valid || !hold_result.has_finite_output())
        {
            return false;
        }
        clear_terminal_hold();
        active_plan_ = hold_result;
        active_plan_target_ = current;
        active_plan_final_target_ = final_target;
        active_plan_start_time_ = now;
        active_plan_trajectory_id_ = ++trajectory_id_;
        has_active_plan_ = true;
        emergency_brake_active_ = false;
        waiting_for_replan_ = false;
        waiting_retry_time_s_ = 0.0;
    }

    peer_deconfliction_active_ = true;
    peer_deconfliction_owner_id_ = owner_agent_id;
    peer_deconfliction_conflict_peer_ids_ = conflict_peer_ids;
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_, true);
    publish_status(
        PLANNER_STATE_HOLD, reason, &active_plan_, true, peer_data_valid, now);
    return true;
}

bool EgoSwarmPlannerNode::handle_peer_deconfliction(
    const ego_swarm::Vector3& current,
    const ego_swarm::Vector3& final_target,
    const ego_swarm::VehiclePlanningState& planning_state,
    const rclcpp::Time& now,
    const bool peer_data_valid)
{
    if (!peer_deconfliction_enabled_)
    {
        return false;
    }

    const double now_s = now.seconds();
    const auto input = make_peer_deconfliction_input(
        current, final_target, now_s, peer_data_valid);
    const double speed_mps = std::sqrt(
        ego_swarm::squared_norm(planning_state.velocity));
    const bool stationary = std::isfinite(speed_mps) && speed_mps <= 0.2;

    if (peer_deconfliction_active_)
    {
        if (!input.peers_are_fresh || input.obstacle_map == nullptr ||
            !input.obstacle_map->is_valid() ||
            !input.obstacle_map->is_fresh(input.now_s))
        {
            clear_peer_deconfliction_state();
            return false;
        }

        if (peer_deconfliction_released(input))
        {
            clear_peer_deconfliction_state();
            if (!stationary)
            {
                publish_emergency_brake(
                    planning_state, final_target,
                    "peer deconfliction clearance reached; braking before mission replan",
                    now, peer_data_valid);
                return true;
            }
            invalidate_active_plan();
            return false;
        }

        if (peer_deconfliction_owner_id_ != agent_index_)
        {
            return publish_peer_deconfliction_hold(
                current, final_target, peer_deconfliction_owner_id_,
                peer_deconfliction_conflict_peer_ids_,
                "Peer deconfliction: waiting for owner to clear", now,
                peer_data_valid);
        }

        if (has_active_plan_ && active_plan_.trajectory.is_valid())
        {
            const double elapsed_s =
                (now - active_plan_start_time_).seconds();
            const auto validation = peer_deconfliction_planner_.validate_remaining(
                active_plan_.trajectory, elapsed_s, input, true);
            if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
                elapsed_s <= active_plan_.trajectory.duration_s() &&
                validation.is_safe)
            {
                publish_plan(
                    active_plan_, current, now, active_plan_start_time_,
                    active_plan_trajectory_id_, false);
                publish_status(
                    PLANNER_STATE_RUNNING,
                    "Peer deconfliction escape in progress",
                    &active_plan_, true, peer_data_valid, now);
                return true;
            }
        }

        if (!stationary)
        {
            clear_peer_deconfliction_state();
            publish_emergency_brake(
                planning_state, final_target,
                "optimized_emergency_brake: recovery trajectory expired or unsafe",
                now, peer_data_valid);
            return true;
        }
        invalidate_active_plan();
    }

    const auto decision = peer_deconfliction_planner_.decide(input);
    if (decision.action == ego_swarm::PeerDeconflictionAction::NONE)
    {
        return false;
    }
    if (decision.action == ego_swarm::PeerDeconflictionAction::BLOCKED)
    {
        return false;
    }
    if (!stationary)
    {
        return false;
    }

    if (decision.action == ego_swarm::PeerDeconflictionAction::HOLD)
    {
        return publish_peer_deconfliction_hold(
            current, final_target, decision.owner_agent_id,
            decision.conflict_peer_ids,
            "Peer deconfliction: holding non-owner", now, peer_data_valid);
    }

    if (decision.action != ego_swarm::PeerDeconflictionAction::ESCAPE ||
        !decision.plan.is_valid || !decision.plan.trajectory.is_valid())
    {
        return false;
    }

    clear_terminal_hold();
    active_plan_ = decision.plan;
    active_plan_target_ = decision.escape_target;
    active_plan_final_target_ = final_target;
    active_plan_start_time_ = now;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    emergency_brake_active_ = false;
    waiting_for_replan_ = false;
    waiting_retry_time_s_ = 0.0;
    peer_deconfliction_active_ = true;
    peer_deconfliction_owner_id_ = decision.owner_agent_id;
    peer_deconfliction_conflict_peer_ids_ = decision.conflict_peer_ids;
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_, false);
    publish_status(
        PLANNER_STATE_RUNNING,
        "Peer deconfliction escape started; owner=" +
            std::to_string(decision.owner_agent_id),
        &active_plan_, true, peer_data_valid, now);
    return true;
}

std::vector<int> EgoSwarmPlannerNode::stable_member_ids() const
{
    if (!stable_members_.empty())
    {
        return stable_members_;
    }

    std::vector<int> members;
    if (has_own_state_ && is_finite_state_position(own_state_))
    {
        members.push_back(agent_index_);
    }
    for (const int index : currently_discovered_peers_)
    {
        const auto iterator = peers_.find(index);
        if (iterator != peers_.end() && iterator->second.has_state &&
            is_finite_state_position(iterator->second.state))
        {
            members.push_back(index);
        }
    }
    std::sort(members.begin(), members.end());
    members.erase(std::unique(members.begin(), members.end()), members.end());
    return members;
}

bool EgoSwarmPlannerNode::epoch_is_valid(
    const taiphoon_agent::msg::FormationEpoch& epoch) const
{
    if (!epoch.valid || epoch.sequence == 0U || epoch.proposer_agent_id <= 0 ||
        epoch.sender_agent_id <= 0 ||
        (epoch.status != taiphoon_agent::msg::FormationEpoch::PROPOSAL &&
         epoch.status != taiphoon_agent::msg::FormationEpoch::ADOPTED) ||
        epoch.member_agent_ids.empty() ||
        epoch.member_agent_ids.size() != epoch.member_latitudes_deg.size() ||
        epoch.member_agent_ids.size() != epoch.member_longitudes_deg.size() ||
        epoch.member_agent_ids.size() != epoch.member_altitudes_m.size() ||
        epoch.parent_assignment_agent_ids.size() !=
            epoch.parent_assignment_slot_indices.size())
    {
        return false;
    }
    if (epoch.formation_type > taiphoon_agent::msg::FormationEpoch::VERT_LINEAR ||
        !std::isfinite(epoch.radius_m) || !std::isfinite(epoch.spacing_m) ||
        !std::isfinite(epoch.vertical_offset_m) || epoch.radius_m < 0.0 ||
        epoch.spacing_m < 0.0 || !std::isfinite(epoch.center_latitude_deg) ||
        !std::isfinite(epoch.center_longitude_deg) ||
        !std::isfinite(epoch.center_altitude_m) ||
        !ego_swarm::SwarmFrame::is_valid_geodetic(ego_swarm::GeoCoordinate{
            epoch.center_latitude_deg, epoch.center_longitude_deg,
            epoch.center_altitude_m}))
    {
        return false;
    }
    for (std::size_t index = 0U; index < epoch.member_agent_ids.size(); ++index)
    {
        if (epoch.member_agent_ids.at(index) <= 0 ||
            (index > 0U && epoch.member_agent_ids.at(index - 1U) >=
                epoch.member_agent_ids.at(index)) ||
            !std::isfinite(epoch.member_latitudes_deg.at(index)) ||
            !std::isfinite(epoch.member_longitudes_deg.at(index)) ||
            !std::isfinite(epoch.member_altitudes_m.at(index)) ||
            !ego_swarm::SwarmFrame::is_valid_geodetic(ego_swarm::GeoCoordinate{
                epoch.member_latitudes_deg.at(index),
                epoch.member_longitudes_deg.at(index),
                epoch.member_altitudes_m.at(index)}))
        {
            return false;
        }
    }
    return epoch.snapshot_hash == formation_epoch_hash(epoch);
}

bool EgoSwarmPlannerNode::epoch_is_same_snapshot(
    const taiphoon_agent::msg::FormationEpoch& first,
    const taiphoon_agent::msg::FormationEpoch& second) const
{
    return first.sequence == second.sequence &&
        first.snapshot_hash == second.snapshot_hash &&
        first.proposer_agent_id == second.proposer_agent_id;
}

bool EgoSwarmPlannerNode::epoch_is_higher_priority(
    const taiphoon_agent::msg::FormationEpoch& first,
    const taiphoon_agent::msg::FormationEpoch& second) const
{
    if (first.sequence != second.sequence)
    {
        return first.sequence > second.sequence;
    }
    return first.proposer_agent_id < second.proposer_agent_id;
}

bool EgoSwarmPlannerNode::epoch_parent_matches_current(
    const taiphoon_agent::msg::FormationEpoch& epoch) const
{
    if (!has_accepted_formation_epoch_)
    {
        return epoch.sequence == 1U && epoch.parent_sequence == 0U &&
            epoch.parent_snapshot_hash == 0U;
    }
    return epoch.sequence > accepted_formation_epoch_.sequence &&
        epoch.parent_sequence == accepted_formation_epoch_.sequence &&
        epoch.parent_snapshot_hash == accepted_formation_epoch_.snapshot_hash;
}

bool EgoSwarmPlannerNode::make_formation_proposal(
    const std::vector<int>& member_ids,
    const ego_swarm::Vector3* requested_center)
{
    if (!has_own_state_ || !update_swarm_frame())
    {
        RCLCPP_WARN(get_logger(),
                    "Formation proposal deferred until local WGS84 Home is ready");
        return false;
    }
    std::vector<int> members = member_ids;
    std::sort(members.begin(), members.end());
    members.erase(std::unique(members.begin(), members.end()), members.end());
    if (members.empty() ||
        !std::binary_search(members.begin(), members.end(), agent_index_))
    {
        RCLCPP_WARN(get_logger(),
                    "Formation proposal requires this Agent to be a member");
        return false;
    }

    taiphoon_agent::msg::FormationEpoch proposal;
    proposal.valid = true;
    proposal.creation_stamp = this->now();
    proposal.sequence = has_accepted_formation_epoch_
        ? accepted_formation_epoch_.sequence + 1U : 1U;
    proposal.parent_sequence = has_accepted_formation_epoch_
        ? accepted_formation_epoch_.sequence : 0U;
    proposal.parent_snapshot_hash = has_accepted_formation_epoch_
        ? accepted_formation_epoch_.snapshot_hash : 0U;
    proposal.proposer_agent_id = agent_index_;
    proposal.sender_agent_id = agent_index_;
    proposal.status = taiphoon_agent::msg::FormationEpoch::PROPOSAL;
    proposal.formation_type = static_cast<uint8_t>(formation_config_.type);
    proposal.radius_m = formation_config_.radius_m;
    proposal.spacing_m = formation_config_.spacing_m;
    proposal.vertical_offset_m = formation_config_.vertical_offset_m;

    if (has_accepted_formation_epoch_)
    {
        // 一般 membership 變更沿用 parent center；群組 MOVE 則以共同
        // swarm_ned 計算出的新 center 建立下一個 Epoch。
        ego_swarm::GeoCoordinate proposal_center{
            accepted_formation_epoch_.center_latitude_deg,
            accepted_formation_epoch_.center_longitude_deg,
            accepted_formation_epoch_.center_altitude_m};
        if (requested_center != nullptr)
        {
            if (!ego_swarm::is_finite_vector(*requested_center) ||
                !swarm_frame_.from_swarm_ned(*requested_center, proposal_center))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting Formation proposal: requested center is invalid");
                return false;
            }
        }
        proposal.center_latitude_deg = proposal_center.latitude_deg;
        proposal.center_longitude_deg = proposal_center.longitude_deg;
        proposal.center_altitude_m = proposal_center.altitude_m;
        for (const auto& entry : formation_assignment_)
        {
            proposal.parent_assignment_agent_ids.push_back(entry.first);
            proposal.parent_assignment_slot_indices.push_back(
                static_cast<int32_t>(entry.second));
        }
    }

    double latitude_sum = 0.0;
    double longitude_sum = 0.0;
    ego_swarm::Vector3 target_position_sum{0.0, 0.0, 0.0};
    bool all_member_targets_available = true;
    for (const int member_id : members)
    {
        taiphoon_agent::msg::AgentState state{};
        if (member_id == agent_index_)
        {
            state = own_state_;
        }
        else
        {
            const auto iterator = peers_.find(member_id);
            if (iterator == peers_.end() || !iterator->second.has_state)
            {
                return false;
            }
            state = iterator->second.state;
        }
        if (!is_finite_state_position(state))
        {
            return false;
        }
        const double latitude = static_cast<double>(state.position_global.at(0));
        const double longitude = static_cast<double>(state.position_global.at(1));
        const double altitude = static_cast<double>(state.position_global.at(2));
        proposal.member_agent_ids.push_back(member_id);
        proposal.member_latitudes_deg.push_back(latitude);
        proposal.member_longitudes_deg.push_back(longitude);
        proposal.member_altitudes_m.push_back(altitude);
        latitude_sum += latitude;
        longitude_sum += longitude;

        ego_swarm::Vector3 member_target{};
        bool member_target_available = false;
        if (member_id == agent_index_ && has_goal_ &&
            ego_swarm::is_finite_vector(goal_position_))
        {
            member_target = goal_position_;
            member_target_available = true;
        }
        else if (member_id != agent_index_)
        {
            ego_swarm::PeerTrajectory peer_trajectory;
            const double now_s = this->now().seconds();
            if (std::isfinite(now_s) && peer_trajectories_.get_fresh(
                    member_id, now_s, peer_trajectory) &&
                peer_trajectory.trajectory.is_valid())
            {
                const auto endpoint = peer_trajectory.trajectory.evaluate(
                    peer_trajectory.trajectory.duration_s());
                if (endpoint.is_valid)
                {
                    member_target = endpoint.position;
                    member_target_available = true;
                }
            }
        }
        // 沒有可用的 peer target 時，只有在所有成員都缺少 target
        // 的情況下才回退到同步位置中心，避免混用 target／current。
        if (member_target_available)
        {
            target_position_sum = ego_swarm::add_vectors(
                target_position_sum, member_target);
        }
        all_member_targets_available = all_member_targets_available &&
            member_target_available;
    }
    if (!has_accepted_formation_epoch_)
    {
        if (has_formation_center_target_)
        {
            ego_swarm::GeoCoordinate target_center{};
            if (!swarm_frame_.from_swarm_ned(
                    formation_center_, target_center))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting formation proposal: saved center target is invalid");
                return false;
            }
            proposal.center_latitude_deg = target_center.latitude_deg;
            proposal.center_longitude_deg = target_center.longitude_deg;
            proposal.center_altitude_m = target_center.altitude_m;
        }
        else if (all_member_targets_available)
        {
            const double count = static_cast<double>(members.size());
            const ego_swarm::Vector3 target_center_ned =
                ego_swarm::scale_vector(target_position_sum, 1.0 / count);
            ego_swarm::GeoCoordinate target_center{};
            if (!swarm_frame_.from_swarm_ned(
                    target_center_ned, target_center))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting formation proposal: member target center is invalid");
                return false;
            }
            proposal.center_latitude_deg = target_center.latitude_deg;
            proposal.center_longitude_deg = target_center.longitude_deg;
            proposal.center_altitude_m = target_center.altitude_m;
        }
        else
        {
            const double count = static_cast<double>(members.size());
            proposal.center_latitude_deg = latitude_sum / count;
            proposal.center_longitude_deg = longitude_sum / count;
            // 第一次若沒有完整成員 target，才以同步成員位置建立初始
            // 中心；之後的 CIRCLE/HORI/VERT 均沿用保存的目標中心。
            proposal.center_altitude_m = swarm_frame_.origin().altitude_m;
        }
    }
    proposal.snapshot_hash = formation_epoch_hash(proposal);

    candidate_formation_epoch_ = proposal;
    has_candidate_formation_epoch_ = true;
    candidate_adopted_agents_.clear();
    candidate_adopted_agents_.insert(agent_index_);
    publish_formation_epoch(taiphoon_agent::msg::FormationEpoch::PROPOSAL);
    publish_formation_epoch(taiphoon_agent::msg::FormationEpoch::ADOPTED);
    RCLCPP_INFO(
        get_logger(),
        "Proposed FormationEpoch seq=%u hash=%u proposer=%d members=%zu",
        proposal.sequence, proposal.snapshot_hash, proposal.proposer_agent_id,
        proposal.member_agent_ids.size());
    maybe_activate_formation_epoch();
    return true;
}

bool EgoSwarmPlannerNode::update_formation_assignment()
{
    if (!has_accepted_formation_epoch_ || !swarm_frame_.has_origin())
    {
        return false;
    }
    std::vector<int> ids;
    std::vector<ego_swarm::Vector3> positions;
    ids.reserve(accepted_formation_epoch_.member_agent_ids.size());
    positions.reserve(ids.capacity());
    for (std::size_t index = 0U;
         index < accepted_formation_epoch_.member_agent_ids.size(); ++index)
    {
        ids.push_back(accepted_formation_epoch_.member_agent_ids.at(index));
        ego_swarm::Vector3 position{};
        if (!swarm_frame_.to_swarm_ned(ego_swarm::GeoCoordinate{
                accepted_formation_epoch_.member_latitudes_deg.at(index),
                accepted_formation_epoch_.member_longitudes_deg.at(index),
                accepted_formation_epoch_.member_altitudes_m.at(index)}, position))
        {
            return false;
        }
        positions.push_back(position);
    }
    ego_swarm::Vector3 center{};
    if (!swarm_frame_.to_swarm_ned(ego_swarm::GeoCoordinate{
            accepted_formation_epoch_.center_latitude_deg,
            accepted_formation_epoch_.center_longitude_deg,
            accepted_formation_epoch_.center_altitude_m}, center))
    {
        return false;
    }
    const ego_swarm::Vector3 canonical_position = center;
    for (auto& position : positions)
    {
        position = ego_swarm::subtract_vectors(position, canonical_position);
    }
    std::map<int, std::size_t> previous_assignment;
    for (std::size_t index = 0U;
         index < accepted_formation_epoch_.parent_assignment_agent_ids.size();
         ++index)
    {
        const int agent_id = accepted_formation_epoch_.parent_assignment_agent_ids.at(index);
        const int slot = accepted_formation_epoch_.parent_assignment_slot_indices.at(index);
        if (agent_id > 0 && slot >= 0)
        {
            previous_assignment[agent_id] = static_cast<std::size_t>(slot);
        }
    }
    ego_swarm::FormationAssignmentResult assignment;
    if (!ego_swarm::FormationAssignment::assign(
            ids, positions, formation_config_, previous_assignment,
            formation_slot_switch_penalty_m_, formation_crossing_penalty_m_,
            assignment) ||
        assignment.agent_to_slot.find(agent_index_) ==
            assignment.agent_to_slot.end())
    {
        return false;
    }
    formation_assignment_ = assignment.agent_to_slot;
    formation_center_ = center;
    has_formation_center_target_ = true;
    return true;
}

void EgoSwarmPlannerNode::adopt_formation_epoch(
    const taiphoon_agent::msg::FormationEpoch& epoch)
{
    if (!epoch_is_valid(epoch))
    {
        RCLCPP_WARN(get_logger(), "Rejecting invalid FormationEpoch message");
        return;
    }
    bool is_same_sequence_higher_priority = false;
    if (has_accepted_formation_epoch_ &&
        !epoch_is_same_snapshot(epoch, accepted_formation_epoch_))
    {
        if (epoch.sequence < accepted_formation_epoch_.sequence)
        {
            return;
        }
        if (epoch.sequence == accepted_formation_epoch_.sequence)
        {
            // 同 sequence 以較小 proposer ID 勝出。較低優先 proposer
            // 可能先抵達並被暫時採用，因此不能把晚到的 winner 永久
            // 視為過期訊息；允許它重新收斂，並重新等待全員 ADOPTED。
            if (epoch.proposer_agent_id >=
                accepted_formation_epoch_.proposer_agent_id ||
                epoch.parent_sequence != accepted_formation_epoch_.parent_sequence ||
                epoch.parent_snapshot_hash !=
                    accepted_formation_epoch_.parent_snapshot_hash)
            {
                return;
            }
            is_same_sequence_higher_priority = true;
        }
    }
    // 尚未被納入 parent epoch 的新 Agent 沒有 parent 的本地 accepted
    // 狀態；FormationEpoch 本身已攜帶完整 snapshot，因此可先接受
    // 這份 proposal 作為候選並保持 hold，等全體 members ADOPTED 後
    // 才啟用。否則新 Agent 若錯過 parent heartbeat 會永久卡住。
    if (has_accepted_formation_epoch_ && !is_same_sequence_higher_priority &&
        !epoch_is_same_snapshot(epoch, accepted_formation_epoch_) &&
        !epoch_parent_matches_current(epoch))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Rejecting FormationEpoch seq=%u: parent does not match current "
            "accepted epoch", epoch.sequence);
        return;
    }

    if (!has_candidate_formation_epoch_ ||
        epoch_is_higher_priority(epoch, candidate_formation_epoch_))
    {
        candidate_formation_epoch_ = epoch;
        has_candidate_formation_epoch_ = true;
        candidate_adopted_agents_.clear();
    }
    if (!epoch_is_same_snapshot(epoch, candidate_formation_epoch_))
    {
        return;
    }
    if (epoch.status == taiphoon_agent::msg::FormationEpoch::ADOPTED &&
        std::binary_search(
            candidate_formation_epoch_.member_agent_ids.begin(),
            candidate_formation_epoch_.member_agent_ids.end(),
            epoch.sender_agent_id))
    {
        candidate_adopted_agents_.insert(epoch.sender_agent_id);
    }
    if (std::binary_search(
            candidate_formation_epoch_.member_agent_ids.begin(),
            candidate_formation_epoch_.member_agent_ids.end(), agent_index_))
    {
        candidate_adopted_agents_.insert(agent_index_);
        if (epoch.sender_agent_id != agent_index_ &&
            epoch.status == taiphoon_agent::msg::FormationEpoch::PROPOSAL)
        {
            publish_formation_epoch(taiphoon_agent::msg::FormationEpoch::ADOPTED);
        }
    }
    maybe_activate_formation_epoch();
}

void EgoSwarmPlannerNode::formation_epoch_cb(
    const taiphoon_agent::msg::FormationEpoch::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "formation_epoch_cb received nullptr");
        return;
    }
    adopt_formation_epoch(*msg);
}

void EgoSwarmPlannerNode::maybe_activate_formation_epoch()
{
    if (!has_candidate_formation_epoch_ ||
        !std::binary_search(
            candidate_formation_epoch_.member_agent_ids.begin(),
            candidate_formation_epoch_.member_agent_ids.end(), agent_index_))
    {
        return;
    }
    // Transient Local heartbeat 會反覆送出同一個 ADOPTED snapshot；已啟用
    // 的 epoch 不得再次 invalidate 目前 trajectory 或重建 assignment。
    if (has_accepted_formation_epoch_ &&
        epoch_is_same_snapshot(
            candidate_formation_epoch_, accepted_formation_epoch_))
    {
        return;
    }
    for (const int member_id : candidate_formation_epoch_.member_agent_ids)
    {
        if (candidate_adopted_agents_.find(member_id) ==
            candidate_adopted_agents_.end())
        {
            return;
        }
    }

    const auto previous_epoch = accepted_formation_epoch_;
    const bool had_previous_epoch = has_accepted_formation_epoch_;
    accepted_formation_epoch_ = candidate_formation_epoch_;
    has_accepted_formation_epoch_ = true;
    formation_config_.type = static_cast<ego_swarm::FormationType>(
        accepted_formation_epoch_.formation_type);
    formation_config_.radius_m = accepted_formation_epoch_.radius_m;
    formation_config_.spacing_m = accepted_formation_epoch_.spacing_m;
    formation_config_.vertical_offset_m =
        accepted_formation_epoch_.vertical_offset_m;
    if (!update_formation_assignment())
    {
        accepted_formation_epoch_ = previous_epoch;
        has_accepted_formation_epoch_ = had_previous_epoch;
        return;
    }
    has_formation_ = true;
    has_goal_ = false;
    preserve_altitude_ = false;
    formation_membership_change_pending_ = false;
    formation_initial_plan_pending_ = true;
    invalidate_active_plan();
    RCLCPP_INFO(
        get_logger(),
        "Adopted FormationEpoch seq=%u hash=%u assignment_slot=%zu",
        accepted_formation_epoch_.sequence,
        accepted_formation_epoch_.snapshot_hash,
        formation_assignment_.at(agent_index_));
}

void EgoSwarmPlannerNode::publish_formation_epoch(const uint8_t status)
{
    if (formation_epoch_pub_ == nullptr || !has_candidate_formation_epoch_)
    {
        return;
    }
    auto message = candidate_formation_epoch_;
    message.sender_agent_id = agent_index_;
    message.status = status;
    message.valid = true;
    formation_epoch_pub_->publish(message);
    last_epoch_publish_s_ = this->now().seconds();
}

void EgoSwarmPlannerNode::obstacle_cb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "obstacle_cb received nullptr");
        return;
    }
    if (msg->header.frame_id != "swarm_ned")
    {
        RCLCPP_WARN(get_logger(), "Rejecting obstacle cloud with frame '%s'",
                    msg->header.frame_id.c_str());
        return;
    }

    std::vector<ego_swarm::Vector3> points;
    try
    {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        const std::size_t point_count = static_cast<std::size_t>(msg->width) *
            static_cast<std::size_t>(msg->height);
        points.reserve(point_count);
        for (std::size_t index = 0U; index < point_count;
             ++index, ++iter_x, ++iter_y, ++iter_z)
        {
            const ego_swarm::Vector3 point{
                static_cast<double>(*iter_x), static_cast<double>(*iter_y),
                static_cast<double>(*iter_z)};
            if (!ego_swarm::is_finite_vector(point))
            {
                RCLCPP_WARN(get_logger(),
                            "Rejecting obstacle cloud with non-finite point");
                return;
            }
            points.push_back(point);
        }
    }
    catch (const std::exception& error)
    {
        RCLCPP_WARN(get_logger(), "Rejecting malformed PointCloud2: %s",
                    error.what());
        return;
    }

    const double stamp_s = static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1.0e-9;
    if (!std::isfinite(stamp_s) || stamp_s < 0.0 ||
        !obstacle_map_.update(points, stamp_s))
    {
        RCLCPP_WARN(get_logger(),
                    "Rejecting obstacle cloud with invalid timestamp or data");
        return;
    }
}

void EgoSwarmPlannerNode::trajectory_cb(
    const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "trajectory_cb received nullptr");
        return;
    }
    if (msg->agent_index == 0U ||
        msg->agent_index == static_cast<uint32_t>(agent_index_) ||
        !msg->valid || msg->order != ego_swarm::BSPLINE_DEGREE)
    {
        return;
    }

    if (msg->coordinate_frame !=
            taiphoon_agent::msg::AgentTrajectory::
                COORDINATE_FRAME_WGS84_GEODETIC ||
        msg->frame_revision != ego_swarm::GLOBAL_WGS84_FRAME_REVISION)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: unsupported global frame "
            "format=%u revision=%u",
            msg->agent_index, msg->coordinate_frame, msg->frame_revision);
        return;
    }

    if (frame_revision_ == 0U || !swarm_frame_.has_origin())
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: local Home frame is not ready",
            msg->agent_index);
        return;
    }

    if (!std::isfinite(msg->knot_interval_s) ||
        msg->knot_interval_s <= 0.0F ||
        !std::isfinite(msg->duration_s) || msg->duration_s <= 0.0F)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: invalid duration or knot "
            "interval",
            msg->agent_index);
        return;
    }

    if (msg->control_points.empty() ||
        msg->control_points.size() % 3U != 0U)
    {
        RCLCPP_WARN(get_logger(),
                    "Rejecting malformed peer trajectory control points");
        return;
    }

    std::vector<ego_swarm::Vector3> points;
    points.reserve(msg->control_points.size() / 3U);
    for (std::size_t index = 0U;
         index < msg->control_points.size(); index += 3U)
    {
        const ego_swarm::GeoCoordinate global_point{
            msg->control_points.at(index),
            msg->control_points.at(index + 1U),
            msg->control_points.at(index + 2U)};
        ego_swarm::Vector3 local_point{};
        if (!ego_swarm::SwarmFrame::is_valid_geodetic(global_point) ||
            !swarm_frame_.to_swarm_ned(global_point, local_point))
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting peer trajectory agent=%u: invalid WGS84 point",
                msg->agent_index);
            return;
        }
        points.push_back(local_point);
    }

    ego_swarm::PeerTrajectory peer;
    peer.agent_id = static_cast<int>(msg->agent_index);
    peer.trajectory_id = msg->trajectory_id;
    peer.frame_revision = frame_revision_;
    peer.terminal_hold = msg->terminal_hold;
    peer.message_time_s = static_cast<double>(msg->stamp.sec) +
        static_cast<double>(msg->stamp.nanosec) * 1.0e-9;
    peer.start_time_s = static_cast<double>(msg->start_time.sec) +
        static_cast<double>(msg->start_time.nanosec) * 1.0e-9;
    peer.receive_time_s = this->now().seconds();
    peer.trajectory = ego_swarm::BsplineTrajectory(
        points, static_cast<double>(msg->knot_interval_s));
    peer.end_time_s = peer.start_time_s + peer.trajectory.duration_s();

    if (!std::isfinite(peer.message_time_s) ||
        !std::isfinite(peer.start_time_s) ||
        !std::isfinite(peer.receive_time_s) ||
        !peer.trajectory.is_valid() ||
        !std::isfinite(peer.end_time_s) ||
        !is_consistent_duration(
            static_cast<double>(msg->duration_s),
            peer.trajectory.duration_s()))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: duration %.6f s does not "
            "match B-spline duration %.6f s",
            msg->agent_index,
            static_cast<double>(msg->duration_s),
            peer.trajectory.duration_s());
        return;
    }

    if (!peer_trajectories_.update(peer))
    {
        uint32_t latest_trajectory_id = 0U;
        const bool has_latest_trajectory_id =
            peer_trajectories_.get_latest_trajectory_id(
                static_cast<int>(msg->agent_index), latest_trajectory_id);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejecting peer trajectory agent=%u id=%u: peer table latest="
            "%u known=%s",
            msg->agent_index, msg->trajectory_id, latest_trajectory_id,
            has_latest_trajectory_id ? "true" : "false");
    }
}

void EgoSwarmPlannerNode::planner_loop_cb()
{
    const rclcpp::Time now = this->now();
    const double now_s = now.seconds();
    if (!std::isfinite(now_s))
    {
        publish_status(PLANNER_STATE_HOLD, "Planner time unavailable",
                       nullptr, false, false, now);
        return;
    }
    if (has_candidate_formation_epoch_ &&
        (!std::isfinite(last_epoch_publish_s_) ||
         now_s - last_epoch_publish_s_ >= FORMATION_EPOCH_HEARTBEAT_PERIOD_S))
    {
        publish_formation_epoch(
            candidate_adopted_agents_.find(agent_index_) !=
                candidate_adopted_agents_.end()
                ? taiphoon_agent::msg::FormationEpoch::ADOPTED
                : taiphoon_agent::msg::FormationEpoch::PROPOSAL);
    }
    std::vector<int> expected_peer_ids;
    // PeerMatcher 先更新 currently_discovered_peers_，再由下一批 DDS
    // callback 填入 peers_。若只從 peers_ 建 expected list，這段 race
    // 期間 all_fresh(empty) 會錯誤放行一條沒有 peer occupancy 的直線。
    expected_peer_ids.reserve(currently_discovered_peers_.size());
    for (const int peer_id : currently_discovered_peers_)
    {
        if (peer_id > 0 && peer_id != agent_index_)
        {
            expected_peer_ids.push_back(peer_id);
        }
    }
    const bool peer_data_valid = peer_trajectories_.all_fresh(
        now_s, expected_peer_ids);
    if (!has_own_state_ || !update_swarm_frame())
    {
        (void)peer_trajectories_.prune(now_s);
        publish_status(PLANNER_STATE_HOLD, "Local WGS84 Home unavailable",
                       nullptr, false, peer_data_valid, now);
        return;
    }

    // 命令可能早於第一筆有限 WGS84 AgentState 到達；保留命令並在
    // local Home 建立後重試，避免一次性的全域 topic 發送被悄悄丟失。
    if (formation_command_pending_ && !has_candidate_formation_epoch_ &&
        (!has_accepted_formation_epoch_ || !has_formation_))
    {
        if (make_formation_proposal(stable_member_ids()))
        {
            formation_command_pending_ = false;
        }
    }

    // 沒有任務或編隊目標時，若仍在執行 emergency brake，必須先持續
    // 發布同一條煞車軌跡；煞停完成後才切換成 terminal hold heartbeat。
    if (!has_goal_ && !has_formation_)
    {
        if (has_active_plan_ && active_plan_.trajectory.is_valid())
        {
            const double active_elapsed_s =
                (now - active_plan_start_time_).seconds();
            if (std::isfinite(active_elapsed_s) && active_elapsed_s <=
                    active_plan_.trajectory.duration_s())
            {
                ego_swarm::Vector3 current{};
                if (get_position_in_swarm_frame(own_state_, current))
                {
                    publish_plan(
                        active_plan_, current, now, active_plan_start_time_,
                        active_plan_trajectory_id_);
                }
                publish_status(
                    PLANNER_STATE_HOLD, "Emergency brake in progress",
                    &active_plan_, true, peer_data_valid, now);
                (void)peer_trajectories_.prune(now_s);
                return;
            }

            const auto endpoint = active_plan_.trajectory.evaluate(
                active_plan_.trajectory.duration_s());
            if (endpoint.is_valid)
            {
                invalidate_active_plan();
                terminal_hold_position_ = endpoint.position;
                terminal_hold_plan_ = ego_swarm::make_exact_hold_result(
                    terminal_hold_position_, planner_config_);
                terminal_hold_start_time_ = now;
                terminal_hold_trajectory_id_ = ++trajectory_id_;
                has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
            }
            else
            {
                invalidate_active_plan();
            }
        }

        // TAKEOFF／LAND 等 BasicController 任務不會經過本 planner 的
        // mission goal callback。若待命時先建立了 terminal hold，飛機
        // 之後起飛便不能繼續廣播起飛前的位置，否則其他 Agent 會把
        // 舊高度當成真實 occupancy，planner 可能錯誤放行同高度直線。
        // 只在沒有 active brake 的待命路徑更新 peer heartbeat；這些
        // occupancy 資料不應取得本機 BasicController 的控制權。
        ego_swarm::Vector3 current{};
        const bool current_position_available =
            get_position_in_swarm_frame(own_state_, current);
        if (has_terminal_hold_ && current_position_available &&
            ego_swarm::distance_between_centers(
                current, terminal_hold_position_) >
            local_arrival_tolerance_m_)
        {
            terminal_hold_position_ = current;
            terminal_hold_plan_ = ego_swarm::make_exact_hold_result(
                current, planner_config_);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
        }

        if (has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid())
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true,
                TrajectoryPublishRoute::PEER_ONLY);
            publish_status(
                PLANNER_STATE_HOLD,
                "Terminal hold; peer occupancy heartbeat active",
                &terminal_hold_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return;
        }

        // 待命中的 Agent 也必須廣播自己的固定位置，讓其他 Agent 在
        // 下一個任務開始前就能建立完整 peer occupancy snapshot；不能把
        // 「目前沒有移動任務」誤當成「空氣中沒有 peer」。
        if (!has_terminal_hold_ && current_position_available)
        {
            terminal_hold_position_ = current;
            terminal_hold_plan_ = ego_swarm::make_exact_hold_result(
                current, planner_config_);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
        }
        if (has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid())
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true,
                TrajectoryPublishRoute::PEER_ONLY);
            publish_status(
                PLANNER_STATE_HOLD,
                "Terminal hold; peer occupancy heartbeat active",
                &terminal_hold_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return;
        }

        (void)peer_trajectories_.prune(now_s);
        publish_status(
            PLANNER_STATE_HOLD,
            "No mission target; BasicController keeps its hold setpoint",
            nullptr, true, peer_data_valid, now);
        return;
    }

    ego_swarm::Vector3 current{};
    if (!get_position_in_swarm_frame(own_state_, current))
    {
        (void)peer_trajectories_.prune(now_s);
        publish_status(PLANNER_STATE_HOLD, "Own GPS position unavailable",
                       nullptr, false, peer_data_valid, now);
        return;
    }

    ego_swarm::VehiclePlanningState planning_state =
        make_planning_state(current, now_s);

    // 無論目前是否已經進入 safety recovery，Final Target 都必須保留為最後
    // 的任務目標。Recovery 只能改變當下執行的安全軌跡，不能把任務
    // 目標悄悄改成目前位置，否則煞停後永遠不會重新規劃原任務。
    ego_swarm::Vector3 final_target{};
    if (!get_current_final_target(final_target))
    {
        publish_emergency_brake(
            planning_state, current, "target unavailable", now,
            peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    // 近距離 recovery 必須先於 live hard gate 處理：若起點已低於
    // peer_collision_min_distance，正常 validator 會合理地拒絕 sample 0，
    // 但 recovery 需要在「不再惡化、逐步拉開」的窄化契約下取得一次移動
    // 機會。所有無法證明 recovery 安全的情況仍回到下面既有 hold/brake。
    if (handle_peer_deconfliction(
            current, final_target, planning_state, now, peer_data_valid))
    {
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    // Peer trajectory 是預測資料；實際飛機可能因 PX4 estimator 或控制
    // 延遲偏離宣告路徑，因此每個 planner tick 仍必須檢查 live WGS84
    // 位置。低於硬性碰撞距離立即煞停；進入額外 1 m 警戒帶且本機正
    // 執行移動軌跡時提前煞停，保留飛控反應時間。
    const auto live_peer_distance = live_peer_min_distance(current);
    const double collision_distance = std::max(
        formation_min_distance_m_,
        planner_config_.peer_collision_min_distance_m);
    const double current_speed_mps = std::sqrt(
        ego_swarm::squared_norm(planning_state.velocity));
    const double stopping_distance_m =
        std::isfinite(current_speed_mps) && current_speed_mps > 0.0 &&
        std::isfinite(planner_config_.max_acceleration_mps2) &&
        planner_config_.max_acceleration_mps2 > 0.0
        ? (current_speed_mps * current_speed_mps) /
            (2.0 * planner_config_.max_acceleration_mps2)
        : 0.0;
    const double live_warning_distance = collision_distance +
        std::max(
            LIVE_PEER_REACTION_BUFFER_M,
            stopping_distance_m + LIVE_PEER_REACTION_BUFFER_M);
    const bool is_moving = std::isfinite(current_speed_mps) &&
        current_speed_mps > 0.05;
    const bool has_valid_active_route = has_active_plan_ &&
        !emergency_brake_active_ && !waiting_for_replan_ &&
        active_plan_.trajectory.is_valid();
    // 實際飛控位置與 AgentTrajectory 可能存在 tracking delay；進入反應
    // 警戒帶且尚未有已驗證的 active route 才直接煞停，不能等到 live
    // distance 已低於 3 m 才處置。已有 route 時，後面的 common validator
    // 會用同一份 peer snapshot 檢查整條路徑；若此時僅因起飛／隊形的
    // 初始間距落在 reaction buffer 內便煞停，永遠沒有機會執行已規劃
    // 的水平繞行。
    if (ego_swarm::live_peer_collision_detected(
            live_peer_distance, collision_distance))
    {
        // 舊 active route 已經不能再作為安全恢復路徑；保留 Final Target，讓
        // brake 完成後以最後任務目標重新進入 optimizer。若目前已在
        // collision margin 內，先煞停／固定原高度仍是必要的 fail-closed
        // 動作，不能發布一條從不安全起點穿越 peer 的假安全路徑。
        if (!emergency_brake_active_)
        {
            invalidate_active_plan();
        }
        if (!is_moving)
        {
            (void)publish_waiting_hold(
                current, final_target,
                "optimized_no_feasible_horizontal_path; replan final target",
                now, peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        publish_emergency_brake(
            planning_state, final_target,
            "optimized_emergency_brake: live peer position inside collision margin", now,
            peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }
    if (is_moving && live_peer_distance.has_peer &&
        live_peer_distance.data_valid &&
        live_peer_distance.minimum_distance_m < live_warning_distance &&
        !has_valid_active_route && !has_goal_ && !has_formation_)
    {
        publish_emergency_brake(
            planning_state, current,
            "optimized_emergency_brake: live peer warning distance reached",
            now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    if (has_formation_ && formation_membership_change_pending_)
    {
        // Discovery 在成員集合穩定前可能只缺一個 scan；此期間停止
        // 舊 epoch 的任何移動，避免新舊成員集合同時釋放 trajectory。
        if (has_active_plan_)
        {
            invalidate_active_plan();
        }
        if (!has_terminal_hold_ ||
            ego_swarm::distance_between_centers(
                terminal_hold_position_, current) >
            local_arrival_tolerance_m_)
        {
            terminal_hold_position_ = current;
            terminal_hold_plan_ = ego_swarm::make_exact_hold_result(
                current, planner_config_);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
        }
        if (has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid())
        {
            publish_plan(
                terminal_hold_plan_, current, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true);
            publish_status(
                PLANNER_STATE_HOLD,
                "Formation membership change pending; holding safely",
                &terminal_hold_plan_, true, peer_data_valid, now);
        }
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    const auto arrival = evaluate_target_arrival(current, final_target);
    // 編隊的槽位與中心由 FormationEpoch 的 WGS84 snapshot 定義；local
    // NED 只是在本機控制器內的投影，可能因 PX4 Home/odometry 與 GPS
    // frozen transform 的小幅差異而比 global 位置多出誤差。編隊到位
    // 時以共同 global NED 判斷，避免已在槽位內的 Agent 永遠無法建立
    // terminal hold；一般 MOVE 仍要求 local 與 global 都通過。
    const bool formation_global_arrived = has_formation_ &&
        arrival.global_valid &&
        arrival.global_error_m <= global_arrival_tolerance_m_;
    if (arrival.reached || formation_global_arrived)
    {
        const double speed_mps = std::sqrt(
            ego_swarm::squared_norm(planning_state.velocity));

        // 編隊是持續存在的任務，不是單一 Agent 到槽位後就結束的
        // one-shot move。保留 has_formation_，讓後續 Agent 能以此機的
        // Final Target terminal hold 判斷 transition readiness；同時避免每個
        // planner tick 重建相同的 hold trajectory。
        const bool formation_target_hold_active = has_formation_ &&
            has_terminal_hold_ &&
            ego_swarm::distance_between_centers(
                terminal_hold_position_, final_target) <=
            global_arrival_tolerance_m_;
        if (formation_target_hold_active)
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true);
            publish_status(
                PLANNER_STATE_HOLD,
                evaluate_formation_convergence(now_s).is_converged
                    ? "formation_converged; terminal hold active"
                    : "Terminal hold; formation slot occupied",
                &terminal_hold_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return;
        }

        has_goal_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        // 編隊槽位到達以共同位置誤差判斷；不要再用差分速度或
        // 飛控殘餘速度阻止建立 terminal hold。PX4 會繼續追蹤固定
        // setpoint，而 planner 只需維持該槽位的 occupancy heartbeat。
        // 一般 MOVE 仍保留原本 0.05 m/s 的較嚴格停止條件。
        const bool moving_too_fast_for_move = !has_formation_ &&
            std::isfinite(speed_mps) && speed_mps > 0.05;
        if (moving_too_fast_for_move)
        {
            publish_emergency_brake(
                planning_state, final_target, "target reached while moving", now,
                peer_data_valid);
        }
        else
        {
            // 到達判斷允許位置誤差，因此 hold 必須鎖定目標座標，而不是
            // 使用仍可能漂移的即時位置。
            terminal_hold_position_ = final_target;
            terminal_hold_plan_ = ego_swarm::make_exact_hold_result(
                final_target, planner_config_);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
            if (has_terminal_hold_)
            {
                publish_plan(
                    terminal_hold_plan_, final_target, now,
                    terminal_hold_start_time_, terminal_hold_trajectory_id_,
                    true);
            }
            publish_status(
                PLANNER_STATE_HOLD,
                has_formation_ &&
                    evaluate_formation_convergence(now_s).is_converged
                    ? "formation_converged; terminal hold active"
                    : "Terminal hold; peer occupancy heartbeat active",
                has_terminal_hold_ ? &terminal_hold_plan_ : nullptr,
                true, peer_data_valid, now);
        }
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    if (!peer_data_valid)
    {
        if (has_formation_ && formation_initial_plan_pending_)
        {
            (void)publish_waiting_hold(
                current, final_target,
                "Formation sync waiting for peer trajectory", now, false);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        publish_emergency_brake(
            planning_state, final_target, "peer trajectory stale or invalid", now,
            false);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    const auto obstacle_status = obstacle_map_.status(now_s);
    if (obstacle_status != perception::ObstacleMapStatus::Fresh)
    {
        const std::string reason = obstacle_map_status_reason(now_s);
        publish_emergency_brake(
            planning_state, final_target, reason, now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    // 編隊 transition 只能使用直線或 North/East 側向候選；不允許 planner
    // 以額外的 above/below candidate 繞過同儕。VERT_LINEAR 的目標 Z 仍可
    // 沿目標方向調整，但不會再增加垂直避碰偏移。
    const bool fixed_z_path = preserve_altitude_ ||
        (has_formation_ && formation_config_.type !=
            ego_swarm::FormationType::VERT_LINEAR);
    const bool linear_z_path = has_formation_ &&
        formation_config_.type == ego_swarm::FormationType::VERT_LINEAR;
    const ego_swarm::PathConstraint path_constraint = fixed_z_path
        ? ego_swarm::PathConstraint::FIXED_Z
        : (linear_z_path ? ego_swarm::PathConstraint::LINEAR_Z
                         : ego_swarm::PathConstraint::FREE_3D);
    if (fixed_z_path)
    {
        // 水平移動只改變 X/Y；使用命令建立時的目標 Z 作為整條
        // trajectory 的固定高度，避免 GPS callback 延遲把水平命令
        // 轉成非預期的垂直修正。
        current.at(2) = final_target.at(2);
        planning_state.position = current;
    }

    const bool should_reserve_trajectory =
        ego_swarm::should_reserve_moving_trajectory(
            has_formation_, planner_feature_flags_);
    const double plan_start_delay_s = should_reserve_trajectory
        ? reservation_lead_time_s_ : 0.0;
    const rclcpp::Time plan_start_time = now +
        rclcpp::Duration::from_seconds(plan_start_delay_s);
    const double plan_start_time_s = plan_start_time.seconds();

    std::vector<ego_swarm::PeerTrajectory> active_peers;
    peer_trajectories_.get_active(now_s, active_peers);
    ego_swarm::LocalGoalEnvironment local_goal_environment;
    local_goal_environment.now_s = now_s;
    local_goal_environment.candidate_start_time_s = plan_start_time_s;
    local_goal_environment.self_agent_id = agent_index_;
    local_goal_environment.peers_are_fresh = peer_data_valid;
    local_goal_environment.peers = active_peers;
    local_goal_environment.obstacle_map = &obstacle_map_;
    local_goal_environment.peer_collision_min_distance_m =
        planner_config_.peer_collision_min_distance_m;
    local_goal_environment.obstacle_clearance_m =
        planner_config_.obstacle_clearance_m;
    local_goal_environment.maximum_velocity_mps =
        planner_config_.max_velocity_mps;
    const double local_goal_detour_offset_m = std::max(
        planner_config_.peer_collision_min_distance_m +
            planner_feature_flags_.local_goal_detour_margin_m,
        planner_config_.obstacle_clearance_m + 1.0);
    const auto planning_target_result = ego_swarm::select_planning_target(
        planner_feature_flags_.enable_local_goal_planning,
        current,
        final_target,
        path_constraint,
        ego_swarm::LocalGoalConfig{
            planner_feature_flags_.local_goal_horizon_m,
            1.0,
            local_goal_detour_offset_m},
        local_goal_environment);
    if (!planning_target_result.is_valid)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Local Goal selection failed (%s); fail-closed hold/brake",
            ego_swarm::local_goal_failure_reason_string(
                planning_target_result.failure_reason).c_str());
        const double speed_mps = std::sqrt(
            ego_swarm::squared_norm(planning_state.velocity));
        if (std::isfinite(speed_mps) && speed_mps <= 0.2)
        {
            (void)publish_waiting_hold(
                current, final_target,
                "local_goal_selection_failed", now, peer_data_valid);
        }
        else
        {
            publish_emergency_brake(
                planning_state, final_target,
                "optimized_emergency_brake: local_goal_selection_failed",
                now, peer_data_valid);
        }
        (void)peer_trajectories_.prune(now_s);
        return;
    }
    const ego_swarm::Vector3 planning_target = planning_target_result.goal;

    // 等待中的固定位置路線只保留短時間，讓衝突中的 Agent 有機會
    // 發布較早路線或新的水平路線；不把等待轉成永久 hold。
    if (waiting_for_replan_ && has_active_plan_)
    {
        if (std::isfinite(waiting_retry_time_s_) &&
            now_s < waiting_retry_time_s_)
        {
            publish_plan(
                active_plan_, current, now, active_plan_start_time_,
                active_plan_trajectory_id_, active_plan_.is_hold);
            publish_status(
                PLANNER_STATE_HOLD, "Waiting for local route arbitration",
                &active_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        invalidate_active_plan();
    }

    const bool final_target_unchanged =
        ego_swarm::active_plan_matches_final_target(
            has_active_plan_, active_plan_final_target_, final_target);
    const double active_elapsed_s = has_active_plan_
        ? (now - active_plan_start_time_).seconds() : 0.0;
    const bool active_plan_running = has_active_plan_ &&
        final_target_unchanged &&
        active_plan_.trajectory.is_valid() &&
        std::isfinite(active_elapsed_s) && active_elapsed_s <=
            active_plan_.trajectory.duration_s();
    bool preserve_active_plan_on_candidate_failure = false;
    if (active_plan_running)
    {
        const auto safety_result = validate_plan(
            active_plan_.trajectory, planning_state, now_s,
            active_plan_start_time_.seconds());
        if (!safety_result.is_safe)
        {
            if (safety_result.reason ==
                    ego_swarm::SafetyFailureReason::PEER_COLLISION ||
                safety_result.reason ==
                    ego_swarm::SafetyFailureReason::OBSTACLE_COLLISION)
            {
                // 已選路線遇到新的同儕路線或障礙物時，先撤回本機規劃
                // 並重新走水平候選；只有重新規劃也失敗，才進入煞停。
                invalidate_active_plan();
            }
            else
            {
                publish_emergency_brake(
                    planning_state, final_target,
                    ego_swarm::ContinuousSafetyValidator::reason_string(
                        safety_result.reason),
                    now, peer_data_valid);
                (void)peer_trajectories_.prune(now_s);
                return;
            }
        }
        else
        {
            const ego_swarm::ReplanTriggerInput replan_input{
                planner_feature_flags_.enable_local_goal_planning &&
                !emergency_brake_active_,
                true,
                final_target_unchanged,
                true,
                active_elapsed_s,
                active_plan_.trajectory.duration_s(),
                now_s,
                has_last_replan_time_,
                last_replan_time_s_,
                planner_feature_flags_.local_goal_replan_ratio,
                planner_feature_flags_.minimum_replan_interval_s};
            const auto replan_decision =
                ego_swarm::evaluate_replan_trigger(replan_input);
            if (!replan_decision.should_replan)
            {
                // 重新發布同一條軌跡時保留原始 start_time 與
                // trajectory_id，
                // 讓 BasicController 依原本的時間軸持續取樣，
                // 不會每 100 ms 重置進度。
                publish_plan(
                    active_plan_, current, now, active_plan_start_time_,
                    active_plan_trajectory_id_);
                publish_status(
                    active_plan_.is_hold
                        ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
                    active_plan_.is_hold
                        ? "Holding until replanning is safe" : "ok",
                    &active_plan_, true, peer_data_valid, now);
                (void)peer_trajectories_.prune(now_s);
                return;
            }
            preserve_active_plan_on_candidate_failure =
                replan_decision.reason ==
                ego_swarm::ReplanTriggerReason::SEGMENT_PROGRESS;
        }
    }

    ego_swarm::PlanningSnapshot snapshot;
    snapshot.self_state = planning_state;
    snapshot.local_goal = planning_target;
    snapshot.planning_time_s = now_s;
    snapshot.trajectory_start_time_s = plan_start_time_s;
    snapshot.frame_revision = frame_revision_;
    snapshot.self_agent_id = agent_index_;
    snapshot.path_constraint = path_constraint;
    snapshot.fixed_z_m = final_target.at(2);
    snapshot.obstacle_map = &obstacle_map_;
    snapshot.peers_are_fresh = peer_data_valid;
    snapshot.peers = active_peers;

    // Progress-based replan 的 candidate 若失敗，先保留仍在
    // duration 內且通過最新 safety validation 的舊 trajectory；
    // Final Target 改變、unsafe
    // 或 segment 結束則不適用此 fallback。
    const auto retain_active_plan_if_safe = [&]() -> bool
        {
            if (!preserve_active_plan_on_candidate_failure ||
                !has_active_plan_ || emergency_brake_active_ ||
                waiting_for_replan_ || !active_plan_.trajectory.is_valid())
            {
                return false;
            }
            const double elapsed_s =
                (now - active_plan_start_time_).seconds();
            if (!std::isfinite(elapsed_s) || elapsed_s < 0.0 ||
                elapsed_s > active_plan_.trajectory.duration_s())
            {
                return false;
            }
            const auto active_safety = validate_plan(
                active_plan_.trajectory, planning_state, now_s,
                active_plan_start_time_.seconds());
            if (!active_safety.is_safe)
            {
                return false;
            }
            publish_plan(
                active_plan_, current, now, active_plan_start_time_,
                active_plan_trajectory_id_);
            publish_status(
                active_plan_.is_hold
                    ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
                "Candidate replan failed; retaining safe active trajectory",
                &active_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return true;
        };

    ego_swarm::PlannerResult result = selected_planner_->plan(snapshot);
    if (!result.is_valid || !result.trajectory.is_valid() ||
        !result.has_finite_output())
    {
        if (retain_active_plan_if_safe())
        {
            return;
        }
        const std::string failure_reason = optimized_failure_reason(
            result.termination);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Optimized planner failed (%s) termination=%s planning_ms=%.2f "
            "budget_ms=%.2f iterations=%zu search_radius_m=%.2f "
            "route_search=%s route_nodes=%zu route_ms=%.2f min_peer_m=%.2f; "
            "fail-closed hold/brake",
            failure_reason.c_str(),
            ego_swarm::planner_termination_reason_string(
                result.termination).c_str(),
            result.planning_time_ms, optimizer_time_budget_ms_,
            result.iteration_count, planner_config_.optimizer_search_radius_m,
            result.route_search_used ? "true" : "false",
            result.route_expanded_nodes, result.route_search_time_ms,
            result.minimum_peer_distance_m);
        const double speed_mps = std::sqrt(
            ego_swarm::squared_norm(planning_state.velocity));
        if (std::isfinite(speed_mps) && speed_mps <= 0.2)
        {
            (void)publish_waiting_hold(
                current, final_target,
                failure_reason, now,
                peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        publish_emergency_brake(
            planning_state, final_target,
            "optimized_emergency_brake: " + failure_reason,
            now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    double accepted_start_time_s = result.trajectory_start_time_s;
    bool reservation_accepted = !should_reserve_trajectory;
    if (should_reserve_trajectory)
    {
        ego_swarm::PlanningSnapshot reservation_snapshot = snapshot;
        reservation_snapshot.trajectory_start_time_s =
            result.trajectory_start_time_s;
        const auto reservation = ego_swarm::FormationReservationPlanner::reserve(
            result.trajectory, reservation_snapshot,
            ego_swarm::required_peer_separation_m(
                has_formation_, formation_min_distance_m_,
                planner_config_.peer_collision_min_distance_m),
            reservation_lead_time_s_, reservation_step_s_,
            reservation_horizon_s_);
        if (!reservation.is_reserved)
        {
            if (retain_active_plan_if_safe())
            {
                return;
            }
            const double speed_mps = std::sqrt(
                ego_swarm::squared_norm(planning_state.velocity));
            if (std::isfinite(speed_mps) && speed_mps <= 0.2)
            {
                (void)publish_waiting_hold(
                    current, final_target,
                    "optimized_no_feasible_horizontal_path", now,
                    peer_data_valid);
                (void)peer_trajectories_.prune(now_s);
                return;
            }
            publish_emergency_brake(
                planning_state, final_target,
                "optimized_emergency_brake: "
                "optimized_no_feasible_horizontal_path", now,
                peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        accepted_start_time_s = reservation.start_time_s;
        result.trajectory_start_time_s = accepted_start_time_s;
        reservation_accepted = true;
    }
    const auto safety_result = validate_plan(
        result.trajectory, planning_state, now_s, accepted_start_time_s,
        &snapshot.peers);
    const auto acceptance_decision =
        ego_swarm::evaluate_candidate_acceptance(
        ego_swarm::CandidateAcceptanceInput{
            result.is_valid && result.trajectory.is_valid(),
            result.has_finite_output(),
            should_reserve_trajectory,
            reservation_accepted,
            safety_result.is_safe});
    if (!acceptance_decision.should_accept)
    {
        if (retain_active_plan_if_safe())
        {
            return;
        }
        const double speed_mps = std::sqrt(
            ego_swarm::squared_norm(planning_state.velocity));
        if (safety_result.reason ==
                ego_swarm::SafetyFailureReason::PEER_COLLISION &&
            has_formation_ &&
            (formation_initial_plan_pending_ ||
             (std::isfinite(speed_mps) && speed_mps <= 0.2)))
        {
            (void)publish_waiting_hold(
                current, final_target,
                "optimized_no_feasible_horizontal_path", now,
                peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        publish_emergency_brake(
            planning_state, final_target,
            ego_swarm::ContinuousSafetyValidator::reason_string(
                safety_result.reason),
            now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    clear_terminal_hold();
    const rclcpp::Time accepted_start_time = now +
        rclcpp::Duration::from_seconds(accepted_start_time_s - now_s);
    active_plan_ = result;
    active_plan_target_ = planning_target;
    active_plan_final_target_ = final_target;
    active_plan_start_time_ = accepted_start_time;
    has_last_replan_time_ = true;
    last_replan_time_s_ = now_s;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    emergency_brake_active_ = false;
    waiting_for_replan_ = false;
    waiting_retry_time_s_ = 0.0;
    formation_initial_plan_pending_ = false;
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_);
    publish_status(
        active_plan_.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
        active_plan_.is_hold
            ? "Holding until replanning is safe" : "ok",
        &active_plan_, true, peer_data_valid, now);
    (void)peer_trajectories_.prune(now_s);
}

void EgoSwarmPlannerNode::scan_peers_cb()
{
    // rclcpp 的 get_node_names() 只回傳 basename；PeerMatcher 需要完整
    // namespace 才能判斷 /fmu<N>/basic_controller_node 是否在線。
    std::vector<std::string> fully_qualified_node_names;
    for (const auto& entry :
         get_node_graph_interface()->get_node_names_and_namespaces())
    {
        std::string namespace_name = entry.second;
        if (namespace_name.empty() || namespace_name == "/")
        {
            namespace_name.clear();
        }
        else if (namespace_name.back() == '/')
        {
            namespace_name.pop_back();
        }
        fully_qualified_node_names.push_back(
            namespace_name + "/" + entry.first);
    }
    const auto found = formation::PeerMatcher::filter_agent_nodes(
        fully_qualified_node_names, agent_namespace_);
    std::set<int> discovered(found.begin(), found.end());
    currently_discovered_peers_ = discovered;
    for (const int index : found)
    {
        discovery_absent_scan_count_[index] = 0;
        if (peer_subscriptions_.find(index) != peer_subscriptions_.end())
        {
            continue;
        }
        const std::string topic = "/fmu" + std::to_string(index) +
            "_agent/state";
        auto subscription = create_subscription<taiphoon_agent::msg::AgentState>(
            topic, rclcpp::QoS(10).best_effort(),
            [this, index](
                const taiphoon_agent::msg::AgentState::SharedPtr msg)
            {
                peer_state_cb(index, msg);
            });
        if (subscription != nullptr)
        {
            peer_subscriptions_[index] = subscription;
        }
    }

    for (auto iterator = peer_subscriptions_.begin();
         iterator != peer_subscriptions_.end();)
    {
        if (discovered.find(iterator->first) == discovered.end())
        {
            const int absent_scans = ++discovery_absent_scan_count_[iterator->first];
            if (absent_scans >= formation_member_stable_scan_count_)
            {
                const int removed_id = iterator->first;
                peers_.erase(removed_id);
                peer_trajectories_.erase(removed_id);
                discovery_absent_scan_count_.erase(removed_id);
                iterator = peer_subscriptions_.erase(iterator);
            }
            else
            {
                ++iterator;
            }
        }
        else
        {
            ++iterator;
        }
    }

    std::vector<int> candidate_members;
    if (has_own_state_ && is_finite_state_position(own_state_))
    {
        candidate_members.push_back(agent_index_);
    }
    const auto now = std::chrono::steady_clock::now();
    for (const int index : currently_discovered_peers_)
    {
        const auto peer_iterator = peers_.find(index);
        if (peer_iterator == peers_.end() || !peer_iterator->second.has_state ||
            !is_finite_state_position(peer_iterator->second.state))
        {
            continue;
        }
        const double age = std::chrono::duration<double>(
            now - peer_iterator->second.last_arrival).count();
        if (std::isfinite(age) && age >= 0.0 &&
            age <= peer_trajectory_timeout_s_)
        {
            candidate_members.push_back(index);
        }
    }
    std::sort(candidate_members.begin(), candidate_members.end());
    candidate_members.erase(
        std::unique(candidate_members.begin(), candidate_members.end()),
        candidate_members.end());
    if (has_accepted_formation_epoch_ && has_formation_)
    {
        if (candidate_members != accepted_formation_epoch_.member_agent_ids)
        {
            formation_membership_change_pending_ = true;
        }
        else if (!has_candidate_formation_epoch_ ||
                 epoch_is_same_snapshot(
                     candidate_formation_epoch_, accepted_formation_epoch_))
        {
            formation_membership_change_pending_ = false;
        }
    }
    if (candidate_members == discovery_candidate_members_)
    {
        ++discovery_same_scan_count_;
    }
    else
    {
        discovery_candidate_members_ = candidate_members;
        discovery_same_scan_count_ = 1;
    }
    if (discovery_same_scan_count_ >= formation_member_stable_scan_count_ &&
        candidate_members != stable_members_)
    {
        stable_members_ = candidate_members;
    }
    // 若目前穩定成員已回到 accepted epoch，先前尚未完成的不同成員
    // proposal 已經失去作用。清除它，避免後續 Agent 恢復時被誤認為
    // 已有相同候選，無法依照目前存活編隊建立新的 parent-child epoch。
    if (discovery_same_scan_count_ >= formation_member_stable_scan_count_ &&
        has_accepted_formation_epoch_ && has_candidate_formation_epoch_ &&
        candidate_members == accepted_formation_epoch_.member_agent_ids &&
        !epoch_is_same_snapshot(
            candidate_formation_epoch_, accepted_formation_epoch_))
    {
        RCLCPP_INFO(
            get_logger(),
            "Discarding stale FormationEpoch candidate after membership "
            "returned to accepted set");
        has_candidate_formation_epoch_ = false;
        candidate_adopted_agents_.clear();
    }
    // stable_members_ 可能在上一輪 discovery 已先收斂，之後才完成
    // FormationEpoch。不能只依賴 stable_members_ 再次變動，否則 Agent
    // 失效後會永遠保留舊 epoch；只要目前穩定集合仍與 accepted epoch
    // 不同，且尚未存在相同成員候選，就建立新的 parent-child epoch。
    const bool membership_requires_reconfiguration =
        has_accepted_formation_epoch_ && has_formation_ &&
        candidate_members != accepted_formation_epoch_.member_agent_ids;
    const bool same_membership_candidate = has_candidate_formation_epoch_ &&
        candidate_formation_epoch_.member_agent_ids == candidate_members;
    if (discovery_same_scan_count_ >= formation_member_stable_scan_count_ &&
        membership_requires_reconfiguration && !same_membership_candidate)
    {
        RCLCPP_INFO(
            get_logger(),
            "Stable member set differs from accepted epoch; proposing "
            "FormationEpoch reconfiguration members=%zu",
            candidate_members.size());
        (void)make_formation_proposal(candidate_members);
    }
}

bool EgoSwarmPlannerNode::update_swarm_frame()
{
    if (!has_own_state_ || !is_finite_state_position(own_state_))
    {
        return false;
    }

    if (swarm_frame_.has_origin())
    {
        return update_local_swarm_transform();
    }

    // 全球 WGS84 trajectory 不再要求所有 Agent 共用同一個 origin。每個
    // planner 只 freeze 自己的 Home，接收 peer 時再轉成本機 NED。
    const ego_swarm::GeoCoordinate origin = to_geo(own_state_);
    if (!ego_swarm::SwarmFrame::is_valid_geodetic(origin) ||
        !swarm_frame_.set_origin(origin))
    {
        return false;
    }

    frame_origin_ = origin;
    frame_origin_agent_index_ = agent_index_;
    frame_revision_ = swarm_frame_.frame_revision();
    RCLCPP_INFO(
        get_logger(),
        "Selected and froze local Home for Agent %d, local revision=%u",
        frame_origin_agent_index_, frame_revision_);
    return swarm_frame_.has_origin() && update_local_swarm_transform();
}

bool EgoSwarmPlannerNode::update_local_swarm_transform()
{
    if (has_local_swarm_transform_)
    {
        return true;
    }
    if (!has_own_state_ ||
        !ego_swarm::is_finite_vector({
            static_cast<double>(own_state_.position_ned[0]),
            static_cast<double>(own_state_.position_ned[1]),
            static_cast<double>(own_state_.position_ned[2])}))
    {
        return false;
    }

    ego_swarm::Vector3 current_swarm{};
    if (!get_position_in_swarm_frame(own_state_, current_swarm))
    {
        return false;
    }

    const ego_swarm::Vector3 current_local{
        static_cast<double>(own_state_.position_ned[0]),
        static_cast<double>(own_state_.position_ned[1]),
        static_cast<double>(own_state_.position_ned[2])};
    local_swarm_offset_ = ego_swarm::subtract_vectors(
        current_local, current_swarm);
    has_local_swarm_transform_ =
        ego_swarm::is_finite_vector(local_swarm_offset_);
    return has_local_swarm_transform_;
}

bool EgoSwarmPlannerNode::get_position_in_swarm_frame(
    const taiphoon_agent::msg::AgentState& state,
    ego_swarm::Vector3& position) const
{
    if (!is_finite_state_position(state))
    {
        position = ego_swarm::Vector3{0.0, 0.0, 0.0};
        return false;
    }
    return swarm_frame_.to_swarm_ned(to_geo(state), position);
}

bool EgoSwarmPlannerNode::get_target_in_local_ned(
    const ego_swarm::Vector3& target,
    ego_swarm::Vector3& local_target) const
{
    local_target = ego_swarm::Vector3{0.0, 0.0, 0.0};
    if (!has_local_swarm_transform_ || !ego_swarm::is_finite_vector(target))
    {
        return false;
    }
    local_target = ego_swarm::add_vectors(target, local_swarm_offset_);
    return ego_swarm::is_finite_vector(local_target);
}

ego_swarm::PositionArrivalResult EgoSwarmPlannerNode::evaluate_target_arrival(
    const ego_swarm::Vector3& current_global_ned,
    const ego_swarm::Vector3& final_target) const
{
    ego_swarm::PositionArrivalResult result;
    ego_swarm::Vector3 target_local{};
    if (!get_target_in_local_ned(final_target, target_local))
    {
        return result;
    }

    const ego_swarm::Vector3 current_local{
        static_cast<double>(own_state_.position_ned[0]),
        static_cast<double>(own_state_.position_ned[1]),
        static_cast<double>(own_state_.position_ned[2])};
    return ego_swarm::evaluate_position_arrival(
        current_local, target_local, current_global_ned, final_target,
        local_arrival_tolerance_m_, global_arrival_tolerance_m_);
}

void EgoSwarmPlannerNode::invalidate_active_plan()
{
    has_active_plan_ = false;
    emergency_brake_active_ = false;
    waiting_for_replan_ = false;
    waiting_retry_time_s_ = 0.0;
    active_plan_ = ego_swarm::PlannerResult{};
    active_plan_target_ = ego_swarm::Vector3{0.0, 0.0, 0.0};
    active_plan_final_target_ = ego_swarm::Vector3{0.0, 0.0, 0.0};
    active_plan_start_time_ = rclcpp::Time(
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME);
    has_last_replan_time_ = false;
    last_replan_time_s_ = 0.0;
    active_plan_trajectory_id_ = 0U;
    clear_terminal_hold();
}

void EgoSwarmPlannerNode::clear_terminal_hold()
{
    has_terminal_hold_ = false;
    terminal_hold_plan_ = ego_swarm::PlannerResult{};
    terminal_hold_position_ = ego_swarm::Vector3{0.0, 0.0, 0.0};
    terminal_hold_start_time_ = rclcpp::Time(
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME);
    terminal_hold_trajectory_id_ = 0U;
}

bool EgoSwarmPlannerNode::get_formation_target_for_agent(
    const int agent_id, ego_swarm::Vector3& target) const
{
    if (!has_formation_ || !has_accepted_formation_epoch_)
    {
        return false;
    }
    const auto assignment = formation_assignment_.find(agent_id);
    if (assignment == formation_assignment_.end())
    {
        return false;
    }
    std::vector<ego_swarm::Vector3> offsets;
    if (!ego_swarm::FormationSlotPlanner::compute_slot_offsets(
            accepted_formation_epoch_.member_agent_ids.size(),
            formation_config_, offsets) ||
        assignment->second >= offsets.size())
    {
        return false;
    }
    std::vector<ego_swarm::FormationSlot> slots;
    slots.reserve(offsets.size());
    for (std::size_t index = 0U; index < offsets.size(); ++index)
    {
        ego_swarm::FormationSlot geometry_slot;
        geometry_slot.rank = index;
        geometry_slot.offset_ned = offsets.at(index);
        geometry_slot.is_valid = true;
        slots.push_back(geometry_slot);
    }
    if (!ego_swarm::FormationSlotPlanner::are_slots_safe(
            slots, formation_min_distance_m_))
    {
        return false;
    }
    target = ego_swarm::add_vectors(
        formation_center_, offsets.at(assignment->second));
    return ego_swarm::is_finite_vector(target);
}

ego_swarm::FormationConvergenceResult
EgoSwarmPlannerNode::evaluate_formation_convergence(const double now_s) const
{
    ego_swarm::FormationConvergenceResult result;
    if (!has_formation_ || !has_accepted_formation_epoch_ ||
        !std::isfinite(now_s))
    {
        return result;
    }

    std::vector<ego_swarm::FormationConvergenceObservation> observations;
    observations.reserve(accepted_formation_epoch_.member_agent_ids.size());
    const auto now_steady = std::chrono::steady_clock::now();
    for (const int member_id : accepted_formation_epoch_.member_agent_ids)
    {
        ego_swarm::FormationConvergenceObservation observation;
        observation.agent_id = member_id;
        observation.has_same_epoch =
            candidate_adopted_agents_.find(member_id) !=
            candidate_adopted_agents_.end();
        if (!get_formation_target_for_agent(member_id, observation.target))
        {
            observations.push_back(observation);
            continue;
        }

        if (member_id == agent_index_)
        {
            observation.has_fresh_state =
                get_position_in_swarm_frame(own_state_, observation.position);
            observation.has_fresh_occupancy =
                has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid();
        }
        else
        {
            const auto peer_entry = peers_.find(member_id);
            if (peer_entry != peers_.end() && peer_entry->second.has_state)
            {
                const PeerState& peer = peer_entry->second;
                const double age_s = std::chrono::duration<double>(
                    now_steady - peer.last_arrival).count();
                observation.has_fresh_state = age_s >= 0.0 &&
                    age_s <= peer_trajectory_timeout_s_ &&
                    get_position_in_swarm_frame(
                        peer.state, observation.position);

                ego_swarm::PeerTrajectory peer_trajectory;
                observation.has_fresh_occupancy =
                    peer_trajectories_.get_fresh(
                        member_id, now_s, peer_trajectory) &&
                    peer_trajectory.terminal_hold;
            }
        }
        observations.push_back(observation);
    }
    return ego_swarm::FormationConvergenceEvaluator::evaluate(
        observations, global_arrival_tolerance_m_);
}

bool EgoSwarmPlannerNode::get_current_final_target(
    ego_swarm::Vector3& final_target) const
{
    if (has_formation_)
    {
        return get_formation_target_for_agent(agent_index_, final_target);
    }

    if (!has_goal_ || !ego_swarm::is_finite_vector(goal_position_))
    {
        return false;
    }
    final_target = goal_position_;
    return true;
}

ego_swarm::VehiclePlanningState EgoSwarmPlannerNode::make_planning_state(
    const ego_swarm::Vector3& position,
    const double now_s) const
{
    ego_swarm::VehiclePlanningState state;
    state.position = position;
    state.timestamp_s = now_s;
    if (has_vehicle_motion_state_ &&
        std::isfinite(vehicle_motion_state_.timestamp_s) &&
        now_s >= vehicle_motion_state_.timestamp_s &&
        now_s - vehicle_motion_state_.timestamp_s <= 1.0 &&
        ego_swarm::is_finite_vector(vehicle_motion_state_.velocity) &&
        ego_swarm::is_finite_vector(vehicle_motion_state_.acceleration))
    {
        state.velocity = vehicle_motion_state_.velocity;
        state.acceleration = vehicle_motion_state_.acceleration;
        return state;
    }

    if (has_active_plan_ && active_plan_.trajectory.is_valid())
    {
        const double elapsed_s =
            now_s - active_plan_start_time_.seconds();
        if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
            elapsed_s <= active_plan_.trajectory.duration_s())
        {
            const auto sample = active_plan_.trajectory.evaluate_state(
                elapsed_s);
            if (sample.is_valid)
            {
                state.velocity = sample.velocity;
                state.acceleration = sample.acceleration;
            }
        }
    }
    return state;
}

ego_swarm::SafetyValidationResult EgoSwarmPlannerNode::validate_plan(
    const ego_swarm::BsplineTrajectory& trajectory,
    const ego_swarm::VehiclePlanningState& state,
    const double now_s,
    const double trajectory_start_time_s,
    const std::vector<ego_swarm::PeerTrajectory>* peer_snapshot) const
{
    ego_swarm::SafetyValidationContext context;
    context.self_state = state;
    context.now_s = now_s;
    context.trajectory_start_time_s = trajectory_start_time_s;
    context.max_time_skew_s = max_peer_time_skew_s_;
    context.max_future_start_delay_s = reservation_lead_time_s_ +
        reservation_horizon_s_ + 1.0;
    context.frame_revision = frame_revision_;
    context.trajectory_frame_revision = frame_revision_;
    context.self_agent_id = agent_index_;
    ego_swarm::PeerTrajectoryTable snapshot_table;
    if (peer_snapshot != nullptr)
    {
        (void)snapshot_table.set_config(
            ego_swarm::PeerTrajectoryTableConfig{
                peer_trajectory_timeout_s_, max_peer_time_skew_s_,
                reservation_lead_time_s_ + reservation_horizon_s_ + 1.0});
        for (const auto& peer : *peer_snapshot)
        {
            (void)snapshot_table.update(peer);
        }
        context.peer_table = &snapshot_table;
    }
    else
    {
        context.peer_table = &peer_trajectories_;
    }
    context.obstacle_map = &obstacle_map_;
    return safety_validator_.validate(trajectory, context);
}

bool EgoSwarmPlannerNode::publish_emergency_brake(
    const ego_swarm::VehiclePlanningState& state,
    const ego_swarm::Vector3& active_target,
    const std::string& reason,
    const rclcpp::Time& now,
    const bool peer_data_valid)
{
    // Emergency brake 代表目前的 terminal hold 已不再描述實際控制
    // 狀態；若保留舊 hold，後續 heartbeat 可能帶著較小 trajectory ID
    // 覆寫/回退 PeerTrajectoryTable，讓其他 Agent 永遠等不到新的
    // predecessor 狀態。煞停完成並重新到位後會以新 ID 建立 hold。
    if (emergency_brake_active_ && has_active_plan_ &&
        active_plan_.trajectory.is_valid())
    {
        const double elapsed_s = (now - active_plan_start_time_).seconds();
        if (std::isfinite(elapsed_s) && elapsed_s >= 0.0 &&
            elapsed_s <= active_plan_.trajectory.duration_s() +
                TRAJECTORY_DURATION_ABSOLUTE_EPSILON_S)
        {
            publish_plan(
                active_plan_, state.position, now, active_plan_start_time_,
                active_plan_trajectory_id_);
            publish_status(
                PLANNER_STATE_HOLD, "Emergency brake in progress",
                &active_plan_, true, peer_data_valid, now);
            return true;
        }
        emergency_brake_active_ = false;
        const double speed_mps = std::sqrt(
            ego_swarm::squared_norm(state.velocity));
        if (std::isfinite(speed_mps) && speed_mps <= 0.05)
        {
            return publish_waiting_hold(
                state.position, active_target,
                "optimized_no_feasible_horizontal_path; brake complete",
                now, peer_data_valid);
        }
    }

    clear_terminal_hold();
    const bool fixed_z_path = preserve_altitude_ ||
        (has_formation_ && has_accepted_formation_epoch_ &&
         formation_config_.type != ego_swarm::FormationType::VERT_LINEAR);
    const ego_swarm::PathConstraint brake_path_constraint = fixed_z_path
        ? ego_swarm::PathConstraint::FIXED_Z
        : ego_swarm::PathConstraint::FREE_3D;
    const auto brake_result = emergency_brake_planner_.plan(
        state, brake_path_constraint);
    if (!brake_result.is_valid || !brake_result.has_finite_output())
    {
        RCLCPP_ERROR(
            get_logger(),
            "Emergency brake generation failed; retaining last finite setpoint");
        publish_status(
            PLANNER_STATE_ERROR, "Emergency brake generation failed", nullptr,
            true, peer_data_valid, now);
        return false;
    }

    active_plan_ = brake_result;
    active_plan_target_ = ego_swarm::is_finite_vector(active_target)
        ? active_target : state.position;
    active_plan_final_target_ = active_plan_target_;
    active_plan_start_time_ = now;
    has_last_replan_time_ = false;
    last_replan_time_s_ = 0.0;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    emergency_brake_active_ = true;
    waiting_for_replan_ = false;
    waiting_retry_time_s_ = 0.0;
    publish_plan(
        active_plan_, state.position, now, active_plan_start_time_,
        active_plan_trajectory_id_);
    const std::string status_reason = reason.rfind(
        "optimized_emergency_brake", 0U) == 0U
        ? reason : "Emergency brake: " + reason;
    publish_status(
        PLANNER_STATE_HOLD, status_reason, nullptr, true,
        peer_data_valid, now);
    return true;
}

bool EgoSwarmPlannerNode::publish_waiting_hold(
    const ego_swarm::Vector3& current,
    const ego_swarm::Vector3& target,
    const std::string& reason,
    const rclcpp::Time& now,
    const bool peer_data_valid)
{
    if (!ego_swarm::is_finite_vector(current) ||
        !ego_swarm::is_finite_vector(target) || !std::isfinite(now.seconds()))
    {
        return false;
    }
    const auto hold_result = ego_swarm::make_exact_hold_result(
        current, planner_config_);
    if (!hold_result.is_valid || !hold_result.has_finite_output())
    {
        RCLCPP_ERROR(get_logger(), "Failed to create local waiting hold");
        return false;
    }

    clear_terminal_hold();
    active_plan_ = hold_result;
    active_plan_target_ = target;
    active_plan_final_target_ = target;
    active_plan_start_time_ = now;
    has_last_replan_time_ = false;
    last_replan_time_s_ = 0.0;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    emergency_brake_active_ = false;
    waiting_for_replan_ = true;
    waiting_retry_time_s_ = now.seconds() + PEER_REPLAN_RETRY_DELAY_S;
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_, true);
    publish_status(
        PLANNER_STATE_HOLD, reason, nullptr, true, peer_data_valid, now);
    return true;
}

void EgoSwarmPlannerNode::set_goal_from_command(const std::string& command)
{
    const auto parsed = utils::parse_command_string(command);
    if (parsed.command == utils::AgentCommand::FORMATION_CANCEL)
    {
        ego_swarm::Vector3 preserved_target{};
        if (has_formation_ && get_formation_target_for_agent(
                agent_index_, preserved_target))
        {
            goal_position_ = preserved_target;
            has_goal_ = true;
            has_position_target_ = true;
        }
        has_formation_ = false;
        formation_initial_plan_pending_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        return;
    }

    if (parsed.command == utils::AgentCommand::YAW)
    {
        if (!parsed.has_param || !std::isfinite(parsed.param_value))
        {
            return;
        }
        double base_yaw_rad = 0.0;
        if (has_target_yaw_)
        {
            base_yaw_rad = target_yaw_rad_;
        }
        else if (!attitude_is_fresh() ||
                 !ego_swarm::quaternion_to_ned_yaw(
                     own_attitude_.q, base_yaw_rad))
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting YAW: no fresh attitude is available to initialize "
                "the target yaw");
            return;
        }
        double next_yaw_rad = 0.0;
        if (!normalize_yaw_rad(
                base_yaw_rad + parsed.param_value *
                    ego_swarm::DEGREES_TO_RADIANS,
                next_yaw_rad))
        {
            return;
        }
        target_yaw_rad_ = next_yaw_rad;
        has_target_yaw_ = true;
        return;
    }

    if (!has_own_state_ || !update_swarm_frame())
    {
        RCLCPP_WARN(get_logger(),
                    "Mission intent deferred until local WGS84 Home is ready");
        return;
    }

    ego_swarm::Vector3 current{};
    if (!get_position_in_swarm_frame(own_state_, current))
    {
        return;
    }

    if (parsed.command == utils::AgentCommand::MOVE_POINT &&
        parsed.has_multi_params)
    {
        goal_position_ = ego_swarm::Vector3{
            parsed.param_value, parsed.param2, parsed.param3};
        has_goal_ = ego_swarm::is_finite_vector(goal_position_);
        has_position_target_ = has_goal_;
        has_formation_ = false;
        formation_initial_plan_pending_ = false;
        preserve_altitude_ = false;
        if (std::isfinite(parsed.param4))
        {
            double absolute_yaw_rad = 0.0;
            if (normalize_yaw_rad(
                    parsed.param4 * ego_swarm::DEGREES_TO_RADIANS,
                    absolute_yaw_rad))
            {
                target_yaw_rad_ = absolute_yaw_rad;
                has_target_yaw_ = true;
            }
        }
        invalidate_active_plan();
        return;
    }

    if (!parsed.has_param || !std::isfinite(parsed.param_value))
    {
        return;
    }
    ego_swarm::Vector3 command_base = current;
    if (has_formation_)
    {
        if (!get_formation_target_for_agent(agent_index_, command_base))
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting relative MOVE: current formation target is invalid");
            return;
        }
    }
    else if (has_position_target_ &&
             ego_swarm::is_finite_vector(goal_position_))
    {
        command_base = goal_position_;
    }
    goal_position_ = command_base;
    bool preserve_altitude = false;
    double forward_m = 0.0;
    double right_m = 0.0;
    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
            forward_m = parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_BACKWARD:
            forward_m = -parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_LEFT:
            right_m = -parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_RIGHT:
            right_m = parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_UP:
            goal_position_.at(2) -= parsed.param_value;
            break;
        case utils::AgentCommand::MOVE_DOWN:
            goal_position_.at(2) += parsed.param_value;
            break;
        default:
            return;
    }
    if (preserve_altitude)
    {
        if (!has_target_yaw_)
        {
            if (!attitude_is_fresh() ||
                !ego_swarm::quaternion_to_ned_yaw(
                    own_attitude_.q, target_yaw_rad_))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting horizontal MOVE: target yaw cannot be initialized "
                    "from a fresh attitude");
                return;
            }
            has_target_yaw_ = true;
        }
        ego_swarm::Vector3 body_offset{};
        if (!ego_swarm::body_horizontal_offset_to_ned(
                target_yaw_rad_, forward_m, right_m, body_offset))
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting horizontal MOVE: target yaw cannot produce a "
                "finite body-to-NED transform");
            return;
        }
        // 位移以保存的 target yaw 計算一次；後續實際 yaw 變化不會
        // 旋轉既有目標，也不會改變下一個相對命令的累積基準。
        goal_position_ = ego_swarm::add_vectors(goal_position_, body_offset);
    }
    has_goal_ = ego_swarm::is_finite_vector(goal_position_);
    has_position_target_ = has_goal_;
    has_formation_ = false;
    formation_initial_plan_pending_ = false;
    preserve_altitude_ = preserve_altitude;
    invalidate_active_plan();
}

bool EgoSwarmPlannerNode::attitude_is_fresh() const
{
    double yaw_rad = 0.0;
    if (!has_own_attitude_ ||
        !ego_swarm::quaternion_to_ned_yaw(own_attitude_.q, yaw_rad))
    {
        return false;
    }
    const auto age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - attitude_last_arrival_).count();
    return std::isfinite(age) && age >= 0.0 &&
        age <= move_attitude_timeout_s_;
}

void EgoSwarmPlannerNode::publish_plan(
    const ego_swarm::PlannerResult& result,
    const ego_swarm::Vector3& origin_position,
    const rclcpp::Time& now,
    const rclcpp::Time& start_time,
    const uint32_t trajectory_id,
    const bool terminal_hold,
    const TrajectoryPublishRoute route)
{
    if (trajectory_id == 0U || trajectory_id < last_published_trajectory_id_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Suppressing non-monotonic trajectory id=%u; last published=%u",
            trajectory_id, last_published_trajectory_id_);
        return;
    }
    if (!result.has_finite_output())
    {
        return;
    }
    const auto& trajectory = result.trajectory;
    if (!trajectory.is_valid())
    {
        return;
    }

    taiphoon_agent::msg::AgentTrajectory message;
    message.stamp = now;
    message.start_time = start_time;
    message.trajectory_id = trajectory_id;
    message.agent_index = static_cast<uint32_t>(agent_index_);
    message.coordinate_frame =
        taiphoon_agent::msg::AgentTrajectory::
            COORDINATE_FRAME_WGS84_GEODETIC;
    message.frame_revision = ego_swarm::GLOBAL_WGS84_FRAME_REVISION;
    message.order = static_cast<uint8_t>(ego_swarm::BSPLINE_DEGREE);
    message.knot_interval_s = static_cast<float>(
        trajectory.knot_interval_s());
    message.duration_s = static_cast<float>(trajectory.duration_s());
    message.terminal_hold = terminal_hold;
    message.valid = true;
    message.control_points.reserve(trajectory.control_points().size() * 3U);
    for (const auto& point : trajectory.control_points())
    {
        ego_swarm::GeoCoordinate global_point{};
        if (!swarm_frame_.from_swarm_ned(point, global_point))
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot publish trajectory id=%u: NED to WGS84 failed",
                trajectory_id);
            return;
        }
        message.control_points.push_back(global_point.latitude_deg);
        message.control_points.push_back(global_point.longitude_deg);
        message.control_points.push_back(global_point.altitude_m);
    }
    (void)origin_position;
    trajectory_pub_->publish(message);
    if (route == TrajectoryPublishRoute::PEER_AND_LOCAL)
    {
        local_trajectory_pub_->publish(message);
    }
    last_published_trajectory_id_ = trajectory_id;
}

std::string EgoSwarmPlannerNode::obstacle_map_status_reason(
    const double now_s) const
{
    const auto status = obstacle_map_.status(now_s);
    if (require_obstacle_cloud_)
    {
        switch (status)
        {
            case perception::ObstacleMapStatus::Missing:
                return "Required obstacle map is missing";
            case perception::ObstacleMapStatus::Stale:
                return "Required obstacle map is stale";
            case perception::ObstacleMapStatus::Invalid:
                return "Required obstacle map is invalid";
            case perception::ObstacleMapStatus::Fresh:
                return "ok";
        }
    }
    else
    {
        switch (status)
        {
            case perception::ObstacleMapStatus::Missing:
                return "Obstacle map missing; waiting for explicit_empty source";
            case perception::ObstacleMapStatus::Stale:
                return "Obstacle map stale; waiting for a fresh source";
            case perception::ObstacleMapStatus::Invalid:
                return "Obstacle map invalid; waiting for a valid source";
            case perception::ObstacleMapStatus::Fresh:
                return "ok";
        }
    }
    return "Obstacle map status unavailable";
}

void EgoSwarmPlannerNode::publish_status(
    const uint8_t state,
    const std::string& reason,
    const ego_swarm::PlannerResult* result,
    const bool localization_valid,
    const bool peer_data_valid,
    const rclcpp::Time& stamp)
{
    if (status_pub_ == nullptr)
    {
        return;
    }
    taiphoon_agent::msg::PlannerStatus status;
    status.stamp = stamp;
    status.state = state;
    status.trajectory_id = trajectory_id_;
    status.map_valid = obstacle_map_.is_fresh(stamp.seconds());
    status.localization_valid = localization_valid;
    status.peer_data_valid = peer_data_valid;
    status.planning_time_ms = result == nullptr
        ? 0.0F : static_cast<float>(result->planning_time_ms);
    status.min_obstacle_distance_m = result == nullptr
        ? 0.0F : static_cast<float>(result->minimum_obstacle_distance_m);
    double minimum_peer_distance_m = result == nullptr
        ? 0.0 : result->minimum_peer_distance_m;
    if (result == nullptr)
    {
        ego_swarm::Vector3 current{};
        if (get_position_in_swarm_frame(own_state_, current))
        {
            const auto live_peer_distance = live_peer_min_distance(current);
            if (live_peer_distance.has_peer && live_peer_distance.data_valid)
            {
                minimum_peer_distance_m =
                    live_peer_distance.minimum_distance_m;
            }
        }
    }
    status.min_peer_distance_m = static_cast<float>(minimum_peer_distance_m);
    status.reason = reason;
    status_pub_->publish(status);
}

int EgoSwarmPlannerNode::compute_self_index() const
{
    const std::size_t underscore = agent_namespace_.find('_');
    if (underscore == std::string::npos ||
        agent_namespace_.substr(0, 3) != "fmu")
    {
        return 1;
    }
    try
    {
        return std::max(1, std::stoi(agent_namespace_.substr(
            3, underscore - 3)));
    }
    catch (const std::exception& error)
    {
        RCLCPP_WARN(get_logger(), "Cannot parse agent index: %s",
                    error.what());
        return 1;
    }
}

}  // namespace nodes
}  // namespace taiphoon
