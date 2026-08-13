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
    declare_parameter<double>("obstacle_clearance_m",
                             DEFAULT_OBSTACLE_CLEARANCE_M);
    declare_parameter<double>("knot_interval_s", 1.0);
    declare_parameter<double>("max_velocity_mps", DEFAULT_MAX_VELOCITY_MPS);
    declare_parameter<double>("max_acceleration_mps2",
                             DEFAULT_MAX_ACCELERATION_MPS2);
    declare_parameter<double>("max_jerk_mps3", DEFAULT_MAX_JERK_MPS3);
    // 保留 Phase 3 參數名稱，既有 compose / launch 不需同步切換。
    declare_parameter<double>("max_velocity_m_s", 2.0);
    declare_parameter<double>("max_acceleration_m_s2", 1.0);
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
    if (!planner_.set_config(planner_config_))
    {
        RCLCPP_ERROR(get_logger(), "EGO-Swarm planner parameters are invalid");
        return;
    }
    planner_config_ = planner_.config();
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
                peer_trajectory_timeout_s_, max_peer_time_skew_s_}))
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
                "compat_origin_agent=%d peer_distance=%.2f m "
                "obstacle_clearance=%.2f m peer_timeout=%.2f s "
                "max_peer_time_skew=%.2f s obstacle_timeout=%.2f s "
                "require_obstacle_cloud=%s move_attitude_timeout=%.2f s "
                "slot_switch_penalty=%.2f m member_stable_scans=%d",
                agent_index_, agent_namespace_.c_str(), planning_rate_hz_,
                swarm_origin_agent_index_,
                planner_config_.peer_collision_min_distance_m,
                planner_config_.obstacle_clearance_m,
                peer_trajectory_timeout_s_, max_peer_time_skew_s_,
                obstacle_cloud_timeout_s_,
                require_obstacle_cloud_ ? "true" : "false",
                move_attitude_timeout_s_, formation_slot_switch_penalty_m_,
                formation_member_stable_scan_count_);
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
        case utils::AgentCommand::FORMATION_CANCEL:
            set_goal_from_command(msg->data);
            break;
        default:
            // MODE_*, ARM, TAKEOFF, LAND 與 YAW 由 BasicController 處理。
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
        has_formation_ = false;
        has_accepted_formation_epoch_ = false;
        has_candidate_formation_epoch_ = false;
        formation_command_pending_ = false;
        formation_membership_change_pending_ = false;
        candidate_adopted_agents_.clear();
        formation_assignment_.clear();
        preserve_altitude_ = false;
        invalidate_active_plan();
        return;
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
    const std::vector<int> members = stable_member_ids();
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

bool EgoSwarmPlannerNode::formation_config_matches(
    const ego_swarm::FormationConfig& first,
    const ego_swarm::FormationConfig& second) const
{
    return first.type == second.type &&
        std::abs(first.radius_m - second.radius_m) <= 1.0e-9 &&
        std::abs(first.spacing_m - second.spacing_m) <= 1.0e-9 &&
        std::abs(first.vertical_offset_m - second.vertical_offset_m) <= 1.0e-9;
}

bool EgoSwarmPlannerNode::formation_transition_order(
    std::vector<int>& out_order) const
{
    out_order.clear();
    if (!has_accepted_formation_epoch_ || !swarm_frame_.has_origin())
    {
        return false;
    }
    std::vector<ego_swarm::Vector3> offsets;
    if (!ego_swarm::FormationSlotPlanner::compute_slot_offsets(
            accepted_formation_epoch_.member_agent_ids.size(),
            formation_config_, offsets) ||
        accepted_formation_epoch_.member_agent_ids.size() !=
            accepted_formation_epoch_.member_latitudes_deg.size() ||
        accepted_formation_epoch_.member_agent_ids.size() !=
            accepted_formation_epoch_.member_longitudes_deg.size() ||
        accepted_formation_epoch_.member_agent_ids.size() !=
            accepted_formation_epoch_.member_altitudes_m.size())
    {
        return false;
    }

    std::vector<ego_swarm::FormationTransitionObservation> observations;
    observations.reserve(accepted_formation_epoch_.member_agent_ids.size());
    for (std::size_t index = 0U;
         index < accepted_formation_epoch_.member_agent_ids.size(); ++index)
    {
        const int agent_id = accepted_formation_epoch_.member_agent_ids.at(index);
        const auto assignment = formation_assignment_.find(agent_id);
        if (assignment == formation_assignment_.end() ||
            assignment->second >= offsets.size())
        {
            return false;
        }
        ego_swarm::Vector3 position{};
        if (!swarm_frame_.to_swarm_ned(ego_swarm::GeoCoordinate{
                accepted_formation_epoch_.member_latitudes_deg.at(index),
                accepted_formation_epoch_.member_longitudes_deg.at(index),
                accepted_formation_epoch_.member_altitudes_m.at(index)},
                position))
        {
            return false;
        }
        const ego_swarm::Vector3 target = ego_swarm::add_vectors(
            formation_center_, offsets.at(assignment->second));
        observations.push_back(ego_swarm::FormationTransitionObservation{
            agent_id, position, target, ego_swarm::Vector3{0.0, 0.0, 0.0},
            true});
    }

    const double minimum_distance_m = std::max(
        formation_min_distance_m_, planner_config_.peer_collision_min_distance_m);
    return ego_swarm::FormationTransition::compute_safe_order(
        accepted_formation_epoch_.member_agent_ids, observations,
        minimum_distance_m, planner_config_.candidate_offset_m, out_order);
}

bool EgoSwarmPlannerNode::formation_transition_ready() const
{
    if (!has_formation_ || !has_accepted_formation_epoch_ ||
        !has_own_state_ || !swarm_frame_.has_origin())
    {
        return false;
    }

    std::vector<int> order;
    if (!formation_transition_order(order))
    {
        return false;
    }
    const auto self_iterator = std::find(order.begin(), order.end(), agent_index_);
    if (self_iterator == order.end())
    {
        return false;
    }

    std::vector<ego_swarm::Vector3> offsets;
    if (!ego_swarm::FormationSlotPlanner::compute_slot_offsets(
            accepted_formation_epoch_.member_agent_ids.size(),
            formation_config_, offsets))
    {
        return false;
    }
    const double now_s = this->now().seconds();
    if (!std::isfinite(now_s))
    {
        return false;
    }

    for (auto iterator = order.begin(); iterator != self_iterator; ++iterator)
    {
        const int predecessor_id = *iterator;
        const auto assignment = formation_assignment_.find(predecessor_id);
        const auto member_iterator = std::find(
            accepted_formation_epoch_.member_agent_ids.begin(),
            accepted_formation_epoch_.member_agent_ids.end(), predecessor_id);
        const auto peer = peers_.find(predecessor_id);
        if (assignment == formation_assignment_.end() ||
            assignment->second >= offsets.size() ||
            member_iterator == accepted_formation_epoch_.member_agent_ids.end() ||
            peer == peers_.end() || !peer->second.has_state ||
            !is_finite_state_position(peer->second.state))
        {
            return false;
        }

        ego_swarm::Vector3 predecessor_position{};
        if (!get_position_in_swarm_frame(
                peer->second.state, predecessor_position))
        {
            return false;
        }
        const ego_swarm::Vector3 predecessor_target =
            ego_swarm::add_vectors(
                formation_center_, offsets.at(assignment->second));
        ego_swarm::PeerTrajectory predecessor_trajectory{};
        if (!peer_trajectories_.get_fresh(
                predecessor_id, now_s, predecessor_trajectory))
        {
            return false;
        }
        const double trajectory_time_s = now_s -
            predecessor_trajectory.start_time_s;
        const ego_swarm::TrajectorySample sample =
            predecessor_trajectory.trajectory.evaluate(trajectory_time_s);
        if (!sample.is_valid ||
            !predecessor_trajectory.terminal_hold ||
            ego_swarm::distance_between_centers(
                predecessor_position, predecessor_target) >
                global_arrival_tolerance_m_ ||
            std::sqrt(ego_swarm::squared_norm(sample.velocity)) > 0.2)
        {
            return false;
        }
    }
    return true;
}

double EgoSwarmPlannerNode::live_peer_min_distance(
    const ego_swarm::Vector3& current) const
{
    if (!ego_swarm::is_finite_vector(current))
    {
        return 0.0;
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
    for (const int peer_id : peer_ids)
    {
        if (peer_id == agent_index_)
        {
            continue;
        }
        const auto peer_iterator = peers_.find(peer_id);
        if (peer_iterator == peers_.end() ||
            !peer_iterator->second.has_state ||
            !is_finite_state_position(peer_iterator->second.state))
        {
            return 0.0;
        }
        ego_swarm::Vector3 peer_position{};
        if (!get_position_in_swarm_frame(
                peer_iterator->second.state, peer_position))
        {
            return 0.0;
        }
        const double distance = ego_swarm::distance_between_centers(
            current, peer_position);
        if (!std::isfinite(distance))
        {
            return 0.0;
        }
        minimum_distance = std::min(minimum_distance, distance);
    }
    return minimum_distance;
}

double EgoSwarmPlannerNode::live_peer_path_min_distance(
    const ego_swarm::Vector3& current,
    const ego_swarm::Vector3& target) const
{
    if (!ego_swarm::is_finite_vector(current) ||
        !ego_swarm::is_finite_vector(target))
    {
        return 0.0;
    }

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

    double minimum_distance = std::numeric_limits<double>::infinity();
    for (const int peer_id : peer_ids)
    {
        if (peer_id == agent_index_)
        {
            continue;
        }
        const auto peer_iterator = peers_.find(peer_id);
        if (peer_iterator == peers_.end() ||
            !peer_iterator->second.has_state ||
            !is_finite_state_position(peer_iterator->second.state))
        {
            return 0.0;
        }
        ego_swarm::Vector3 peer_position{};
        if (!get_position_in_swarm_frame(
                peer_iterator->second.state, peer_position))
        {
            return 0.0;
        }
        const double distance = ego_swarm::distance_to_segment(
            peer_position, current, target);
        if (!std::isfinite(distance))
        {
            return 0.0;
        }
        minimum_distance = std::min(minimum_distance, distance);
    }
    return minimum_distance;
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
    const std::vector<int>& member_ids)
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
        // Membership 變更只換槽位與 ownership，中心沿用 parent center。
        proposal.center_latitude_deg =
            accepted_formation_epoch_.center_latitude_deg;
        proposal.center_longitude_deg =
            accepted_formation_epoch_.center_longitude_deg;
        proposal.center_altitude_m = accepted_formation_epoch_.center_altitude_m;
        for (const auto& entry : formation_assignment_)
        {
            proposal.parent_assignment_agent_ids.push_back(entry.first);
            proposal.parent_assignment_slot_indices.push_back(
                static_cast<int32_t>(entry.second));
        }
    }

    double latitude_sum = 0.0;
    double longitude_sum = 0.0;
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
    }
    if (!has_accepted_formation_epoch_)
    {
        const double count = static_cast<double>(members.size());
        proposal.center_latitude_deg = latitude_sum / count;
        proposal.center_longitude_deg = longitude_sum / count;
        // CIRCLE/HORI 的高度參數是既有 NED 命令語意：相對本機 Home 的
        // 絕對飛行高度。center altitude 因此使用 proposer 的 frozen Home，
        // 不能使用成員目前平均高度，否則起飛高度誤差會被再次寫入目標。
        proposal.center_altitude_m = swarm_frame_.origin().altitude_m;
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
            formation_slot_switch_penalty_m_, assignment) ||
        assignment.agent_to_slot.find(agent_index_) ==
            assignment.agent_to_slot.end())
    {
        return false;
    }
    formation_assignment_ = assignment.agent_to_slot;
    formation_center_ = center;
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
    expected_peer_ids.reserve(peers_.size());
    for (const auto& entry : peers_)
    {
        expected_peer_ids.push_back(entry.first);
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
                terminal_hold_plan_ = planner_.make_exact_hold_result(
                    terminal_hold_position_);
                terminal_hold_start_time_ = now;
                terminal_hold_trajectory_id_ = ++trajectory_id_;
                has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
            }
            else
            {
                invalidate_active_plan();
            }
        }

        if (has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid())
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true);
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
        ego_swarm::Vector3 current{};
        if (!has_terminal_hold_ &&
            get_position_in_swarm_frame(own_state_, current))
        {
            terminal_hold_position_ = current;
            terminal_hold_plan_ = planner_.make_exact_hold_result(current);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
        }
        if (has_terminal_hold_ && terminal_hold_plan_.trajectory.is_valid())
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true);
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

    const ego_swarm::VehiclePlanningState planning_state =
        make_planning_state(current, now_s);

    // Peer trajectory 是預測資料；實際飛機可能因 PX4 estimator 或控制
    // 延遲偏離宣告路徑，因此每個 planner tick 仍必須檢查 live WGS84
    // 位置。低於硬性碰撞距離立即煞停；進入額外 1 m 警戒帶且本機正
    // 執行移動軌跡時提前煞停，保留飛控反應時間。
    const double live_peer_distance = live_peer_min_distance(current);
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
    // EmergencyBrakePlanner 的結果標記為 is_hold，但它仍可能包含一段
    // 非零速度的減速軌跡。煞車期間只沿用同一條時間軸，不能在每個
    // planner tick 又被 live warning 重新建立，否則 trajectory ID 會
    // 無限增加，飛機反而永遠停在 warning 狀態。只有一般移動軌跡才
    // 需要由 live peer warning 觸發新的 emergency brake。
    const bool active_motion = has_active_plan_ &&
        active_plan_.trajectory.is_valid() && !active_plan_.is_hold &&
        !has_terminal_hold_;
    if (!std::isfinite(live_peer_distance) ||
        live_peer_distance < collision_distance ||
        (active_motion && live_peer_distance < live_warning_distance))
    {
        const std::string reason = live_peer_distance < collision_distance
            ? "live peer position inside collision margin"
            : "live peer proximity warning";
        publish_emergency_brake(
            planning_state, current, reason, now, peer_data_valid);
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
            terminal_hold_plan_ = planner_.make_exact_hold_result(current);
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

    if (has_formation_ && !formation_transition_ready())
    {
        // 編隊轉換只釋放一台已由 canonical snapshot 判定安全的 Agent；
        // 其他 Agent 固定在目前位置。這避免多條直線候選同時穿越舊隊形，
        // 但順序不是硬編碼，而是由 FormationTransition 依幾何安全性與
        // Agent ID tie-break deterministic 產生。
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
            terminal_hold_plan_ = planner_.make_exact_hold_result(current);
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
                "Formation transition waiting for safe predecessor",
                &terminal_hold_plan_, true, peer_data_valid, now);
        }
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    ego_swarm::Vector3 target{};
    if (!get_current_target(target))
    {
        publish_emergency_brake(
            planning_state, current, "target unavailable", now,
            peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    if (has_formation_)
    {
        // safety validator 以同儕 trajectory 做時間同步檢查；這裡再用
        // 最新實體位置檢查 current -> target 的直接走廊。若 snapshot
        // 已因估測漂移而過時，不得仍依舊 snapshot 釋放一條可能穿過
        // 現場機體的路徑。
        const double live_path_distance = live_peer_path_min_distance(
            current, target);
        if (!std::isfinite(live_path_distance) ||
            live_path_distance < collision_distance)
        {
            publish_emergency_brake(
                planning_state, target,
                "formation live path is occupied", now, peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
    }

    const auto arrival = evaluate_target_arrival(current, target);
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
        // target terminal hold 判斷 transition readiness；同時避免每個
        // planner tick 重建相同的 hold trajectory。
        const bool formation_target_hold_active = has_formation_ &&
            has_terminal_hold_ &&
            ego_swarm::distance_between_centers(
                terminal_hold_position_, target) <=
            global_arrival_tolerance_m_;
        if (formation_target_hold_active)
        {
            publish_plan(
                terminal_hold_plan_, terminal_hold_position_, now,
                terminal_hold_start_time_, terminal_hold_trajectory_id_, true);
            publish_status(
                PLANNER_STATE_HOLD,
                "Terminal hold; formation slot occupied",
                &terminal_hold_plan_, true, peer_data_valid, now);
            (void)peer_trajectories_.prune(now_s);
            return;
        }

        has_goal_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        // 編隊 transition_ready() 以 0.2 m/s 判斷前序 Agent 已穩定
        // 佔位。若到達分支仍使用 0.05 m/s，飛控在槽位附近的正常
        // 殘餘速度會讓本機永遠進入 brake/replan，無法發布 terminal
        // hold，後續 Agent 也就永遠不會被釋放。一般 MOVE 仍保留
        // 原本 0.05 m/s 的較嚴格停止條件。
        const double formation_settle_speed_mps = 0.2;
        const double settle_speed_mps = has_formation_
            ? formation_settle_speed_mps : 0.05;
        if (std::isfinite(speed_mps) && speed_mps > settle_speed_mps)
        {
            publish_emergency_brake(
                planning_state, target, "target reached while moving", now,
                peer_data_valid);
        }
        else
        {
            // 到達判斷允許位置誤差，因此 hold 必須鎖定目標座標，而不是
            // 使用仍可能漂移的即時位置。
            terminal_hold_position_ = target;
            terminal_hold_plan_ = planner_.make_exact_hold_result(target);
            terminal_hold_start_time_ = now;
            terminal_hold_trajectory_id_ = ++trajectory_id_;
            has_terminal_hold_ = terminal_hold_plan_.trajectory.is_valid();
            if (has_terminal_hold_)
            {
                publish_plan(
                    terminal_hold_plan_, target, now,
                    terminal_hold_start_time_, terminal_hold_trajectory_id_,
                    true);
            }
            publish_status(
                PLANNER_STATE_HOLD,
                "Terminal hold; peer occupancy heartbeat active",
                has_terminal_hold_ ? &terminal_hold_plan_ : nullptr,
                true, peer_data_valid, now);
        }
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    if (!peer_data_valid)
    {
        publish_emergency_brake(
            planning_state, target, "peer trajectory stale or invalid", now,
            false);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    const auto obstacle_status = obstacle_map_.status(now_s);
    if (obstacle_status != perception::ObstacleMapStatus::Fresh)
    {
        const std::string reason = obstacle_map_status_reason(now_s);
        publish_emergency_brake(
            planning_state, target, reason, now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    // 編隊 transition 只能使用直線或 North/East 側向候選；不允許 planner
    // 以額外的 above/below candidate 繞過同儕。VERT_LINEAR 的目標 Z 仍可
    // 沿目標方向調整，但不會再增加垂直避碰偏移。
    const bool preserve_altitude = has_formation_ || preserve_altitude_;
    if (preserve_altitude)
    {
        // 水平移動只改變 X/Y；使用命令建立時的目標 Z 作為整條
        // trajectory 的固定高度，避免 GPS callback 延遲把水平命令
        // 轉成非預期的垂直修正。
        current.at(2) = target.at(2);
    }

    const bool target_unchanged = has_active_plan_ &&
        ego_swarm::distance_between_centers(
            active_plan_target_, target) <= 1.0e-3;
    const double active_elapsed_s = has_active_plan_
        ? (now - active_plan_start_time_).seconds() : 0.0;
    const bool active_plan_running = has_active_plan_ && target_unchanged &&
        active_plan_.trajectory.is_valid() &&
        std::isfinite(active_elapsed_s) && active_elapsed_s <=
            active_plan_.trajectory.duration_s();
    if (active_plan_running)
    {
        const auto safety_result = validate_plan(
            active_plan_.trajectory, planning_state, now_s,
            active_plan_start_time_.seconds());
        if (!safety_result.is_safe)
        {
            publish_emergency_brake(
                planning_state, target,
                ego_swarm::ContinuousSafetyValidator::reason_string(
                    safety_result.reason),
                now, peer_data_valid);
            (void)peer_trajectories_.prune(now_s);
            return;
        }
        // 重新發布同一條軌跡時保留原始 start_time 與 trajectory_id，讓
        // BasicController 依原本的時間軸持續取樣，不會每 100 ms 重置進度。
        publish_plan(
            active_plan_, current, now, active_plan_start_time_,
            active_plan_trajectory_id_);
        publish_status(
            active_plan_.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
            active_plan_.is_hold ? "Holding until replanning is safe" : "ok",
            &active_plan_, true, peer_data_valid, now);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    const auto result = planner_.plan(
        current, target, obstacle_map_, peer_trajectories_,
        now_s, agent_index_, preserve_altitude);
    if (!result.is_valid || !result.trajectory.is_valid())
    {
        publish_emergency_brake(
            planning_state, target, "planner failure", now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    const auto safety_result = validate_plan(
        result.trajectory, planning_state, now_s, now_s);
    if (!safety_result.is_safe)
    {
        publish_emergency_brake(
            planning_state, target,
            ego_swarm::ContinuousSafetyValidator::reason_string(
                safety_result.reason),
            now, peer_data_valid);
        (void)peer_trajectories_.prune(now_s);
        return;
    }

    clear_terminal_hold();
    active_plan_ = result;
    active_plan_target_ = target;
    active_plan_start_time_ = now;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_);
    publish_status(
        active_plan_.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
        active_plan_.is_hold ? "Holding until replanning is safe" : "ok",
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
        const std::vector<int> previous_members = stable_members_;
        stable_members_ = candidate_members;
        if (has_accepted_formation_epoch_ && has_formation_ &&
            candidate_members != accepted_formation_epoch_.member_agent_ids &&
            candidate_members != previous_members)
        {
            RCLCPP_INFO(
                get_logger(),
                "Stable member set changed; proposing FormationEpoch reconfiguration "
                "members=%zu", candidate_members.size());
            (void)make_formation_proposal(candidate_members);
        }
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
    const ego_swarm::Vector3& target) const
{
    ego_swarm::PositionArrivalResult result;
    ego_swarm::Vector3 target_local{};
    if (!get_target_in_local_ned(target, target_local))
    {
        return result;
    }

    const ego_swarm::Vector3 current_local{
        static_cast<double>(own_state_.position_ned[0]),
        static_cast<double>(own_state_.position_ned[1]),
        static_cast<double>(own_state_.position_ned[2])};
    return ego_swarm::evaluate_position_arrival(
        current_local, target_local, current_global_ned, target,
        local_arrival_tolerance_m_, global_arrival_tolerance_m_);
}

void EgoSwarmPlannerNode::invalidate_active_plan()
{
    has_active_plan_ = false;
    active_plan_ = ego_swarm::PlannerResult{};
    active_plan_target_ = ego_swarm::Vector3{0.0, 0.0, 0.0};
    active_plan_start_time_ = rclcpp::Time(
        static_cast<int32_t>(0), static_cast<uint32_t>(0), RCL_ROS_TIME);
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

bool EgoSwarmPlannerNode::get_current_target(ego_swarm::Vector3& target) const
{
    if (has_formation_)
    {
        const auto assignment = formation_assignment_.find(agent_index_);
        if (!has_accepted_formation_epoch_ ||
            assignment == formation_assignment_.end())
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

    if (!has_goal_ || !ego_swarm::is_finite_vector(goal_position_))
    {
        return false;
    }
    target = goal_position_;
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
    const double trajectory_start_time_s) const
{
    ego_swarm::SafetyValidationContext context;
    context.self_state = state;
    context.now_s = now_s;
    context.trajectory_start_time_s = trajectory_start_time_s;
    context.max_time_skew_s = max_peer_time_skew_s_;
    context.frame_revision = frame_revision_;
    context.trajectory_frame_revision = frame_revision_;
    context.self_agent_id = agent_index_;
    context.peer_table = &peer_trajectories_;
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
    clear_terminal_hold();
    const auto brake_result = emergency_brake_planner_.plan(state);
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
    active_plan_start_time_ = now;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = true;
    publish_plan(
        active_plan_, state.position, now, active_plan_start_time_,
        active_plan_trajectory_id_);
    publish_status(
        PLANNER_STATE_HOLD, "Emergency brake: " + reason, &active_plan_,
        true, peer_data_valid, now);
    return true;
}

void EgoSwarmPlannerNode::set_goal_from_command(const std::string& command)
{
    const auto parsed = utils::parse_command_string(command);
    if (parsed.command == utils::AgentCommand::FORMATION_CANCEL)
    {
        has_formation_ = false;
        has_goal_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
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
        has_formation_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        return;
    }

    if (!parsed.has_param || !std::isfinite(parsed.param_value))
    {
        return;
    }
    goal_position_ = current;
    bool preserve_altitude = false;
    double forward_m = 0.0;
    double right_m = 0.0;
    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
            if (!attitude_is_fresh())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting horizontal MOVE: attitude is missing, "
                    "invalid, or older than %.2f s; holding current target",
                    move_attitude_timeout_s_);
                return;
            }
            forward_m = parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_BACKWARD:
            if (!attitude_is_fresh())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting horizontal MOVE: attitude is missing, "
                    "invalid, or older than %.2f s; holding current target",
                    move_attitude_timeout_s_);
                return;
            }
            forward_m = -parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_LEFT:
            if (!attitude_is_fresh())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting horizontal MOVE: attitude is missing, "
                    "invalid, or older than %.2f s; holding current target",
                    move_attitude_timeout_s_);
                return;
            }
            right_m = -parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_RIGHT:
            if (!attitude_is_fresh())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Rejecting horizontal MOVE: attitude is missing, "
                    "invalid, or older than %.2f s; holding current target",
                    move_attitude_timeout_s_);
                return;
            }
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
        double yaw_rad = 0.0;
        ego_swarm::Vector3 body_offset{};
        if (!ego_swarm::quaternion_to_ned_yaw(own_attitude_.q, yaw_rad) ||
            !ego_swarm::body_horizontal_offset_to_ned(
                yaw_rad, forward_m, right_m, body_offset))
        {
            RCLCPP_WARN(
                get_logger(),
                "Rejecting horizontal MOVE: attitude cannot produce finite "
                "body-to-NED transform");
            return;
        }
        // 這個 offset 只在命令 callback 計算一次；後續 yaw 變化不會
        // 旋轉既有目標。
        goal_position_ = ego_swarm::add_vectors(goal_position_, body_offset);
    }
    has_goal_ = ego_swarm::is_finite_vector(goal_position_);
    has_formation_ = false;
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
    const bool terminal_hold)
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
    local_trajectory_pub_->publish(message);
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
    status.planning_time_ms = 0.0F;
    status.min_obstacle_distance_m = result == nullptr
        ? 0.0F : static_cast<float>(result->minimum_obstacle_distance_m);
    status.min_peer_distance_m = result == nullptr
        ? 0.0F : static_cast<float>(result->minimum_peer_distance_m);
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
