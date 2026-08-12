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
}

EgoSwarmPlannerNode::EgoSwarmPlannerNode()
    : rclcpp::Node("ego_swarm_planner")
{
    declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    declare_parameter<int>("agent_index", 1);
    declare_parameter<double>("planning_rate_hz", DEFAULT_EGO_PLANNER_RATE_HZ);
    declare_parameter<double>("formation_min_distance_m",
                             DEFAULT_FORMATION_MIN_DISTANCE_M);
    declare_parameter<double>("peer_collision_min_distance_m",
                             DEFAULT_PEER_COLLISION_MIN_DISTANCE_M);
    declare_parameter<double>("obstacle_clearance_m",
                             DEFAULT_OBSTACLE_CLEARANCE_M);
    declare_parameter<double>("knot_interval_s", 1.0);
    declare_parameter<double>("candidate_offset_m", 3.0);

    agent_namespace_ = get_parameter("agent_namespace").as_string();
    agent_index_ = get_parameter("agent_index").as_int();
    if (agent_index_ <= 0)
    {
        agent_index_ = compute_self_index();
    }

    planning_rate_hz_ = get_parameter("planning_rate_hz").as_double();
    if (!std::isfinite(planning_rate_hz_) || planning_rate_hz_ <= 0.0)
    {
        planning_rate_hz_ = DEFAULT_EGO_PLANNER_RATE_HZ;
    }

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
    planner_config_.candidate_offset_m = get_parameter(
        "candidate_offset_m").as_double();
    if (!planner_.set_config(planner_config_))
    {
        RCLCPP_ERROR(get_logger(), "EGO-Swarm planner parameters are invalid");
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
                "peer_distance=%.2f m obstacle_clearance=%.2f m",
                agent_index_, agent_namespace_.c_str(), planning_rate_hz_,
                planner_config_.peer_collision_min_distance_m,
                planner_config_.obstacle_clearance_m);
}

bool EgoSwarmPlannerNode::initialize_subscribers()
{
    const std::string own_state_topic = "/" + agent_namespace_ + "/state";
    own_state_sub_ = create_subscription<taiphoon_agent::msg::AgentState>(
        own_state_topic, rclcpp::QoS(10).best_effort(),
        std::bind(&EgoSwarmPlannerNode::own_state_cb, this,
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

    return own_state_sub_ != nullptr && ned_target_sub_ != nullptr &&
        mission_intent_sub_ != nullptr && all_command_sub_ != nullptr &&
        obstacle_sub_ != nullptr && trajectory_sub_ != nullptr;
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

    return trajectory_pub_ != nullptr && local_trajectory_pub_ != nullptr &&
        status_pub_ != nullptr;
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
        return;
    }

    if (parsed.command != utils::AgentCommand::FORMATION_CIRCLE &&
        parsed.command != utils::AgentCommand::FORMATION_HORI_LINEAR &&
        parsed.command != utils::AgentCommand::FORMATION_VERT_LINEAR)
    {
        return;
    }
    if (!parsed.has_multi_params || !std::isfinite(parsed.param_value) ||
        !std::isfinite(parsed.param2))
    {
        RCLCPP_WARN(get_logger(), "Rejecting invalid formation command");
        return;
    }

    if (parsed.command == utils::AgentCommand::FORMATION_CIRCLE)
    {
        formation_config_.type = ego_swarm::FormationType::CIRCLE;
        formation_config_.radius_m = parsed.param_value;
        formation_config_.vertical_offset_m = -parsed.param2;
    }
    else if (parsed.command == utils::AgentCommand::FORMATION_HORI_LINEAR)
    {
        formation_config_.type = ego_swarm::FormationType::HORI_LINEAR;
        formation_config_.spacing_m = parsed.param_value;
        formation_config_.vertical_offset_m = -parsed.param2;
    }
    else
    {
        formation_config_.type = ego_swarm::FormationType::VERT_LINEAR;
        formation_config_.spacing_m = parsed.param_value;
        formation_config_.vertical_offset_m = 0.0;
    }

    if (!update_swarm_frame())
    {
        RCLCPP_WARN(get_logger(),
                    "Formation accepted but GPS common frame is not ready");
    }
    else
    {
        std::vector<int> ids;
        ids.push_back(agent_index_);
        for (const auto& entry : peers_)
        {
            if (entry.second.has_state &&
                is_finite_state_position(entry.second.state))
            {
                ids.push_back(entry.first);
            }
        }
        formation_center_ = ego_swarm::Vector3{0.0, 0.0, 0.0};
        std::size_t count = 0U;
        ego_swarm::Vector3 position{};
        if (has_own_state_ && get_position_in_swarm_frame(
                own_state_, position))
        {
            formation_center_ = ego_swarm::add_vectors(
                formation_center_, position);
            ++count;
        }
        for (const auto& entry : peers_)
        {
            if (entry.second.has_state && get_position_in_swarm_frame(
                    entry.second.state, position))
            {
                formation_center_ = ego_swarm::add_vectors(
                    formation_center_, position);
                ++count;
            }
        }
        if (count > 0U)
        {
            formation_center_ = ego_swarm::scale_vector(
                formation_center_, 1.0 / static_cast<double>(count));
            has_formation_ = true;
        }
    }
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

    obstacle_points_ = points;
    has_obstacle_cloud_ = true;
}

void EgoSwarmPlannerNode::trajectory_cb(
    const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(get_logger(), "trajectory_cb received nullptr");
        return;
    }
    if (msg->agent_index == static_cast<uint32_t>(agent_index_) ||
        !msg->valid || msg->order != ego_swarm::BSPLINE_DEGREE)
    {
        return;
    }

    if (frame_revision_ == 0U || msg->frame_revision != frame_revision_)
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: frame revision %u does not "
            "match local revision %u",
            msg->agent_index, msg->frame_revision, frame_revision_);
        return;
    }

    const ego_swarm::GeoCoordinate message_origin{
        msg->frame_origin_global[0],
        msg->frame_origin_global[1],
        msg->frame_origin_global[2]};
    if (!ego_swarm::SwarmFrame::are_origins_compatible(
            frame_origin_, message_origin))
    {
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: frame origin does not "
            "match the local swarm frame",
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
        const ego_swarm::Vector3 converted{
            static_cast<double>(msg->control_points.at(index)),
            static_cast<double>(msg->control_points.at(index + 1U)),
            static_cast<double>(msg->control_points.at(index + 2U))};
        if (!ego_swarm::is_finite_vector(converted))
        {
            return;
        }
        points.push_back(converted);
    }

    ego_swarm::PeerTrajectory peer;
    peer.agent_id = static_cast<int>(msg->agent_index);
    peer.start_time_s = static_cast<double>(msg->start_time.sec) +
        static_cast<double>(msg->start_time.nanosec) * 1.0e-9;
    peer.trajectory = ego_swarm::BsplineTrajectory(
        points, static_cast<double>(msg->knot_interval_s));

    if (!peer.trajectory.is_valid() ||
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
        RCLCPP_WARN(
            get_logger(),
            "Rejecting peer trajectory agent=%u: peer trajectory table "
            "rejected the validated trajectory",
            msg->agent_index);
    }
}

void EgoSwarmPlannerNode::planner_loop_cb()
{
    const rclcpp::Time now = this->now();
    if (!has_own_state_ || !update_swarm_frame())
    {
        publish_status(PLANNER_STATE_HOLD, "GPS common frame unavailable",
                       nullptr, false, !peers_.empty(), now);
        return;
    }

    ego_swarm::Vector3 current{};
    if (!get_position_in_swarm_frame(own_state_, current))
    {
        publish_status(PLANNER_STATE_HOLD, "Own GPS position unavailable",
                       nullptr, false, !peers_.empty(), now);
        return;
    }

    ego_swarm::Vector3 target{};
    if (!get_current_target(target))
    {
        target = current;
    }

    const auto result = planner_.plan(
        current, target, obstacle_points_, peer_trajectories_,
        now.seconds(), agent_index_);
    publish_plan(result, current, now);
    publish_status(
        result.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
        result.is_hold ? "No safe candidate; holding position" : "ok",
        &result, true, !peers_.empty(), now);
}

void EgoSwarmPlannerNode::scan_peers_cb()
{
    const auto found = formation::PeerMatcher::filter_agent_nodes(
        get_node_names(), agent_namespace_);
    std::set<int> discovered(found.begin(), found.end());
    for (const int index : found)
    {
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
            peers_.erase(iterator->first);
            iterator = peer_subscriptions_.erase(iterator);
        }
        else
        {
            ++iterator;
        }
    }
}

bool EgoSwarmPlannerNode::update_swarm_frame()
{
    std::map<int, ego_swarm::GeoCoordinate> candidates;
    if (has_own_state_ && is_finite_state_position(own_state_))
    {
        candidates[agent_index_] = to_geo(own_state_);
    }
    for (const auto& entry : peers_)
    {
        if (entry.second.has_state &&
            is_finite_state_position(entry.second.state))
        {
            candidates[entry.first] = to_geo(entry.second.state);
        }
    }
    if (candidates.empty())
    {
        return false;
    }

    // 原點 Agent 仍有有效 GPS 時，固定既有 swarm_ned 原點；不能以其
    // 移動中的最新 GPS 重新設定原點，否則每次更新都會產生新 revision。
    const auto current_origin = candidates.find(frame_origin_agent_index_);
    if (current_origin != candidates.end() && swarm_frame_.has_origin())
    {
        return true;
    }

    const auto next_origin = candidates.begin();
    const int next_origin_agent_index = next_origin->first;
    const ego_swarm::GeoCoordinate origin = next_origin->second;
    if (!ego_swarm::SwarmFrame::is_valid_geodetic(origin) ||
        !swarm_frame_.set_origin(origin))
    {
        return false;
    }

    const int previous_origin_agent_index = frame_origin_agent_index_;
    frame_origin_ = origin;
    frame_origin_agent_index_ = next_origin_agent_index;
    ++frame_revision_;
    if (frame_revision_ == 0U)
    {
        frame_revision_ = 1U;
    }
    if (previous_origin_agent_index < 0)
    {
        RCLCPP_INFO(
            get_logger(),
            "Selected Agent %d as swarm_ned frame origin, revision=%u",
            frame_origin_agent_index_, frame_revision_);
    }
    else
    {
        RCLCPP_WARN(
            get_logger(),
            "Swarm_ned frame origin Agent %d unavailable; switched to "
            "Agent %d, revision=%u",
            previous_origin_agent_index, frame_origin_agent_index_,
            frame_revision_);
    }
    return swarm_frame_.has_origin();
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

bool EgoSwarmPlannerNode::get_current_target(ego_swarm::Vector3& target) const
{
    if (has_formation_)
    {
        std::vector<int> ids{agent_index_};
        for (const auto& entry : peers_)
        {
            if (entry.second.has_state &&
                is_finite_state_position(entry.second.state))
            {
                ids.push_back(entry.first);
            }
        }
        ego_swarm::FormationSlot slot;
        if (!ego_swarm::FormationSlotPlanner::compute_slot(
                agent_index_, ids, formation_config_, slot))
        {
            return false;
        }
        std::vector<ego_swarm::FormationSlot> slots;
        if (!ego_swarm::FormationSlotPlanner::compute_all_slots(
                ids, formation_config_, slots) ||
            !ego_swarm::FormationSlotPlanner::are_slots_safe(
                slots, formation_min_distance_m_))
        {
            return false;
        }
        target = ego_swarm::add_vectors(formation_center_, slot.offset_ned);
        return ego_swarm::is_finite_vector(target);
    }

    if (!has_goal_ || !ego_swarm::is_finite_vector(goal_position_))
    {
        return false;
    }
    target = goal_position_;
    return true;
}

void EgoSwarmPlannerNode::set_goal_from_command(const std::string& command)
{
    const auto parsed = utils::parse_command_string(command);
    if (parsed.command == utils::AgentCommand::FORMATION_CANCEL)
    {
        has_formation_ = false;
        has_goal_ = false;
        return;
    }
    if (!has_own_state_ || !update_swarm_frame())
    {
        RCLCPP_WARN(get_logger(),
                    "Mission intent deferred until GPS common frame is ready");
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
        return;
    }

    if (!parsed.has_param || !std::isfinite(parsed.param_value))
    {
        return;
    }
    goal_position_ = current;
    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
            goal_position_.at(0) += parsed.param_value;
            break;
        case utils::AgentCommand::MOVE_BACKWARD:
            goal_position_.at(0) -= parsed.param_value;
            break;
        case utils::AgentCommand::MOVE_LEFT:
            goal_position_.at(1) -= parsed.param_value;
            break;
        case utils::AgentCommand::MOVE_RIGHT:
            goal_position_.at(1) += parsed.param_value;
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
    has_goal_ = ego_swarm::is_finite_vector(goal_position_);
    has_formation_ = false;
}

void EgoSwarmPlannerNode::publish_plan(
    const ego_swarm::PlannerResult& result,
    const ego_swarm::Vector3& origin_position,
    const rclcpp::Time& now)
{
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
    message.start_time = now;
    message.trajectory_id = ++trajectory_id_;
    message.agent_index = static_cast<uint32_t>(agent_index_);
    message.frame_revision = frame_revision_;
    message.order = static_cast<uint8_t>(ego_swarm::BSPLINE_DEGREE);
    message.knot_interval_s = static_cast<float>(
        trajectory.knot_interval_s());
    message.duration_s = static_cast<float>(trajectory.duration_s());
    message.frame_origin_global = {
        frame_origin_.latitude_deg, frame_origin_.longitude_deg,
        frame_origin_.altitude_m};
    message.valid = true;
    message.control_points.reserve(trajectory.control_points().size() * 3U);
    for (const auto& point : trajectory.control_points())
    {
        message.control_points.push_back(static_cast<float>(point.at(0)));
        message.control_points.push_back(static_cast<float>(point.at(1)));
        message.control_points.push_back(static_cast<float>(point.at(2)));
    }
    (void)origin_position;
    trajectory_pub_->publish(message);
    local_trajectory_pub_->publish(message);
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
    status.map_valid = has_obstacle_cloud_;
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
