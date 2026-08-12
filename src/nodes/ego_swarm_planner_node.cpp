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
    declare_parameter<int>("swarm_origin_agent_index", 1);
    declare_parameter<double>("planning_rate_hz", DEFAULT_EGO_PLANNER_RATE_HZ);
    declare_parameter<double>("formation_min_distance_m",
                             DEFAULT_FORMATION_MIN_DISTANCE_M);
    declare_parameter<double>("peer_collision_min_distance_m",
                             DEFAULT_PEER_COLLISION_MIN_DISTANCE_M);
    declare_parameter<double>("obstacle_clearance_m",
                             DEFAULT_OBSTACLE_CLEARANCE_M);
    declare_parameter<double>("knot_interval_s", 1.0);
    declare_parameter<double>("max_velocity_m_s", 2.0);
    declare_parameter<double>("max_acceleration_m_s2", 1.0);
    declare_parameter<double>("candidate_offset_m", 3.0);
    declare_parameter<double>("local_arrival_tolerance_m",
                             DEFAULT_LOCAL_ARRIVAL_TOLERANCE_M);
    declare_parameter<double>("global_arrival_tolerance_m",
                             DEFAULT_GLOBAL_ARRIVAL_TOLERANCE_M);

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
                "origin_agent=%d peer_distance=%.2f m "
                "obstacle_clearance=%.2f m",
                agent_index_, agent_namespace_.c_str(), planning_rate_hz_,
                swarm_origin_agent_index_,
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

    return own_state_sub_ != nullptr && ned_target_sub_ != nullptr &&
        mission_intent_sub_ != nullptr && agent_command_sub_ != nullptr &&
        all_command_sub_ != nullptr &&
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
    preserve_altitude_ = false;
    invalidate_active_plan();

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

    // 沒有任務或編隊目標時，不應把每一回合的目前位置重新發布成
    // timed trajectory。那會讓 BasicController 追蹤一個隨漂移移動的
    // 目標，失去起飛後固定 hold position／altitude 的錨點。
    if (!has_goal_ && !has_formation_)
    {
        publish_status(
            PLANNER_STATE_HOLD,
            "No mission target; BasicController keeps its hold setpoint",
            nullptr, true, !peers_.empty(), now);
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

    const auto arrival = evaluate_target_arrival(current, target);
    if (arrival.reached)
    {
        // 先發布一條以「目標座標」填滿控制點的固定 hold。不能使用
        // current，因為到達判斷允許誤差，且即時 odometry 仍可能漂移。
        const auto hold_result = planner_.make_exact_hold_result(target);
        has_goal_ = false;
        has_formation_ = false;
        preserve_altitude_ = false;
        invalidate_active_plan();
        if (hold_result.trajectory.is_valid())
        {
            const uint32_t hold_id = ++trajectory_id_;
            publish_plan(hold_result, target, now, now, hold_id);
        }
        publish_status(
            PLANNER_STATE_HOLD,
            "No mission target; BasicController keeps its hold setpoint",
            nullptr, true, !peers_.empty(), now);
        return;
    }

    const bool preserve_altitude = preserve_altitude_ && !has_formation_;
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
        // 重新發布同一條軌跡時保留原始 start_time 與 trajectory_id，讓
        // BasicController 依原本的時間軸持續取樣，不會每 100 ms 重置進度。
        publish_plan(
            active_plan_, current, now, active_plan_start_time_,
            active_plan_trajectory_id_);
        publish_status(
            active_plan_.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
            active_plan_.is_hold ? "Holding until replanning is safe" : "ok",
            &active_plan_, true, !peers_.empty(), now);
        return;
    }

    const auto result = planner_.plan(
        current, target, obstacle_points_, peer_trajectories_,
        now.seconds(), agent_index_, preserve_altitude);
    active_plan_ = result;
    active_plan_target_ = target;
    active_plan_start_time_ = now;
    active_plan_trajectory_id_ = ++trajectory_id_;
    has_active_plan_ = active_plan_.trajectory.is_valid();
    publish_plan(
        active_plan_, current, now, active_plan_start_time_,
        active_plan_trajectory_id_);
    publish_status(
        active_plan_.is_hold ? PLANNER_STATE_HOLD : PLANNER_STATE_RUNNING,
        active_plan_.is_hold ? "No safe candidate; holding position" : "ok",
        &active_plan_, true, !peers_.empty(), now);
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

    // 第一次只接受指定的 deterministic origin。所有 Agent 都等待同一個
    // origin GPS，避免因啟動順序不同而各自選出不同的 swarm_ned frame。
    if (swarm_frame_.has_origin())
    {
        return update_local_swarm_transform();
    }

    const auto origin_candidate = candidates.find(swarm_origin_agent_index_);
    if (origin_candidate == candidates.end())
    {
        return false;
    }

    const ego_swarm::GeoCoordinate origin = origin_candidate->second;
    if (!ego_swarm::SwarmFrame::is_valid_geodetic(origin) ||
        !swarm_frame_.set_origin(origin))
    {
        return false;
    }

    frame_origin_ = origin;
    frame_origin_agent_index_ = swarm_origin_agent_index_;
    ++frame_revision_;
    if (frame_revision_ == 0U)
    {
        frame_revision_ = 1U;
    }
    RCLCPP_INFO(
        get_logger(),
        "Selected and froze Agent %d as swarm_ned frame origin, revision=%u",
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
        preserve_altitude_ = false;
        invalidate_active_plan();
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
    switch (parsed.command)
    {
        case utils::AgentCommand::MOVE_FORWARD:
            goal_position_.at(0) += parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_BACKWARD:
            goal_position_.at(0) -= parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_LEFT:
            goal_position_.at(1) -= parsed.param_value;
            preserve_altitude = true;
            break;
        case utils::AgentCommand::MOVE_RIGHT:
            goal_position_.at(1) += parsed.param_value;
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
    has_goal_ = ego_swarm::is_finite_vector(goal_position_);
    has_formation_ = false;
    preserve_altitude_ = preserve_altitude;
    invalidate_active_plan();
}

void EgoSwarmPlannerNode::publish_plan(
    const ego_swarm::PlannerResult& result,
    const ego_swarm::Vector3& origin_position,
    const rclcpp::Time& now,
    const rclcpp::Time& start_time,
    const uint32_t trajectory_id)
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
    message.start_time = start_time;
    message.trajectory_id = trajectory_id;
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
