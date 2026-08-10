/**
 * @file TrajectoryController.cpp
 * @brief 軌跡控制層節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "trajectory/TrajectoryController.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <set>
#include <utility>

namespace taiphoon
{
namespace nodes
{

// =============================================================================
// 建構子
// =============================================================================

TrajectoryController::TrajectoryController()
    : rclcpp::Node("trajectory_controller")
{
    this->declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    this->declare_parameter<double>("trajectory_update_rate_hz",
                                    DEFAULT_TRAJECTORY_RATE_HZ);
    this->declare_parameter<double>("waypoint_spacing_m",
                                    DEFAULT_WAYPOINT_SPACING_M);
    this->declare_parameter<double>("flight_speed_mps",
                                    DEFAULT_FLIGHT_SPEED_MPS);
    this->declare_parameter<double>("safety_margin_m",
                                    static_cast<double>(DEFAULT_SAFETY_MARGIN_M));
    this->declare_parameter<double>("peer_timeout_s",
                                    DEFAULT_TRAJECTORY_PEER_TIMEOUT_S);

    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    trajectory_update_rate_hz_ =
        this->get_parameter("trajectory_update_rate_hz").as_double();
    waypoint_spacing_m_ =
        this->get_parameter("waypoint_spacing_m").as_double();
    flight_speed_mps_ =
        this->get_parameter("flight_speed_mps").as_double();
    const double requested_safety_margin_m =
        this->get_parameter("safety_margin_m").as_double();
    safety_margin_m_ = std::isfinite(requested_safety_margin_m)
        ? std::max(
            static_cast<float>(requested_safety_margin_m),
            DEFAULT_SAFETY_MARGIN_M)
        : DEFAULT_SAFETY_MARGIN_M;
    peer_timeout_s_ = this->get_parameter("peer_timeout_s").as_double();

    RCLCPP_INFO(this->get_logger(),
                "TrajectoryController: namespace=%s, rate=%.1f Hz, "
                "spacing=%.2f m, speed=%.1f m/s, margin=%.1f m, "
                "peer_timeout=%.1f s",
                agent_namespace_.c_str(), trajectory_update_rate_hz_,
                waypoint_spacing_m_, flight_speed_mps_,
                static_cast<double>(safety_margin_m_), peer_timeout_s_);

    // --- 初始化 ---
    if (!initialize_subscribers())
    {
        RCLCPP_ERROR(this->get_logger(), "Subscribers init failed");
        return;
    }

    if (!initialize_publishers())
    {
        RCLCPP_ERROR(this->get_logger(), "Publishers init failed");
        return;
    }

    // --- 建立控制迴圈計時器 ---
    auto ctrl_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / trajectory_update_rate_hz_));
    trajectory_loop_timer_ = this->create_wall_timer(
        ctrl_period_ms,
        std::bind(&TrajectoryController::trajectory_loop_cb, this));

    // --- 建立同儕掃描計時器 ---
    auto scan_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 * DEFAULT_PEER_SCAN_INTERVAL_S));
    peer_scan_timer_ = this->create_wall_timer(
        scan_period_ms,
        std::bind(&TrajectoryController::scan_peers_cb, this));

    RCLCPP_INFO(this->get_logger(),
                "TrajectoryController initialized successfully");
}

// =============================================================================
// 初始化
// =============================================================================

bool TrajectoryController::initialize_subscribers()
{
    // 1. NED 目標 (來自 MissionPlanner)
    std::string ned_topic = "/" + agent_namespace_ + "/ned_target";
    ned_target_sub_ =
        this->create_subscription<taiphoon_agent::msg::NedTarget>(
            ned_topic,
            rclcpp::QoS(10).best_effort(),
            std::bind(&TrajectoryController::ned_target_cb,
                      this, std::placeholders::_1));

    if (ned_target_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to subscribe to %s", ned_topic.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", ned_topic.c_str());

    // 2. 自身 AgentState (目前位置)
    std::string state_topic = "/" + agent_namespace_ + "/state";
    own_state_sub_ =
        this->create_subscription<taiphoon_agent::msg::AgentState>(
            state_topic,
            rclcpp::QoS(10).best_effort(),
            std::bind(&TrajectoryController::own_state_cb,
                      this, std::placeholders::_1));

    if (own_state_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to subscribe to %s", state_topic.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", state_topic.c_str());

    // 3. 障礙物介面 (預留，不處理)
    std::string obstacle_topic = "/" + agent_namespace_ + "/obstacles";
    obstacle_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            obstacle_topic,
            rclcpp::QoS(10).best_effort(),
            std::bind(&TrajectoryController::obstacle_cb,
                      this, std::placeholders::_1));

    if (obstacle_sub_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to subscribe to %s (reserved interface)",
                    obstacle_topic.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Reserved obstacle interface: %s", obstacle_topic.c_str());
    }

    return true;
}

bool TrajectoryController::initialize_publishers()
{
    std::string waypoint_topic = "/" + agent_namespace_ + "/waypoints";
    waypoint_pub_ =
        this->create_publisher<taiphoon_agent::msg::WaypointSequence>(
            waypoint_topic, rclcpp::QoS(10));

    if (waypoint_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create waypoint publisher on '%s'",
                     waypoint_topic.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Waypoint publisher: %s", waypoint_topic.c_str());
    return true;
}

int TrajectoryController::compute_self_index() const
{
    size_t underscore_pos = agent_namespace_.find('_');
    if (underscore_pos == std::string::npos ||
        agent_namespace_.substr(0, 3) != "fmu")
    {
        return 0;
    }
    try
    {
        std::string num_str = agent_namespace_.substr(3, underscore_pos - 3);
        int idx = std::stoi(num_str) - 1;
        return (idx >= 0) ? idx : 0;
    }
    catch (const std::exception& error)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to parse agent index from '%s': %s",
                    agent_namespace_.c_str(), error.what());
        return 0;
    }
}

// =============================================================================
// 同儕發現
// =============================================================================

void TrajectoryController::scan_peers_cb()
{
    const auto node_names = this->get_node_names();
    const auto found_indices = formation::PeerMatcher::filter_agent_nodes(
        node_names, agent_namespace_);
    std::set<int> discovered_indices;

    for (const int agent_index : found_indices)
    {
        discovered_indices.insert(agent_index);
        peer_states_[agent_index].is_discovered = true;

        if (peer_subs_.find(agent_index) != peer_subs_.end())
        {
            continue;
        }

        const std::string topic = "/fmu" + std::to_string(agent_index) +
            "_agent/state";
        auto sub = this->create_subscription<taiphoon_agent::msg::AgentState>(
            topic,
            rclcpp::QoS(10).best_effort(),
            [this, agent_index](
                const taiphoon_agent::msg::AgentState::SharedPtr msg)
            {
                this->peer_state_cb(agent_index, msg);
            });

        if (sub != nullptr)
        {
            peer_subs_[agent_index] = sub;
            peer_states_[agent_index] = PeerState{};
            RCLCPP_INFO(this->get_logger(),
                        "Discovered peer %d at %s",
                        agent_index, topic.c_str());
        }
    }

    std::set<int> known_indices;
    for (const auto& peer : peer_states_)
    {
        known_indices.insert(peer.first);
    }

    const auto removed_indices = formation::PeerMatcher::find_removed_indices(
        known_indices, discovered_indices);
    for (const int agent_index : removed_indices)
    {
        peer_subs_.erase(agent_index);
        peer_states_.erase(agent_index);
        RCLCPP_INFO(this->get_logger(),
                    "Removed disappeared peer %d from trajectory safety",
                    agent_index);
    }
}

// =============================================================================
// 回呼實作
// =============================================================================

void TrajectoryController::ned_target_cb(
    const taiphoon_agent::msg::NedTarget::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "ned_target_cb received nullptr, ignoring");
        return;
    }

    if (!std::isfinite(msg->x) || !std::isfinite(msg->y) ||
        !std::isfinite(msg->z) ||
        (!std::isfinite(msg->yaw) && !std::isnan(msg->yaw)))
    {
        RCLCPP_WARN(this->get_logger(),
                    "ned_target_cb received an invalid target, ignoring");
        return;
    }

    target_ned_[0] = msg->x;
    target_ned_[1] = msg->y;
    target_ned_[2] = msg->z;
    target_ned_[3] = msg->yaw;
    has_target_ = true;
}

void TrajectoryController::own_state_cb(
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "own_state_cb received nullptr, ignoring");
        return;
    }

    const bool is_position_valid =
        std::isfinite(msg->position_ned[0]) &&
        std::isfinite(msg->position_ned[1]) &&
        std::isfinite(msg->position_ned[2]);
    if (!msg->online || !msg->px4_state_valid || !is_position_valid)
    {
        has_current_position_ = false;
        return;
    }

    current_position_ned_[0] = msg->position_ned[0];
    current_position_ned_[1] = msg->position_ned[1];
    current_position_ned_[2] = msg->position_ned[2];
    has_current_position_ = true;

    current_position_global_[0] = msg->position_global[0];
    current_position_global_[1] = msg->position_global[1];
    current_position_global_[2] = msg->position_global[2];
    has_current_global_position_ =
        std::isfinite(current_position_global_.at(0)) &&
        std::isfinite(current_position_global_.at(1)) &&
        std::isfinite(current_position_global_.at(2));
}

void TrajectoryController::peer_state_cb(
    int agent_index,
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "peer_state_cb received nullptr for peer %d, ignoring",
                    agent_index);
        return;
    }

    auto& peer_state = peer_states_[agent_index];
    peer_state.last_arrival = std::chrono::steady_clock::now();
    peer_state.is_discovered = true;
    peer_state.is_online = msg->online;
    peer_state.is_px4_state_valid = msg->px4_state_valid;

    const bool is_position_valid =
        std::isfinite(msg->position_ned[0]) &&
        std::isfinite(msg->position_ned[1]) &&
        std::isfinite(msg->position_ned[2]);
    peer_state.has_position = is_position_valid;
    if (!is_position_valid)
    {
        return;
    }

    peer_state.position_ned[0] = msg->position_ned[0];
    peer_state.position_ned[1] = msg->position_ned[1];
    peer_state.position_ned[2] = msg->position_ned[2];
    peer_state.position_global[0] = msg->position_global[0];
    peer_state.position_global[1] = msg->position_global[1];
    peer_state.position_global[2] = msg->position_global[2];
}

void TrajectoryController::obstacle_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "obstacle_cb received nullptr, ignoring");
        return;
    }

    // 預留介面：未來外部避障節點可發布障礙物資訊至此 topic
    // 目前不實作處理邏輯
    (void)msg;
}

// =============================================================================
// 軌跡控制迴圈
// =============================================================================

bool TrajectoryController::build_peer_positions(
    std::vector<std::array<float, 3>>& peer_positions) const
{
    peer_positions.clear();
    peer_positions.reserve(peer_states_.size());
    const auto now = std::chrono::steady_clock::now();
    for (const auto& entry : peer_states_)
    {
        const auto& peer_state = entry.second;
        const bool has_arrival = peer_state.last_arrival !=
            std::chrono::steady_clock::time_point{};
        const double age_s = has_arrival
            ? std::chrono::duration<double>(now - peer_state.last_arrival).count()
            : std::numeric_limits<double>::infinity();
        const bool is_peer_global_valid =
            std::isfinite(peer_state.position_global.at(0)) &&
            std::isfinite(peer_state.position_global.at(1)) &&
            std::isfinite(peer_state.position_global.at(2));
        if (!peer_state.is_discovered || !peer_state.has_position ||
            !peer_state.is_online || !peer_state.is_px4_state_valid ||
            age_s > peer_timeout_s_ || !has_current_global_position_ ||
            !is_peer_global_valid)
        {
            return false;
        }

        std::array<float, 3> common_position{};
        if (!formation::FormationCalculator::compute_common_frame_position(
                current_position_ned_, current_position_global_,
                peer_state.position_global, common_position))
        {
            return false;
        }
        peer_positions.push_back(common_position);
    }
    return true;
}

void TrajectoryController::trajectory_loop_cb()
{
    // 需要目標與目前位置才能規劃
    if (!has_target_ || !has_current_position_)
    {
        return;
    }

    const std::array<float, 3> target_position = {
        target_ned_[0], target_ned_[1], target_ned_[2]};
    std::vector<std::array<float, 3>> peer_positions;
    const bool are_peer_states_valid = build_peer_positions(peer_positions);
    // 不安全時只發布目前位置保持序列，禁止以時間延遲掩蓋空間衝突。
    auto waypoints = are_peer_states_valid
        ? trajectory::TrajectoryPlanner::generate_safe_path(
            current_position_ned_, target_position, peer_positions,
            static_cast<float>(waypoint_spacing_m_), safety_margin_m_)
        : trajectory::TrajectoryPlanner::generate_path(
            current_position_ned_, current_position_ned_,
            static_cast<float>(waypoint_spacing_m_));
    const float target_dx = target_position.at(0) - current_position_ned_.at(0);
    const float target_dy = target_position.at(1) - current_position_ned_.at(1);
    const float target_dz = target_position.at(2) - current_position_ned_.at(2);
    const bool is_movement_requested =
        target_dx * target_dx + target_dy * target_dy + target_dz * target_dz >
        std::numeric_limits<float>::epsilon();
    const bool is_holding_for_safety =
        is_movement_requested && waypoints.size() == 1U;
    if (is_holding_for_safety)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Unsafe trajectory blocked — holding position");
    }

    trajectory::TrajectoryPlanner::compute_arrival_times(
        waypoints, this->now().seconds(), flight_speed_mps_);

    // 發布 waypoint 序列
    auto msg = taiphoon_agent::msg::WaypointSequence();
    msg.stamp = this->now();

    for (const auto& wp : waypoints)
    {
        taiphoon_agent::msg::Waypoint wp_msg;
        wp_msg.position_ned[0] = wp.position_ned[0];
        wp_msg.position_ned[1] = wp.position_ned[1];
        wp_msg.position_ned[2] = wp.position_ned[2];

        // 將相對時間轉換為 ROS2 Time
        int64_t sec = static_cast<int64_t>(wp.arrival_time_s);
        wp_msg.arrival_time.sec = static_cast<int32_t>(sec);
        wp_msg.arrival_time.nanosec = static_cast<uint32_t>(
            (wp.arrival_time_s - static_cast<double>(sec)) * 1e9);

        msg.waypoints.push_back(wp_msg);
    }

    waypoint_pub_->publish(msg);
}

}  // namespace nodes
}  // namespace taiphoon
