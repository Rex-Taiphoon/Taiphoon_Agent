/**
 * @file TrajectoryController.cpp
 * @brief 軌跡控制層節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "trajectory/TrajectoryController.hpp"

#include <cmath>
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

    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    trajectory_update_rate_hz_ =
        this->get_parameter("trajectory_update_rate_hz").as_double();
    waypoint_spacing_m_ =
        this->get_parameter("waypoint_spacing_m").as_double();
    flight_speed_mps_ =
        this->get_parameter("flight_speed_mps").as_double();
    safety_margin_m_ =
        static_cast<float>(
            this->get_parameter("safety_margin_m").as_double());

    RCLCPP_INFO(this->get_logger(),
                "TrajectoryController: namespace=%s, rate=%.1f Hz, "
                "spacing=%.2f m, speed=%.1f m/s, margin=%.1f m",
                agent_namespace_.c_str(), trajectory_update_rate_hz_,
                waypoint_spacing_m_, flight_speed_mps_,
                static_cast<double>(safety_margin_m_));

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
    catch (...)
    {
        return 0;
    }
}

// =============================================================================
// 同儕發現
// =============================================================================

void TrajectoryController::scan_peers_cb()
{
    auto topic_names_and_types = this->get_topic_names_and_types();
    std::vector<std::string> all_topics;
    for (const auto& entry : topic_names_and_types)
    {
        all_topics.push_back(entry.first);
    }

    auto found_peers = formation::PeerMatcher::filter_peers(
        all_topics, agent_namespace_);

    for (const auto& peer : found_peers)
    {
        int idx = peer.first;
        const std::string& topic = peer.second;

        if (peer_subs_.find(idx) != peer_subs_.end())
        {
            continue;
        }

        auto sub = this->create_subscription<taiphoon_agent::msg::AgentState>(
            topic,
            rclcpp::QoS(10).best_effort(),
            [this, idx](const taiphoon_agent::msg::AgentState::SharedPtr msg)
            {
                this->peer_state_cb(idx, msg);
            });

        if (sub != nullptr)
        {
            peer_subs_[idx] = sub;
            peer_positions_[idx] = {0.0F, 0.0F, 0.0F};
            RCLCPP_INFO(this->get_logger(),
                        "Discovered peer %d at %s", idx, topic.c_str());
        }
    }

    // 清除離線同儕
    std::map<int, std::string> current_peers;
    for (const auto& peer : found_peers)
    {
        current_peers[peer.first] = peer.second;
    }
    std::vector<int> stale;
    for (const auto& entry : peer_subs_)
    {
        if (current_peers.find(entry.first) == current_peers.end())
        {
            stale.push_back(entry.first);
        }
    }
    for (int idx : stale)
    {
        peer_subs_.erase(idx);
        peer_positions_.erase(idx);
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

    target_ned_[0] = msg->x;
    target_ned_[1] = msg->y;
    target_ned_[2] = msg->z;
    target_ned_[3] = msg->yaw;
    has_target_ = true;
}

void TrajectoryController::own_state_cb(
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr) return;

    current_position_ned_[0] = msg->position_ned[0];
    current_position_ned_[1] = msg->position_ned[1];
    current_position_ned_[2] = msg->position_ned[2];
    has_current_position_ = true;
}

void TrajectoryController::peer_state_cb(
    int agent_index,
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr) return;

    peer_positions_[agent_index][0] = msg->position_ned[0];
    peer_positions_[agent_index][1] = msg->position_ned[1];
    peer_positions_[agent_index][2] = msg->position_ned[2];
}

void TrajectoryController::obstacle_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    // 預留介面：未來外部避障節點可發布障礙物資訊至此 topic
    // 目前不實作處理邏輯
    (void)msg;
}

// =============================================================================
// 軌跡控制迴圈
// =============================================================================

std::vector<std::array<float, 3>>
TrajectoryController::build_peer_positions() const
{
    std::vector<std::array<float, 3>> result;
    for (const auto& entry : peer_positions_)
    {
        result.push_back(entry.second);
    }
    return result;
}

void TrajectoryController::trajectory_loop_cb()
{
    // 需要目標與目前位置才能規劃
    if (!has_target_ || !has_current_position_)
    {
        return;
    }

    // 生成路徑
    auto waypoints = trajectory::TrajectoryPlanner::generate_path(
        current_position_ned_,
        {target_ned_[0], target_ned_[1], target_ned_[2]},
        static_cast<float>(waypoint_spacing_m_));

    // 計算到達時間
    trajectory::TrajectoryPlanner::compute_arrival_times(
        waypoints, this->now().seconds(), flight_speed_mps_);

    // 檢查不安全 waypoint
    auto peer_pos = build_peer_positions();
    auto unsafe_indices = trajectory::TrajectoryPlanner::find_unsafe_waypoints(
        waypoints, peer_pos, safety_margin_m_);

    // 若有不安全 waypoint，簡單時間偏移策略：延遲後續 waypoint 5 秒
    if (!unsafe_indices.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "%zu unsafe waypoints — applying time delay",
                             unsafe_indices.size());

        double extra_delay = 5.0;
        for (size_t idx : unsafe_indices)
        {
            for (size_t j = idx; j < waypoints.size(); ++j)
            {
                waypoints[j].arrival_time_s += extra_delay;
            }
        }
    }

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
