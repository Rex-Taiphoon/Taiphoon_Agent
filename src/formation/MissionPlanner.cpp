/**
 * @file MissionPlanner.cpp
 * @brief 任務規劃層節點實作
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 */

#include "formation/MissionPlanner.hpp"

#include <cmath>
#include <sstream>
#include <algorithm>
#include <regex>
#include <utility>
#include <set>

namespace taiphoon
{
namespace nodes
{

// =============================================================================
// 建構子
// =============================================================================

MissionPlanner::MissionPlanner()
    : rclcpp::Node("mission_planner")
{
    // 宣告 ROS2 參數
    this->declare_parameter<std::string>("agent_namespace", "fmu1_agent");
    this->declare_parameter<double>("formation_update_rate_hz",
                                    DEFAULT_FORMATION_RATE_HZ);
    this->declare_parameter<double>("peer_scan_interval_s",
                                    DEFAULT_PEER_SCAN_INTERVAL_S);

    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    formation_update_rate_hz_ =
        this->get_parameter("formation_update_rate_hz").as_double();
    peer_scan_interval_s_ =
        this->get_parameter("peer_scan_interval_s").as_double();

    self_index_ = compute_self_index();

    RCLCPP_INFO(this->get_logger(),
                "MissionPlanner: agent_namespace=%s, self_index=%d, "
                "formation_rate=%.1f Hz, peer_scan=%.1f s",
                agent_namespace_.c_str(), self_index_,
                formation_update_rate_hz_, peer_scan_interval_s_);

    // --- 初始化各組件 ---
    if (!initialize_command_subscriber())
    {
        RCLCPP_ERROR(this->get_logger(), "Command subscriber init failed");
        return;
    }

    if (!initialize_ned_target_publisher())
    {
        RCLCPP_ERROR(this->get_logger(), "NED target publisher init failed");
        return;
    }

    // --- 初始同儕發現 ---
    initial_peer_discovery();

    // --- 建立編隊控制迴圈計時器 ---
    auto ctrl_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 / formation_update_rate_hz_));
    formation_loop_timer_ = this->create_wall_timer(
        ctrl_period_ms,
        std::bind(&MissionPlanner::formation_loop_cb, this));

    // --- 建立同儕掃描計時器 ---
    auto scan_period_ms = std::chrono::milliseconds(
        static_cast<int>(1000.0 * peer_scan_interval_s_));
    peer_scan_timer_ = this->create_wall_timer(
        scan_period_ms,
        std::bind(&MissionPlanner::scan_peers_cb, this));

    RCLCPP_INFO(this->get_logger(),
                "MissionPlanner initialized successfully");
}

// =============================================================================
// 初始化
// =============================================================================

bool MissionPlanner::initialize_command_subscriber()
{
    all_agent_command_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/fmu_all_agent/command",
            rclcpp::QoS(10).best_effort(),
            std::bind(&MissionPlanner::all_agent_command_cb,
                      this, std::placeholders::_1));

    if (all_agent_command_sub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to subscribe to /fmu_all_agent/command");
        return false;
    }
    return true;
}

bool MissionPlanner::initialize_ned_target_publisher()
{
    std::string topic = "/" + agent_namespace_ + "/ned_target";
    ned_target_pub_ =
        this->create_publisher<taiphoon_agent::msg::NedTarget>(
            topic, rclcpp::QoS(10));

    if (ned_target_pub_ == nullptr)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create NED target publisher on '%s'",
                     topic.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "NED target publisher: %s", topic.c_str());
    return true;
}

int MissionPlanner::compute_self_index() const
{
    // "fmu1_agent" → 0, "fmu2_agent" → 1, ...
    // 從結尾反查下底線位置，找出 fmu 後的數字
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

void MissionPlanner::initial_peer_discovery()
{
    // 使用 peer scan 的邏輯
    scan_peers_cb();
}

void MissionPlanner::scan_peers_cb()
{
    // 使用 node 名稱取代 topic 名稱進行同儕發現
    // node 在程序崩潰時立即從 ROS2 graph 消失，不像 topic 有 discovery 快取殘留
    auto node_names = this->get_node_names();

    // 過濾出 agent 節點索引 (不含自身)
    auto found_indices = formation::PeerMatcher::filter_agent_nodes(
        node_names, agent_namespace_);

    // 加入自身：自身的 AgentState 也需納入 peers_ 供編隊計算
    int self_idx = compute_self_index() + 1;  // 1-based
    std::set<int> required_indices(found_indices.begin(), found_indices.end());
    required_indices.insert(self_idx);

    // 處理新發現的同儕 (訂閱其 AgentState)
    for (int idx : required_indices)
    {
        // 如果已經訂閱過，跳過
        if (peer_subs_.find(idx) != peer_subs_.end())
        {
            continue;
        }

        // 訂閱 AgentState topic：/fmu<N>_agent/state
        std::string topic = "/fmu" + std::to_string(idx) + "_agent/state";

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
            peers_[idx] = formation::PeerPosition();  // 初始化位置
            RCLCPP_INFO(this->get_logger(),
                        "Discovered agent %d on %s", idx, topic.c_str());
        }
    }

    // 檢查離線同儕：已訂閱但 node 已不存在
    std::vector<int> stale_indices;
    for (const auto& entry : peer_subs_)
    {
        if (required_indices.find(entry.first) == required_indices.end())
        {
            stale_indices.push_back(entry.first);
        }
    }

    for (int idx : stale_indices)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Agent %d went offline, removing subscription", idx);
        peer_subs_.erase(idx);
        peers_.erase(idx);
    }
}

// =============================================================================
// 回呼
// =============================================================================

void MissionPlanner::all_agent_command_cb(
    const std_msgs::msg::String::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_WARN(this->get_logger(),
                    "all_agent_command_cb received nullptr, ignoring");
        return;
    }

    auto parsed = utils::parse_command_string(msg->data);

    switch (parsed.command)
    {
        case utils::AgentCommand::FORMATION_CIRCLE:
            formation_type_ = formation::FormationType::CIRCLE;
            formation_param1_ = parsed.param_value;  // 半徑
            formation_param2_ = parsed.param2;        // 高度
            RCLCPP_INFO(this->get_logger(),
                        "FORMATION_CIRCLE: radius=%.2f, altitude=%.2f",
                        formation_param1_, formation_param2_);
            break;

        case utils::AgentCommand::FORMATION_HORI_LINEAR:
            formation_type_ = formation::FormationType::HORI_LINEAR;
            formation_param1_ = parsed.param_value;  // 間距
            formation_param2_ = parsed.param2;        // 高度
            RCLCPP_INFO(this->get_logger(),
                        "FORMATION_HORI_LINEAR: spacing=%.2f, altitude=%.2f",
                        formation_param1_, formation_param2_);
            break;

        case utils::AgentCommand::FORMATION_VERT_LINEAR:
            formation_type_ = formation::FormationType::VERT_LINEAR;
            formation_param1_ = parsed.param_value;  // 間距
            RCLCPP_INFO(this->get_logger(),
                        "FORMATION_VERT_LINEAR: spacing=%.2f",
                        formation_param1_);
            break;

        case utils::AgentCommand::FORMATION_CANCEL:
            formation_type_ = formation::FormationType::NONE;
            formation_param1_ = 0.0F;
            formation_param2_ = 0.0F;
            RCLCPP_INFO(this->get_logger(), "FORMATION_CANCEL");
            break;

        default:
            RCLCPP_WARN(this->get_logger(),
                        "Unknown formation command: %s", msg->data.c_str());
            break;
    }
}

void MissionPlanner::peer_state_cb(
    int agent_index,
    const taiphoon_agent::msg::AgentState::SharedPtr msg)
{
    if (msg == nullptr) return;

    // 更新同儕位置
    auto it = peers_.find(agent_index);
    if (it == peers_.end())
    {
        // 尚未註冊（應不會發生，因為訂閱是在發現時建立的）
        peers_[agent_index] = formation::PeerPosition();
        it = peers_.find(agent_index);
    }

    it->second.position_ned[0] = msg->position_ned[0];
    it->second.position_ned[1] = msg->position_ned[1];
    it->second.position_ned[2] = msg->position_ned[2];
    it->second.online = msg->online;
}

// =============================================================================
// 編隊控制迴圈
// =============================================================================

std::vector<formation::PeerPosition> MissionPlanner::build_sorted_peers() const
{
    std::vector<formation::PeerPosition> result;
    // map 已經依 key (agent_index) 排序，直接取出
    for (const auto& entry : peers_)
    {
        result.push_back(entry.second);
    }
    return result;
}

void MissionPlanner::formation_loop_cb()
{
    // 沒有啟用編隊則不發布
    if (formation_type_ == formation::FormationType::NONE) return;

    // 取得排序後的同儕位置
    auto sorted_peers = build_sorted_peers();

    // 檢查是否有任何線上無人機
    size_t online_count = formation::FormationCalculator::count_online(sorted_peers);
    if (online_count == 0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "No peers online, skipping formation update");
        return;
    }

    // 計算質心
    auto centroid = formation::FormationCalculator::compute_centroid(sorted_peers);

    // 組建編隊參數
    formation::FormationParams params;
    params.param1 = formation_param1_;
    params.param2 = formation_param2_;

    // 動態計算 effective_self_index：自身在所有 online agent 中的排序位置
    // 避免因中間 agent 離線導致 self_index 不連續（如 0, 2）
    int self_key = compute_self_index() + 1;  // 1-based，對應 peers_ key
    std::vector<int> alive_keys;
    for (const auto& entry : peers_)
    {
        if (entry.second.online)
        {
            alive_keys.push_back(entry.first);
        }
    }
    std::sort(alive_keys.begin(), alive_keys.end());

    size_t effective_self_index = 0;
    for (size_t i = 0; i < alive_keys.size(); ++i)
    {
        if (alive_keys[i] == self_key)
        {
            effective_self_index = i;
            break;
        }
    }

    // 計算自身目標位置
    auto target = formation::FormationCalculator::compute_formation_target(
        centroid, formation_type_, params,
        effective_self_index, online_count);

    // 變化檢測：避免重複發布相同目標
    float pos_dx = target[0] - last_published_target_[0];
    float pos_dy = target[1] - last_published_target_[1];
    float pos_dz = target[2] - last_published_target_[2];
    float pos_dist = std::sqrt(pos_dx * pos_dx + pos_dy * pos_dy + pos_dz * pos_dz);

    float yaw_diff = 0.0F;
    if (!std::isnan(target[3]) && !std::isnan(last_published_target_[3]))
    {
        yaw_diff = std::abs(target[3] - last_published_target_[3]);
    }
    else if (!std::isnan(target[3]) || !std::isnan(last_published_target_[3]))
    {
        yaw_diff = YAW_CHANGE_THRESHOLD_DEG + 1.0F;
    }

    if (pos_dist < POSITION_CHANGE_THRESHOLD_M &&
        yaw_diff < YAW_CHANGE_THRESHOLD_DEG)
    {
        return;
    }

    // 發布 NED 目標
    auto ned_msg = taiphoon_agent::msg::NedTarget();
    ned_msg.x = target[0];
    ned_msg.y = target[1];
    ned_msg.z = target[2];
    ned_msg.yaw = target[3];

    ned_target_pub_->publish(ned_msg);
    last_published_target_ = target;
}

}  // namespace nodes
}  // namespace taiphoon
