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
    this->declare_parameter<double>("peer_timeout_s",
                                    DEFAULT_PEER_TIMEOUT_S);

    agent_namespace_ = this->get_parameter("agent_namespace").as_string();
    formation_update_rate_hz_ =
        this->get_parameter("formation_update_rate_hz").as_double();
    peer_scan_interval_s_ =
        this->get_parameter("peer_scan_interval_s").as_double();
    peer_timeout_s_ =
        this->get_parameter("peer_timeout_s").as_double();

    self_index_ = compute_self_index();

    RCLCPP_INFO(this->get_logger(),
                "MissionPlanner: agent_namespace=%s, self_index=%d, "
                "formation_rate=%.1f Hz, peer_scan=%.1f s, peer_timeout=%.1f s",
                agent_namespace_.c_str(), self_index_,
                formation_update_rate_hz_, peer_scan_interval_s_,
                peer_timeout_s_);

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
    // 使用 node 名稱進行同儕發現：agent 節點（/fmu<N>_agent/... 主模式、
    // /fmu<N>/basic_controller_node 相容模式）存在性即為離線判定依據
    auto node_names = this->get_node_names();

    // 過濾出 agent 節點索引 (不含自身)
    auto found_indices = formation::PeerMatcher::filter_agent_nodes(
        node_names, agent_namespace_);

    // 加入自身：自身的 AgentState 也需納入 peers_ 供編隊計算
    int self_idx = compute_self_index() + 1;  // 1-based
    std::set<int> required_indices(found_indices.begin(), found_indices.end());
    required_indices.insert(self_idx);

    // 處理新發現的同儕 (訂閱其 AgentState)
    // 訂閱建立後不因 node 暫時消失而刪除，避免 DDS graph 滯留造成
    // 線上旗標反覆震盪；離線僅以 online 旗標反映
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
            peer_liveness_[idx] = PeerLiveness();
            RCLCPP_INFO(this->get_logger(),
                        "Discovered agent %d on %s", idx, topic.c_str());
        }
    }

    // 更新各同儕的節點存在性（不刪除 peers_ / 訂閱）
    update_peer_liveness(required_indices);
}

void MissionPlanner::update_peer_liveness(
    const std::set<int>& required_indices)
{
    for (auto& entry : peer_liveness_)
    {
        bool present =
            required_indices.find(entry.first) != required_indices.end();
        if (!present && entry.second.node_present)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Agent %d went offline (fmu%d_agent node not found)",
                        entry.first, entry.first);
        }
        entry.second.node_present = present;
    }
}

// =============================================================================
// 回呼
// =============================================================================

bool MissionPlanner::activate_formation_if_safe(
    formation::FormationType type, float param1, float param2)
{
    sweep_peer_online_status();
    const size_t online_count = formation::FormationCalculator::count_online(
        build_sorted_peers());
    formation::FormationParams params;
    params.param1 = param1;
    params.param2 = param2;

    if (online_count < 2)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Formation command rejected: online agents < 2");
        return false;
    }
    if (!formation::FormationCalculator::is_formation_geometry_safe(
            type, params, online_count))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Formation command rejected: target slots violate "
                    "the %.2f m safety margin",
                    static_cast<double>(
                        formation::MIN_FORMATION_SLOT_DISTANCE_M));
        return false;
    }

    formation_type_ = type;
    formation_param1_ = param1;
    formation_param2_ = param2;
    snapshot_centroid();
    return true;
}

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
            if (activate_formation_if_safe(
                    formation::FormationType::CIRCLE,
                    parsed.param_value, parsed.param2))
            {
                RCLCPP_INFO(this->get_logger(),
                            "FORMATION_CIRCLE: radius=%.2f, altitude=%.2f",
                            formation_param1_, formation_param2_);
            }
            break;

        case utils::AgentCommand::FORMATION_HORI_LINEAR:
            if (activate_formation_if_safe(
                    formation::FormationType::HORI_LINEAR,
                    parsed.param_value, parsed.param2))
            {
                RCLCPP_INFO(this->get_logger(),
                            "FORMATION_HORI_LINEAR: spacing=%.2f, altitude=%.2f",
                            formation_param1_, formation_param2_);
            }
            break;

        case utils::AgentCommand::FORMATION_VERT_LINEAR:
            if (activate_formation_if_safe(
                    formation::FormationType::VERT_LINEAR,
                    parsed.param_value, parsed.param2))
            {
                RCLCPP_INFO(this->get_logger(),
                            "FORMATION_VERT_LINEAR: spacing=%.2f",
                            formation_param1_);
            }
            break;

        case utils::AgentCommand::FORMATION_CANCEL:
            formation_type_ = formation::FormationType::NONE;
            formation_param1_ = 0.0F;
            formation_param2_ = 0.0F;
            snapshot_online_set_.clear();
            RCLCPP_INFO(this->get_logger(), "FORMATION_CANCEL");
            break;

        default:
            // 非編隊命令（如 MODE_OFFBOARD、ARM、TAKEOFF）由
            // AgentCommandHandler 處理，此處靜默忽略
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
        peer_liveness_[agent_index] = PeerLiveness();
        it = peers_.find(agent_index);
    }

    it->second.position_ned[0] = msg->position_ned[0];
    it->second.position_ned[1] = msg->position_ned[1];
    it->second.position_ned[2] = msg->position_ned[2];
    it->second.position_global[0] = msg->position_global[0];
    it->second.position_global[1] = msg->position_global[1];
    it->second.position_global[2] = msg->position_global[2];

    // online 旗標由 sweep_peer_online_status() 統一計算，
    // 這裡只記錄原始資訊（到達時間與發送端 online 旗標）
    peer_liveness_[agent_index].last_state_arrival =
        std::chrono::steady_clock::now();
    peer_liveness_[agent_index].last_msg_online = msg->online;
}

void MissionPlanner::sweep_peer_online_status()
{
    auto now = std::chrono::steady_clock::now();

    for (auto& entry : peers_)
    {
        bool node_present = false;
        bool last_msg_online = false;
        std::chrono::steady_clock::time_point last_arrival{};

        auto liveness_it = peer_liveness_.find(entry.first);
        if (liveness_it != peer_liveness_.end())
        {
            node_present = liveness_it->second.node_present;
            last_msg_online = liveness_it->second.last_msg_online;
            last_arrival = liveness_it->second.last_state_arrival;
        }

        bool was_online = entry.second.online;
        bool is_fresh = !formation::FormationCalculator::is_peer_stale(
            last_arrival, now, peer_timeout_s_);
        entry.second.online = node_present && last_msg_online && is_fresh;

        if (was_online && !entry.second.online)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Agent %d marked offline (state timeout or node lost)",
                        entry.first);
        }
        else if (!was_online && entry.second.online)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Agent %d back online", entry.first);
        }
    }
}

void MissionPlanner::snapshot_centroid()
{
    // 先以最新時效更新線上旗標，快照才反映當下成員集合
    sweep_peer_online_status();

    int self_key = compute_self_index() + 1;  // 1-based，對應 peers_ key
    auto self_it = peers_.find(self_key);

    // 記錄快照時的線上索引集合（含自身）
    snapshot_online_set_.clear();
    for (const auto& entry : peers_)
    {
        if (entry.second.online)
        {
            snapshot_online_set_.insert(entry.first);
        }
    }

    if (self_it == peers_.end())
    {
        // 自身尚無記錄：無法以 GPS 換算，退回原點
        formation_centroid_ = {0.0F, 0.0F, 0.0F};
        RCLCPP_WARN(this->get_logger(),
                    "Snapshot centroid: self entry missing, using (0,0,0)");
        return;
    }

    auto sorted_peers = build_sorted_peers();
    auto reference = formation::FormationCalculator::compute_reference_centroid(
        sorted_peers, self_it->second.position_ned,
        self_it->second.position_global);
    formation_centroid_ = reference.centroid;

    if (!reference.gps_based)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Snapshot centroid without GPS correction (raw local frames)");
    }

    RCLCPP_INFO(this->get_logger(),
                "Formation centroid snapshot: (%.2f, %.2f, %.2f), %zu online",
                static_cast<double>(formation_centroid_[0]),
                static_cast<double>(formation_centroid_[1]),
                static_cast<double>(formation_centroid_[2]),
                snapshot_online_set_.size());
}

void MissionPlanner::cancel_formation_for_low_members()
{
    RCLCPP_WARN(this->get_logger(),
                "Formation cancelled (online < 2)");

    // 發布一次「自身目前位置」目標，使無人機原地懸停，
    // 避免 BasicController 繼續追最後一個編隊目標
    int self_key = compute_self_index() + 1;  // 1-based，對應 peers_ key
    auto self_it = peers_.find(self_key);
    if (self_it != peers_.end())
    {
        auto ned_msg = taiphoon_agent::msg::NedTarget();
        ned_msg.x = self_it->second.position_ned[0];
        ned_msg.y = self_it->second.position_ned[1];
        ned_msg.z = self_it->second.position_ned[2];
        ned_msg.yaw = std::numeric_limits<float>::quiet_NaN();

        ned_target_pub_->publish(ned_msg);
        last_published_target_ = {
            ned_msg.x, ned_msg.y, ned_msg.z, ned_msg.yaw};

        RCLCPP_INFO(this->get_logger(),
                    "Published hold target at current position "
                    "(%.2f, %.2f, %.2f)",
                    static_cast<double>(ned_msg.x),
                    static_cast<double>(ned_msg.y),
                    static_cast<double>(ned_msg.z));
    }

    // 直接取消編隊（不自動恢復，需重新廣播編隊指令）
    formation_type_ = formation::FormationType::NONE;
    formation_param1_ = 0.0F;
    formation_param2_ = 0.0F;
    snapshot_online_set_.clear();
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

    // 先更新各同儕的線上狀態（節點存在 + state 時效）
    sweep_peer_online_status();

    // 自身 state 逾時或離線時暫停發布（安全護欄：不對失聯飛控下指令）
    int self_key = compute_self_index() + 1;  // 1-based，對應 peers_ key
    auto self_it = peers_.find(self_key);
    if (self_it == peers_.end() || !self_it->second.online)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Self state stale or offline, skip formation "
                             "target publish");
        return;
    }

    // 目前線上索引集合（含自身）
    std::set<int> current_online_set;
    for (const auto& entry : peers_)
    {
        if (entry.second.online)
        {
            current_online_set.insert(entry.first);
        }
    }

    // 線上少於 2 台：直接取消編隊並原地懸停，避免單機追隨自我參考目標直飛
    if (current_online_set.size() < 2)
    {
        cancel_formation_for_low_members();
        return;
    }

    // 成員集合變化（離線/加入）→ 以目前位置重算質心快照
    if (current_online_set != snapshot_online_set_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Re-snapshot centroid (members changed)");
        snapshot_centroid();
    }

    size_t online_count = current_online_set.size();

    // 組建編隊參數
    formation::FormationParams params;
    params.param1 = formation_param1_;
    params.param2 = formation_param2_;

    if (!formation::FormationCalculator::is_formation_geometry_safe(
            formation_type_, params, online_count))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Formation target publish blocked: target slots violate "
            "the %.2f m safety margin",
            static_cast<double>(formation::MIN_FORMATION_SLOT_DISTANCE_M));
        return;
    }

    // 動態計算 effective_self_index：自身在所有 online agent 中的排序位置
    // 避免因中間 agent 離線導致 self_index 不連續（如 0, 2）
    std::vector<int> alive_keys(current_online_set.begin(),
                                current_online_set.end());
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
        formation_centroid_, formation_type_, params,
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
