/**
 * @file MissionPlanner.hpp
 * @brief 任務規劃層節點 — 同儕發現、編隊計算、目標發布
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 取代 FormationNode，新增同儕發現機制：
 * 1. 動態掃描 ROS2 topic 發現線上同儕 Agent
 * 2. 訂閱同儕的 AgentState topic 取得位置
 * 3. 接收 /fmu_all_agent/command 編隊命令
 * 4. 計算自身目標 NED 位置並發布至 /{namespace}/ned_target
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"

#include "formation/FormationCalculator.hpp"
#include "formation/PeerMatcher.hpp"
#include "utils/CommandTypes.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_FORMATION_RATE_HZ = 5.0;
constexpr double DEFAULT_PEER_SCAN_INTERVAL_S = 3.0;
constexpr float POSITION_CHANGE_THRESHOLD_M = 0.2F;
constexpr float YAW_CHANGE_THRESHOLD_DEG = 1.0F;

/**
 * @brief 任務規劃層節點
 *
 * 每個 Taiphoon_Agent 執行一個 MissionPlanner，
 * 透過同儕發現動態追蹤線上 Agent，計算自身在編隊中的目標位置。
 */
class MissionPlanner : public rclcpp::Node
{
public:
    MissionPlanner();
    ~MissionPlanner() override = default;

private:
    // ==================== 回呼 ====================

    /**
     * @brief 全體 Agent 命令回呼 (解析 FORMATION_* 命令)
     */
    void all_agent_command_cb(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief 同儕 AgentState 回呼
     *
     * @param agent_index 同儕 agent 索引
     * @param msg AgentState 訊息
     */
    void peer_state_cb(int agent_index,
                       const taiphoon_agent::msg::AgentState::SharedPtr msg);

    /**
     * @brief 編隊控制迴圈 (由計時器驅動，5Hz)
     */
    void formation_loop_cb();

    /**
     * @brief 同儕掃描計時器回呼 (3 秒週期)
     */
    void scan_peers_cb();

    // ==================== 初始化 ====================

    /**
     * @brief 建立編隊命令訂閱者
     */
    bool initialize_command_subscriber();

    /**
     * @brief 建立 NED 目標發布者
     */
    bool initialize_ned_target_publisher();

    /**
     * @brief 初始同儕發現 (建構時呼叫一次)
     */
    void initial_peer_discovery();

    /**
     * @brief 從 agent_namespace 推算自身索引
     *
     * "fmu1_agent" → 0, "fmu2_agent" → 1, ...
     */
    int compute_self_index() const;

    /**
     * @brief 將 peers_ map 轉為排序後的向量，供 FormationCalculator 使用
     */
    std::vector<formation::PeerPosition> build_sorted_peers() const;

    // ==================== 參數 ====================

    std::string agent_namespace_{"fmu1_agent"};
    double formation_update_rate_hz_{DEFAULT_FORMATION_RATE_HZ};
    double peer_scan_interval_s_{DEFAULT_PEER_SCAN_INTERVAL_S};
    int self_index_{0};

    // ==================== 編隊狀態 ====================

    formation::FormationType formation_type_{formation::FormationType::NONE};
    float formation_param1_{0.0F};  ///< 半徑 (circle) / 間距 (linear)
    float formation_param2_{0.0F};  ///< 高度 (共用)

    /// 同儕狀態: agent_index → PeerPosition
    std::map<int, formation::PeerPosition> peers_;

    /// 同儕訂閱者: agent_index → Subscription
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        peer_subs_;

    // ==================== 發布/訂閱 ====================

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        all_agent_command_sub_;
    rclcpp::Publisher<taiphoon_agent::msg::NedTarget>::SharedPtr
        ned_target_pub_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr formation_loop_timer_;
    rclcpp::TimerBase::SharedPtr peer_scan_timer_;

    // ==================== 變化檢測 ====================

    std::array<float, 4> last_published_target_{
        0.0F, 0.0F, 0.0F, std::numeric_limits<float>::quiet_NaN()};
};

}  // namespace nodes
}  // namespace taiphoon
