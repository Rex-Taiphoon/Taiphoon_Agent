/**
 * @file TrajectoryController.hpp
 * @brief 軌跡控制層節點 — 路徑規劃、碰撞檢查、waypoint 發布
 * @author Taiphoon Agent Team
 * @date 2026-07-29
 *
 * 接收目標 NED 位置與同儕狀態，規劃無碰撞路徑。
 * 輸出 waypoint 序列至 /{agent_namespace}/waypoints。
 */

#pragma once

#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <memory>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/waypoint_sequence.hpp"
#include "taiphoon_agent/msg/waypoint.hpp"
#include "std_msgs/msg/string.hpp"

#include "trajectory/TrajectoryPlanner.hpp"
#include "formation/FormationCalculator.hpp"
#include "formation/PeerMatcher.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_TRAJECTORY_RATE_HZ = 10.0;
constexpr double DEFAULT_WAYPOINT_SPACING_M = 1.0;
constexpr double DEFAULT_FLIGHT_SPEED_MPS = 5.0;
constexpr float DEFAULT_SAFETY_MARGIN_M = 3.0F;
constexpr double DEFAULT_PEER_SCAN_INTERVAL_S = 3.0;
constexpr double DEFAULT_TRAJECTORY_PEER_TIMEOUT_S = 2.0;

/**
 * @brief 軌跡控制層節點
 *
 * 每個 Taiphoon_Agent 執行一個 TrajectoryController，
 * 負責：
 * 1. 接收目標 NED 位置（來自 MissionPlanner）
 * 2. 訂閱自身與同儕的 AgentState（目前位置）
 * 3. 規劃無碰撞直線路徑
 * 4. 輸出 waypoint 序列至 /{namespace}/waypoints
 * 5. 預留 /{namespace}/obstacles 訂閱介面
 */
class TrajectoryController : public rclcpp::Node
{
public:
    TrajectoryController();
    ~TrajectoryController() override = default;

private:
    /**
     * @brief 同儕狀態快取
     *
     * 使用本機穩定時鐘判斷資料新鮮度，避免不同 Agent 的 ROS 時鐘誤差。
     */
    struct PeerState
    {
        std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};
        std::array<float, 3> position_global{
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN()};
        std::chrono::steady_clock::time_point last_arrival{};
        bool has_position{false};
        bool is_online{false};
        bool is_px4_state_valid{false};
        bool is_discovered{false};
    };

    // ==================== 回呼 ====================

    void ned_target_cb(const taiphoon_agent::msg::NedTarget::SharedPtr msg);
    void own_state_cb(const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void peer_state_cb(int agent_index,
                       const taiphoon_agent::msg::AgentState::SharedPtr msg);
    void obstacle_cb(const std_msgs::msg::String::SharedPtr msg);
    void trajectory_loop_cb();
    void scan_peers_cb();

    // ==================== 初始化 ====================

    bool initialize_subscribers();
    bool initialize_publishers();

    /**
     * @brief 從 agent_namespace 推算自身索引
     */
    int compute_self_index() const;

    /**
     * @brief 將 peers_ map 轉為排序後的向量
     */
    bool build_peer_positions(
        std::vector<std::array<float, 3>>& peer_positions) const;

    // ==================== 參數 ====================

    std::string agent_namespace_{"uav_1_agent"};
    double trajectory_update_rate_hz_{DEFAULT_TRAJECTORY_RATE_HZ};
    double waypoint_spacing_m_{DEFAULT_WAYPOINT_SPACING_M};
    double flight_speed_mps_{DEFAULT_FLIGHT_SPEED_MPS};
    float safety_margin_m_{DEFAULT_SAFETY_MARGIN_M};
    double peer_timeout_s_{DEFAULT_TRAJECTORY_PEER_TIMEOUT_S};

    // ==================== 目標與狀態 ====================

    /// 是否有目前的目標 NED
    bool has_target_{false};
    std::array<float, 4> target_ned_{0.0F, 0.0F, 0.0F,
        std::numeric_limits<float>::quiet_NaN()};

    /// 自身目前位置（來自 AgentState）
    bool has_current_position_{false};
    std::array<float, 3> current_position_ned_{0.0F, 0.0F, 0.0F};
    bool has_current_global_position_{false};
    std::array<float, 3> current_position_global_{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN()};

    /// 同儕狀態: agent_index → 狀態與接收時間
    std::map<int, PeerState> peer_states_;

    /// 同儕訂閱者: agent_index → Subscription
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        peer_subs_;

    // ==================== 發布/訂閱 ====================

    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr
        ned_target_sub_;
    rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr
        own_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        obstacle_sub_;

    rclcpp::Publisher<taiphoon_agent::msg::WaypointSequence>::SharedPtr
        waypoint_pub_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr trajectory_loop_timer_;
    rclcpp::TimerBase::SharedPtr peer_scan_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
