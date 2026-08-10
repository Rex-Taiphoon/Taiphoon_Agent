/**
 * @file basic_controller_node.hpp
 * @brief BasicController ROS2 節點，封裝 BasicController 並管理生命週期
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 *
 * 整合 BasicController 與 AgentCommandHandler，
 * 提供 ROS2 節點層級的飛行控制、狀態發布與位置安全監控。
 */

#pragma once

#include <string>
#include <memory>
#include <array>
#include <chrono>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"

#include "controllers/BasicController.hpp"
#include "controllers/AgentCommandHandler.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "safety/PositionSafetyMonitor.hpp"
#include "safety/CollisionSafetyMonitor.hpp"
#include "safety/StatePublisher.hpp"
#include "formation/FormationCalculator.hpp"
#include "formation/PeerMatcher.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_TARGET_ALTITUDE_M = 3.0;
constexpr double DEFAULT_HOVER_DURATION_S = 3.0;
constexpr double CONTROL_LOOP_RATE_HZ = 20.0;
constexpr double DEFAULT_STATE_PUBLISH_RATE_HZ = 10.0;
constexpr double DEFAULT_POSITION_DEVIATION_THRESHOLD_M = 2.0;
constexpr double DEFAULT_COLLISION_SAFETY_MARGIN_M = 3.0;
constexpr double DEFAULT_COLLISION_PEER_TIMEOUT_S = 2.0;
constexpr double DEFAULT_COLLISION_PEER_SCAN_INTERVAL_S = 3.0;

/// 防撞關卡使用的已知同儕狀態
struct CollisionPeerState
{
    taiphoon_agent::msg::AgentState state;
    std::chrono::steady_clock::time_point last_arrival{};
    bool has_state{false};
    bool is_discovered{false};
};

/**
 * @brief BasicController ROS2 節點類別
 *
 * 負責：
 * 1. 訂閱 PX4 里程計、GPS 與電池主題以獲取完整狀態資訊
 * 2. 以固定頻率驅動 BasicController 發布控制命令
 * 3. 發布 Agent 對外狀態（AgentState、AgentAttitude）
 * 4. 位置安全監控：偏差超標時自動觸發 hover
 * 5. 整合 AgentCommandHandler 以接收高階命令
 */
class BasicControllerNode : public rclcpp::Node
{
public:
    BasicControllerNode();
    ~BasicControllerNode() override = default;

private:
    /**
     * @brief PX4 里程計回呼
     *
     * @param msg 里程計訊息
     */
    void odometry_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    /**
     * @brief PX4 全域 GPS 位置回呼
     *
     * @param msg GPS 位置訊息
     */
    void gps_cb(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);

    /**
     * @brief PX4 電池狀態回呼
     *
     * @param msg 電池狀態訊息
     */
    void battery_cb(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

    /**
     * @brief NED 目標位置回呼
     *
     * 接收來自 formation_node 的絕對 NED 位置指令
     *
     * @param msg NED 目標訊息
     */
    void ned_target_cb(const taiphoon_agent::msg::NedTarget::SharedPtr msg);

    /**
     * @brief 控制迴圈計時器回呼
     *
     * 以固定頻率驅動控制器發布 Offboard 控制命令，
     * 並進行位置安全監控
     */
    void control_loop_cb();

    /**
     * @brief Agent 狀態發布計時器回呼
     *
     * 以 state_publish_rate_hz 頻率發布 AgentState 與 AgentAttitude
     */
    void state_publish_cb();

    /**
     * @brief 高階命令處理回呼
     *
     * 將 AgentCommandHandler 解析的命令轉譯為 BasicController 的呼叫
     *
     * @param cmd 解析後的高階命令
     */
    void on_agent_command(const utils::ParsedCommand& cmd);

    /**
     * @brief 接收同儕 AgentState 並更新防撞快取
     * @param agent_index 同儕 Agent 編號
     * @param msg 同儕 AgentState
     */
    void collision_peer_state_cb(
        int agent_index,
        const taiphoon_agent::msg::AgentState::SharedPtr msg);

    /** @brief 掃描同儕 AgentState topic 並維持訂閱生命週期 */
    void scan_collision_peers_cb();

    /**
     * @brief 建立已轉換到自身 NED frame 的同儕觀測
     * @return 防撞監控器輸入；無效同儕仍保留並標記 is_valid=false
     */
    std::vector<safety::PeerCollisionObservation>
        build_collision_peer_observations() const;

    /** @brief 評估最新移動目標並執行套用或原地保持 */
    void evaluate_collision_target();

    /**
     * @brief 登記控制器剛產生的移動目標並立即評估
     * @param yaw_deg 目標偏航角；NaN 表示維持既有偏航
     */
    void register_controller_motion_target(float yaw_deg);

    /**
     * @brief 判斷命令是否會產生位置移動目標
     * @param command 高階命令
     * @return true 表示成功執行後需進入防撞關卡
     */
    static bool is_position_motion_command(utils::AgentCommand command);

    /**
     * @brief 判斷命令是否應清除待執行移動目標並略過防撞保持
     * @param command 高階命令
     * @return true 表示 RTL、Land 或 Disarm 安全中止命令
     */
    static bool is_collision_bypass_command(utils::AgentCommand command);

    /**
     * @brief 初始化 AgentCommandHandler
     *
     * @return true 初始化成功
     */
    bool initialize_agent_command_handler();

    /**
     * @brief 初始化里程計訂閱者
     *
     * @return true 初始化成功
     */
    bool initialize_odometry_subscriber();

    /**
     * @brief 初始化 GPS 訂閱者
     *
     * @return true 初始化成功
     */
    bool initialize_gps_subscriber();

    /**
     * @brief 初始化電池狀態訂閱者
     *
     * @return true 初始化成功
     */
    bool initialize_battery_subscriber();

    /**
     * @brief 初始化 NED 目標訂閱者
     *
     * @return true 初始化成功
     */
    bool initialize_ned_target_subscriber();

    /**
     * @brief 初始化 Agent 狀態發布者
     *
     * @return true 初始化成功
     */
    bool initialize_state_publishers();

    // ==================== 參數 ====================

    double target_altitude_m_{DEFAULT_TARGET_ALTITUDE_M};
    double hover_duration_s_{DEFAULT_HOVER_DURATION_S};
    double startup_delay_s_{0.0};
    int target_system_{1};
    std::string agent_namespace_{"fmu1_agent"};
    double state_publish_rate_hz_{DEFAULT_STATE_PUBLISH_RATE_HZ};
    double position_deviation_threshold_m_{DEFAULT_POSITION_DEVIATION_THRESHOLD_M};
    double collision_safety_margin_m_{DEFAULT_COLLISION_SAFETY_MARGIN_M};
    double collision_peer_timeout_s_{DEFAULT_COLLISION_PEER_TIMEOUT_S};

    // ==================== 時間 ====================

    rclcpp::Time start_time_;

    // ==================== 訂閱者 ====================

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        odometry_sub_;

    /// PX4 全域 GPS 位置訂閱者
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr
        gps_sub_;

    /// PX4 電池狀態訂閱者
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr
        battery_sub_;

    /// NED 目標位置訂閱者 (來自 formation_node)
    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr
        ned_target_sub_;

    /// 動態同儕 AgentState 訂閱者
    std::map<int, rclcpp::Subscription<taiphoon_agent::msg::AgentState>::SharedPtr>
        collision_peer_subs_;

    // ==================== 發布者 ====================

    /// Agent 對外狀態發布者
    rclcpp::Publisher<taiphoon_agent::msg::AgentState>::SharedPtr
        agent_state_pub_;

    /// Agent 姿態發布者
    rclcpp::Publisher<taiphoon_agent::msg::AgentAttitude>::SharedPtr
        agent_attitude_pub_;

    // ==================== 控制器 ====================

    std::unique_ptr<controllers::BasicController> controller_;

    // ==================== 安全監控 ====================

    /// 位置偏差安全監控器
    safety::PositionSafetyMonitor position_safety_monitor_;

    /// 多機移動目標防撞關卡
    safety::CollisionTargetGate collision_target_gate_;

    /// 已知同儕狀態；同儕消失時保留以採取保守保持
    std::map<int, CollisionPeerState> collision_peer_states_;

    // ==================== 命令處理器 ====================

    std::unique_ptr<controllers::AgentCommandHandler> agent_cmd_handler_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr state_publish_timer_;
    rclcpp::TimerBase::SharedPtr collision_peer_scan_timer_;

    // ==================== PX4 資料快取 ====================

    /// 最新里程計訊息快取
    px4_msgs::msg::VehicleOdometry latest_odometry_;

    /// 最新 GPS 位置訊息快取
    px4_msgs::msg::VehicleGlobalPosition latest_gps_;

    /// 最新電池狀態訊息快取
    px4_msgs::msg::BatteryStatus latest_battery_;

    /// 是否已收到第一筆里程計資料
    bool has_odometry_data_{false};

    /// 是否已收到第一筆 GPS 資料
    bool has_gps_data_{false};

    /// 是否已收到第一筆電池資料
    bool has_battery_data_{false};

    // ==================== 狀態旗標 ====================

    bool sequence_started_{false};
    bool auto_start_sequence_{true};
};

}  // namespace nodes
}  // namespace taiphoon
