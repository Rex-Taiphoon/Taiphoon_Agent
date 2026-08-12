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
#include <mutex>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/battery_status.hpp"

#include "controllers/BasicController.hpp"
#include "controllers/AgentCommandHandler.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "taiphoon_agent/msg/agent_trajectory.hpp"
#include "std_msgs/msg/string.hpp"
#include "ego_swarm/SwarmFrame.hpp"
#include "safety/PositionSafetyMonitor.hpp"
#include "safety/StatePublisher.hpp"
#include "formation/FormationCalculator.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_TARGET_ALTITUDE_M = 3.0;
constexpr double DEFAULT_HOVER_DURATION_S = 3.0;
constexpr double CONTROL_LOOP_RATE_HZ = 20.0;
constexpr double DEFAULT_STATE_PUBLISH_RATE_HZ = 10.0;
constexpr double DEFAULT_POSITION_DEVIATION_THRESHOLD_M = 2.0;

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
     * @brief AgentTrajectory 回呼
     *
     * 將 WGS84 控制點依本機 Home 轉換為 local NED，交由唯一的
     * BasicController PX4 setpoint publisher 取樣發布。
     *
     * @param msg EGO-Swarm 定時軌跡訊息
     */
    void agent_trajectory_cb(
        const taiphoon_agent::msg::AgentTrajectory::SharedPtr msg);

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
     * @brief 初始化 EGO-Swarm AgentTrajectory 訂閱者
     * @return true 表示訂閱者已建立
     */
    bool initialize_agent_trajectory_subscriber();

    /**
     * @brief 建立送往 EGO-Swarm Planner 的任務意圖發布者
     * @return true 表示發布者建立成功
     */
    bool initialize_mission_intent_publisher();

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

    /// 共用 EGO-Swarm 軌跡訂閱者；只接受屬於本機 agent_index 的訊息
    rclcpp::Subscription<taiphoon_agent::msg::AgentTrajectory>::SharedPtr
        agent_trajectory_sub_;

    // ==================== 發布者 ====================

    /// Agent 對外狀態發布者
    rclcpp::Publisher<taiphoon_agent::msg::AgentState>::SharedPtr
        agent_state_pub_;

    /// Agent 姿態發布者
    rclcpp::Publisher<taiphoon_agent::msg::AgentAttitude>::SharedPtr
        agent_attitude_pub_;

    /// 送往 EGO-Swarm Planner 的任務意圖發布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_intent_pub_;

    // ==================== 控制器 ====================

    std::unique_ptr<controllers::BasicController> controller_;

    // ==================== 安全監控 ====================

    /// 位置偏差安全監控器
    safety::PositionSafetyMonitor position_safety_monitor_;

    // ==================== 命令處理器 ====================

    std::unique_ptr<controllers::AgentCommandHandler> agent_cmd_handler_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr state_publish_timer_;

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

    /// 節點層保留最近一筆已接受軌跡，控制器另有執行時緩衝區
    controllers::TimedTrajectory latest_agent_trajectory_{};
    bool has_latest_agent_trajectory_{false};
    mutable std::mutex agent_trajectory_mutex_;

    /// 自身 Home 對應的全球軌跡轉換 frame；不使用發送端 origin。
    ego_swarm::SwarmFrame trajectory_frame_{};
    ego_swarm::Vector3 trajectory_local_offset_{0.0, 0.0, 0.0};
    bool has_trajectory_frame_{false};

    // ==================== 狀態旗標 ====================

    bool sequence_started_{false};
    bool auto_start_sequence_{true};
};

}  // namespace nodes
}  // namespace taiphoon
