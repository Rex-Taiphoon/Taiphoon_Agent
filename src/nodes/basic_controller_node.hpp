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

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "controllers/BasicController.hpp"
#include "controllers/AgentCommandHandler.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"
#include "taiphoon_agent/msg/agent_state.hpp"
#include "taiphoon_agent/msg/agent_attitude.hpp"
#include "safety/PositionSafetyMonitor.hpp"
#include "safety/StatePublisher.hpp"

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
 * 1. 訂閱 PX4 里程計主題以獲取位置資訊
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

    // ==================== 時間 ====================

    rclcpp::Time start_time_;

    // ==================== 訂閱者 ====================

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        odometry_sub_;

    /// NED 目標位置訂閱者 (來自 formation_node)
    rclcpp::Subscription<taiphoon_agent::msg::NedTarget>::SharedPtr
        ned_target_sub_;

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

    // ==================== 命令處理器 ====================

    std::unique_ptr<controllers::AgentCommandHandler> agent_cmd_handler_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr state_publish_timer_;

    // ==================== 里程計快取 ====================

    /// 最新里程計訊息快取（用於狀態發布）
    px4_msgs::msg::VehicleOdometry latest_odometry_;

    /// 是否已收到第一筆里程計資料
    bool has_odometry_data_{false};

    // ==================== 狀態旗標 ====================

    bool sequence_started_{false};
    bool auto_start_sequence_{true};
};

}  // namespace nodes
}  // namespace taiphoon
