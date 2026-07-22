/**
 * @file basic_controller_node.hpp
 * @brief BasicController ROS2 節點，封裝 BasicController 並管理生命週期
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 *
 * 整合 BasicController 與 AgentCommandHandler，
 * 提供 ROS2 節點層級的飛行控制與高階命令處理。
 */

#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "controllers/BasicController.hpp"
#include "controllers/AgentCommandHandler.hpp"

namespace taiphoon
{
namespace nodes
{

constexpr double DEFAULT_TARGET_ALTITUDE_M = 3.0;
constexpr double DEFAULT_HOVER_DURATION_S = 3.0;
constexpr double CONTROL_LOOP_RATE_HZ = 20.0;

/**
 * @brief BasicController ROS2 節點類別
 *
 * 負責：
 * 1. 訂閱 PX4 里程計主題以獲取位置資訊
 * 2. 以固定頻率驅動 BasicController 發布控制命令
 * 3. 整合 AgentCommandHandler 以接收高階命令
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
     * @brief 控制迴圈計時器回呼
     *
     * 以固定頻率驅動控制器發布 Offboard 控制命令
     */
    void control_loop_cb();

    /**
     * @brief 觸發起飛-降落任務序列
     */
    void trigger_takeoff_land_sequence();

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

    // ==================== 參數 ====================

    double target_altitude_m_{DEFAULT_TARGET_ALTITUDE_M};
    double hover_duration_s_{DEFAULT_HOVER_DURATION_S};
    double startup_delay_s_{0.0};
    int target_system_{1};
    std::string agent_namespace_{"fmu1"};

    // ==================== 時間 ====================

    rclcpp::Time start_time_;

    // ==================== 訂閱者 ====================

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
        odometry_sub_;

    // ==================== 控制器 ====================

    std::unique_ptr<controllers::BasicController> controller_;

    // ==================== 命令處理器 ====================

    std::unique_ptr<controllers::AgentCommandHandler> agent_cmd_handler_;

    // ==================== 計時器 ====================

    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // ==================== 狀態旗標 ====================

    bool sequence_started_{false};
};

}  // namespace nodes
}  // namespace taiphoon
