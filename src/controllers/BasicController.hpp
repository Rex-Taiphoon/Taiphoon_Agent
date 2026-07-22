/**
 * @file BasicController.hpp
 * @brief Basic flight controller implementing IBasicFlightController
 * @author Taiphoon Agent Team
 * @date 2026-07-22
 */

#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"

#include "core/IBasicFlightController.hpp"

namespace taiphoon
{
namespace controllers
{

/// 飛行狀態機
enum class FlightState : uint8_t
{
    IDLE = 0,
    OFFBOARD_INIT,   ///< 等待 Offboard 模式啟用
    ARMED,
    TAKEOFF,
    HOVER,
    RTL,             ///< 返航 (PX4 自動降落並上鎖)
    LAND,            ///< 降落模式
    MOVING,          ///< 水平移動中
    ERROR
};

/// 暖機心跳時間 (毫秒)
constexpr int HEARTBEAT_WARMUP_MS = 2000;

/// 模式切換後 VehicleCommand 回應超時 (毫秒)
constexpr int VEHICLE_CMD_ECHO_TIMEOUT_MS = 3000;

/// 起飛超時 (毫秒)
constexpr int TAKEOFF_TIMEOUT_MS = 30000;

/// 高度日誌記錄間隔 (控制週期數)
constexpr int ALTITUDE_LOG_INTERVAL = 20;

/// 高度到達比例 (0.95 = 95% 視為到達)
constexpr float ALTITUDE_ARRIVAL_RATIO = 0.95F;

/// RTL 完成等待超時 (毫秒)
constexpr int RTL_TIMEOUT_MS = 60000;

/// 預設 ROS2 QoS depth
constexpr int DEFAULT_QOS_DEPTH = 10;

/// 預設目標系統 ID
constexpr uint8_t DEFAULT_TARGET_SYSTEM = 1;

/**
 * @brief 基礎飛行控制器
 *
 * 實作 IBasicFlightController 介面，透過 PX4 Offboard 模式控制無人機。
 * 提供起飛、降落、返航、水平移動、模式切換等基礎飛行功能。
 */
class BasicController : public core::IBasicFlightController
{
public:
    /**
     * @brief 建構子
     *
     * @param node ROS2 節點指標 (必須不為 nullptr)
     * @param target_system PX4 MAVLink 系統 ID (fmu1=1, fmu2=2, ...)
     */
    explicit BasicController(rclcpp::Node* node,
                             uint8_t target_system = DEFAULT_TARGET_SYSTEM);

    ~BasicController() override = default;

    // ==================== IBasicFlightController 介面實作 ====================

    /** @brief 初始化所有 Publisher 與 Subscriber */
    bool initialize() override;

    /** @brief 清理控制器資源 */
    void shutdown() override;

    /** @brief 檢查控制器是否已初始化 */
    bool is_initialized() const override;

    /** @brief 發送 Arm (解鎖) 命令 */
    bool arm() override;

    /** @brief 發送 Disarm (上鎖) 命令 */
    bool disarm() override;

    /** @brief 起飛至指定高度 (公尺) */
    bool takeoff(float altitude_m) override;

    /** @brief 在空中停留指定秒數 */
    bool hold_position(float duration_s) override;

    /** @brief 降落 */
    bool land() override;

    /** @brief 執行起飛-停留-降落完整任務序列 */
    bool execute_takeoff_land_sequence(
        float altitude_m, float hover_duration_s) override;

    /** @brief 取得目前飛行狀態字串 */
    const char* get_state() const override;

    // ==================== 高階命令新增方法 ====================

    /**
     * @brief 切換至 RTL (Return to Launch) 模式
     *
     * PX4 會自動處理降落、落地偵測與自動上鎖
     *
     * @return true 命令發送成功
     */
    bool switch_to_rtl_mode();

    /**
     * @brief 切換至 LAND 模式
     *
     * PX4 會自動處理降落程序
     *
     * @return true 命令發送成功
     */
    bool switch_to_land_mode();

    /**
     * @brief 切換至 OFFBOARD 模式
     *
     * @return true 命令發送成功
     */
    bool switch_to_offboard_mode();

    /**
     * @brief 從目前位置向前/後移動指定距離
     *
     * @param distance_m 移動距離 (正數為向前，負數為向後)
     * @return true 移動命令設定成功
     */
    bool move_forward(float distance_m);

    /**
     * @brief 從目前位置向左/右移動指定距離
     *
     * @param distance_m 移動距離 (正數為向右，負數為向左)
     * @return true 移動命令設定成功
     */
    bool move_right(float distance_m);

    // ==================== 公開輔助方法 ====================

    /**
     * @brief 更新車輛里程計 (由節點回呼驅動)
     *
     * @param msg VehicleOdometry 訊息
     */
    void update_odometry(const px4_msgs::msg::VehicleOdometry& msg);

    /**
     * @brief 發布 Offboard 控制模式與軌跡設定點 (計時器回呼)
     */
    void publish_control_commands();

    /**
     * @brief 取得目前高度
     *
     * @return float 目前高度 (公尺)
     */
    float get_current_altitude() const;

    /**
     * @brief 取得目前位置 (NED 座標)
     *
     * @return std::array<float, 3> [x, y, z]
     */
    std::array<float, 3> get_current_position_ned() const;

private:
    // ==================== Publisher 初始化 (單一職責) ====================

    /**
     * @brief 建立 OffboardControlMode Publisher
     *
     * 發布至 fmu/in/offboard_control_mode
     *
     * @return true 建立成功
     */
    bool create_offboard_control_mode_publisher();

    /**
     * @brief 建立 TrajectorySetpoint Publisher
     *
     * 發布至 fmu/in/trajectory_setpoint
     *
     * @return true 建立成功
     */
    bool create_trajectory_setpoint_publisher();

    /**
     * @brief 建立 VehicleCommand Publisher
     *
     * 發布至 fmu/in/vehicle_command
     *
     * @return true 建立成功
     */
    bool create_vehicle_command_publisher();

    // ==================== Subscriber 初始化 (單一職責) ====================

    /**
     * @brief 建立 VehicleCommandAck Subscriber
     *
     * 訂閱 fmu/out/vehicle_command_ack_v1，用於確認命令回應
     *
     * @return true 建立成功
     */
    bool create_vehicle_command_ack_subscriber();

    /**
     * @brief 建立 VehicleStatus Subscriber
     *
     * 訂閱 fmu/out/vehicle_status_v4，用於監控解鎖/上鎖狀態
     *
     * @return true 建立成功
     */
    bool create_vehicle_status_subscriber();

    // ==================== 狀態機處理 (單一職責) ====================

    /**
     * @brief 處理 OFFBOARD_INIT 狀態
     *
     * 發送心跳暖機，嘗試切換至 Offboard 模式，等待回應後解鎖
     *
     * @param msg OffboardControlMode 訊息 (用於發布心跳)
     */
    void handle_offboard_init_state(
        px4_msgs::msg::OffboardControlMode& /*msg*/);

    /**
     * @brief 處理 ARMED 狀態
     *
     * 等待 Arm 命令確認後觸發起飛
     */
    void handle_armed_state(
        px4_msgs::msg::OffboardControlMode& /*msg*/);

    /**
     * @brief 處理 TAKEOFF 狀態
     *
     * 設定上升軌跡設定點，監控高度到達狀態
     *
     * @param msg TrajectorySetpoint 訊息 (更新位置)
     */
    void handle_takeoff_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 HOVER 狀態
     *
     * 在目標高度停留指定時間後切換至 RTL
     *
     * @param msg TrajectorySetpoint 訊息 (保持高度)
     */
    void handle_hover_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 RTL 狀態
     *
     * 發送 RTL 命令，等待 PX4 完成降落並上鎖
     *
     * @param msg TrajectorySetpoint 訊息 (停用位置設定)
     */
    void handle_rtl_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 LAND 狀態
     *
     * 發送 LAND 命令，等待 PX4 完成降落
     *
     * @param msg TrajectorySetpoint 訊息 (停用位置設定)
     */
    void handle_land_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    /**
     * @brief 處理 MOVING 狀態
     *
     * 執行水平移動至目標位置
     *
     * @param msg TrajectorySetpoint 訊息 (更新目標位置)
     */
    void handle_moving_state(
        px4_msgs::msg::TrajectorySetpoint& msg);

    // ==================== 輔助方法 ====================

    /**
     * @brief 發送 Arm/Disarm VehicleCommand
     *
     * @param do_arm true 為 Arm，false 為 Disarm
     * @return true 發送成功
     */
    bool send_arm_command(bool do_arm);

    /**
     * @brief 發送切換至 Offboard 模式的 VehicleCommand
     *
     * PX4 1.18: command=176 (VEHICLE_CMD_DO_SET_MODE),
     * param1=1.0 (custom mode), param2=6.0 (Offboard for multicopter)
     *
     * @return true 發送成功
     */
    bool send_offboard_mode_command();

    /**
     * @brief 發送 RTL (Return to Launch) 命令
     *
     * PX4 1.18: command=20 (VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
     *
     * @return true 發送成功
     */
    bool send_rtl_command();

    /**
     * @brief 發送 LAND 命令
     *
     * PX4 1.18: command=21 (VEHICLE_CMD_NAV_LAND)
     *
     * @return true 發送成功
     */
    bool send_land_command();

    /**
     * @brief 驗證設定點數值 (無 NaN/Inf)
     *
     * @param x X 座標
     * @param y Y 座標
     * @param z Z 座標
     * @return true 數值有效
     */
    bool validate_setpoint(float x, float y, float z) const;

    /**
     * @brief VehicleCommandAck 回呼 (PX4 輸出主題)
     *
     * @param msg VehicleCommandAck 訊息
     */
    void vehicle_command_ack_cb(
        const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

    /**
     * @brief VehicleStatus 回呼 (監控解鎖/上鎖狀態)
     *
     * @param msg VehicleStatus 訊息
     */
    void vehicle_status_cb(
        const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    // ==================== 成員變數 ====================

    /// ROS2 節點指標
    rclcpp::Node* node_;

    // --- Publishers ---
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
        offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
        trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
        vehicle_command_pub_;

    // --- Subscribers ---
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr
        vehicle_command_ack_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
        vehicle_status_sub_;

    /// PX4 MAVLink 系統 ID (fmu1=1, fmu2=2, ...)
    uint8_t target_system_{DEFAULT_TARGET_SYSTEM};

    /// 目前高度 (公尺)
    std::atomic<float> current_altitude_m_{0.0F};

    /// 目標高度 (公尺)
    std::atomic<float> target_altitude_m_{0.0F};

    /// 停留持續時間 (秒)
    std::atomic<float> hover_duration_s_{0.0F};

    /// 目前飛行狀態
    std::atomic<FlightState> flight_state_{FlightState::IDLE};

    /// 初始化完成旗標
    std::atomic<bool> is_initialized_{false};

    /// 計時相關
    rclcpp::Time hover_start_time_;
    rclcpp::Time offboard_start_time_;
    rclcpp::Time takeoff_start_time_;
    rclcpp::Time move_start_time_;
    bool is_hover_timer_active_{false};
    bool is_offboard_timer_active_{false};
    bool offboard_cmd_sending_{false};
    int altitude_log_counter_{0};

    /// VehicleCommandAck 確認旗標
    std::atomic<bool> offboard_cmd_echo_received_{false};
    std::atomic<bool> arm_cmd_echo_received_{false};

    /// 車輛解鎖狀態 (1=DISARMED, 2=ARMED, 參閱 px4_msgs/VehicleStatus)
    std::atomic<uint8_t> arming_state_{0};

    /// RTL 命令已發送旗標
    bool rtl_cmd_sent_{false};

    /// LAND 命令已發送旗標
    bool land_cmd_sent_{false};

    /// 目前位置 (NED 座標系) [x, y, z]
    std::array<float, 3> current_position_ned_{0.0F, 0.0F, 0.0F};

    /// 目標位置 (NED 座標系) [x, y, z]，用於移動命令
    std::array<float, 3> target_position_ned_{0.0F, 0.0F, 0.0F};

    /// 移動到位元差 (公尺)，低於此值視為已到達
    float move_arrival_threshold_m_{0.2F};
};

}  // namespace controllers
}  // namespace taiphoon

