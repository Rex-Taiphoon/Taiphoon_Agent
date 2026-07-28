/**
 * @file formation_node.hpp
 * @brief 編隊節點 — 接收編隊指令，計算多機相對位置，發布 NED 目標
 * @author Taiphoon Agent Team
 * @date 2026-07-28
 *
 * 訂閱：
 *   - /fmu{N}/fmu/out/vehicle_odometry (所有無人機的位置)
 *   - /fmu_all_agent/command (FORMATION_* 編隊命令)
 *
 * 發布：
 *   - /fmu{N}_agent/ned_target (自身目標 NED 位置)
 *
 * 編隊類型：
 *   - FORMATION_CIRCLE R alt     (環形編隊)
 *   - FORMATION_HORI_LINEAR s alt (水平直列)
 *   - FORMATION_VERT_LINEAR s    (垂直直列)
 */

#pragma once

#include <string>
#include <vector>
#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "taiphoon_agent/msg/ned_target.hpp"

#include "utils/CommandTypes.hpp"

namespace taiphoon
{
namespace nodes
{

/// 編隊類型
enum class FormationType : uint8_t
{
    NONE = 0,         ///< 無編隊
    CIRCLE,           ///< 環形編隊
    HORI_LINEAR,      ///< 水平直列
    VERT_LINEAR       ///< 垂直直列
};

/// 單一無人機的位置與狀態
struct DroneState
{
    std::array<float, 3> position_ned{0.0F, 0.0F, 0.0F};  ///< NED 位置
    rclcpp::Time last_update;                               ///< 最後更新時間
    bool online{false};                                     ///< 是否在線
};

/// 預設參數
constexpr double DEFAULT_DRONE_COUNT = 3.0;
constexpr double DEFAULT_FORMATION_RATE_HZ = 5.0;
constexpr double DRONE_TIMEOUT_S = 2.0;       ///< 無人機離線逾時 (秒)
constexpr float POSITION_CHANGE_THRESHOLD_M = 0.2F;  ///< 位置變化發布閾值
constexpr float YAW_CHANGE_THRESHOLD_DEG = 1.0F;     ///< 偏航變化發布閾值

/**
 * @brief 編隊節點
 *
 * 每個 Taiphoon_Agent 執行一個 FormationNode，
 * 獨立計算自身在編隊中的目標位置。
 */
class FormationNode : public rclcpp::Node
{
public:
    FormationNode();
    ~FormationNode() override = default;

private:
    // ==================== 回呼 ====================

    /**
     * @brief 里程計回呼 (泛用，根據 index 寫入對應 DroneState)
     */
    void odometry_cb(size_t drone_index,
                     const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    /**
     * @brief 全體 Agent 命令回呼 (解析 FORMATION_* 命令)
     */
    void all_agent_command_cb(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief 編隊控制迴圈 (由計時器驅動，5Hz)
     */
    void formation_loop_cb();

    // ==================== 編隊邏輯 ====================

    /**
     * @brief 計算所有線上無人機的質心
     *
     * @return 質心 NED 座標 (x, y, z)
     */
    std::array<float, 3> compute_centroid() const;

    /**
     * @brief 計算編隊中此無人機的目標位置
     *
     * @param centroid 所有無人機的質心
     * @return 目標 NED 位置 (x, y, z) 與偏航角 (yaw_deg)
     */
    std::array<float, 4> compute_formation_target(
        const std::array<float, 3>& centroid) const;

    /**
     * @brief 取得線上無人機數量
     */
    size_t get_online_count() const;

    // ==================== 初始化 ====================

    /**
     * @brief 建立所有無人機的里程計訂閱者
     *
     * @return true 建立成功
     */
    bool initialize_odometry_subscribers();

    /**
     * @brief 建立編隊命令訂閱者
     *
     * @return true 建立成功
     */
    bool initialize_command_subscriber();

    /**
     * @brief 建立 NED 目標發布者
     *
     * @return true 建立成功
     */
    bool initialize_ned_target_publisher();

    // ==================== 參數 ====================

    int drone_count_{3};
    std::string agent_namespace_{"fmu1_agent"};
    double formation_update_rate_hz_{DEFAULT_FORMATION_RATE_HZ};
    int self_index_{0};  ///< 自身在編隊中的索引 (0-based)

    // ==================== 編隊狀態 ====================

    FormationType formation_type_{FormationType::NONE};
    float formation_param1_{0.0F};  ///< 半徑 (circle) / 間距 (linear)
    float formation_param2_{0.0F};  ///< 高度 (共用)

    /// 所有無人機的狀態
    std::vector<DroneState> drone_states_;

    /// 上次發布的目標 (用於變化檢測)
    std::array<float, 4> last_published_target_{0.0F, 0.0F, 0.0F,
        std::numeric_limits<float>::quiet_NaN()};

    // ==================== 訂閱/發布 ====================

    /// 所有無人機的里程計訂閱者
    std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr>
        odometry_subs_;

    /// 全體 Agent 命令訂閱者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        all_agent_command_sub_;

    /// NED 目標發布者
    rclcpp::Publisher<taiphoon_agent::msg::NedTarget>::SharedPtr
        ned_target_pub_;

    /// 編隊控制迴圈計時器
    rclcpp::TimerBase::SharedPtr formation_loop_timer_;
};

}  // namespace nodes
}  // namespace taiphoon
